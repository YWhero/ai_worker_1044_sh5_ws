"""ROS Jazzy sidecar for the isolated Isaac physics process (no ROS in Isaac)."""
import base64
import collections
import json
import math
import socket
import threading
import time
import uuid
from pathlib import Path

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMessage
from robotis_interfaces.msg import HandPressures, TactileSensor
from sensor_msgs.msg import CameraInfo, CompressedImage, JointState, LaserScan
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from trajectory_msgs.msg import JointTrajectory

ARM_HAND = ([f'arm_{s}_joint{i}' for s in ('l', 'r') for i in range(1, 8)]
            + [f'finger_{s}_joint{i}' for s in ('l', 'r') for i in range(1, 21)])
CAMERAS = {
    'head_left': ('/zed/zed_node/left/image_rect_color', 'zedm_left_camera_optical_frame'),
    'head_right': ('/zed/zed_node/right/image_rect_color', 'zedm_right_camera_optical_frame'),
    'wrist_left': ('/camera_left/camera_left/color/image_rect_raw', 'camera_l_color_optical_frame'),
    'wrist_right': ('/camera_right/camera_right/color/image_rect_raw', 'camera_r_color_optical_frame'),
}


def stamp(value):
    ns = round(float(value) * 1e9)
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


class SocketClient:
    def __init__(self, port):
        self.port, self.socket = port, None
        self.lock = threading.Lock()
        self.pending = collections.deque(maxlen=256)
        self.responses = {}
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()

    def read(self):
        while not self.stopped.is_set():
            try:
                connection = socket.create_connection(('127.0.0.1', self.port), timeout=2)
                connection.settimeout(2)
                with self.lock:
                    self.socket = connection
                data = b''
                while not self.stopped.is_set():
                    try:
                        chunk = connection.recv(262144)
                    except socket.timeout:
                        continue
                    if not chunk:
                        break
                    data += chunk
                    if len(data) > 24 * 1024 * 1024:
                        raise ValueError('Isaac packet exceeds 24 MiB')
                    while b'\n' in data:
                        line, data = data.split(b'\n', 1)
                        message = json.loads(line)
                        with self.lock:
                            if message.get('kind') == 'command_result':
                                request = self.responses.get(message.get('request_id'))
                                if request:
                                    request[1].update(message)
                                    request[0].set()
                            elif message.get('kind') == 'state':
                                self.pending.append(message)
                connection.close()
            except (OSError, ValueError):
                pass
            finally:
                with self.lock:
                    self.socket = None
            self.stopped.wait(1)

    def send(self, message):
        payload = (json.dumps(message, allow_nan=False) + '\n').encode()
        with self.lock:
            if self.socket is None:
                raise ConnectionError('Isaac simulator is disconnected')
            self.socket.sendall(payload)

    def drain(self):
        with self.lock:
            messages = list(self.pending)
            self.pending.clear()
        return messages

    def request(self, message, timeout=15):
        token = uuid.uuid4().hex
        event, result = threading.Event(), {}
        with self.lock:
            self.responses[token] = event, result
        try:
            self.send(dict(message, request_id=token))
            if not event.wait(timeout):
                return False, 'Isaac reset acknowledgement timed out'
            return bool(result.get('ok')), result.get('message', '')
        finally:
            with self.lock:
                self.responses.pop(token, None)

    def close(self):
        self.stopped.set()
        with self.lock:
            if self.socket:
                self.socket.shutdown(socket.SHUT_RDWR)


class IsaacBridge(Node):
    def __init__(self):
        super().__init__('hx5_isaac_bridge')
        port = self.declare_parameter('socket_port', 7866).value
        metadata = Path(self.declare_parameter('metadata_path', '/isaac_assets/scene_metadata.json').value)
        self.metadata_data = json.loads(metadata.read_text()) if metadata.exists() else {}
        self.client = SocketClient(port)
        initial_spawn = self.metadata_data.get('spawn')
        self.spawn = (initial_spawn[0], initial_spawn[1], math.radians(initial_spawn[3])) if initial_spawn else None
        self.previous = None
        self.epoch = None
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.joints = self.create_publisher(JointState, '/joint_states', 10)
        self.arm_hand = self.create_publisher(JointState, '/arm_hand/joint_states', 10)
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.clock = self.create_publisher(ClockMessage, '/clock', 10)
        self.scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.images, self.infos, self.pressures, self.forces = {}, {}, {}, {}
        for key, (topic, _) in CAMERAS.items():
            self.images[key] = self.create_publisher(CompressedImage, topic + '/compressed', qos_profile_sensor_data)
            info_topic = topic.replace('image_rect_color', 'camera_info').replace('image_rect_raw', 'camera_info')
            self.infos[key] = self.create_publisher(CameraInfo, info_topic, qos_profile_sensor_data)
        for side in ('left', 'right'):
            self.pressures[side] = self.create_publisher(HandPressures, f'/{side}_hand/finger_pressures', 10)
            self.forces[side] = self.create_publisher(Float64MultiArray, f'/simulation/{side}_hand/taxel_forces_newtons', 10)
        self.description = self.create_publisher(String, '/simulation/tactile_model',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.recording = self.create_publisher(String, '/simulation/recording_trigger', 10)
        self.create_subscription(String, '/leader/joystick_controller/tact_trigger', self.record, 10)
        commands = {
            # The shared preset adapter forwards seven SH5 arm axes here
            # and converts legacy arm+gripper input into HX5 hand commands.
            # Consume its output once rather than both input and output.
            'left': '/simulation/arm_l_controller/joint_trajectory',
            'right': '/simulation/arm_r_controller/joint_trajectory',
            'left_hand': '/leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory',
            'right_hand': '/leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory',
            'head': '/leader/joystick_controller_left/joint_trajectory',
            'lift': '/leader/joystick_controller_right/joint_trajectory',
        }
        for group, topic in commands.items():
            self.create_subscription(JointTrajectory, topic, lambda msg, g=group: self.trajectory(g, msg), 10)
        self.create_subscription(Twist, '/cmd_vel', self.velocity, 10)
        self.create_service(Trigger, '/simulation/reset', self.reset, callback_group=ReentrantCallbackGroup())
        self.create_service(Trigger, '/simulation/shutdown', self.shutdown, callback_group=ReentrantCallbackGroup())
        self.create_service(Trigger, '/simulation/diagnostics', self.diagnostics, callback_group=ReentrantCallbackGroup())
        # A wall timer must receive the first /clock while ROS simulation time is zero.
        self.create_timer(.005, self.receive, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(1, self.publish_metadata, clock=Clock(clock_type=ClockType.STEADY_TIME))
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'base_link', 'isaac_scan'
        transform.transform.translation.z = .344
        transform.transform.rotation.w = 1.
        self.static_tf.sendTransform(transform)
        self.publish_metadata()

    def publish_metadata(self):
        self.description.publish(String(data=json.dumps({
            'schema_version': 'cyclo_isaac_mcap', 'robot_type': 'ffw_sh5_rev1',
            'tactile_source': 'physx_contact_force_projected_grid_v1',
            'full_scale_newtons_per_taxel': 8., 'taxels_per_finger': 9,
            'hardware_calibrated': False, 'force_mode': 'contact_force_magnitude',
            'mobile_odometry_source': 'isaac_measured_base_pose_relative_spawn',
            'environment': 'isaac_logistics_cell_ws',
        })))

    def record(self, message):
        mapped = {'left': 'right_long_time', 'right': 'left_long_time'}.get(message.data)
        if mapped:
            self.recording.publish(String(data=mapped))

    def command(self, message):
        try:
            self.client.send(message)
        except (OSError, ValueError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.)

    def trajectory(self, group, message):
        points = []
        for point in message.points:
            item = {'positions': list(point.positions), 'time_from_start': point.time_from_start.sec + point.time_from_start.nanosec * 1e-9}
            if point.velocities:
                item['velocities'] = list(point.velocities)
            points.append(item)
        self.command({'kind': 'trajectory', 'group': group, 'joint_names': list(message.joint_names), 'points': points})

    def velocity(self, message):
        self.command({'kind': 'basevelocity', 'vx': message.linear.x, 'vy': message.linear.y, 'w': message.angular.z})

    def reset(self, _request, response):
        response.success, response.message = self.client.request({'kind': 'reset'})
        return response

    def shutdown(self, _request, response):
        response.success, response.message = self.client.request({'kind': 'shutdown'})
        return response

    def diagnostics(self, _request, response):
        response.success, response.message = self.client.request({'kind': 'inspect'})
        return response

    def receive(self):
        for message in self.client.drain():
            try:
                self.state(message)
            except (ValueError, KeyError, TypeError) as error:
                self.get_logger().warning(f'Invalid measured Isaac state: {error}', throttle_duration_sec=5.)

    def state(self, message):
        names, positions = message['joint_names'], message['positions']
        if len(names) != len(positions) or len(set(names)) != len(names) or not all(math.isfinite(x) for x in positions):
            raise ValueError('Invalid joint vector')
        values = dict(zip(names, positions))
        if not all(name in values for name in ARM_HAND):
            raise ValueError('Incomplete SH5/HX5 joints')
        timestamp = stamp(message['time'])
        self.clock.publish(ClockMessage(clock=timestamp))
        joints = JointState(name=names, position=positions)
        joints.header.stamp, joints.header.frame_id = timestamp, 'base_link'
        velocities = message.get('velocities', [])
        if len(velocities) == len(names):
            joints.velocity = velocities
        self.joints.publish(joints)
        selected = JointState(header=joints.header, name=ARM_HAND, position=[values[n] for n in ARM_HAND])
        if joints.velocity:
            speeds = dict(zip(names, velocities))
            selected.velocity = [speeds[n] for n in ARM_HAND]
        self.arm_hand.publish(selected)
        p, q = message['base_position'], message['base_orientation']
        yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
        if self.spawn is None:
            self.spawn = (p[0], p[1], yaw)
        sx, sy, syaw = self.spawn
        c, s = math.cos(syaw), math.sin(syaw)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = timestamp, 'odom', 'base_link'
        odom.pose.pose.position.x = c * (p[0] - sx) + s * (p[1] - sy)
        odom.pose.pose.position.y = -s * (p[0] - sx) + c * (p[1] - sy)
        odom.pose.pose.position.z = p[2] - self.metadata_data.get('spawn', [0., 0., 0.])[2]
        qc, qs = math.cos(syaw / 2), math.sin(syaw / 2)
        orientation = odom.pose.pose.orientation
        orientation.x, orientation.y = qc * q[0] + qs * q[1], qc * q[1] - qs * q[0]
        orientation.z, orientation.w = qc * q[2] - qs * q[3], qc * q[3] + qs * q[2]
        linear, angular = message.get('base_linear_velocity'), message.get('base_angular_velocity')
        if linear is not None:
            odom.twist.twist.linear.x = math.cos(yaw) * linear[0] + math.sin(yaw) * linear[1]
            odom.twist.twist.linear.y = -math.sin(yaw) * linear[0] + math.cos(yaw) * linear[1]
        if angular is not None:
            odom.twist.twist.angular.z = angular[2]
        self.odom.publish(odom)
        tf = TransformStamped(header=odom.header, child_frame_id='base_link')
        tf.transform.translation.x, tf.transform.translation.y = odom.pose.pose.position.x, odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)
        if 'scan' in message:
            source = message['scan']
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp(source.get('time', message['time'])), source.get('frame_id', 'isaac_scan')
            for key in ('angle_min', 'angle_increment', 'range_min', 'range_max', 'scan_time'):
                setattr(scan, key, float(source[key]))
            scan.ranges = [math.inf if x is None else float(x) for x in source['ranges']]
            scan.angle_max = scan.angle_min + max(0, len(scan.ranges) - 1) * scan.angle_increment
            self.scan.publish(scan)
        for key, source in message.get('images', {}).items():
            if key not in CAMERAS:
                continue
            image = CompressedImage(format='jpeg', data=base64.b64decode(source['jpeg'], validate=True))
            image.header.stamp, image.header.frame_id = stamp(source.get('time', message['time'])), CAMERAS[key][1]
            self.images[key].publish(image)
            info = CameraInfo(header=image.header, width=source['width'], height=source['height'], distortion_model='plumb_bob')
            calibration = self.metadata_data.get('cameras', {}).get(key, {})
            fallback_fx = info.width * calibration.get('focal_length_mm', 12.) / calibration.get('horizontal_aperture_mm', 20.955)
            fallback_fy = info.height * calibration.get('focal_length_mm', 12.) / calibration.get('vertical_aperture_mm', 20.955*188/336)
            fx = float(source.get('fx', calibration.get('fx', fallback_fx)))
            fy = float(source.get('fy', calibration.get('fy', fallback_fy)))
            cx = float(source.get('cx', calibration.get('cx', .5*info.width)))
            cy = float(source.get('cy', calibration.get('cy', .5*info.height)))
            info.d = [0.]*5
            info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
            info.p = [fx, 0., cx, float(calibration.get('projection_tx',0.)), 0., fy, cy, 0., 0., 0., 1., 0.]
            info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            self.infos[key].publish(info)
        for side, short in (('left', 'l'), ('right', 'r')):
            sensors = message.get('contacts', {}).get(side, {})
            keys = [f'finger_{short}_sensor{i}' for i in range(1, 6)]
            if not all(key in sensors and sensors[key].get('valid') for key in keys):
                continue
            output, forces = HandPressures(hand_name=side), []
            output.header.stamp, output.header.frame_id = timestamp, f'hx5_d20_{side}_base'
            for key in keys:
                cells = sensors[key]['forces']
                if len(cells) != 9 or any(not math.isfinite(x) or x < 0 for x in cells):
                    raise ValueError('Invalid tactile forces')
                forces.extend(cells)
                output.sensors.append(TactileSensor(sensor_name=key,
                    pressure_names=[f'Present Pressure {i}' for i in range(1, 10)],
                    pressure_values=[min(255, round(255 * x / 8.)) for x in cells]))
            self.pressures[side].publish(output)
            self.forces[side].publish(Float64MultiArray(data=forces))


def main():
    rclpy.init()
    node = IsaacBridge()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.client.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
