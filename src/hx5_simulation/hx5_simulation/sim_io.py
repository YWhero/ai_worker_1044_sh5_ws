from functools import partial
import copy
import json
import math
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from robotis_interfaces.msg import HandPressures, TactileSensor
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import String, Float64MultiArray
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformBroadcaster, TransformException

from hx5_simulation.model import CAMERAS, JOINT_NAMES, MODEL, SPAWN_X, SPAWN_Y, SPAWN_YAW
from hx5_simulation.tactile import contact_forces, pressure_values


def spawn_relative_odometry(message):
    """Transform Gazebo's world pose into the fixed spawn odometry frame.

    Gazebo OdometryPublisher reports world coordinates; subtracting position
    alone would miss the spawn heading and rotate the robot's navigation axes.
    The published twist is already in the robot base frame.
    """
    output = copy.deepcopy(message)
    output.header.frame_id = 'odom'
    output.child_frame_id = 'base_link'
    position = output.pose.pose.position
    dx, dy = position.x - SPAWN_X, position.y - SPAWN_Y
    position.x = math.cos(SPAWN_YAW) * dx + math.sin(SPAWN_YAW) * dy
    position.y = -math.sin(SPAWN_YAW) * dx + math.cos(SPAWN_YAW) * dy
    position.z = 0.0
    q = message.pose.pose.orientation
    c, s = math.cos(SPAWN_YAW / 2), math.sin(SPAWN_YAW / 2)
    orientation = output.pose.pose.orientation
    orientation.x, orientation.y = c * q.x + s * q.y, c * q.y - s * q.x
    orientation.z, orientation.w = c * q.z - s * q.w, c * q.w + s * q.z
    return output


class SimIO(Node):
    def __init__(self):
        super().__init__('hx5_sim_io')
        self.scale = self.declare_parameter('tactile_full_scale_newtons', 8.0).value
        pressure_values([0.0] * 9, self.scale)
        self.frames = Buffer(cache_time=Duration(seconds=2.0), node=self)
        self.samples = {}
        self.cv = CvBridge()
        self.record_trigger = self.create_publisher(String, '/simulation/recording_trigger', 10)
        self.create_subscription(String, '/leader/joystick_controller/tact_trigger', self.joystick, 10)
        self.joints = self.create_publisher(JointState, '/arm_hand/joint_states', 10)
        self.base_odom = self.create_publisher(Odometry, '/odom', 10)
        self.base_tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/simulation/world_odom', self.world_odometry,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.joint_state, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/simulation/gazebo_tf', self.transforms, qos_profile_sensor_data)
        self.pressures = {}
        self.forces = {}
        self.images = {}
        for key, (_, _, topic) in CAMERAS.items():
            self.images[key] = self.create_publisher(CompressedImage, topic + '/compressed', qos_profile_sensor_data)
            self.create_subscription(Image, '/simulation/camera/' + key, partial(self.camera, key), qos_profile_sensor_data)
        for side, hand in (('l', 'left'), ('r', 'right')):
            self.pressures[side] = self.create_publisher(HandPressures, f'/{hand}_hand/finger_pressures', 10)
            self.forces[side] = self.create_publisher(Float64MultiArray, f'/simulation/{hand}_hand/taxel_forces_newtons', 10)
            for finger in range(1, 6):
                self.create_subscription(Contacts, f'/hx5/contact/{side}/finger{finger}',
                                         partial(self.contact, side, finger), qos_profile_sensor_data)
        self.metadata = self.create_publisher(String, '/simulation/tactile_model',
                                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.metadata_message = String(data=json.dumps({
            'schema_version': 'cyclo_gazebo_mcap', 'robot_type': 'ffw_sh5_rev1',
            'tactile_source': 'gazebo_contact_force_projected_grid_v1',
            'full_scale_newtons_per_taxel': self.scale, 'taxels_per_finger': 9,
            'hardware_calibrated': False, 'force_mode': 'contact_force_magnitude',
            'mobile_odometry_source': 'gazebo_odometry_publisher_world_pose_relative_spawn',
            'wheel_odometry_topic': '/simulation/wheel_odom',
        }))
        self.publish_metadata()
        self.create_timer(1.0, self.publish_metadata)
        self.create_timer(0.04, self.publish_tactile)

    def world_odometry(self, message):
        output = spawn_relative_odometry(message)
        self.base_odom.publish(output)
        transform = TransformStamped(header=output.header, child_frame_id=output.child_frame_id)
        transform.transform.translation.x = output.pose.pose.position.x
        transform.transform.translation.y = output.pose.pose.position.y
        transform.transform.translation.z = output.pose.pose.position.z
        transform.transform.rotation = output.pose.pose.orientation
        self.base_tf.sendTransform(transform)

    def publish_metadata(self):
        self.metadata.publish(self.metadata_message)

    def joystick(self, message):
        mapped = {'left': 'right_long_time', 'right': 'left_long_time'}.get(message.data)
        if mapped:
            self.record_trigger.publish(String(data=mapped))

    def joint_state(self, message):
        values = dict(zip(message.name, message.position))
        if not all(name in values for name in JOINT_NAMES):
            return
        output = JointState(header=message.header, name=JOINT_NAMES,
                            position=[values[name] for name in JOINT_NAMES])
        for field in ('velocity', 'effort'):
            source = getattr(message, field)
            if len(source) == len(message.name):
                mapping = dict(zip(message.name, source))
                setattr(output, field, [mapping[name] for name in JOINT_NAMES])
        self.joints.publish(output)

    def transforms(self, message):
        for transform in message.transforms:
            if transform.header.frame_id and transform.child_frame_id:
                self.frames.set_transform(transform, 'gazebo')

    def camera(self, key, message):
        try:
            frame = self.cv.imgmsg_to_cv2(message, desired_encoding='bgr8')
            success, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if success:
                self.images[key].publish(CompressedImage(header=message.header, format='jpeg', data=encoded.tobytes()))
        except (cv2.error, ValueError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)

    def contact(self, side, finger, message):
        try:
            if message.contacts:
                pose = self.frames.lookup_transform('l_sorting_workcell',
                    f'{MODEL}/finger_{side}_link{finger * 4}', Time())
                if (self.get_clock().now() - Time.from_msg(pose.header.stamp)).nanoseconds > 250_000_000:
                    return
                forces = contact_forces(message, pose.transform, finger == 1, MODEL, right=side == 'r')
            else:
                forces = [0.0] * 9
            self.samples[side, finger] = (time.monotonic(), forces, message.header.stamp)
        except (TransformException, ValueError) as error:
            self.samples.pop((side, finger), None)
            self.get_logger().warning(f'Tactile sample unavailable: {error}', throttle_duration_sec=5.0)

    def publish_tactile(self):
        for side, hand in (('l', 'left'), ('r', 'right')):
            samples = [self.samples.get((side, finger)) for finger in range(1, 6)]
            if any(sample is None or time.monotonic() - sample[0] > 0.5 for sample in samples):
                continue
            output = HandPressures(hand_name=hand)
            output.header.stamp = min((sample[2] for sample in samples), key=lambda stamp: (stamp.sec, stamp.nanosec))
            output.header.frame_id = f'hx5_d20_{hand}_base'
            raw = []
            for finger, sample in enumerate(samples, 1):
                output.sensors.append(TactileSensor(
                    sensor_name=f'finger_{side}_sensor{finger}',
                    pressure_names=[f'Present Pressure {index}' for index in range(1, 10)],
                    pressure_values=pressure_values(sample[1], self.scale)))
                raw.extend(sample[1])
            self.pressures[side].publish(output)
            self.forces[side].publish(Float64MultiArray(data=raw))


def main(args=None):
    rclpy.init(args=args)
    node = SimIO()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
