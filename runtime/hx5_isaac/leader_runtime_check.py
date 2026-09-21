"""Observe the Isaac follower and hand adapter before starting real LG2 hardware.

HandPreset status is periodic in simulation time. Read its live node parameters
instead of treating missing transient-local replay within three wall seconds as
an absent profile. Duplicate publishers must never select a random instance.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

MAPPING_PATH = '/hx5_isaac_runtime/official_1044_hand_mapping.json'
MAPPING_SHA256 = '04eeb1756e19ce442b53aeb692460326415a7e2dc9d63f835bdbe607c4fc98c4'
MAPPING_NAME = 'ROBOTIS 1044 SH5 release/grasp'
PARAMETERS = ('mapping_profile', 'robot_description', 'use_sim_time')


def joint_limits(robot):
    result = {}
    for joint in robot.findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        limit = joint.find('limit')
        result[joint.get('name')] = (joint.get('type'),
            *(float(limit.get(key)) if limit is not None and limit.get(key) is not None else None
              for key in ('lower', 'upper')))
    return result


def verify_mapping(parameters, robot, read_bytes=lambda path: Path(path).read_bytes()):
    if parameters.get('mapping_profile') != MAPPING_PATH:
        raise RuntimeError('Isaac HandPreset node has no pinned official 1044 mapping_profile: '
                           f'{parameters.get("mapping_profile")!r}')
    if parameters.get('use_sim_time') is not True:
        raise RuntimeError('Isaac HandPreset node must use simulation time')
    data = read_bytes(MAPPING_PATH)
    if hashlib.sha256(data).hexdigest() != MAPPING_SHA256:
        raise RuntimeError('Isaac official 1044 mapping file differs from the pinned release/grasp source')
    try:
        loaded = ET.fromstring(parameters.get('robot_description', ''))
    except (ET.ParseError, TypeError) as error:
        raise RuntimeError('Isaac HandPreset node has no valid robot_description') from error
    if joint_limits(loaded) != joint_limits(robot):
        raise RuntimeError('Isaac HandPreset node joint limits differ from the current SH5/HX5 follower')
    return json.loads(data)['name']


def check_publishers(publishers):
    for topic, details in publishers.items():
        if len(details) > 1:
            raise RuntimeError(f'Duplicate Isaac publishers on {topic}: {json.dumps(details)}. '
                               'Remove orphaned Isaac ROS processes before starting physical Leader.')


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Observation:
    def __init__(self):
        self.clock = self.joints = None
        self.clock_received = self.joints_received = self.clock_advanced = None
        self.clock_advances = 0
        self.status = None
        self.status_error = None

    def receive_clock(self, stamp, now):
        if self.clock is not None and seconds(stamp) > seconds(self.clock):
            self.clock_advances += 1
            self.clock_advanced = now
        self.clock = stamp
        self.clock_received = now

    def receive_joints(self, message, now):
        self.joints = message
        self.joints_received = now

    def receive_status(self, data):
        try:
            self.status = json.loads(data)
            if not isinstance(self.status, dict):
                raise ValueError('Status must be a JSON object')
            self.status_error = None
        except (ValueError, TypeError) as error:
            self.status = None
            self.status_error = str(error)

    def feedback(self, required, now, max_wall_age=1.5):
        if self.clock is None or self.joints is None:
            raise RuntimeError('Waiting for measured Isaac /joint_states and /clock')
        if not self.clock_advances or self.clock_advanced is None or now - self.clock_advanced > max_wall_age:
            raise RuntimeError('Waiting for advancing Isaac simulation clock (paused or stale)')
        if now - self.joints_received > max_wall_age:
            raise RuntimeError('Measured Isaac joint feedback is stale in wall time')
        joints = self.joints
        if len(joints.name) != len(joints.position) or len(set(joints.name)) != len(joints.name):
            raise RuntimeError('Incomplete or duplicate measured Isaac joint names')
        values = dict(zip(joints.name, joints.position))
        if not required.issubset(values) or any(not math.isfinite(values[name]) for name in required):
            raise RuntimeError('Incomplete or invalid measured Isaac SH5/HX5 follower state')
        age = seconds(self.clock) - seconds(joints.header.stamp)
        if not -.1 <= age <= .5:
            raise RuntimeError(f'Isaac joint feedback is stale relative to /clock: {age:.3f}s')
        return age

def main():
    import rclpy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rcl_interfaces.srv import GetParameters, SetParametersAtomically

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=15.0, help='Readiness deadline in wall seconds (3..30)')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or not 3 <= args.timeout <= 30:
        parser.error('--timeout must be finite and in [3, 30] seconds')
    robot = ET.parse('/isaac_assets/robot.urdf').getroot()
    required = set(joint_limits(robot))
    rclpy.init()
    node = rclpy.create_node('isaac_physical_leader_readiness')
    observed = Observation()
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    subs = [node.create_subscription(Clock, '/clock', lambda m: observed.receive_clock(m.clock, time.monotonic()), qos),
            node.create_subscription(JointState, '/joint_states', lambda m: observed.receive_joints(m, time.monotonic()), qos),
            node.create_subscription(String, '/leader/hand_preset/status',
                lambda m: observed.receive_status(m.data),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    setter = node.create_client(SetParametersAtomically, '/leader/hand_preset/set')
    getter = node.create_client(GetParameters, '/hx5_sim_hand_presets/get_parameters')
    future = mapping_name = unique_since = queried_gid = None
    pending = 'Waiting for Isaac publishers'
    publishers = {}
    try:
        until = time.monotonic() + args.timeout
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            if node.count_publishers('/leader/joint_states'):
                raise RuntimeError('Another physical Skeleton Leader is already publishing in domain 115')
            publishers = {topic: [{'node': info.node_namespace.rstrip('/') + '/' + info.node_name,
                                   'gid': bytes(info.endpoint_gid).hex()}
                                  for info in node.get_publishers_info_by_topic(topic)]
                          for topic in ('/clock', '/joint_states', '/leader/hand_preset/status')}
            check_publishers(publishers)
            if not all(len(details) == 1 for details in publishers.values()):
                unique_since = None
                pending = 'Waiting for exactly one /clock, /joint_states and HandPreset status publisher'
                continue
            if unique_since is None:
                unique_since = now
            current = publishers['/leader/hand_preset/status'][0]
            if current['node'] != '/hx5_sim_hand_presets':
                raise RuntimeError(f'Unexpected Isaac HandPreset status publisher: {current}')
            if queried_gid is not None and queried_gid != current['gid']:
                future = mapping_name = queried_gid = None
                unique_since = now
            if future is None and now - unique_since >= 1 and getter.service_is_ready():
                future = getter.call_async(GetParameters.Request(names=list(PARAMETERS)))
                queried_gid = current['gid']
            if future is not None and future.done() and mapping_name is None:
                response = future.result()
                if response is None or len(response.values) != len(PARAMETERS):
                    raise RuntimeError('Isaac HandPreset parameter service returned incomplete data')
                parameters = {name: {1: value.bool_value, 4: value.string_value}.get(value.type)
                              for name, value in zip(PARAMETERS, response.values)}
                mapping_name = verify_mapping(parameters, robot)
            if mapping_name is None:
                pending = 'Waiting for the unique HandPreset node parameter response'
                continue
            if not setter.service_is_ready():
                pending = 'Waiting for Isaac HandPreset service'
                continue
            try:
                age = observed.feedback(required, now)
            except RuntimeError as error:
                pending = str(error)
                continue
            print(json.dumps({'ready': True, 'measured_joints': len(required), 'feedback_age_sim_sec': age,
                              'hand_mapping': mapping_name, 'mapping_verified_by': 'live_node_parameters_and_pinned_file',
                              'hand_status_received': observed.status is not None, 'domain': 115}))
            return
        raise RuntimeError(f'Timed out waiting for Isaac Leader readiness: {pending}; '
                           f'hand_status_received={observed.status is not None}; publishers={json.dumps(publishers)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ValueError, ET.ParseError) as error:
        print(f'Isaac Leader readiness failed: {error}', file=sys.stderr)
        raise SystemExit(1)
