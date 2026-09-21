#!/usr/bin/env python3
"""Read-only SH5/HX5 Isaac ROS contract probe; no commands or service calls."""

from __future__ import annotations

import argparse
from collections import deque
from io import BytesIO
import json
import math
import os
from numbers import Integral, Real
from pathlib import Path
import time


ARMS = [f'arm_{side}_joint{i}' for side in ('l', 'r') for i in range(1, 8)]
HANDS = [f'finger_{side}_joint{i}' for side in ('l', 'r') for i in range(1, 21)]
AUX = ['head_joint1', 'head_joint2', 'lift_joint']
JOINTS = ARMS + HANDS + AUX
CAMERAS = [
    '/zed/zed_node/left/image_rect_color',
    '/zed/zed_node/right/image_rect_color',
    '/camera_left/camera_left/color/image_rect_raw',
    '/camera_right/camera_right/color/image_rect_raw',
]
CAMERA_INFOS = [
    '/zed/zed_node/left/camera_info', '/zed/zed_node/right/camera_info',
    '/camera_left/camera_left/color/camera_info', '/camera_right/camera_right/color/camera_info',
]
CAMERA_FRAMES = [
    'zedm_left_camera_optical_frame', 'zedm_right_camera_optical_frame',
    'camera_l_color_optical_frame', 'camera_r_color_optical_frame',
]
CAMERA_SIZES = dict(zip(CAMERA_FRAMES, [(672, 376)]*2 + [(424, 240)]*2))


def json_scalar(value):
    """ROS array sums can be NumPy scalars; retain numeric JSON types."""
    if isinstance(value, Integral):
        return int(value)
    if isinstance(value, Real):
        return float(value)
    raise TypeError(f'not JSON serializable: {type(value).__name__}')


def stamp_seconds(stamp):
    return int(stamp.sec) + int(stamp.nanosec) / 1_000_000_000


def finite(values):
    return all(math.isfinite(float(value)) for value in values)


def joint_values(message, required):
    names, positions = list(message.name), list(message.position)
    if len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError('JointState names/positions must align and names must be unique')
    values = dict(zip(names, map(float, positions)))
    missing = sorted(set(required) - values.keys())
    if missing:
        raise ValueError(f'missing joints: {", ".join(missing)}')
    if not finite(values.values()):
        raise ValueError('non-finite joint position')
    return values


def pressure_values(message, side):
    if message.hand_name != {'l': 'left', 'r': 'right'}[side]:
        raise ValueError('tactile hand_name does not match this topic')
    sensors = {sensor.sensor_name: sensor for sensor in message.sensors}
    if len(sensors) != len(message.sensors):
        raise ValueError('duplicate tactile sensor_name')
    expected = [f'finger_{side}_sensor{i}' for i in range(1, 6)]
    missing = sorted(set(expected) - sensors.keys())
    if missing:
        raise ValueError(f'missing fingertip sensors: {", ".join(missing)}')
    if set(sensors) != set(expected):
        raise ValueError('expected exactly the five fingertip sensors for this hand')
    if [sensor.sensor_name for sensor in message.sensors] != expected:
        raise ValueError('fingertip sensor order must be 1..5 for policy tensor mapping')
    sums = {}
    for name in expected:
        if list(sensors[name].pressure_names) != [f'Present Pressure {i}' for i in range(1, 10)]:
            raise ValueError(f'{name}: pressure_names must be Present Pressure 1..9 in order')
        values = list(sensors[name].pressure_values)
        if len(values) != 9 or not finite(values) or any(
            float(value) != int(value) or not 0 <= value <= 255 for value in values
        ):
            raise ValueError(f'{name}: expected nine uint8 pressure values (0..255)')
        sums[name] = sum(map(int, values))
    return sums


def camera_summary(message, frame, expected_size=None):
    from PIL import Image, UnidentifiedImageError

    data = bytes(message.data)
    expected_size = expected_size or CAMERA_SIZES[frame]
    if 'jpeg' not in message.format.lower() or not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):
        raise ValueError('expected nonempty complete JPEG payload')
    if message.header.frame_id != frame:
        raise ValueError(f'expected camera frame_id {frame}')
    try:
        with Image.open(BytesIO(data)) as image:
            image.load()
            if image.format != 'JPEG' or image.size != expected_size or image.mode != 'RGB':
                raise ValueError(f'expected decodable {expected_size[0]}x{expected_size[1]} RGB JPEG')
            return {'jpeg_bytes': len(data), 'width': int(image.width),
                    'height': int(image.height), 'frame_id': frame}
    except (UnidentifiedImageError, OSError) as error:
        raise ValueError('JPEG cannot be decoded') from error


def camera_info_summary(message, frame, expected_size=None):
    expected_size = expected_size or CAMERA_SIZES[frame]
    if (int(message.width), int(message.height)) != expected_size or len(message.k) != 9 or not finite(message.k) or message.k[0] <= 0 or message.k[4] <= 0:
        raise ValueError(f'invalid CameraInfo: expected {expected_size[0]}x{expected_size[1]} and finite positive intrinsics')
    if message.header.frame_id != frame:
        raise ValueError(f'expected CameraInfo frame_id {frame}')
    return {'width': int(message.width), 'height': int(message.height), 'frame_id': frame}


def scan_summary(message):
    if not finite([message.range_min, message.range_max, message.angle_min,
                   message.angle_max, message.angle_increment]):
        raise ValueError('non-finite scan bounds')
    if not 0 <= message.range_min < message.range_max or message.angle_increment <= 0:
        raise ValueError('invalid scan range or angular bounds')
    ranges = list(message.ranges)
    if not ranges or abs(message.angle_min + (len(ranges) - 1) * message.angle_increment
                         - message.angle_max) > message.angle_increment * 1.5:
        raise ValueError('scan angles do not match its sample count')
    valid = []
    for value in ranges:
        if value == math.inf:  # ROS convention for no return within range_max.
            continue
        if not math.isfinite(value) or not message.range_min <= value <= message.range_max:
            raise ValueError('scan has NaN/negative infinity/out-of-range finite values')
        valid.append(value)
    if not valid:
        raise ValueError('scan contains no finite obstacle ranges in this workcell')
    if message.header.frame_id != 'isaac_scan':
        raise ValueError('expected scan frame_id isaac_scan')
    return {'samples': len(ranges), 'finite_ranges': len(valid), 'minimum_m': float(min(valid)),
            'frame_id': message.header.frame_id}


def xyz(vector):
    return [float(vector.x), float(vector.y), float(vector.z)]


def xyzw(quaternion):
    values = xyz(quaternion) + [float(quaternion.w)]
    if not finite(values) or abs(sum(value * value for value in values) - 1.0) > 0.001:
        raise ValueError('quaternion is non-finite or not normalized')
    return values


def pose_error(position, quaternion, other_position, other_quaternion):
    if not finite(position + other_position + quaternion + other_quaternion):
        raise ValueError('non-finite TF/odometry pose')
    distance = math.dist(position, other_position)
    dot = min(1.0, abs(sum(left * right for left, right in zip(quaternion, other_quaternion))))
    return distance, 2.0 * math.acos(dot)


def has_tf_path(parent_sets, child, ancestor):
    seen = set()
    while child != ancestor:
        if child in seen or len(parent_sets.get(child, ())) != 1:
            return False
        seen.add(child)
        child = next(iter(parent_sets[child]))
    return True


class Samples:
    def __init__(self):
        self.count = 0
        self.last_wall = None
        self.first_stamp = None
        self.last_stamp = None
        self.regressions = 0
        self.error = ''
        self.error_count = 0
        self.last_error = ''
        self.summary = {}

    def update(self, stamp, summary=None, error=''):
        if self.last_stamp is not None and stamp < self.last_stamp - 1e-9:
            self.regressions += 1
        if self.first_stamp is None:
            self.first_stamp = stamp
        self.count += 1
        self.last_stamp = stamp
        self.last_wall = time.monotonic()
        self.error = error
        if error:
            self.error_count += 1
            self.last_error = error
        self.summary = summary or {}

    def report(self, now, clock, max_age, advance=True):
        failures = []
        if not self.count:
            return {'count': 0, 'passed': False, 'failures': ['no messages']}
        wall_age = now - self.last_wall
        sim_age = None if clock is None else clock - self.last_stamp
        if wall_age > max_age:
            failures.append(f'callback stale: {wall_age:.3f}s')
        if sim_age is not None and (sim_age > max_age or sim_age < -0.1):
            failures.append(f'stamp differs from /clock: age {sim_age:.3f}s')
        if self.regressions:
            failures.append(f'{self.regressions} timestamp regressions')
        if advance and (self.count < 2 or self.last_stamp <= self.first_stamp):
            failures.append('time did not advance during probe')
        if self.error_count:
            failures.append(f'{self.error_count} malformed messages; last: {self.last_error}')
        return {'count': self.count, 'passed': not failures, 'failures': failures,
                'wall_age_sec': round(wall_age, 6), 'simulation_age_sec': sim_age,
                'stamp_first': self.first_stamp, 'stamp_last': self.last_stamp,
                'malformed_count': self.error_count, **self.summary}


def collect(args):
    import rclpy
    from nav_msgs.msg import Odometry
    from robotis_interfaces.msg import HandPressures
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, CompressedImage, JointState, LaserScan
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init(args=[])
    node = rclpy.create_node('hx5_isaac_read_only_probe')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    records, subscriptions = {}, []
    first_joints, latest_joints, deltas = {}, {}, {}
    transforms, parent_sets = {}, {}
    dynamic_transforms, tf_errors = set(), set()
    odometry = deque(maxlen=2000)

    def subscribe(topic, message_type, parser, selected_qos=qos):
        record = records.setdefault(topic, Samples())

        def callback(message):
            stamp = stamp_seconds(message.clock if topic == '/clock' else message.header.stamp)
            try:
                summary = parser(message)
                record.update(stamp, summary)
            except (ValueError, TypeError, OverflowError) as error:
                record.update(stamp, error=str(error))
        subscriptions.append(node.create_subscription(message_type, topic, callback, selected_qos))

    def parse_joints(required):
        def parser(message):
            values = joint_values(message, required)
            for name, value in values.items():
                if name not in JOINTS:
                    continue
                first_joints.setdefault(name, value)
                latest_joints[name] = value
                deltas[name] = max(deltas.get(name, 0), abs(value - first_joints[name]))
            return {'joint_count': len(values), 'required_joint_count': len(required)}
        return parser

    subscribe('/clock', Clock, lambda message: {})
    subscribe('/joint_states', JointState, parse_joints(AUX))
    subscribe('/arm_hand/joint_states', JointState, parse_joints(ARMS + HANDS))
    subscribe('/left_hand/finger_pressures', HandPressures,
              lambda message: {'raw_pressure_sums': pressure_values(message, 'l')})
    subscribe('/right_hand/finger_pressures', HandPressures,
              lambda message: {'raw_pressure_sums': pressure_values(message, 'r')})

    camera_sizes = dict(CAMERA_SIZES)
    if args.metadata:
        metadata = json.loads(args.metadata.read_text())
        camera_sizes.update({spec['optical_frame']: tuple(spec['resolution'])
                             for spec in metadata['cameras'].values()})
    for camera, info, frame in zip(CAMERAS, CAMERA_INFOS, CAMERA_FRAMES):
        subscribe(camera + '/compressed', CompressedImage,
                  lambda message, frame=frame: camera_summary(message, frame, camera_sizes[frame]))
        subscribe(info, CameraInfo,
                  lambda message, frame=frame: camera_info_summary(message, frame, camera_sizes[frame]))
    subscribe('/scan', LaserScan, scan_summary)

    def odom_parser(message):
        if message.header.frame_id != args.odom_frame or message.child_frame_id != args.base_frame:
            raise ValueError(f'expected {args.odom_frame} -> {args.base_frame} odometry')
        position, quaternion = xyz(message.pose.pose.position), xyzw(message.pose.pose.orientation)
        if not finite(position + xyz(message.twist.twist.linear) + xyz(message.twist.twist.angular)):
            raise ValueError('non-finite odometry pose/twist')
        odometry.append((stamp_seconds(message.header.stamp), position, quaternion))
        return {'position_m': position, 'quaternion_xyzw': quaternion}
    subscribe('/odom', Odometry, odom_parser)

    def tf_callback(message, static=False):
        for transform in message.transforms:
            child, parent = transform.child_frame_id, transform.header.frame_id
            try:
                if not child or not parent or not finite(xyz(transform.transform.translation)):
                    raise ValueError('empty TF frame or non-finite translation')
                xyzw(transform.transform.rotation)
            except ValueError as error:
                tf_errors.add(f'TF {parent} -> {child}: {error}')
            parent_sets.setdefault(child, set()).add(parent)
            transforms[(parent, child)] = transform
            if not static:
                dynamic_transforms.add((parent, child))
    subscriptions.append(node.create_subscription(TFMessage, '/tf', tf_callback, qos))
    subscriptions.append(node.create_subscription(TFMessage, '/tf_static', lambda message: tf_callback(message, static=True),
                         QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0, deadline - time.monotonic())))
        clock = records['/clock'].last_stamp
        now = time.monotonic()
        topics = {topic: record.report(now, clock, args.max_age) for topic, record in records.items()}
        failures = [f'{topic}: {failure}' for topic, report in topics.items() for failure in report['failures']]
        failures.extend(sorted(tf_errors))
        publishers = {topic: node.count_publishers(topic) for topic in ('/clock', '/odom')}
        for topic, count in publishers.items():
            if count != 1:
                failures.append(f'{topic}: expected exactly one publisher, found {count}')
        if set(JOINTS) - latest_joints.keys():
            failures.append('combined joint feedback does not contain all 57 finite positions')
        for child, parents in parent_sets.items():
            if len(parents) > 1:
                failures.append(f'TF {child}: multiple parents {sorted(parents)}')
        tf_report = {}
        transform = transforms.get((args.odom_frame, args.base_frame))
        if transform is None or not odometry:
            failures.append('missing odom -> base_link TF or valid odometry')
        else:
            try:
                tf_stamp = stamp_seconds(transform.header.stamp)
                nearest = min(odometry, key=lambda sample: abs(sample[0] - tf_stamp))
                distance, angle = pose_error(xyz(transform.transform.translation), xyzw(transform.transform.rotation), nearest[1], nearest[2])
                tf_report = {'stamp_gap_sec': abs(tf_stamp - nearest[0]), 'position_error_m': distance, 'orientation_error_rad': angle}
                if clock is None or not -0.1 <= clock - tf_stamp <= args.max_age:
                    failures.append('odom -> base TF timestamp is stale or in the future')
                if abs(tf_stamp - nearest[0]) > 0.05 or distance > 0.01 or angle > 0.02:
                    failures.append('odom -> base TF does not match simultaneous odometry')
            except ValueError as error:
                failures.append(f'TF: {error}')
        sensor_frames = [topics['/scan'].get('frame_id')]
        for camera, info in zip(CAMERAS, CAMERA_INFOS):
            image_frame = topics[camera + '/compressed'].get('frame_id')
            info_frame = topics[info].get('frame_id')
            sensor_frames.append(image_frame)
            if image_frame and image_frame != info_frame:
                failures.append(f'{camera}: CameraInfo and JPEG frame_id differ')
        for frame in filter(None, sensor_frames):
            if not has_tf_path(parent_sets, frame, args.base_frame):
                failures.append(f'TF: no unique path from sensor {frame} to {args.base_frame}')
                continue
            child = frame
            while child != args.base_frame:
                parent = next(iter(parent_sets[child]))
                edge = (parent, child)
                if edge in dynamic_transforms:
                    age = None if clock is None else clock - stamp_seconds(transforms[edge].header.stamp)
                    if age is None or not -0.1 <= age <= args.max_age:
                        failures.append(f'TF {parent} -> {child}: stale or future dynamic sensor transform')
                child = parent
        if args.expect_pose:
            expected = json.loads(Path(args.expect_pose).read_text())
            if not isinstance(expected, dict) or not expected or not set(expected).issubset(JOINTS) or not finite(expected.values()):
                raise ValueError('--expect-pose must be a nonempty JSON object of known joint names and finite positions')
            errors = {name: abs(latest_joints.get(name, math.inf) - float(value)) for name, value in expected.items()}
            if max(errors.values()) > args.target_tolerance:
                failures.append(f'target pose not reached: maximum error {max(errors.values())}')
        if args.min_joint_change is not None and max(deltas.values(), default=0) < args.min_joint_change:
            failures.append('no joint changed by the requested minimum during this read-only probe')
        return {'result': 'passed' if not failures else 'failed', 'read_only': True,
                'ros_domain_id': args.domain, 'duration_sec': args.duration, 'topics': topics,
                'publisher_counts': publishers, 'odom_tf': tf_report,
                'joint_max_change_radians_or_m': deltas, 'failures': failures,
                'scope': 'State/transport validation; no command publication, grasp-success, policy-success, or route-success certification.'}
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=8.0)
    parser.add_argument('--max-age', type=float, default=1.0)
    parser.add_argument('--domain', type=int, default=115)
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--expect-pose', type=Path)
    parser.add_argument('--metadata', type=Path, default=Path('/isaac_assets/scene_metadata.json'),
                        help='Expected camera profile from the built stage receipt')
    parser.add_argument('--target-tolerance', type=float, default=0.02)
    parser.add_argument('--min-joint-change', type=float)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 120:
        parser.error('--duration must be finite and between 1 and 120 seconds')
    for name in ('max_age', 'target_tolerance'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'--{name.replace("_", "-")} must be finite and positive')
    if args.min_joint_change is not None and (not math.isfinite(args.min_joint_change) or args.min_joint_change <= 0):
        parser.error('--min-joint-change must be finite and positive')
    if args.domain != 115 or os.environ.get('ROS_DOMAIN_ID') != str(args.domain):
        parser.error('this Isaac probe requires ROS_DOMAIN_ID=115; use the isolated Isaac ROS container')
    try:
        report = collect(args)
    except (ImportError, ValueError, OSError, RuntimeError) as error:
        report = {'result': 'error', 'read_only': True, 'error': str(error)}
    content = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False, default=json_scalar)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(content + '\n')
    print(content)
    return 0 if report['result'] == 'passed' else (2 if report['result'] == 'error' else 1)


if __name__ == '__main__':
    raise SystemExit(main())
