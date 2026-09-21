#!/usr/bin/env python3
"""Read-only post-launch LG2 controller and measured-state readiness check.

Run in the private Isaac ROS domain after launching the physical leader. This
does not activate following, publish commands, or open hardware devices.
"""
import argparse
import json
import math
import os
import time

DOMAIN = '115'
SERVICE = '/leader/controller_manager/list_controllers'
TOPIC = '/leader/joint_states'
CONTROLLERS = {
    'joint_trajectory_command_broadcaster': 'joint_trajectory_command_broadcaster/JointTrajectoryCommandBroadcaster',
    'spring_actuator_controller_left': 'spring_actuator_controller/SpringActuatorController',
    'spring_actuator_controller_right': 'spring_actuator_controller/SpringActuatorController',
    'joystick_controller': 'joystick_controller/JoystickController',
    'joint_state_broadcaster': 'joint_state_broadcaster/JointStateBroadcaster',
}
JOINTS = tuple(f'arm_{side}_joint{number}' for side in ('l', 'r') for number in range(1, 8)) + (
    'gripper_l_joint1', 'gripper_r_joint1')


def controller_states(response):
    if response is None:
        raise RuntimeError('Leader controller service returned no response')
    entries = response.controller
    states = {entry.name: entry for entry in entries}
    if len(states) != len(entries):
        raise RuntimeError('Leader controller service returned duplicate controller names')
    problems = []
    for name, expected_type in CONTROLLERS.items():
        entry = states.get(name)
        if entry is None:
            problems.append(name + '=missing')
        elif entry.state != 'active':
            problems.append(name + '=' + entry.state)
        elif entry.type != expected_type:
            problems.append(name + '=unexpected type ' + entry.type)
    if problems:
        raise RuntimeError('Waiting for official active LG2 controllers: ' + ', '.join(problems))
    return {name: states[name].state for name in CONTROLLERS}


def joint_sample(message):
    names, positions = message.name, message.position
    if len(names) != len(positions) or len(set(names)) != len(names):
        raise RuntimeError('Leader feedback has incomplete or duplicate joint names')
    values = dict(zip(names, positions))
    if not set(JOINTS).issubset(values):
        raise RuntimeError('Leader feedback must contain all 14 arm and 2 gripper joints')
    if any(not math.isfinite(value) for value in positions):
        raise RuntimeError('Leader feedback contains nonfinite joint positions')
    stamp = message.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        raise RuntimeError('Leader feedback has an invalid header timestamp')
    stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
    if stamp_ns <= 0:
        raise RuntimeError('Leader feedback has a zero header timestamp')
    return values, stamp_ns


class Feedback:
    def __init__(self):
        self.received = self.advanced = self.first_stamp = self.last_stamp = None
        self.values = None
        self.samples = 0
        self.error = 'Waiting for measured leader joint feedback'

    def receive(self, message, now):
        try:
            values, stamp = joint_sample(message)
        except (RuntimeError, TypeError, ValueError) as error:
            self.values = None
            self.first_stamp = self.last_stamp = self.advanced = None
            self.error = str(error)
            return
        self.received, self.values = now, values
        self.samples += 1
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.first_stamp, self.last_stamp, self.advanced = stamp, stamp, None
            self.error = 'Leader feedback header timestamp regressed; waiting for advancing samples'
            return
        if self.first_stamp is None:
            self.first_stamp = stamp
        if self.last_stamp is not None and stamp > self.last_stamp:
            self.advanced = now
        self.last_stamp = stamp
        self.error = None

    def report(self, now, max_age=1.0):
        if self.values is None:
            raise RuntimeError(self.error)
        if self.received is None or not 0 <= now - self.received <= max_age:
            raise RuntimeError('Measured leader joint feedback is stale in wall time')
        if self.advanced is None or not 0 <= now - self.advanced <= max_age:
            raise RuntimeError(self.error or 'Waiting for advancing leader feedback header timestamps')
        return {'measured_joints': len(self.values), 'required_joints': len(JOINTS),
                'feedback_age_wall_sec': round(now-self.received, 6),
                'feedback_stamp_progress_ns': self.last_stamp-self.first_stamp,
                'feedback_samples': self.samples}


def wait_for_readiness(node, client, observed, spin_once, request_factory,
                       timeout=30.0, monotonic=time.monotonic):
    """Bound service discovery, async replies, controller startup and feedback together."""
    if not math.isfinite(timeout) or not 1 <= timeout <= 30:
        raise ValueError('Readiness timeout must be finite and in [1, 30] wall seconds')
    deadline = monotonic() + timeout
    future = states = states_received = None
    next_query = 0.0
    pending = 'Waiting for leader controller service and joint publisher'
    while monotonic() < deadline:
        spin_once(node, timeout_sec=min(.05, max(0.0, deadline-monotonic())))
        now = monotonic()
        publishers = node.count_publishers(TOPIC)
        if publishers > 1:
            raise RuntimeError(f'Expected exactly one measured leader joint publisher, found {publishers}')
        if future is not None and future.done():
            try:
                states = controller_states(future.result())
                states_received = now
            except RuntimeError as error:
                states = None
                pending = str(error)
            future = None
            next_query = now + .5
        if future is None and now >= next_query and client.service_is_ready():
            future = client.call_async(request_factory())
        if publishers != 1:
            pending = 'Waiting for exactly one measured leader joint publisher'
            continue
        if states is None or now-states_received > 1.0:
            if not client.service_is_ready():
                pending = 'Waiting for leader controller service'
            elif future is not None:
                pending = 'Waiting for active-controller service response'
            continue
        try:
            feedback = observed.report(now)
        except RuntimeError as error:
            pending = str(error)
            continue
        return {'ready': True, 'domain': int(DOMAIN), 'controllers': states,
                'publisher_count': publishers, **feedback,
                'scope': 'Leader controller and measured-state readiness'}
    raise RuntimeError(f'Timed out after {timeout:g} wall seconds: {pending}')


def check(timeout):
    if os.environ.get('ROS_DOMAIN_ID') != DOMAIN:
        raise RuntimeError('Private Isaac ROS_DOMAIN_ID=115 is required')
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from sensor_msgs.msg import JointState
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = None
    try:
        node = rclpy.create_node('isaac_lg2_hardware_readiness')
        observed = Feedback()
        subscription = node.create_subscription(JointState, TOPIC,
            lambda message: observed.receive(message, time.monotonic()),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        client = node.create_client(ListControllers, SERVICE)
        return wait_for_readiness(node, client, observed, rclpy.spin_once,
                                  ListControllers.Request, timeout)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='Maximum post-launch readiness wait in wall seconds (1..30)')
    args = parser.parse_args(argv)
    try:
        if not math.isfinite(args.timeout) or not 1 <= args.timeout <= 30:
            raise ValueError('--timeout must be finite and in [1, 30] wall seconds')
        result = check(args.timeout)
    except Exception as error:
        print(json.dumps({'ready': False, 'domain': int(DOMAIN), 'error': str(error)}))
        return 1
    print(json.dumps(result))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
