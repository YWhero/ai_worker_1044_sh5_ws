#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Validate SG2 arm trajectories at the Gazebo controller boundary."""

from copy import deepcopy

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory


# These are the user-facing SG2 limits. The simulated URDF has a small amount
# of solver headroom at the zero boundary, but commands must remain inside the
# physical robot's range.
SG2_JOINT_LIMITS = {
    'arm_l_joint1': (-3.14, 3.14),
    'arm_l_joint2': (0.0, 3.14),
    'arm_l_joint3': (-3.14, 3.14),
    'arm_l_joint4': (-2.9361, 1.0786),
    'arm_l_joint5': (-3.14, 3.14),
    'arm_l_joint6': (-1.57, 1.57),
    'arm_l_joint7': (-1.8201, 1.5804),
    'gripper_l_joint1': (0.0, 1.05),
    'arm_r_joint1': (-3.14, 3.14),
    'arm_r_joint2': (-3.14, 0.0),
    'arm_r_joint3': (-3.14, 3.14),
    'arm_r_joint4': (-2.9361, 1.0786),
    'arm_r_joint5': (-3.14, 3.14),
    'arm_r_joint6': (-1.57, 1.57),
    'arm_r_joint7': (-1.5804, 1.8201),
    'gripper_r_joint1': (0.0, 1.05),
}

def clamp_positions(joint_names, positions, limits=SG2_JOINT_LIMITS):
    """Return clamped positions and details of values that were changed."""
    if len(positions) != len(joint_names):
        raise ValueError(
            f'position count {len(positions)} does not match '
            f'joint count {len(joint_names)}'
        )

    clamped = list(positions)
    changes = []
    for index, joint_name in enumerate(joint_names):
        joint_limit = limits.get(joint_name)
        if joint_limit is None:
            continue
        lower, upper = joint_limit
        original = clamped[index]
        bounded = min(max(original, lower), upper)
        if bounded != original:
            clamped[index] = bounded
            changes.append((joint_name, original, bounded))
    return clamped, changes


class SimJointTrajectoryGuard(Node):
    """Clamp all command sources before they reach simulated arm controllers."""

    def __init__(self):
        super().__init__('sim_joint_trajectory_guard')

        self.declare_parameter(
            'left_input_topic',
            '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
        )
        self.declare_parameter(
            'right_input_topic',
            '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory',
        )
        self.declare_parameter(
            'left_output_topic',
            '/simulation/arm_l_controller/joint_trajectory',
        )
        self.declare_parameter(
            'right_output_topic',
            '/simulation/arm_r_controller/joint_trajectory',
        )

        left_input = self.get_parameter('left_input_topic').value
        right_input = self.get_parameter('right_input_topic').value
        left_output = self.get_parameter('left_output_topic').value
        right_output = self.get_parameter('right_output_topic').value

        self._left_publisher = self.create_publisher(
            JointTrajectory, left_output, 10
        )
        self._right_publisher = self.create_publisher(
            JointTrajectory, right_output, 10
        )
        self._left_subscription = self.create_subscription(
            JointTrajectory,
            left_input,
            lambda message: self._forward(message, self._left_publisher),
            10,
        )
        self._right_subscription = self.create_subscription(
            JointTrajectory,
            right_input,
            lambda message: self._forward(message, self._right_publisher),
            10,
        )

        self.get_logger().info(
            'Guarding simulated arm trajectories: '
            f'{left_input} -> {left_output}, {right_input} -> {right_output}'
        )

    def _forward(self, message, publisher):
        guarded = deepcopy(message)
        changes = []

        try:
            for point in guarded.points:
                if not point.positions:
                    continue
                point.positions, point_changes = clamp_positions(
                    guarded.joint_names, point.positions
                )
                changes.extend(point_changes)
        except ValueError as error:
            self.get_logger().error(
                f'Dropping malformed joint trajectory: {error}',
                throttle_duration_sec=5.0,
            )
            return

        if changes:
            summary = ', '.join(
                f'{joint}: {original:.6f} -> {bounded:.6f}'
                for joint, original, bounded in changes[:4]
            )
            if len(changes) > 4:
                summary += f', and {len(changes) - 4} more'
            self.get_logger().warning(
                f'Clamped out-of-range simulated trajectory command ({summary})',
                throttle_duration_sec=5.0,
            )

        publisher.publish(guarded)


def main(args=None):
    rclpy.init(args=args)
    node = SimJointTrajectoryGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
