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
#
# Author: Seongwoo Kim

"""Action node for rotating the mobile base by a specified angle."""

import math
import threading
import time
from typing import TYPE_CHECKING

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from orchestrator.bt.actions.base_action import BaseAction
from orchestrator.bt.bt_core import NodeStatus
from orchestrator.bt.constants import *  # noqa: F403
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

if TYPE_CHECKING:
    from rclpy.node import Node


class Rotate(BaseAction):
    """Rotate the mobile base by a target angle around the vertical axis.

    Reads the configured mobile Odometry state topic for closed-loop control
    and stops once the rotation completes within tolerance.
    """

    @classmethod
    def from_xml_params(cls, context, name: str, params: dict):
        action = cls(
            node=context.node,
            angle_deg=params.get('angle_deg', DEFAULT_ROTATION_ANGLE_DEG),  # noqa: F405
            angular_velocity=params.get(
                'angular_velocity', ROTATION_ANGULAR_VELOCITY),  # noqa: F405
            tolerance_deg=params.get(
                'tolerance_deg', ROTATION_TOLERANCE_DEG),  # noqa: F405
            timeout_sec=params.get('timeout_sec', 60.0),
            topic_config=context.topic_config,
        )
        action.name = name
        return action

    @staticmethod
    def angle_diff_deg(a, b):
        """Calculate the difference between two angles in degrees."""
        d = a - b
        while d > ANGLE_NORMALIZATION_180:  # noqa: F405
            d -= ANGLE_NORMALIZATION_360  # noqa: F405
        while d < -ANGLE_NORMALIZATION_180:  # noqa: F405
            d += ANGLE_NORMALIZATION_360  # noqa: F405
        return d

    def __init__(
            self,
            node: 'Node',
            angle_deg: float = DEFAULT_ROTATION_ANGLE_DEG,  # noqa: F405
            topic_config: dict = None,
            angular_velocity: float = ROTATION_ANGULAR_VELOCITY,  # noqa: F405
            tolerance_deg: float = ROTATION_TOLERANCE_DEG,  # noqa: F405
            timeout_sec: float = 60.0,
    ):
        """Initialize the Rotate action."""
        super().__init__(node, name='Rotate')
        self.angle_deg = float(angle_deg)
        self.angular_velocity = float(angular_velocity)
        self.tolerance_deg = float(tolerance_deg)
        self.timeout_sec = float(timeout_sec)
        if not math.isfinite(self.angle_deg) or any(
                not math.isfinite(value) or value <= 0.0
                for value in (
                    self.angular_velocity, self.tolerance_deg,
                    self.timeout_sec)):
            raise ValueError(
                'Rotate requires a finite angle and positive speed, '
                'tolerance and timeout')
        self.topic_config = topic_config or {}
        if not isinstance(self.topic_config, dict):
            self.topic_config = {}
        self.kp = ROTATION_KP  # noqa: F405
        self.min_angular_velocity = min(
            self.angular_velocity, ROTATION_MIN_ANGULAR_VELOCITY)  # noqa: F405
        self._lock = threading.Lock()
        self._rotated_deg = 0.0
        self._odom_received_at = None
        self.odom_start_yaw = None
        self.odom_last_yaw = None

        qos_profile = QoSProfile(
            depth=QOS_QUEUE_DEPTH,  # noqa: F405
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.publishers = {}
        topic_map = self.topic_config.get('topic_map', {})
        topic_type_map = self.topic_config.get('topic_type_map', {})
        if self.topic_config and 'topic_map' in self.topic_config:
            for joint_group, topic in topic_map.items():
                if (
                    joint_group == 'leader_mobile'
                    and topic_type_map.get(
                        joint_group,
                        'geometry_msgs/msg/Twist',
                    ) == 'geometry_msgs/msg/Twist'
                ):
                    pub = self.node.create_publisher(
                        Twist,
                        topic,
                        qos_profile
                    )
                    self.publishers[joint_group] = pub

        self.odom_topic = ''
        for joint_group, topic in topic_map.items():
            if (
                joint_group.startswith('follower_')
                and topic_type_map.get(joint_group) == 'nav_msgs/msg/Odometry'
            ):
                self.odom_topic = topic
                break

        self.odom_sub = None
        if self.odom_topic:
            self.odom_sub = self.node.create_subscription(
                Odometry,
                self.odom_topic,
                self._odom_callback,
                qos_profile
            )
        else:
            self.log_warn(
                'Rotate: no Odometry state topic configured for this robot'
            )
        self._stop_event = threading.Event()
        self._thread = None
        self._result = None  # None=running, True=success, False=failure
        self._control_rate = CONTROL_RATE_HZ  # noqa: F405

    def _odom_callback(self, msg):
        """Receive odometry updates and compute yaw angle."""
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        if not math.isfinite(yaw):
            return  # invalid feedback must not refresh the stale-data watchdog
        with self._lock:
            if self.odom_start_yaw is None:
                self.odom_start_yaw = yaw
            elif self.odom_last_yaw is not None:
                self._rotated_deg += self.angle_diff_deg(
                    math.degrees(yaw), math.degrees(self.odom_last_yaw))
            self.odom_last_yaw = yaw
            self._odom_received_at = time.monotonic()

    def _control_loop(self):
        """Control loop that publishes velocity and monitors rotation."""
        rate_sleep = RATE_SLEEP_SEC  # noqa: F405

        if 'leader_mobile' not in self.publishers:
            self.log_error(
                'Rotate: no mobile Twist action topic configured for this robot'
            )
            with self._lock:
                self._result = False
            return

        if self.odom_sub is None:
            self.log_error(
                'Rotate: no mobile Odometry feedback topic configured for this robot'
            )
            with self._lock:
                self._result = False
            return

        timeout_count = 0
        max_init_timeout = ROTATE_INIT_TIMEOUT_TICKS  # noqa: F405
        while (
            self.odom_start_yaw is None and timeout_count < max_init_timeout
            and not self._stop_event.is_set()
        ):
            self._stop_event.wait(0.01)
            timeout_count += 1

        if self._stop_event.is_set():
            self._stop_mobile()
            return
        if self.odom_start_yaw is None:
            self.log_error(
                f'Timeout waiting for odom data on {self.odom_topic}'
            )
            with self._lock:
                self._result = False
            return

        started_at = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()
            with self._lock:
                rotated_deg = self._rotated_deg
                received_at = self._odom_received_at
            if received_at is None or now - received_at > 1.0:
                self.log_error(
                    'Rotate stopped: Odometry feedback stale on '
                    f'{self.odom_topic}')
                self._stop_mobile()
                with self._lock:
                    self._result = False
                return
            if now - started_at > self.timeout_sec:
                self.log_error(
                    f'Rotate timed out after {self.timeout_sec:.1f}s: '
                    f'{rotated_deg:.2f}/{self.angle_deg:.2f} deg')
                self._stop_mobile()
                with self._lock:
                    self._result = False
                return
            error = self.angle_deg - rotated_deg

            if abs(error) <= self.tolerance_deg:
                self._stop_mobile()
                norm_str = f'{rotated_deg:.2f}'
                target_str = str(self.angle_deg)
                msg = (
                    f'[Thread] Rotation complete: {norm_str} deg '
                    f'(target: {target_str} deg)'
                )
                self.log_info(msg)
                with self._lock:
                    self._result = True
                return

            angular_z = self.kp * error
            angular_z = max(-self.angular_velocity,
                            min(self.angular_velocity, angular_z))
            if 0 < abs(angular_z) < self.min_angular_velocity:
                angular_z = (self.min_angular_velocity if angular_z > 0
                             else -self.min_angular_velocity)

            if 'leader_mobile' in self.publishers:
                twist_msg = Twist()
                twist_msg.linear.x = ZERO_VELOCITY  # noqa: F405
                twist_msg.linear.y = ZERO_VELOCITY  # noqa: F405
                twist_msg.angular.z = angular_z
                self.publishers['leader_mobile'].publish(twist_msg)

            self._stop_event.wait(rate_sleep)
        self._stop_mobile()

    def tick(self) -> NodeStatus:
        """Execute the action and return its status."""
        if self._thread is None:
            self._stop_event.clear()
            with self._lock:
                self.odom_start_yaw = None
                self.odom_last_yaw = None
                self._rotated_deg = 0.0
                self._odom_received_at = None
                self._result = None

            self._thread = threading.Thread(
                target=self._control_loop, daemon=True
            )
            self._thread.start()
            angle_str = str(self.angle_deg)
            self.log_info(f'Rotate thread started (target: {angle_str} deg)')
            return NodeStatus.RUNNING

        with self._lock:
            result = self._result

        if result is None:
            return NodeStatus.RUNNING
        return NodeStatus.SUCCESS if result else NodeStatus.FAILURE

    def _stop_mobile(self):
        """Stop the mobile base by publishing zero velocity."""
        if 'leader_mobile' in self.publishers:
            twist_msg = Twist()
            twist_msg.linear.x = ZERO_VELOCITY  # noqa: F405
            twist_msg.linear.y = ZERO_VELOCITY  # noqa: F405
            twist_msg.angular.z = ZERO_VELOCITY  # noqa: F405
            self.publishers['leader_mobile'].publish(twist_msg)
            self.log_info('Mobile base stopped')

    def reset(self):
        """Reset the action to its initial state."""
        super().reset()
        self._stop_event.set()
        self._stop_mobile()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=THREAD_JOIN_TIMEOUT_SEC)  # noqa: F405
        self._thread = None
        with self._lock:
            self._result = None
            self.odom_start_yaw = None
            self.odom_last_yaw = None
            self._rotated_deg = 0.0
            self._odom_received_at = None
