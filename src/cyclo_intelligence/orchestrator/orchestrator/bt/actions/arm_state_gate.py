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

"""Observe configured arm, hand, head, lift and optional tactile conditions.

The gate never publishes commands or stops inference itself. Follow it with
SendCommand STOP in a Sequence to stop inference before the next action.
Tactile thresholds use the raw sum per 3x3 sensor, not calibrated grasp force.
"""

import math
import time
from typing import TYPE_CHECKING, Optional

from orchestrator.bt.actions.base_action import BaseAction
from orchestrator.bt.bt_core import NodeStatus
from orchestrator.bt.constants import QOS_QUEUE_DEPTH
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

if TYPE_CHECKING:
    from rclpy.node import Node


def _coerce_names(value) -> list[str]:
    if value is None:
        return []
    parts = value.split(',') if isinstance(value, str) else value
    if not isinstance(parts, (list, tuple)):
        parts = [parts]
    return [str(part).strip() for part in parts if str(part).strip()]


def _coerce_positions(value) -> list[float]:
    return [float(part) for part in _coerce_names(value)]


def _coerce_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ('true', '1', 'yes', 'on')
    return bool(value)


def _make_targets(group: str, joints, positions) -> list[tuple[str, float]]:
    names = _coerce_names(joints)
    values = _coerce_positions(positions)
    if len(names) != len(values):
        raise ValueError(
            f'ArmStateGate: {group} joint names and positions must have '
            f'the same length ({len(names)} != {len(values)})'
        )
    if len(names) != len(set(names)) or not all(math.isfinite(v) for v in values):
        raise ValueError(f'ArmStateGate: invalid {group} joint targets')
    return list(zip(names, values))


class ArmStateGate(BaseAction):
    """Gate SUCCESS on selected fresh conditions, optionally held continuously.

    Existing SG2 gripper closed-then-opened XML remains supported. HX5 events
    compare any selected finger joints against separate closed/open vectors.
    Contact observes official HandPressures sensor arrays, with an explicit
    raw threshold and sensor count; it does not certify successful grasping.
    All enabled conditions must be satisfied. Empty target groups are ignored.
    """

    @classmethod
    def from_xml_params(cls, context, name: str, params: dict):
        action = cls(node=context.node, topic_config=context.topic_config, **params)
        action.name = name
        return action

    def __init__(
        self,
        node: 'Node',
        left_target_joints: str = '',
        left_target_positions: str = '',
        right_target_joints: str = '',
        right_target_positions: str = '',
        left_hand_target_joints: str = '',
        left_hand_target_positions: str = '',
        right_hand_target_joints: str = '',
        right_hand_target_positions: str = '',
        head_target_joints: str = '',
        head_target_positions: str = '',
        lift_target_joints: str = '',
        lift_target_positions: str = '',
        joint_threshold: float = 0.01,
        detect_left_gripper: bool = False,
        detect_right_gripper: bool = False,
        left_gripper_joint: str = 'gripper_l_joint1',
        right_gripper_joint: str = 'gripper_r_joint1',
        gripper_closed_value: float = 1.0,
        gripper_open_value: float = 0.0,
        gripper_threshold: float = 0.05,
        detect_left_hand: bool = False,
        detect_right_hand: bool = False,
        left_hand_event_joints: str = '',
        left_hand_closed_positions: str = '',
        left_hand_open_positions: str = '',
        right_hand_event_joints: str = '',
        right_hand_closed_positions: str = '',
        right_hand_open_positions: str = '',
        hand_threshold: float = 0.05,
        detect_left_contact: bool = False,
        detect_right_contact: bool = False,
        left_contact_sensor_names: str = '',
        right_contact_sensor_names: str = '',
        left_contact_condition: str = 'contact',
        right_contact_condition: str = 'contact',
        contact_pressure_threshold: float = 30.0,
        contact_min_sensors: int = 1,
        hold_sec: float = 0.0,
        state_max_age_sec: float = 1.0,
        timeout_sec: float = 0.0,
        topic_config: Optional[dict] = None,
    ):
        super().__init__(node, name='ArmStateGate')
        self.left_targets = _make_targets('left', left_target_joints, left_target_positions)
        self.right_targets = _make_targets('right', right_target_joints, right_target_positions)
        self.targets = self.left_targets + self.right_targets
        for group, names, values in (
            ('left_hand', left_hand_target_joints, left_hand_target_positions),
            ('right_hand', right_hand_target_joints, right_hand_target_positions),
            ('head', head_target_joints, head_target_positions),
            ('lift', lift_target_joints, lift_target_positions),
        ):
            self.targets.extend(_make_targets(group, names, values))
        if len({name for name, _ in self.targets}) != len(self.targets):
            raise ValueError('ArmStateGate: select each target joint only once')

        self.joint_threshold = float(joint_threshold)
        self.gripper_threshold = float(gripper_threshold)
        self.hand_threshold = float(hand_threshold)
        self.hold_sec = float(hold_sec)
        self.state_max_age_sec = float(state_max_age_sec)
        self.timeout_sec = float(timeout_sec)
        self.contact_pressure_threshold = float(contact_pressure_threshold)
        self.contact_min_sensors = int(contact_min_sensors)
        scalars = (self.joint_threshold, self.gripper_threshold, self.hand_threshold,
                   self.hold_sec, self.state_max_age_sec, self.timeout_sec,
                   self.contact_pressure_threshold)
        if not all(math.isfinite(v) and v >= 0 for v in scalars):
            raise ValueError('ArmStateGate: thresholds and times must be finite and non-negative')
        if self.state_max_age_sec <= 0 or self.contact_min_sensors < 1:
            raise ValueError('ArmStateGate: state_max_age_sec and contact_min_sensors must be positive')

        self.detect_left_gripper = _coerce_bool(detect_left_gripper)
        self.detect_right_gripper = _coerce_bool(detect_right_gripper)
        self.left_gripper_joint = str(left_gripper_joint)
        self.right_gripper_joint = str(right_gripper_joint)
        self.gripper_closed_value = float(gripper_closed_value)
        self.gripper_open_value = float(gripper_open_value)
        if not all(math.isfinite(v) for v in (self.gripper_closed_value, self.gripper_open_value)):
            raise ValueError('ArmStateGate: gripper event values must be finite')

        self.hand_events = {}
        for side, enabled, names, closed, opened in (
            ('left', detect_left_hand, left_hand_event_joints,
             left_hand_closed_positions, left_hand_open_positions),
            ('right', detect_right_hand, right_hand_event_joints,
             right_hand_closed_positions, right_hand_open_positions),
        ):
            enabled = _coerce_bool(enabled)
            closed_targets = _make_targets(f'{side}_hand_closed', names, closed) if enabled else []
            open_targets = _make_targets(f'{side}_hand_open', names, opened) if enabled else []
            if enabled and (not closed_targets or closed_targets == open_targets):
                raise ValueError(f'ArmStateGate: {side} hand needs distinct non-empty closed/open targets')
            self.hand_events[side] = dict(enabled=enabled, closed=closed_targets,
                                          opened=open_targets, closed_seen=False, opened_seen=False)

        self.contact_conditions = {}
        for side, enabled, names, condition in (
            ('left', detect_left_contact, left_contact_sensor_names, left_contact_condition),
            ('right', detect_right_contact, right_contact_sensor_names, right_contact_condition),
        ):
            condition = str(condition).strip().lower()
            enabled = _coerce_bool(enabled)
            if enabled and condition not in ('contact', 'released'):
                raise ValueError(f'ArmStateGate: invalid {side}_contact_condition')
            sensor_names = _coerce_names(names)
            if len(sensor_names) != len(set(sensor_names)):
                raise ValueError('ArmStateGate: duplicate tactile sensor names')
            if enabled and sensor_names and self.contact_min_sensors > len(sensor_names):
                raise ValueError('ArmStateGate: contact_min_sensors exceeds selected sensors')
            self.contact_conditions[side] = dict(enabled=enabled, names=sensor_names, condition=condition)
        if not (self.targets or self.detect_left_gripper or self.detect_right_gripper
                or any(c['enabled'] for c in self.hand_events.values())
                or any(c['enabled'] for c in self.contact_conditions.values())):
            raise ValueError('ArmStateGate: enable at least one joint, hand event or contact condition')

        self.topic_config = topic_config or {}
        # Sensor-compatible subscription accepts both best-effort and reliable
        # publishers; a reliable reader cannot receive best-effort feedback.
        qos = QoSProfile(depth=QOS_QUEUE_DEPTH, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joint_state_topics = self._configured_topics('sensor_msgs/msg/JointState') or ['/joint_states']
        self.joint_states = {topic: None for topic in self.joint_state_topics}
        self._joint_values = {}
        self.joint_state_subscriptions = [
            self.node.create_subscription(JointState, topic,
                lambda msg, topic_name=topic: self._joint_state_callback(msg, topic_name), qos)
            for topic in self.joint_state_topics
        ]
        self._pressures = {}
        self.pressure_subscriptions = []
        contact_topics = self._configured_topics('robotis_interfaces/msg/HandPressures')
        if any(c['enabled'] for c in self.contact_conditions.values()):
            from robotis_interfaces.msg import HandPressures
            for side, condition in self.contact_conditions.items():
                if not condition['enabled']:
                    continue
                matches = [t for t in contact_topics if f'/{side}_hand/' in t]
                # Match configured semantic keys as well as standard official topics.
                for group, topic in self.topic_config.get('topic_map', {}).items():
                    if (topic in contact_topics and side in group and topic not in matches):
                        matches.append(topic)
                if len(matches) != 1:
                    raise ValueError(f'ArmStateGate: configure one {side} HandPressures topic')
                self.pressure_subscriptions.append(self.node.create_subscription(
                    HandPressures, matches[0],
                    lambda msg, side_name=side: self._pressure_callback(msg, side_name), qos))
        self._start_time = None
        self._matched_since = None
        self._left_gripper_closed = self._left_gripper_opened = False
        self._right_gripper_closed = self._right_gripper_opened = False
        self._missing_joints_warned = set()

    def _configured_topics(self, msg_type):
        topic_map = self.topic_config.get('topic_map', {})
        type_map = self.topic_config.get('topic_type_map', {})
        return list(dict.fromkeys(topic for group, topic in topic_map.items()
                                 if group.startswith('follower_') and type_map.get(group) == msg_type))

    def _joint_state_callback(self, msg, topic_name=None):
        topic = topic_name or self.joint_state_topics[0]
        self.joint_states[topic] = msg
        now = time.monotonic()
        # Preserve per-joint receipt time: a head-only message must not make
        # a stale finger position fresh just because it shares a topic.
        for name, value in zip(msg.name, msg.position):
            self._joint_values[name] = (float(value), now)

    def _pressure_callback(self, msg, side):
        self._pressures[side] = (msg, time.monotonic())

    def _name_to_position(self):
        now = time.monotonic()
        return {name: value for name, (value, stamp) in self._joint_values.copy().items()
                if now - stamp <= self.state_max_age_sec and math.isfinite(value)}

    @staticmethod
    def _is_close(value, target, threshold):
        return math.isfinite(value) and abs(value - target) <= threshold

    def _joint_value(self, name, positions):
        if name not in positions and name not in self._missing_joints_warned:
            self._missing_joints_warned.add(name)
            self.log_warn(f"Joint '{name}' missing, stale or non-finite in configured feedback")
        return positions.get(name)

    def _matches(self, targets, positions, threshold):
        return all((value := self._joint_value(name, positions)) is not None
                   and self._is_close(value, target, threshold) for name, target in targets)

    def _update_gripper(self, enabled, name, closed_seen, opened_seen, positions):
        if not enabled:
            return True, True
        value = self._joint_value(name, positions)
        if value is None:
            return closed_seen, opened_seen
        if not closed_seen and self._is_close(value, self.gripper_closed_value, self.gripper_threshold):
            closed_seen = True
        elif closed_seen and self._is_close(value, self.gripper_open_value, self.gripper_threshold):
            opened_seen = True
        return closed_seen, opened_seen

    def _contact_reached(self, side, condition):
        if not condition['enabled']:
            return True
        message, stamp = self._pressures.get(side, (None, 0))
        if message is None or time.monotonic() - stamp > self.state_max_age_sec:
            return False
        values = {}
        for sensor in message.sensors:
            raw = list(sensor.pressure_values)
            if len(raw) == 9 and all(math.isfinite(v) and 0 <= v <= 255 for v in raw):
                values[sensor.sensor_name] = sum(raw)
        # An unspecified selection means the official HX5 five sensors. A
        # partial or malformed message must not silently shrink that selection.
        names = condition['names'] or [f'finger_{side[0]}_sensor{i}' for i in range(1, 6)]
        if not names or any(name not in values for name in names):
            return False
        if condition['condition'] == 'released':
            # Release requires every selected sensor below the threshold;
            # missing sensors cannot be mistaken for zero pressure.
            return all(values[name] < self.contact_pressure_threshold for name in names)
        return sum(values[name] >= self.contact_pressure_threshold for name in names) >= self.contact_min_sensors

    def _timed_out(self):
        return (self.timeout_sec > 0 and self._start_time is not None
                and time.monotonic() - self._start_time >= self.timeout_sec)

    def tick(self) -> NodeStatus:
        if self._start_time is None:
            self._start_time = time.monotonic()
            self.log_info('Waiting for configured robot state conditions')
            return NodeStatus.RUNNING
        positions = self._name_to_position()
        self._left_gripper_closed, self._left_gripper_opened = self._update_gripper(
            self.detect_left_gripper, self.left_gripper_joint, self._left_gripper_closed,
            self._left_gripper_opened, positions)
        self._right_gripper_closed, self._right_gripper_opened = self._update_gripper(
            self.detect_right_gripper, self.right_gripper_joint, self._right_gripper_closed,
            self._right_gripper_opened, positions)
        hands_done = True
        for event in self.hand_events.values():
            if not event['enabled']:
                continue
            if not event['closed_seen']:
                event['closed_seen'] = self._matches(event['closed'], positions, self.hand_threshold)
            elif not event['opened_seen']:
                event['opened_seen'] = self._matches(event['opened'], positions, self.hand_threshold)
            # Require current open feedback as well as the remembered event.
            hands_done &= event['opened_seen'] and self._matches(event['opened'], positions, self.hand_threshold)
        legacy_done = self._left_gripper_opened and self._right_gripper_opened
        for enabled, name in ((self.detect_left_gripper, self.left_gripper_joint),
                              (self.detect_right_gripper, self.right_gripper_joint)):
            if enabled:
                value = self._joint_value(name, positions)
                legacy_done &= value is not None and self._is_close(
                    value, self.gripper_open_value, self.gripper_threshold,
                )
        matched = (legacy_done and hands_done
                   and self._matches(self.targets, positions, self.joint_threshold)
                   and all(self._contact_reached(side, c) for side, c in self.contact_conditions.items()))
        if matched:
            if self._matched_since is None:
                self._matched_since = time.monotonic()
            if time.monotonic() - self._matched_since >= self.hold_sec:
                self.log_info('Configured robot state conditions reached')
                return NodeStatus.SUCCESS
        else:
            self._matched_since = None
        if self._timed_out():
            self.log_error('Timeout waiting for configured robot state conditions')
            return NodeStatus.FAILURE
        return NodeStatus.RUNNING

    def reset(self):
        super().reset()
        self._start_time = self._matched_since = None
        self._left_gripper_closed = self._left_gripper_opened = False
        self._right_gripper_closed = self._right_gripper_opened = False
        for event in self.hand_events.values():
            event['closed_seen'] = event['opened_seen'] = False
        self._missing_joints_warned.clear()
        self.joint_states = {topic: None for topic in self.joint_state_topics}
        self._joint_values.clear()
        self._pressures.clear()
