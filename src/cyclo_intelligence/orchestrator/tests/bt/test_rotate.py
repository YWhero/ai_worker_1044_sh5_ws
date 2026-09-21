"""Regression tests for closed-loop rotation without moving a ROS robot."""

import math
import time
from types import SimpleNamespace

import pytest

from orchestrator.bt.actions import rotate as rotate_module
from orchestrator.bt.actions.rotate import Rotate


@pytest.fixture(autouse=True)
def twist_message(monkeypatch):
    # The repository's other CPU-only BT tests may install ROS import stubs.
    # Commands remain inspectable in both that environment and a ROS overlay.
    monkeypatch.setattr(rotate_module, 'Twist', lambda: SimpleNamespace(
        linear=SimpleNamespace(x=0.0, y=0.0),
        angular=SimpleNamespace(z=0.0),
    ))


class Node:
    def __init__(self):
        self.commands = []
        self.logs = []

    def create_publisher(self, *args):
        return SimpleNamespace(publish=self.commands.append)

    def create_subscription(self, *args):
        return object()

    def get_logger(self):
        return SimpleNamespace(info=self.logs.append, warn=self.logs.append, error=self.logs.append)


def action(**kwargs):
    return Rotate(Node(), topic_config={
        'topic_map': {'leader_mobile': '/cmd_vel', 'follower_mobile': '/odom'},
        'topic_type_map': {'leader_mobile': 'geometry_msgs/msg/Twist',
                           'follower_mobile': 'nav_msgs/msg/Odometry'},
    }, **kwargs)


def odom(yaw_deg):
    yaw = math.radians(yaw_deg)
    quaternion = SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(orientation=quaternion)))


@pytest.mark.parametrize(('angles', 'target'), [
    ([170, -170, -100, -10, 80], 270),
    ([-170, 170, 100, 10, -80], -270),
    ([0, 90, 180, -90, 0], 360),
])
def test_wrap_crossing_accumulates_signed_rotation(angles, target):
    rotate = action(angle_deg=target)
    for angle in angles:
        rotate._odom_callback(odom(angle))
    assert rotate._rotated_deg == pytest.approx(target)
    rotate._control_loop()
    assert rotate._result is True
    assert rotate.node.commands[-1].angular.z == 0.0


def test_stale_odometry_fails_and_stops():
    rotate = action()
    rotate._odom_callback(odom(0))
    rotate._odom_received_at = time.monotonic() - 2
    rotate._control_loop()
    assert rotate._result is False
    assert rotate.node.commands[-1].angular.z == 0.0
    assert any('stale' in message for message in rotate.node.logs)


def test_nonfinite_odometry_cannot_refresh_feedback():
    rotate = action()
    rotate._odom_callback(odom(0))
    received_at = rotate._odom_received_at
    rotate._odom_callback(odom(float('nan')))
    assert rotate._odom_received_at == received_at
    assert rotate._rotated_deg == 0


def test_rotation_deadline_fails_and_stops_even_with_feedback():
    rotate = action(timeout_sec=0.001)
    rotate._odom_callback(odom(0))
    rotate._control_loop()
    assert rotate._result is False
    assert rotate.node.commands[-1].angular.z == 0.0
    assert any('timed out' in message for message in rotate.node.logs)


def test_reset_stops_mobile_and_clears_rotation():
    rotate = action()
    rotate._odom_callback(odom(0))
    rotate._odom_callback(odom(30))
    rotate.reset()
    assert rotate.node.commands[-1].angular.z == 0.0
    assert rotate._rotated_deg == 0.0
    assert rotate._odom_received_at is None


def test_xml_exposes_speed_tolerance_and_timeout():
    node = Node()
    rotate = Rotate.from_xml_params(
        SimpleNamespace(node=node, topic_config={}), 'fast_rotate',
        {'angle_deg': '-270', 'angular_velocity': '1.5', 'tolerance_deg': '1', 'timeout_sec': '45'},
    )
    assert rotate.name == 'fast_rotate'
    assert rotate.angle_deg == -270
    assert rotate.angular_velocity == 1.5
    assert rotate.tolerance_deg == 1
    assert rotate.timeout_sec == 45


@pytest.mark.parametrize('kwargs', [
    {'angle_deg': 'nan'}, {'angular_velocity': 0},
    {'tolerance_deg': -1}, {'timeout_sec': float('inf')},
])
def test_rejects_unbounded_or_invalid_rotation(kwargs):
    with pytest.raises(ValueError):
        action(**kwargs)
