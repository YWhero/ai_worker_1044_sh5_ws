import math
from types import SimpleNamespace

from nav_msgs.msg import Odometry
import pytest

from hx5_simulation.model import SPAWN_X, SPAWN_Y, SPAWN_YAW
from hx5_simulation.sim_io import SimIO, spawn_relative_odometry


def world_message(x, y, yaw):
    message = Odometry()
    message.header.frame_id = 'l_sorting_workcell'
    message.header.stamp.sec = 17
    message.header.stamp.nanosec = 123456
    message.child_frame_id = 'base_link'
    message.pose.pose.position.x = SPAWN_X + math.cos(SPAWN_YAW) * x - math.sin(SPAWN_YAW) * y
    message.pose.pose.position.y = SPAWN_Y + math.sin(SPAWN_YAW) * x + math.cos(SPAWN_YAW) * y
    message.pose.pose.position.z = 0.04
    message.pose.pose.orientation.z = math.sin((SPAWN_YAW + yaw) / 2)
    message.pose.pose.orientation.w = math.cos((SPAWN_YAW + yaw) / 2)
    message.twist.twist.linear.x = 0.23
    message.twist.twist.linear.y = -0.05
    message.twist.twist.angular.z = 0.61
    return message


@pytest.mark.parametrize('x,y,yaw', [(0.0, 0.0, 0.0), (1.2, -0.3, 0.7), (-0.5, 0.2, -3.13)])
def test_world_pose_is_expressed_in_spawn_axes_and_heading(x, y, yaw):
    source = world_message(x, y, yaw)
    output = spawn_relative_odometry(source)
    assert output.header.frame_id == 'odom'
    assert output.child_frame_id == 'base_link'
    assert output.header.stamp == source.header.stamp
    assert output.pose.pose.position.x == pytest.approx(x)
    assert output.pose.pose.position.y == pytest.approx(y)
    assert output.pose.pose.position.z == 0.0
    q = output.pose.pose.orientation
    actual_yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
    assert actual_yaw == pytest.approx(yaw)
    assert output.twist == source.twist  # Gazebo already computes body-frame velocity
    assert source.header.frame_id == 'l_sorting_workcell'
    assert source.pose.pose.position.z == 0.04


def test_navigation_odometry_and_its_only_base_transform_share_pose_and_stamp():
    odometry, transforms = [], []
    node = SimpleNamespace(base_odom=SimpleNamespace(publish=odometry.append),
                           base_tf=SimpleNamespace(sendTransform=transforms.append))
    SimIO.world_odometry(node, world_message(1.2, -0.3, 0.7))
    assert len(odometry) == len(transforms) == 1
    message, transform = odometry[0], transforms[0]
    assert transform.header == message.header
    assert transform.child_frame_id == message.child_frame_id
    assert transform.transform.translation.x == message.pose.pose.position.x
    assert transform.transform.translation.y == message.pose.pose.position.y
    assert transform.transform.translation.z == message.pose.pose.position.z
    assert transform.transform.rotation == message.pose.pose.orientation
