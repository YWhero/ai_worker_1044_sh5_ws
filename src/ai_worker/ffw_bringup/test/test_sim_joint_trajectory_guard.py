import pytest

from ffw_bringup.sim_joint_trajectory_guard import clamp_positions


def test_clamps_mirrored_joint2_at_the_follower_boundary():
    names = ['arm_l_joint2', 'arm_r_joint2']

    positions, changes = clamp_positions(names, [-0.35, 0.35])

    assert positions == [0.0, 0.0]
    assert changes == [
        ('arm_l_joint2', -0.35, 0.0),
        ('arm_r_joint2', 0.35, 0.0),
    ]


def test_preserves_valid_positions_and_clamps_grippers():
    names = ['arm_l_joint2', 'arm_r_joint2', 'gripper_l_joint1', 'unknown']

    positions, changes = clamp_positions(names, [0.4, -0.4, 1.2, 99.0])

    assert positions == [0.4, -0.4, 1.05, 99.0]
    assert changes == [('gripper_l_joint1', 1.2, 1.05)]


def test_rejects_mismatched_joint_and_position_counts():
    with pytest.raises(ValueError, match='position count 1 does not match joint count 2'):
        clamp_positions(['arm_l_joint1', 'arm_l_joint2'], [0.0])
