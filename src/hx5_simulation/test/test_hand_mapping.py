"""Official endpoints, mechanical limits and the physical LG2 message boundary."""
from copy import deepcopy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from hx5_simulation.hand_mapping import interpolate, normalize, validate_profile

ROOT = Path(__file__).resolve().parents[3]
PROFILE_PATH = ROOT / 'runtime/hx5_isaac/official_1044_hand_mapping.json'
ROBOT_PATH = ROOT / 'simulation/isaac/assets/sh5/robot.urdf'
if not PROFILE_PATH.exists():
    PROFILE_PATH = Path('/hx5_isaac_runtime/official_1044_hand_mapping.json')
    ROBOT_PATH = Path('/isaac_assets/robot.urdf')
PROFILE = json.loads(PROFILE_PATH.read_text())
ROBOT = ET.parse(ROBOT_PATH).getroot()
BOUNDS = {j.get('name'): (float(j.find('limit').get('lower')), float(j.find('limit').get('upper')))
          for j in ROBOT.findall('joint') if j.find('limit') is not None and j.get('type') != 'continuous'}


@pytest.mark.parametrize('value,expected', [(-1, 0), (-.1, 0), (.1, 1/6), (.2, .25), (1.1, 1), (2, 1)])
def test_official_1044_gripper_normalization(value, expected):
    assert normalize(value, *PROFILE['gripper_range']) == pytest.approx(expected)


def test_pinned_official_1044_endpoints_and_legal_interpolation():
    validate_profile(PROFILE, BOUNDS)
    assert 'a7e047128e730050c6bf21f9b74f6d73139c24bc' in PROFILE['source']
    assert PROFILE['left']['release'][:4] == [0, 1.57, 0, 0]
    assert PROFILE['left']['grasp'][:4] == [-.2, 1.57, -.6, -.48]
    assert PROFILE['right']['grasp'][-4:] == [0, .5, 1.4, 1.2]
    for hand, side in (('left', 'l'), ('right', 'r')):
        endpoints = PROFILE[hand]
        for curl in (0, .25, .5, 1):
            positions = interpolate(endpoints['release'], endpoints['grasp'], [curl]*5, BOUNDS, side)
            assert positions == pytest.approx([a + curl*(b-a) for a,b in zip(endpoints['release'], endpoints['grasp'])])
            assert all(BOUNDS[f'finger_{side}_joint{i+1}'][0] <= p <= BOUNDS[f'finger_{side}_joint{i+1}'][1]
                       for i,p in enumerate(positions))


def test_limit_enforcement_and_delayed_thumb_follow_official_normalization():
    values = interpolate([0]*20, [100]*20, [.25]*5, BOUNDS, 'r', thumb_threshold=.5)
    assert values[:4] == [0]*4
    assert values[5] == BOUNDS['finger_r_joint6'][1]
    invalid = deepcopy(PROFILE); invalid['left']['grasp'][2] = math.nan
    with pytest.raises(ValueError, match='finite'):
        validate_profile(invalid, BOUNDS)
    with pytest.raises(ValueError):
        normalize(math.nan, -.1, 1.1)


def test_unconfigured_gazebo_template_keeps_legacy_zero_open_and_proportional_curl():
    for side in ('l','r'):
        template = []
        for number in range(1,21):
            lower,upper = BOUNDS[f'finger_{side}_joint{number}']
            value = (-.4 if side == 'l' else .4) if number == 1 else 0 if number % 4 == 1 else .65*(upper if upper > abs(lower) else lower)
            template.append(value)
        assert interpolate([0]*20, template, [.5]*5, BOUNDS, side) == pytest.approx([value*.5 for value in template])
    assert normalize(.525, 0, 1.05) == .5


def make_adapter(leader_hand_duration=.1):
    from hx5_simulation.hand_presets import HandPresets
    class Publisher:
        def __init__(self): self.messages = []
        def publish(self, message): self.messages.append(message)
    adapter = HandPresets.__new__(HandPresets)
    adapter.bounds = BOUNDS
    adapter.open_positions = {s: PROFILE[h]['release'] for h,s in (('left','l'),('right','r'))}
    adapter.templates = {s: PROFILE[h]['grasp'] for h,s in (('left','l'),('right','r'))}
    adapter.gripper_range = PROFILE['gripper_range']; adapter.thumb_threshold = 0.0
    adapter.leader_hand_duration = HandPresets.validate_leader_hand_duration(leader_hand_duration)
    adapter.presets = {0: {'curls': [0]*5}, 1: {'curls': [1]*5}, 2: {'curls': [1,1,0,0,0]}}
    adapter.active = {'left': 1, 'right': 1}
    adapter.arm_publishers = {s: Publisher() for s in ('l','r')}
    adapter.publishers_by_side = {s: Publisher() for s in ('l','r')}
    adapter.publish_status = lambda: None
    return adapter


def test_actual_eight_axis_leader_callback_produces_only_7_plus_20_for_each_hand():
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    adapter = make_adapter(); actions = []
    for side in ('l','r'):
        names = [f'arm_{side}_joint{i}' for i in range(1,8)]
        # Reorder the source to require real name filtering, rather than slicing.
        input_names = [f'gripper_{side}_joint1'] + names[::-1]
        message = JointTrajectory(joint_names=input_names,
            points=[JointTrajectoryPoint(positions=[.2] + [0.0]*7, velocities=[0.0]*8)])
        adapter.leader(side, message)
        arm = adapter.arm_publishers[side].messages[-1]
        hand = adapter.publishers_by_side[side].messages[-1]
        assert arm.joint_names == names[::-1] and len(arm.points[0].positions) == 7
        assert hand.joint_names == [f'finger_{side}_joint{i}' for i in range(1,21)]
        assert hand.points[0].positions == pytest.approx(adapter.positions(side, [.25]*5))
        index = {name:i for i,name in enumerate(arm.joint_names)}
        actions.extend(arm.points[0].positions[index[name]] for name in names)
        actions.extend(hand.points[0].positions)
        assert list(message.points[0].positions) == [.2]+[0.0]*7
    assert len(actions) == 54 and all(math.isfinite(value) for value in actions)


@pytest.mark.parametrize('duration', [0.0, .1, .25, 1.0])
def test_leader_hand_duration_changes_only_hand_horizon_and_preserves_arm_alignment(duration):
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    adapter = make_adapter(duration)
    for side in ('l', 'r'):
        arm_names = [f'arm_{side}_joint{i}' for i in range(1, 8)]
        point = JointTrajectoryPoint(positions=[0.0] * 7 + [.2])
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 350_000_000
        message = JointTrajectory(joint_names=arm_names + [f'gripper_{side}_joint1'], points=[point])
        adapter.leader(side, message)
        arm = adapter.arm_publishers[side].messages[-1]
        hand = adapter.publishers_by_side[side].messages[-1]
        assert arm.points[0].time_from_start.nanosec == 350_000_000
        actual_duration = hand.points[0].time_from_start.sec + hand.points[0].time_from_start.nanosec * 1e-9
        assert actual_duration == pytest.approx(duration)
        assert hand.points[0].positions == pytest.approx(adapter.positions(side, [.25] * 5))
        assert len(arm.points[0].positions) + len(hand.points[0].positions) == 27
        assert all(BOUNDS[name][0] <= value <= BOUNDS[name][1]
                   for name, value in zip(hand.joint_names, hand.points[0].positions))
        assert message.points[0].time_from_start.nanosec == 350_000_000


def test_default_leader_hand_duration_preserves_gazebo_and_preset_service_transition():
    from rcl_interfaces.msg import Parameter, ParameterValue
    from rcl_interfaces.srv import SetParametersAtomically
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    for duration in (.1, 0.0):
        adapter = make_adapter(duration)
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name='side', value=ParameterValue(type=4, string_value='left')),
            Parameter(name='preset_id', value=ParameterValue(type=2, integer_value=2))])
        assert adapter.service('set', request, SetParametersAtomically.Response()).result.successful
        selected = adapter.publishers_by_side['l'].messages[-1]
        assert selected.points[0].time_from_start.nanosec == 400_000_000
        adapter.leader('l', JointTrajectory(joint_names=['arm_l_joint7', 'gripper_l_joint1'],
                       points=[JointTrajectoryPoint(positions=[0.0, .2])]))
        streamed = adapter.publishers_by_side['l'].messages[-1]
        assert streamed.points[0].time_from_start.nanosec == round(duration * 1e9)
        assert streamed.points[0].positions == pytest.approx(adapter.positions('l', [.25, .25, 0, 0, 0]))


@pytest.mark.parametrize('duration', [-.1, 1.01, math.nan, math.inf, -math.inf])
def test_invalid_leader_duration_is_rejected_before_commands(duration):
    from hx5_simulation.hand_presets import HandPresets
    with pytest.raises(ValueError, match='leader_hand_duration'):
        HandPresets.validate_leader_hand_duration(duration)


def test_service_selected_preset_changes_subsequent_physical_gripper_mapping():
    from rcl_interfaces.msg import Parameter, ParameterValue
    from rcl_interfaces.srv import SetParametersAtomically
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    adapter = make_adapter()
    request = SetParametersAtomically.Request(parameters=[
        Parameter(name='side', value=ParameterValue(type=4, string_value='left')),
        Parameter(name='preset_id', value=ParameterValue(type=2, integer_value=2))])
    response = adapter.service('set', request, SetParametersAtomically.Response())
    assert response.result.successful and adapter.active['left'] == 2
    adapter.leader('l', JointTrajectory(joint_names=['arm_l_joint7','gripper_l_joint1'],
        points=[JointTrajectoryPoint(positions=[.01,.2])]))
    output = adapter.publishers_by_side['l'].messages[-1]
    assert output.points[0].positions == pytest.approx(adapter.positions('l', [.25,.25,0,0,0]))
    assert output.points[0].positions[10] == 0 and output.points[0].positions[2] != 0


@pytest.mark.parametrize('positions', [[.1], [.1, math.nan]])
def test_malformed_leader_input_does_not_publish_any_physics_command(positions):
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    adapter = make_adapter()
    adapter.leader('l', JointTrajectory(joint_names=['arm_l_joint7','gripper_l_joint1'],
        points=[JointTrajectoryPoint(positions=positions)]))
    assert not adapter.arm_publishers['l'].messages and not adapter.publishers_by_side['l'].messages


def test_unknown_arm_joint_does_not_crash_or_publish_invalid_articulation_target():
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    adapter = make_adapter()
    adapter.leader('l', JointTrajectory(joint_names=['arm_l_joint99','gripper_l_joint1'],
        points=[JointTrajectoryPoint(positions=[.1,.2])]))
    assert not adapter.arm_publishers['l'].messages and not adapter.publishers_by_side['l'].messages
