"""The live model uses the same joint frames as the official Isaac asset."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[2]
UI_URDF = ROOT / 'src/cyclo_intelligence/shared/shared/robot_configs/urdf/ffw_sh5_follower.urdf'
NATIVE_URDF = ROOT / 'simulation/isaac/assets/sh5/robot.urdf'
HAND_ROOT = ROOT / 'src/robotis_hand/robotis_hand_description/urdf/hx5_d20_rev2'


def joints(path):
    return {joint.get('name'): joint for joint in ET.parse(path).getroot().findall('.//joint')}


def assert_joint_frames_match(actual, expected):
    assert actual.get('type') == expected.get('type')
    for tag in ('parent', 'child'):
        assert actual.find(tag).get('link') == expected.find(tag).get('link')
    for tag, attributes in (('origin', ('xyz', 'rpy')), ('axis', ('xyz',)),
                            ('limit', ('lower', 'upper', 'velocity', 'effort'))):
        for attribute in attributes:
            actual_raw = actual.find(tag).get(attribute)
            expected_raw = expected.find(tag).get(attribute)
            if actual_raw is None or expected_raw is None:
                # Continuous wheel joints intentionally omit angular bounds.
                assert actual_raw is expected_raw is None
                continue
            actual_values = [float(value) for value in actual_raw.split()]
            expected_values = [float(value) for value in expected_raw.split()]
            assert actual_values == pytest.approx(expected_values, abs=1e-12)


@pytest.mark.parametrize('side', ('l', 'r'))
def test_all_twenty_finger_joint_frames_match_official_hx5_revision_two(side):
    actual = joints(UI_URDF)
    expected = joints(HAND_ROOT / f'hx5_d20_{"left" if side == "l" else "right"}.urdf.xacro')
    for index in range(1, 21):
        name = f'finger_{side}_joint{index}'
        reference = expected[f'${{prefix}}{name}']
        # The official xacro prefix is empty in the SH5 configuration.
        reference = ET.fromstring(ET.tostring(reference, encoding='unicode').replace('${prefix}', ''))
        assert_joint_frames_match(actual[name], reference)


def test_all_sixty_three_native_joint_frames_match_live_visualization():
    actual, expected = joints(UI_URDF), joints(NATIVE_URDF)
    names = [name for name, joint in expected.items() if joint.get('type') != 'fixed']
    assert len(names) == 63
    for name in names:
        assert_joint_frames_match(actual[name], expected[name])


def test_positive_official_head_pitch_turns_camera_forward_vector_down():
    import math
    head = joints(UI_URDF)['head_joint1']
    assert head.find('axis').get('xyz') == '0 1 0'
    pitch = float(head.find('limit').get('upper'))
    # R_y(pitch) maps the +X forward axis to [cos(pitch), 0, -sin(pitch)].
    assert pitch == pytest.approx(0.6951)
    assert -math.sin(pitch) < 0
