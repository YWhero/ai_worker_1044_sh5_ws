"""Readiness must prove a single official mapper and fresh measured physics."""
from copy import deepcopy
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

SCRIPT = Path(__file__).with_name('leader_runtime_check.py')
spec = importlib.util.spec_from_file_location('leader_runtime_check', SCRIPT)
check = importlib.util.module_from_spec(spec); spec.loader.exec_module(check)
ROOT = SCRIPT.parents[2]
PROFILE_BYTES = SCRIPT.with_name('official_1044_hand_mapping.json').read_bytes()
ROBOT = ET.parse(ROOT / 'simulation/isaac/assets/sh5/robot.urdf').getroot()
REQUIRED = set(check.joint_limits(ROBOT))


def stamp(value):
    return SimpleNamespace(sec=int(value), nanosec=round((value-int(value))*1e9))


def parameters():
    return {'mapping_profile': check.MAPPING_PATH, 'use_sim_time': True,
            'robot_description': ET.tostring(ROBOT, encoding='unicode')}


def ready_observation():
    observation = check.Observation()
    observation.receive_clock(stamp(10), now=100)
    observation.receive_clock(stamp(10.02), now=100.1)
    message = SimpleNamespace(name=sorted(REQUIRED), position=[0.0]*len(REQUIRED),
                              header=SimpleNamespace(stamp=stamp(10.01)))
    observation.receive_joints(message, now=100.1)
    return observation


def test_missing_transient_status_does_not_claim_absent_profile_when_live_parameters_prove_mapping():
    observation = ready_observation()
    assert observation.status is None
    assert check.verify_mapping(parameters(), ROBOT, lambda _: PROFILE_BYTES) == check.MAPPING_NAME
    assert len(REQUIRED) == 63
    assert observation.feedback(REQUIRED, now=100.2) == pytest.approx(.01)


@pytest.mark.parametrize('value', [None, '', '/workspace/generic.json', '/other/copied-official.json'])
def test_actual_missing_or_wrong_mapper_path_remains_failure(value):
    data = parameters(); data['mapping_profile'] = value
    with pytest.raises(RuntimeError, match='mapping_profile'):
        check.verify_mapping(data, ROBOT, lambda _: PROFILE_BYTES)


def test_pinned_file_tampering_is_detected_even_with_correct_node_parameter_path():
    data = json.loads(PROFILE_BYTES); data['left']['grasp'][2] += .01
    with pytest.raises(RuntimeError, match='pinned'):
        check.verify_mapping(parameters(), ROBOT, lambda _: json.dumps(data).encode())


def test_mapper_robot_limits_must_match_actual_follower_and_simulation_clock():
    data = parameters(); data['use_sim_time'] = False
    with pytest.raises(RuntimeError, match='simulation time'):
        check.verify_mapping(data, ROBOT, lambda _: PROFILE_BYTES)
    altered = deepcopy(ROBOT); altered.find("joint[@name='finger_l_joint2']/limit").set('upper', '2.0')
    data = parameters(); data['robot_description'] = ET.tostring(altered, encoding='unicode')
    with pytest.raises(RuntimeError, match='joint limits'):
        check.verify_mapping(data, ROBOT, lambda _: PROFILE_BYTES)


def test_five_identically_named_publishers_are_rejected_before_random_parameter_service_selection():
    details = [{'node':'/hx5_sim_hand_presets', 'gid':str(number)} for number in range(5)]
    with pytest.raises(RuntimeError, match='Duplicate.*hand_preset/status') as failure:
        check.check_publishers({'/leader/hand_preset/status': details})
    assert '"gid": "4"' in str(failure.value)


def test_unique_publishers_are_admitted_but_discovery_missing_publisher_is_not_duplicate():
    check.check_publishers({'/clock':[{'node':'/bridge','gid':'one'}], '/leader/hand_preset/status':[]})


def test_old_status_cache_cannot_override_current_mapper_parameters():
    observation = ready_observation()
    observation.receive_status(json.dumps({'mapping_profile':{'name':'Simulation generic'}}))
    assert check.verify_mapping(parameters(), ROBOT, lambda _: PROFILE_BYTES) == check.MAPPING_NAME
    assert observation.status['mapping_profile']['name'] == 'Simulation generic'


@pytest.mark.parametrize('data', ['{}', '{"mapping_profile":null}', '{"mapping_profile":{"name":"Simulation generic"}}', 'not json', '[]'])
def test_missing_wrong_or_malformed_status_never_substitutes_for_live_parameters(data):
    observation = ready_observation(); observation.receive_status(data)
    generic = parameters(); generic['mapping_profile'] = None
    with pytest.raises(RuntimeError, match='mapping_profile'):
        check.verify_mapping(generic, ROBOT, lambda _: PROFILE_BYTES)


@pytest.mark.parametrize('change', ['missing', 'nan', 'duplicate', 'mismatched_lengths', 'stale_sim', 'future_stamp'])
def test_incomplete_invalid_or_stale_physics_feedback_blocks_hardware_start(change):
    observation = ready_observation()
    if change == 'missing':
        observation.joints.name.pop(); observation.joints.position.pop()
    elif change == 'nan': observation.joints.position[0] = math.nan
    elif change == 'duplicate': observation.joints.name[1] = observation.joints.name[0]
    elif change == 'mismatched_lengths': observation.joints.position.pop()
    elif change == 'stale_sim': observation.joints.header.stamp = stamp(9)
    elif change == 'future_stamp': observation.joints.header.stamp = stamp(11)
    with pytest.raises(RuntimeError): observation.feedback(REQUIRED, now=100.2)


def test_cached_samples_with_same_sim_stamp_do_not_pass_paused_simulation():
    observation = ready_observation()
    observation.receive_clock(stamp(10.02), now=103)
    observation.receive_joints(observation.joints, now=103)
    with pytest.raises(RuntimeError, match='advancing'):
        observation.feedback(REQUIRED, now=103.1)


def test_old_joint_receipt_is_rejected_even_when_header_matches_fresh_clock():
    observation = ready_observation()
    observation.receive_clock(stamp(10.03), now=103)
    with pytest.raises(RuntimeError, match='wall time'):
        observation.feedback(REQUIRED, now=103.1)
