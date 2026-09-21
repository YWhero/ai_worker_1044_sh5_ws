"""Physical-port ownership, profile isolation and reset admission checks."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import yaml

DIRECTORY = Path(__file__).resolve().parent
ROOT = DIRECTORY.parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name, DIRECTORY / f'{name}.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


config = load('leader_config')
preflight = load('leader_preflight')
reset = load('reset_simulation')
SOURCE = ROOT / 'src/ai_worker/ffw_bringup/config/ffw_lg2_leader/ffw_lg2_leader_ai_hardware_controller.yaml'
URDF = ROOT / 'simulation/isaac/assets/sh5/robot.urdf'


@pytest.mark.parametrize('mobile', [False, True])
def test_private_profile_preserves_all_controllers_and_derives_actual_sh5_limits(mobile):
    original = SOURCE.read_bytes()
    data = config.make_config(SOURCE, URDF, mobile)['/**']
    assert set(data) == {'controller_manager', 'joint_trajectory_command_broadcaster',
                         'spring_actuator_controller_left', 'spring_actuator_controller_right', 'joystick_controller'}
    assert data['controller_manager']['ros__parameters']['use_sim_time'] is False
    broadcaster = data['joint_trajectory_command_broadcaster']['ros__parameters']
    robot = ET.parse(URDF).getroot()
    for hand, side in (('left','l'),('right','r')):
        for number in range(1,8):
            limit = robot.find(f"joint[@name='arm_{side}_joint{number}']/limit")
            assert broadcaster[f'{hand}_min_positions'][number-1] == float(limit.get('lower'))
            assert broadcaster[f'{hand}_max_positions'][number-1] == float(limit.get('upper'))
        assert broadcaster[f'{hand}_offsets'][-1] == .3
        assert broadcaster[f'{hand}_min_positions'][-1] == 0
        assert broadcaster[f'{hand}_max_positions'][-1] == 1.05
        assert f'{hand}_reverse_joints' not in broadcaster
    joystick = data['joystick_controller']['ros__parameters']
    assert joystick['enable_joystick_axes'] is True and joystick['enable_swerve_mode'] is mobile
    assert joystick['sensorxel_l_joy_min_positions'][1] == -.35
    assert joystick['sensorxel_l_joy_max_positions'][1] == .35
    assert SOURCE.read_bytes() == original


def free_port(*args, **kwargs):
    return SimpleNamespace(returncode=1, stdout='', stderr='')


def identity(path):
    return {'/left':1, '/right':2, '/alias-left':1}[path]


def test_missing_noncharacter_and_aliased_devices_are_rejected(tmp_path):
    with pytest.raises(FileNotFoundError): preflight.device_identity(tmp_path / 'missing')
    path = tmp_path / 'plain-file'; path.write_text('no USB')
    with pytest.raises(RuntimeError, match='character'): preflight.device_identity(path)
    with pytest.raises(RuntimeError, match='distinct'):
        preflight.check_devices('/left', '/alias-left', [], identity, free_port)


def test_only_two_distinct_unowned_leader_devices_are_admitted():
    result = preflight.check_devices('/left', '/right', [], identity, free_port)
    assert result['domain'] == 115 and result['router'].endswith(':7855')


@pytest.mark.parametrize('containers', [
    [{'Name':'/other_hardware', 'State':{'Running':True}, 'HostConfig':{'Devices':[{'PathOnHost':'/alias-left'}]}}],
    [{'Name':'/lg2_leader_1044_hx5_isaac', 'State':{'Running':True}}],
    [{'Name':'/other_hardware', 'State':{'Running':True}, 'Mounts':[{'Source':'/left'}]}],
])
def test_running_hardware_container_or_usb_alias_prevents_double_open(containers):
    with pytest.raises(RuntimeError):
        preflight.check_devices('/left', '/right', containers, identity, free_port)


def test_stopped_hardware_container_does_not_claim_usb():
    preflight.check_devices('/left', '/right', [{'Name':'/old', 'State':{'Running':False},
        'HostConfig':{'Devices':[{'PathOnHost':'/left'}]}}], identity, free_port)


@pytest.mark.parametrize('result', [
    SimpleNamespace(returncode=0, stdout='1234', stderr=''),
    SimpleNamespace(returncode=1, stdout='', stderr='Permission denied'),
    SimpleNamespace(returncode=2, stdout='', stderr='fuser failed'),
])
def test_in_use_or_unverifiable_usb_ownership_prevents_start(result):
    with pytest.raises(RuntimeError):
        preflight.check_devices('/left', '/right', [], identity, lambda *a, **k: result)


def test_leader_presence_blocks_reset_before_any_api_or_ros_request(monkeypatch):
    monkeypatch.setattr(reset.subprocess, 'run', lambda *a, **k: SimpleNamespace(returncode=0, stdout='true\n'))
    monkeypatch.setattr(reset, 'api', lambda *a: pytest.fail('reset must reject before API/ROS requests'))
    with pytest.raises(RuntimeError, match='leader-stop'):
        reset.check_idle()


def test_leader_compose_exposes_only_two_usb_devices_and_correct_domain():
    leader = yaml.safe_load((DIRECTORY / 'leader-compose.yaml').read_text())['services']['leader']
    assert len(leader['devices']) == 2
    assert leader['devices'][0].endswith(':/dev/left_leader') and leader['devices'][1].endswith(':/dev/right_leader')
    assert leader['environment']['ROS_DOMAIN_ID'] == '115'
    assert ':7855' in leader['environment']['ZENOH_CONFIG_OVERRIDE']
    assert leader.get('privileged', False) is False
    assert all(not volume.startswith('/dev:') for volume in leader['volumes'])
    assert leader['stop_signal'] == 'SIGINT'
