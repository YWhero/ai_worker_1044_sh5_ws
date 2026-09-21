"""Reset must reject active publishers before stopping any Gazebo process."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location(
    'hx5_reset', Path(__file__).with_name('reset_simulation.py'))
reset = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reset)


@pytest.fixture
def idle(monkeypatch):
    states = {
        'navigation/status': {'is_up': False},
        'services/bt_node/status': {'state': 'down'},
    }
    status = {
        'recording': {'publishers': 1, 'phase': 0},
        'inference': {'publishers': 1, 'phase': None},
    }
    running = set()
    monkeypatch.setattr(reset, 'api', lambda path: states[path])
    monkeypatch.setattr(reset.subprocess, 'check_output',
                        lambda *args, **kwargs: json.dumps(status))
    monkeypatch.setattr(reset.subprocess, 'run',
                        lambda cmd, **kwargs: SimpleNamespace(
                            returncode=0, stdout=str(cmd[-1] in running).lower()))
    return states, status, running


def test_idle_without_backend_heartbeat_is_allowed(idle):
    reset.check_idle()


@pytest.mark.parametrize('service,field,value', [
    ('navigation/status', 'is_up', True),
    ('services/bt_node/status', 'state', 'up'),
])
def test_active_navigation_or_task_rejected(idle, service, field, value):
    idle[0][service][field] = value
    with pytest.raises(RuntimeError, match='Task Engine'):
        reset.check_idle()


@pytest.mark.parametrize('publishers,phase', [(0, 0), (1, None), (1, 1)])
def test_unavailable_or_unsaved_recording_rejected(idle, publishers, phase):
    idle[1]['recording'] = {'publishers': publishers, 'phase': phase}
    with pytest.raises(RuntimeError, match='save/discard'):
        reset.check_idle()


def test_loaded_inference_rejected(idle):
    idle[1]['inference']['phase'] = 2
    with pytest.raises(RuntimeError, match='inference'):
        reset.check_idle()


@pytest.mark.parametrize('container', [
    'lg2_leader_1044_hx5_sim', 'lerobot_server_1044_hx5_sim',
    'vitacformer_server_1044_hx5_sim', 'groot_server_1044_hx5_sim',
])
def test_running_backend_rejected_even_without_heartbeat(idle, container):
    idle[2].add(container)
    with pytest.raises(RuntimeError, match=container):
        reset.check_idle()


@pytest.mark.parametrize('override,expected', [
    (None, ['gui:=false', 'initial_pose:=navigation', 'rviz:=false']),
    ('inference', ['gui:=false', 'rviz:=false', 'initial_pose:=inference']),
])
def test_reset_can_switch_pose_without_losing_other_launch_options(monkeypatch, override, expected):
    monkeypatch.setattr(reset, 'check_idle', lambda: None)
    monkeypatch.setattr(reset.subprocess, 'check_output', lambda *args, **kwargs:
                        json.dumps(['gui:=false', 'initial_pose:=navigation', 'rviz:=false']))
    commands=[]
    monkeypatch.setattr(reset.subprocess, 'run', lambda command, **kwargs: commands.append(command))
    monkeypatch.setattr('sys.argv', ['reset_simulation.py'] +
                        (['--initial-pose', override] if override else []))
    reset.main()
    assert len(commands) == 1
    assert commands[0][commands[0].index('bash', commands[0].index('-c') + 1) + 1:] == expected
