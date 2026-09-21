"""Exercise manual handoff with fake services, never physical robot calls."""

import os
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from interfaces.msg import InferenceStatus
from interfaces.srv import SendCommand
from orchestrator.internal.communication.container_service_client import ContainerServiceClient as Client
from orchestrator.internal.communication.model_switch import (
    intercept_model_switch_command, switch_inference_model, prepare_inference_model,
)


def fixture(mode=True, fail_at=None):
    client = SimpleNamespace(_service_prefix='/vitacformer')
    client.inference_command = Mock(side_effect=lambda cmd, **_: SimpleNamespace(
        success=cmd != fail_at, message='test failure' if cmd == fail_at else 'ok',
    ))
    node = SimpleNamespace(
        _state_lock=threading.Lock(), _inference_lifecycle_lock=threading.Lock(),
        _model_switch_operation=None, _loaded_inference_policy_path='/models/part1',
        _loaded_inference_publish_to_robot=mode, _inference_cycle_prepared=False,
        _inference_phase=InferenceStatus.INFERENCING,
        _prepared_inference_task_info=SimpleNamespace(task_instruction=['pour']),
        container_service_client=client, on_inference=True, on_recording=True,
        robot_type='ffw_sh5_rev1', _normalize_policy_path=os.path.normpath,
        _determine_service_prefix=lambda _: '/vitacformer',
        _cache_ui_task_info=Mock(), _publish_inference_phase=Mock(),
    )
    request = SimpleNamespace(command=26, task_info=SimpleNamespace(
        policy_path='/models/part2', inference_mode='robot' if mode else 'simulation',
        tags=[],
    ))
    return node, client, request


@pytest.mark.parametrize('mode', [True, False])
def test_switch_keeps_deploy_mode_task_and_recording(mode):
    node, client, request = fixture(mode)
    response = switch_inference_model(node, request, SimpleNamespace())
    assert response.success
    assert response.loaded_policy_path == '/models/part2'
    assert response.inference_phase == InferenceStatus.INFERENCING
    assert [c.args[0] for c in client.inference_command.call_args_list] == [
        Client.CMD_PAUSE, Client.CMD_SWITCH_POLICY, Client.CMD_RESUME,
    ]
    assert client.inference_command.call_args.kwargs == {'publish_to_robot': mode}
    assert node.on_recording
    cached = node._cache_ui_task_info.call_args.args[0]
    assert cached.task_instruction == ['pour']
    assert cached.policy_path == '/models/part2'


def test_generated_response_can_report_a_completed_switch():
    node, client, request = fixture()
    response = switch_inference_model(node, request, SendCommand.Response())
    assert response.success
    assert response.loaded_policy_path == '/models/part2'
    assert response.inference_phase == InferenceStatus.INFERENCING


def test_cached_switch_uses_only_strict_switch_command():
    node, client, request = fixture()
    request.command = 29
    response = switch_inference_model(node, request, SimpleNamespace())
    assert response.success
    assert [c.args[0] for c in client.inference_command.call_args_list] == [
        Client.CMD_PAUSE, Client.CMD_SWITCH_PRELOADED, Client.CMD_RESUME,
    ]


def test_missing_preload_keeps_paused_without_fallback_or_resume():
    node, client, request = fixture(fail_at=Client.CMD_SWITCH_PRELOADED)
    request.command = 29
    response = switch_inference_model(node, request, SimpleNamespace())
    assert not response.success
    assert response.loaded_policy_path == '/models/part1'
    assert response.inference_phase == InferenceStatus.PAUSED
    assert [c.args[0] for c in client.inference_command.call_args_list] == [
        Client.CMD_PAUSE, Client.CMD_SWITCH_PRELOADED,
    ]


def test_preload_rejects_live_inference_and_preserves_paused_session():
    node, client, request = fixture()
    request.command = 27
    assert not prepare_inference_model(node, request, SimpleNamespace()).success
    client.inference_command.assert_not_called()
    node._inference_phase = InferenceStatus.PAUSED
    assert prepare_inference_model(node, request, SimpleNamespace()).success
    assert node._loaded_inference_policy_path == '/models/part1'
    client.inference_command.assert_called_once_with(
        Client.CMD_PRELOAD_POLICY, model_path='/models/part2', robot_type='ffw_sh5_rev1',
    )
    node._publish_inference_phase.assert_not_called()


def test_preload_before_start_uses_temporary_client_without_unloading_or_loading_session(monkeypatch):
    from orchestrator.internal.communication import model_switch
    node, client, request = fixture()
    node.container_service_client = None
    node._client_cb_group = None
    node._inference_phase = InferenceStatus.READY
    node._loaded_inference_policy_path = ''
    request.command = 27
    client.connect, client.disconnect = Mock(), Mock()
    factory = Mock(return_value=client)
    for key in ('CMD_PRELOAD_POLICY', 'CMD_CLEAR_PRELOAD'):
        setattr(factory, key, getattr(Client, key))
    monkeypatch.setattr(model_switch, 'ContainerServiceClient', factory)
    assert prepare_inference_model(node, request, SimpleNamespace()).success
    assert node.container_service_client is None
    client.connect.assert_called_once()
    client.disconnect.assert_called_once()
    assert [c.args[0] for c in client.inference_command.call_args_list] == [9]
    assert node._model_switch_operation is None


def test_preload_blocks_competing_start_and_clear_without_resuming_after_stop():
    node, client, request = fixture()
    request.command = 27
    node._inference_phase = InferenceStatus.PAUSED
    entered, release = threading.Event(), threading.Event()

    def prepare(cmd, **_):
        assert cmd == Client.CMD_PRELOAD_POLICY
        entered.set()
        assert release.wait(2)
        return SimpleNamespace(success=True, message='ready')

    client.inference_command.side_effect = prepare
    response = SimpleNamespace()
    worker = threading.Thread(target=lambda: prepare_inference_model(node, request, response))
    worker.start()
    assert entered.wait(1)
    for command in (2, 6, 11, 26, 27, 28, 29):
        assert not intercept_model_switch_command(node, SimpleNamespace(command=command), SimpleNamespace()).success
    assert intercept_model_switch_command(node, SimpleNamespace(command=10), SimpleNamespace()).success
    release.set()
    worker.join(2)
    assert not worker.is_alive() and response.success
    assert [c.args[0] for c in client.inference_command.call_args_list] == [9]


@pytest.mark.parametrize('fail_at,path', [
    (Client.CMD_PAUSE, '/models/part1'),
    (Client.CMD_SWITCH_POLICY, '/models/part1'),
    (Client.CMD_RESUME, '/models/part2'),
])
def test_failures_report_actual_loaded_model_and_do_not_fall_through(fail_at, path):
    node, client, request = fixture(fail_at=fail_at)
    response = switch_inference_model(node, request, SimpleNamespace())
    assert not response.success
    assert response.loaded_policy_path == path
    assert client.inference_command.call_args.args[0] == fail_at
    assert node._model_switch_operation is None
    assert not node._inference_lifecycle_lock.locked()
    if fail_at == Client.CMD_SWITCH_POLICY:
        assert node._model_switch_needs_retry
        blocked = intercept_model_switch_command(
            node, SimpleNamespace(command=SendCommand.Request.RESUME_INFERENCE), SimpleNamespace(),
        )
        assert not blocked.success


def test_stop_during_loading_cancels_resume_and_duplicate_switch_is_rejected():
    node, client, request = fixture()
    loading, release = threading.Event(), threading.Event()

    def command(cmd, **_):
        if cmd == Client.CMD_SWITCH_POLICY:
            loading.set()
            assert release.wait(2)
        return SimpleNamespace(success=True, message='ok')

    client.inference_command.side_effect = command
    response = SimpleNamespace()
    thread = threading.Thread(target=lambda: switch_inference_model(node, request, response))
    thread.start()
    assert loading.wait(1)
    duplicate = intercept_model_switch_command(node, SimpleNamespace(command=26), SimpleNamespace())
    assert not duplicate.success
    stopped = intercept_model_switch_command(
        node, SimpleNamespace(command=SendCommand.Request.STOP_INFERENCE), SimpleNamespace(),
    )
    assert stopped.success
    release.set()
    thread.join(2)
    assert not thread.is_alive()
    assert response.success and response.inference_phase == InferenceStatus.PAUSED
    assert response.loaded_policy_path == '/models/part2'
    assert Client.CMD_RESUME not in [c.args[0] for c in client.inference_command.call_args_list]


def test_stop_during_activation_is_acknowledged_only_after_output_is_paused():
    node, client, request = fixture()
    resuming, release, stopped = threading.Event(), threading.Event(), threading.Event()
    commands = []

    def command(cmd, **_):
        commands.append(cmd)
        if cmd == Client.CMD_RESUME:
            resuming.set()
            assert release.wait(2)
        return SimpleNamespace(success=True, message='ok')

    client.inference_command.side_effect = command
    worker = threading.Thread(target=lambda: switch_inference_model(node, request, SimpleNamespace()))
    worker.start()
    assert resuming.wait(1)

    def stop():
        result = intercept_model_switch_command(
            node, SimpleNamespace(command=SendCommand.Request.STOP_INFERENCE), SimpleNamespace(),
        )
        assert result.success
        stopped.set()

    stopper = threading.Thread(target=stop)
    stopper.start()
    assert not stopped.wait(0.05)
    release.set()
    worker.join(2)
    stopper.join(2)
    assert stopped.is_set()
    assert commands == [Client.CMD_PAUSE, Client.CMD_SWITCH_POLICY, Client.CMD_RESUME, Client.CMD_PAUSE]
    assert node._publish_inference_phase.call_args.args[0] == InferenceStatus.PAUSED


@pytest.mark.parametrize('change', ['robot', 'backend', 'mode', 'path', 'home', 'idle'])
def test_invalid_requests_do_not_touch_current_output(change):
    node, client, request = fixture()
    if change == 'robot': node.robot_type = 'ffw_sg2_rev1'
    if change == 'backend': client._service_prefix = '/lerobot'
    if change == 'mode': request.task_info.inference_mode = 'simulation'
    if change == 'path': request.task_info.policy_path = '/models/part1/'
    if change == 'home': node._inference_cycle_prepared = True
    if change == 'idle': node.on_inference = False
    assert not switch_inference_model(node, request, SimpleNamespace()).success
    client.inference_command.assert_not_called()
