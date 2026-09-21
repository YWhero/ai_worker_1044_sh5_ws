"""No robot or checkpoint required: handoff lifecycle and engine ordering."""

import sys
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from engine_process.protocol import CMD_GET_ACTION, CMD_SWITCH_POLICY, EngineCommandRequest
from engine_process.worker import EngineWorker
from main_runtime.inference_requester import InferenceRequester
from main_runtime.service_handler import (
    CMD_RESUME, CMD_SWITCH_POLICY as SWITCH, ServiceHandler,
)
from main_runtime.session_state import SessionState


def handler_fixture():
    session = SessionState(
        loaded=True, running=True, paused=True,
        robot_type='ffw_sh5_rev1', action_keys=['arm'],
    )
    requester = Mock()
    requester.switch_policy.return_value = SimpleNamespace(
        success=True, message='switched', action_keys=['arm'],
    )
    loop = Mock()
    handler = ServiceHandler(session, requester, loop, SimpleNamespace)
    return handler, session, requester, loop


def request(**overrides):
    return SimpleNamespace(**{
        'command': SWITCH, 'model_path': '/models/part2',
        'robot_type': 'ffw_sh5_rev1', 'task_instruction': '',
        'publish_to_robot': True, **overrides,
    })


def test_switch_clears_output_and_never_starts_or_recalibrates():
    handler, session, requester, loop = handler_fixture()
    calls = Mock()
    calls.attach_mock(loop, 'loop')
    calls.attach_mock(requester, 'requester')
    result = handler.handle(request())
    assert result.success and session.paused
    assert [call[0] for call in calls.mock_calls] == ['loop.pause', 'requester.switch_policy']
    loop.configure.assert_not_called()
    requester.reset_policy_cycle.assert_not_called()
    assert handler.handle(request(command=CMD_RESUME)).success
    loop.preflight_start.assert_called_once_with(True, '', continuation=True)


def test_switch_requires_paused_session_and_same_robot():
    handler, session, requester, loop = handler_fixture()
    session.paused = False
    assert not handler.handle(request()).success
    session.paused = True
    assert not handler.handle(request(robot_type='ffw_sg2_rev1')).success
    assert not handler.handle(request(model_path='')).success
    requester.switch_policy.assert_not_called()
    loop.start.assert_not_called()


def test_failed_or_incompatible_switch_blocks_resume_until_successful_retry():
    for failure in (
        SimpleNamespace(success=False, message='timeout'),
        SimpleNamespace(success=True, message='switched', action_keys=['wrong']),
    ):
        handler, session, requester, loop = handler_fixture()
        requester.switch_policy.return_value = failure
        assert not handler.handle(request()).success
        assert session.paused
        assert not handler.handle(request(command=CMD_RESUME)).success
        loop.start.assert_not_called()
        requester.switch_policy.return_value = SimpleNamespace(
            success=True, message='switched', action_keys=['arm'],
        )
        assert handler.handle(request()).success
        assert handler.handle(request(command=CMD_RESUME)).success


def test_worker_finishes_old_action_before_replacing_policy():
    entered, release, switched = threading.Event(), threading.Event(), threading.Event()

    def get_action(_):
        entered.set()
        assert release.wait(2)
        return {'success': True, 'action_chunk': [0], 'chunk_size': 1, 'action_dim': 1}

    def switch(_):
        switched.set()
        return {'success': True, 'action_keys': ['arm']}

    worker = EngineWorker(SimpleNamespace(get_action_chunk=get_action, switch_policy=switch))
    action_thread = threading.Thread(target=lambda: worker.handle(EngineCommandRequest(CMD_GET_ACTION)))
    switch_thread = threading.Thread(target=lambda: worker.handle(EngineCommandRequest(CMD_SWITCH_POLICY)))
    action_thread.start()
    assert entered.wait(1)
    switch_thread.start()
    assert not switched.wait(0.05)
    release.set()
    action_thread.join(2)
    switch_thread.join(2)
    assert switched.is_set()


def test_unsupported_engine_fails_without_cleanup():
    engine = SimpleNamespace(cleanup=Mock())
    result = EngineWorker(engine).handle(EngineCommandRequest(CMD_SWITCH_POLICY, seq_id=13))
    assert not result.success and result.seq_id == 13
    engine.cleanup.assert_not_called()


def test_requester_preserves_target_and_discards_stale_switch_response():
    client = Mock()
    client.call.return_value = SimpleNamespace(success=True, seq_id=999)
    requester = InferenceRequester(client, load_policy_timeout_s=42)
    result = requester.switch_policy(request())
    assert not result.success and 'stale' in result.message
    forwarded = client.call.call_args.args[0]
    assert forwarded.command == CMD_SWITCH_POLICY
    assert forwarded.model_path == '/models/part2'
    assert forwarded.robot_type == 'ffw_sh5_rev1'
    assert client.call.call_args.kwargs == {'timeout_s': 42}


@pytest.mark.parametrize('paused,loaded', [(True, True), (False, False)])
def test_preload_before_start_or_while_paused_never_starts_robot(paused, loaded):
    from main_runtime.service_handler import CMD_PRELOAD_POLICY
    handler, session, requester, loop = handler_fixture()
    session.loaded, session.running, session.paused = loaded, loaded, paused
    requester.preload_policy.return_value = SimpleNamespace(success=True, message='ready', action_keys=['arm'])
    assert handler.handle(request(command=CMD_PRELOAD_POLICY)).success
    assert session.loaded == loaded and session.paused == paused
    requester.load_policy.assert_not_called()
    loop.configure.assert_not_called()
    loop.start.assert_not_called()


def test_live_inference_rejects_preload_and_release_before_engine_work():
    from main_runtime.service_handler import CMD_PRELOAD_POLICY, CMD_CLEAR_PRELOAD
    handler, session, requester, loop = handler_fixture()
    session.paused = False
    for command in (CMD_PRELOAD_POLICY, CMD_CLEAR_PRELOAD):
        result = handler.handle(request(command=command))
        assert not result.success and 'Pause inference' in result.message
    requester.preload_policy.assert_not_called()
    requester.clear_preload.assert_not_called()
    loop.pause.assert_not_called()


def test_cached_switch_uses_strict_command_and_failed_cache_blocks_resume():
    from main_runtime.service_handler import CMD_SWITCH_PRELOADED
    handler, session, requester, loop = handler_fixture()
    requester.switch_preloaded_policy.return_value = SimpleNamespace(success=False, message='not preloaded')
    assert not handler.handle(request(command=CMD_SWITCH_PRELOADED)).success
    requester.switch_policy.assert_not_called()
    assert not handler.handle(request(command=CMD_RESUME)).success
    loop.start.assert_not_called()


@pytest.mark.parametrize('method,command', [('preload_policy', 5), ('clear_preload', 6), ('switch_preloaded_policy', 7)])
def test_resident_commands_traverse_requester_and_worker_without_load(method, command):
    engine = SimpleNamespace(load_policy=Mock())
    operation = Mock(return_value={'success': True, 'action_keys': ['arm']})
    setattr(engine, method, operation)
    worker = EngineWorker(engine)
    client = Mock()
    client.call.side_effect = lambda req, **_: worker.handle(req)
    result = getattr(InferenceRequester(client), method)(request())
    assert result.success
    forwarded = operation.call_args.args[0]
    assert forwarded.command == command and forwarded.model_path == '/models/part2'
    engine.load_policy.assert_not_called()
