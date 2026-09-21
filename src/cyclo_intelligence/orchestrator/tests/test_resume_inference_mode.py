#!/usr/bin/env python3

from __future__ import annotations

import json
import threading
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import Mock, patch

from interfaces.msg import InferenceStatus
from interfaces.srv import SendCommand

from orchestrator.internal.communication.container_service_client import (
    ContainerServiceClient,
)
from orchestrator.orchestrator_node import (
    OrchestratorNode,
    PREPARE_NEXT_CYCLE_COMMAND,
)


class _FakeInferenceClient:
    def __init__(self, *, success: bool = True, message: str = "ok") -> None:
        self.calls = []
        self.result = SimpleNamespace(success=success, message=message)

    def inference_command(self, command, **kwargs):
        self.calls.append((command, kwargs))
        return self.result


def _make_node(*, loaded_publish_to_robot: bool, client):
    # The callback branch under test needs no ROS entities. Allocating without
    # Node.__init__ keeps this a side-effect-free unit test.
    node = object.__new__(OrchestratorNode)
    node._state_lock = threading.RLock()
    node._inference_lifecycle_lock = threading.RLock()
    node.on_recording = False
    node.on_inference = True
    node.container_service_client = client
    node._loaded_inference_publish_to_robot = loaded_publish_to_robot
    node._loaded_inference_initial_pose_sync_duration_s = 5.0
    node._initial_pose_sync_status_timer = None
    node._initial_pose_sync_status_generation = 0
    node._initial_pose_sync_hold_pending = False
    node._inference_cycle_prepared = False
    node._publish_inference_phase = Mock()
    node.robot_type = 'ffw_sh5_rev1'
    node.communicator = SimpleNamespace(
        initial_pose_return_ready=Mock(return_value=(True, 'Initial pose reached')),
        publish_initial_pose_return=Mock(
            return_value=(True, 'Initial-pose return requested')
        )
    )
    return node


def _resume(node, inference_mode: str = "", tags=None):
    request = SimpleNamespace(
        command=SendCommand.Request.RESUME_INFERENCE,
        task_info=SimpleNamespace(
            inference_mode=inference_mode,
            task_instruction=[],
            tags=list(tags or []),
        ),
    )
    response = SimpleNamespace(success=None, message="")
    return node.user_interaction_callback(request, response)


def _start(node, inference_mode: str = "", tags=None):
    request = SimpleNamespace(
        command=SendCommand.Request.START_INFERENCE,
        task_info=SimpleNamespace(
            inference_mode=inference_mode,
            task_instruction=[],
            tags=list(tags or []),
        ),
    )
    response = SimpleNamespace(success=None, message="")
    return node.user_interaction_callback(request, response)


def _prepare_next_cycle(node):
    request = SimpleNamespace(
        command=PREPARE_NEXT_CYCLE_COMMAND,
        task_info=SimpleNamespace(),
    )
    response = SimpleNamespace(success=None, message="")
    return node.user_interaction_callback(request, response)


class ResumeInferenceModeTests(unittest.TestCase):
    def test_resume_sync_alias_waits_for_timer_before_inferencing_phase(self):
        for message in ("syncing", "initial pose sync started"):
            with self.subTest(message=message):
                client = _FakeInferenceClient(message=message)
                node = _make_node(loaded_publish_to_robot=True, client=client)
                with patch("orchestrator.orchestrator_node.threading.Timer") as timer:
                    response = _resume(node)
                    self.assertTrue(response.success)
                    node._publish_inference_phase.assert_called_once_with(InferenceStatus.SYNCING)
                    self.assertEqual(timer.call_args.args[0], 5.0)
                    timer.return_value.start.assert_called_once_with()
                    # Drive the actual timer completion callback, without a sleep.
                    timer.call_args.args[1]()
                    self.assertEqual(node._publish_inference_phase.call_args.args,
                                     (InferenceStatus.INFERENCING,))
                    self.assertIsNone(node._initial_pose_sync_status_timer)

    def test_vitacformer_metadata_overrides_legacy_lerobot_service_type(self):
        node = object.__new__(OrchestratorNode)
        logger = Mock()
        node.get_logger = Mock(return_value=logger)

        with TemporaryDirectory() as tmp_dir:
            model_path = Path(tmp_dir) / 'vitacformer_model'
            model_path.mkdir()
            (model_path / 'train_config.json').write_text(json.dumps({
                'architecture': 'ViTacFormer cross-attention',
            }))
            task_info = SimpleNamespace(
                policy_path=str(model_path),
                service_type='lerobot',
            )

            self.assertEqual(
                node._determine_service_prefix(task_info),
                '/vitacformer',
            )

        logger.warning.assert_called_once()

    def test_explicit_service_type_still_wins_for_other_models(self):
        node = object.__new__(OrchestratorNode)
        node.get_logger = Mock(return_value=Mock())
        task_info = SimpleNamespace(
            policy_path='/does/not/exist',
            service_type='lerobot',
        )

        self.assertEqual(
            node._determine_service_prefix(task_info),
            '/lerobot',
        )

    def test_start_rejects_invalid_mode_before_cache_or_backend(self):
        client = _FakeInferenceClient()
        node = _make_node(loaded_publish_to_robot=False, client=client)
        node._cache_ui_task_info = Mock()

        response = _start(node, "simulaton")

        self.assertFalse(response.success)
        self.assertIn("Invalid inference_mode", response.message)
        self.assertEqual(client.calls, [])
        node._cache_ui_task_info.assert_not_called()

    def test_start_rejects_conflicting_field_and_tag_before_backend(self):
        client = _FakeInferenceClient()
        node = _make_node(loaded_publish_to_robot=False, client=client)
        node._cache_ui_task_info = Mock()

        response = _start(
            node,
            "robot",
            tags=["inference_mode:simulation"],
        )

        self.assertFalse(response.success)
        self.assertIn("Conflicting inference modes", response.message)
        self.assertEqual(client.calls, [])
        node._cache_ui_task_info.assert_not_called()

    def test_simulation_to_robot_uses_request_mode_and_updates_cache(self):
        client = _FakeInferenceClient()
        node = _make_node(loaded_publish_to_robot=False, client=client)

        response = _resume(node, "robot")

        self.assertTrue(response.success)
        self.assertEqual(client.calls, [(
            ContainerServiceClient.CMD_RESUME,
            {"task_instruction": "", "publish_to_robot": True},
        )])
        self.assertTrue(node._loaded_inference_publish_to_robot)
        node._publish_inference_phase.assert_called_once_with(
            InferenceStatus.INFERENCING
        )

    def test_robot_to_simulation_uses_request_mode_and_updates_cache(self):
        client = _FakeInferenceClient()
        node = _make_node(loaded_publish_to_robot=True, client=client)

        response = _resume(node, "simulation")

        self.assertTrue(response.success)
        self.assertFalse(client.calls[0][1]["publish_to_robot"])
        self.assertFalse(node._loaded_inference_publish_to_robot)

    def test_payloadless_legacy_resume_reuses_cached_mode(self):
        client = _FakeInferenceClient()
        node = _make_node(loaded_publish_to_robot=True, client=client)

        response = _resume(node, "")

        self.assertTrue(response.success)
        self.assertTrue(client.calls[0][1]["publish_to_robot"])
        self.assertTrue(node._loaded_inference_publish_to_robot)

    def test_prepared_cycle_uses_fresh_reset_and_clears_flag(self):
        client = _FakeInferenceClient()
        node = _make_node(loaded_publish_to_robot=True, client=client)
        node._inference_cycle_prepared = True

        response = _resume(node, "robot")

        self.assertTrue(response.success)
        self.assertEqual(client.calls, [(
            ContainerServiceClient.CMD_RESET_CYCLE,
            {"task_instruction": "", "publish_to_robot": True},
        )])
        self.assertFalse(node._inference_cycle_prepared)

    def test_tag_only_simulation_overrides_cached_robot_mode(self):
        client = _FakeInferenceClient()
        node = _make_node(loaded_publish_to_robot=True, client=client)

        response = _resume(node, tags=["inference_mode:simulation"])

        self.assertTrue(response.success)
        self.assertFalse(client.calls[0][1]["publish_to_robot"])
        self.assertFalse(node._loaded_inference_publish_to_robot)

    def test_invalid_explicit_mode_is_rejected_before_backend_call(self):
        client = _FakeInferenceClient()
        node = _make_node(loaded_publish_to_robot=True, client=client)

        response = _resume(node, "simulaton")

        self.assertFalse(response.success)
        self.assertIn("Invalid inference_mode", response.message)
        self.assertEqual(client.calls, [])
        self.assertTrue(node._loaded_inference_publish_to_robot)
        node._publish_inference_phase.assert_not_called()

    def test_conflicting_field_and_tag_is_rejected_before_backend_call(self):
        client = _FakeInferenceClient()
        node = _make_node(loaded_publish_to_robot=False, client=client)

        response = _resume(
            node,
            "robot",
            tags=["inference_mode:simulation"],
        )

        self.assertFalse(response.success)
        self.assertIn("Conflicting inference modes", response.message)
        self.assertEqual(client.calls, [])
        self.assertFalse(node._loaded_inference_publish_to_robot)
        node._publish_inference_phase.assert_not_called()

    def test_backend_failure_propagates_and_does_not_update_cache(self):
        client = _FakeInferenceClient(success=False, message="resume rejected")
        node = _make_node(loaded_publish_to_robot=False, client=client)

        response = _resume(node, "robot")

        self.assertFalse(response.success)
        self.assertEqual(response.message, "resume rejected")
        self.assertFalse(node._loaded_inference_publish_to_robot)
        node._publish_inference_phase.assert_not_called()

    def test_prepare_next_cycle_pauses_before_requesting_initial_pose(self):
        client = _FakeInferenceClient()
        node = _make_node(loaded_publish_to_robot=True, client=client)

        response = _prepare_next_cycle(node)

        self.assertTrue(response.success)
        self.assertEqual(client.calls, [(ContainerServiceClient.CMD_PAUSE, {})])
        node.communicator.publish_initial_pose_return.assert_called_once_with()
        self.assertTrue(node._inference_cycle_prepared)
        node._publish_inference_phase.assert_called_once_with(
            InferenceStatus.PAUSED
        )

    def test_prepare_next_cycle_does_not_move_if_pause_fails(self):
        client = _FakeInferenceClient(success=False, message="pause failed")
        node = _make_node(loaded_publish_to_robot=True, client=client)

        response = _prepare_next_cycle(node)

        self.assertFalse(response.success)
        self.assertEqual(response.message, "pause failed")
        node.communicator.publish_initial_pose_return.assert_not_called()
        self.assertFalse(node._inference_cycle_prepared)

    def test_prepare_next_cycle_reports_paused_if_pose_request_fails(self):
        client = _FakeInferenceClient()
        node = _make_node(loaded_publish_to_robot=True, client=client)
        node.communicator.publish_initial_pose_return.return_value = (
            False,
            "pose trigger unavailable",
        )

        response = _prepare_next_cycle(node)

        self.assertFalse(response.success)
        self.assertEqual(response.message, "pose trigger unavailable")
        node._publish_inference_phase.assert_called_once_with(
            InferenceStatus.PAUSED
        )
        self.assertTrue(node._inference_cycle_prepared)

    def test_home_then_stop_then_start_still_resets_the_cycle(self):
        client = _FakeInferenceClient()
        node = _make_node(loaded_publish_to_robot=True, client=client)
        node._set_session_active = Mock()
        self.assertTrue(_prepare_next_cycle(node).success)
        request = SimpleNamespace(command=SendCommand.Request.STOP_INFERENCE)
        response = node.user_interaction_callback(request, SimpleNamespace())
        self.assertTrue(response.success)
        self.assertTrue(node._inference_cycle_prepared)
        self.assertTrue(_resume(node).success)
        self.assertEqual(client.calls[-1][0], ContainerServiceClient.CMD_RESET_CYCLE)
        self.assertFalse(node._inference_cycle_prepared)

    def test_start_and_resume_are_blocked_until_home_completes(self):
        for command in (SendCommand.Request.START_INFERENCE, SendCommand.Request.RESUME_INFERENCE):
            client = _FakeInferenceClient()
            node = _make_node(loaded_publish_to_robot=True, client=client)
            node._inference_cycle_prepared = True
            node.communicator.initial_pose_return_ready.return_value = (False, 'Cycle Home is still running')
            response = node.user_interaction_callback(
                SimpleNamespace(command=command), SimpleNamespace(),
            )
            self.assertFalse(response.success)
            self.assertIn('still running', response.message)
            self.assertEqual(client.calls, [])
            self.assertTrue(node._inference_cycle_prepared)

    def test_failed_reset_cannot_fall_back_to_continuation(self):
        client = _FakeInferenceClient(success=False, message='reset failed')
        node = _make_node(loaded_publish_to_robot=True, client=client)
        node._inference_cycle_prepared = True
        for _ in range(2):
            self.assertFalse(_resume(node).success)
            self.assertTrue(node._inference_cycle_prepared)
        self.assertEqual([c[0] for c in client.calls], [ContainerServiceClient.CMD_RESET_CYCLE] * 2)


if __name__ == "__main__":
    unittest.main()
