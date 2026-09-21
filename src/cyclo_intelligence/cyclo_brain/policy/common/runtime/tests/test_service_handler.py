#!/usr/bin/env python3

from __future__ import annotations

import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


RUNTIME_ROOT = Path(__file__).resolve().parents[1]
if str(RUNTIME_ROOT) not in sys.path:
    sys.path.insert(0, str(RUNTIME_ROOT))

from main_runtime.service_handler import (  # noqa: E402
    CMD_LOAD,
    CMD_PAUSE,
    CMD_RESET_CYCLE,
    CMD_RESUME,
    CMD_START,
    CMD_STOP,
    ServiceHandler,
)
from main_runtime.session_state import SessionState  # noqa: E402


class FakeRequester:
    def __init__(self):
        self.reset_cycle_count = 0
        self.reset_cycle_response = SimpleNamespace(
            success=True,
            message="cycle reset",
        )

    def load_policy(self, _request):
        return SimpleNamespace(
            success=True,
            message="loaded",
            action_keys=["arm"],
        )

    def unload_policy(self):
        return SimpleNamespace(success=True, message="unloaded")

    def reset_policy_cycle(self):
        self.reset_cycle_count += 1
        return self.reset_cycle_response


class FakeControlLoop:
    def __init__(self) -> None:
        self.configures = []
        self.starts = []
        self.task_instructions = []
        self.preflights = []
        self.preflight_error = None
        self.block_preflight = False
        self.preflight_entered = threading.Event()
        self.preflight_release = threading.Event()
        self.stop_count = 0

    def configure(self, **kwargs) -> None:
        self.configures.append(kwargs)

    def start(self, publish_to_robot=None) -> None:
        self.starts.append(publish_to_robot)

    def preflight_start(
        self,
        publish_to_robot,
        task_instruction=None,
        *,
        continuation=False,
    ) -> None:
        self.preflights.append(
            (publish_to_robot, task_instruction, continuation)
        )
        self.preflight_entered.set()
        if self.block_preflight:
            if not self.preflight_release.wait(timeout=2.0):
                raise RuntimeError("test preflight release timed out")
        if self.preflight_error is not None:
            raise RuntimeError(self.preflight_error)

    def set_task_instruction(self, task_instruction: str) -> None:
        self.task_instructions.append(task_instruction)

    def pause(self) -> None:
        pass

    def stop(self) -> None:
        self.stop_count += 1

    def deconfigure(self) -> None:
        pass


def make_response(success, message="", action_keys=None):
    return SimpleNamespace(
        success=success,
        message=message,
        action_keys=list(action_keys or []),
    )


class ServiceHandlerPublishModeTests(unittest.TestCase):
    def _handler(self):
        session = SessionState()
        loop = FakeControlLoop()
        requester = FakeRequester()
        handler = ServiceHandler(
            session,
            requester,
            loop,
            make_response,
        )
        return handler, session, loop

    def test_load_configures_dry_run_by_default(self) -> None:
        handler, _session, loop = self._handler()

        response = handler.handle(SimpleNamespace(
            command=CMD_LOAD,
            model_path="/models/policy",
            robot_type="ffw",
            task_instruction="pick",
        ))

        self.assertTrue(response.success)
        self.assertEqual(loop.configures[0]["publish_to_robot"], False)
        self.assertEqual(loop.configures[0]["action_request_mode"], "async")

    def test_load_forwards_requested_timing(self) -> None:
        handler, _session, loop = self._handler()
        response = handler.handle(SimpleNamespace(
            command=CMD_LOAD,
            model_path="/models/policy",
            robot_type="ffw",
            task_instruction="pick",
            control_hz=80,
            inference_hz=30,
            chunk_align_window_s=0.6,
        ))
        self.assertTrue(response.success)
        self.assertEqual(loop.configures[0]["control_hz"], 80)
        self.assertEqual(loop.configures[0]["inference_hz"], 30)
        self.assertEqual(loop.configures[0]["chunk_align_window_s"], 0.6)

    def test_load_without_timing_preserves_environment_defaults(self) -> None:
        handler, _session, loop = self._handler()
        response = handler.handle(SimpleNamespace(
            command=CMD_LOAD,
            model_path="/models/policy",
            robot_type="ffw",
            task_instruction="pick",
        ))
        self.assertTrue(response.success)
        for name in ("control_hz", "inference_hz", "chunk_align_window_s"):
            self.assertEqual(loop.configures[0][name], 0)

    def test_load_configures_robot_publish_when_requested(self) -> None:
        handler, _session, loop = self._handler()

        response = handler.handle(SimpleNamespace(
            command=CMD_LOAD,
            model_path="/models/policy",
            robot_type="ffw",
            task_instruction="pick",
            publish_to_robot=True,
        ))

        self.assertTrue(response.success)
        self.assertEqual(loop.configures[0]["publish_to_robot"], True)

    def test_load_configures_action_request_mode(self) -> None:
        handler, _session, loop = self._handler()

        response = handler.handle(SimpleNamespace(
            command=CMD_LOAD,
            model_path="/models/policy",
            robot_type="ffw",
            task_instruction="pick",
            action_request_mode="sync",
        ))

        self.assertTrue(response.success)
        self.assertEqual(loop.configures[0]["action_request_mode"], "sync")

    def test_tactile_load_uses_ordered_async_even_for_stale_async(self) -> None:
        handler, _session, loop = self._handler()
        with tempfile.TemporaryDirectory() as folder:
            Path(folder, "config.json").write_text(
                '{"type": "tactile_act"}',
                encoding="utf-8",
            )
            response = handler.handle(SimpleNamespace(
                command=CMD_LOAD,
                model_path=folder,
                robot_type="ffw",
                task_instruction="pick",
                action_request_mode="async",
            ))

        self.assertTrue(response.success)
        self.assertEqual(
            loop.configures[0]["action_request_mode"],
            "async_ordered",
        )

    def test_tactile_load_overrides_later_ordered_async_ui_value(self) -> None:
        handler, _session, loop = self._handler()
        with tempfile.TemporaryDirectory() as folder:
            Path(folder, "config.json").write_text(
                '{"type": "tactile_act"}',
                encoding="utf-8",
            )
            response = handler.handle(SimpleNamespace(
                command=CMD_LOAD,
                model_path=folder,
                robot_type="ffw",
                task_instruction="pick",
                action_request_mode="async_ordered",
            ))

        self.assertTrue(response.success)
        self.assertEqual(
            loop.configures[0]["action_request_mode"],
            "async_ordered",
        )

    def test_legacy_async_vanilla_load_stays_async(self) -> None:
        handler, _session, loop = self._handler()
        with tempfile.TemporaryDirectory() as folder:
            Path(folder, "config.json").write_text(
                '{"type": "act"}',
                encoding="utf-8",
            )
            response = handler.handle(SimpleNamespace(
                command=CMD_LOAD,
                model_path=folder,
                robot_type="ffw",
                task_instruction="pick",
                action_request_mode="async",
            ))

        self.assertTrue(response.success)
        self.assertEqual(loop.configures[0]["action_request_mode"], "async")

    def test_backend_env_can_force_runtime_profile(self) -> None:
        handler, _session, loop = self._handler()
        with patch.dict(
            "os.environ",
            {
                "POLICY_ACTION_REQUEST_MODE_OVERRIDE": "async",
                "POLICY_REFILL_MARGIN_S_OVERRIDE": "0.55",
                "POLICY_SOURCE_CHUNK_LIMIT": "20",
            },
        ):
            response = handler.handle(SimpleNamespace(
                command=CMD_LOAD,
                model_path="/models/dedicated-policy",
                robot_type="ffw_sh5_rev1",
                task_instruction="",
                action_request_mode="sync",
            ))

        self.assertTrue(response.success)
        self.assertEqual(loop.configures[0]["action_request_mode"], "async")
        self.assertEqual(loop.configures[0]["refill_margin_s"], 0.55)
        self.assertEqual(loop.configures[0]["source_chunk_limit"], 20)

    def test_start_applies_publish_mode(self) -> None:
        handler, _session, loop = self._handler()
        handler.handle(SimpleNamespace(
            command=CMD_LOAD,
            model_path="/models/policy",
            robot_type="ffw",
            task_instruction="pick",
            publish_to_robot=False,
        ))

        response = handler.handle(SimpleNamespace(
            command=CMD_START,
            publish_to_robot=True,
        ))

        self.assertTrue(response.success)
        self.assertEqual(loop.starts[-1], True)
        self.assertEqual(loop.preflights[-1], (True, "pick", False))

    def test_resume_applies_publish_mode(self) -> None:
        handler, _session, loop = self._handler()
        handler.handle(SimpleNamespace(
            command=CMD_LOAD,
            model_path="/models/policy",
            robot_type="ffw",
            task_instruction="pick",
        ))
        handler.handle(SimpleNamespace(command=CMD_START, publish_to_robot=False))

        response = handler.handle(SimpleNamespace(
            command=CMD_RESUME,
            task_instruction="place",
            publish_to_robot=True,
        ))

        self.assertTrue(response.success)
        self.assertEqual(loop.starts[-1], True)
        self.assertEqual(loop.task_instructions[-1], "place")
        self.assertEqual(loop.preflights[-1], (True, "place", True))

    def test_reset_cycle_resets_episode_state_and_uses_fresh_preflight(self) -> None:
        session = SessionState()
        loop = FakeControlLoop()
        requester = FakeRequester()
        handler = ServiceHandler(session, requester, loop, make_response)
        handler.handle(SimpleNamespace(
            command=CMD_LOAD,
            model_path="/models/policy",
            robot_type="ffw",
            task_instruction="pick",
        ))
        handler.handle(SimpleNamespace(command=CMD_START, publish_to_robot=False))
        handler.handle(SimpleNamespace(command=CMD_PAUSE))

        response = handler.handle(SimpleNamespace(
            command=CMD_RESET_CYCLE,
            task_instruction="",
            publish_to_robot=True,
        ))

        self.assertTrue(response.success)
        self.assertEqual(requester.reset_cycle_count, 1)
        self.assertEqual(loop.preflights[-1], (True, "pick", False))
        self.assertEqual(loop.starts[-1], True)
        self.assertTrue(session.running)
        self.assertFalse(session.paused)

    def test_reset_cycle_requires_paused_session(self) -> None:
        session = SessionState()
        loop = FakeControlLoop()
        requester = FakeRequester()
        handler = ServiceHandler(session, requester, loop, make_response)
        handler.handle(SimpleNamespace(
            command=CMD_LOAD,
            model_path="/models/policy",
            robot_type="ffw",
            task_instruction="pick",
        ))
        handler.handle(SimpleNamespace(command=CMD_START, publish_to_robot=False))

        response = handler.handle(SimpleNamespace(
            command=CMD_RESET_CYCLE,
            task_instruction="",
            publish_to_robot=True,
        ))

        self.assertFalse(response.success)
        self.assertIn("requires PAUSED", response.message)
        self.assertEqual(requester.reset_cycle_count, 0)

    def test_reset_cycle_engine_failure_stays_paused(self) -> None:
        session = SessionState()
        loop = FakeControlLoop()
        requester = FakeRequester()
        requester.reset_cycle_response = SimpleNamespace(
            success=False,
            message="tactile calibration failed",
        )
        handler = ServiceHandler(session, requester, loop, make_response)
        handler.handle(SimpleNamespace(
            command=CMD_LOAD,
            model_path="/models/policy",
            robot_type="ffw",
            task_instruction="pick",
        ))
        handler.handle(SimpleNamespace(command=CMD_START, publish_to_robot=False))
        handler.handle(SimpleNamespace(command=CMD_PAUSE))
        starts_before = list(loop.starts)

        response = handler.handle(SimpleNamespace(
            command=CMD_RESET_CYCLE,
            task_instruction="",
            publish_to_robot=True,
        ))

        self.assertFalse(response.success)
        self.assertIn("tactile calibration failed", response.message)
        self.assertEqual(loop.starts, starts_before)
        self.assertTrue(session.running)
        self.assertTrue(session.paused)

    def test_real_start_preflight_failure_is_returned_in_service_response(self) -> None:
        handler, session, loop = self._handler()
        handler.handle(SimpleNamespace(
            command=CMD_LOAD,
            model_path="/models/policy",
            robot_type="ffw",
            task_instruction="pick",
        ))
        loop.preflight_error = "Real action preflight blocked: arm jump"

        response = handler.handle(SimpleNamespace(
            command=CMD_START,
            publish_to_robot=True,
        ))

        self.assertFalse(response.success)
        self.assertIn("arm jump", response.message)
        self.assertFalse(session.running)
        self.assertEqual(loop.starts, [])

    def test_stop_waits_for_inflight_real_start_preflight(self) -> None:
        handler, session, loop = self._handler()
        load_response = handler.handle(SimpleNamespace(
            command=CMD_LOAD,
            model_path="/models/policy",
            robot_type="ffw",
            task_instruction="pick",
        ))
        self.assertTrue(load_response.success)
        loop.block_preflight = True
        responses = {}

        start_thread = threading.Thread(
            target=lambda: responses.setdefault(
                "start",
                handler.handle(SimpleNamespace(
                    command=CMD_START,
                    publish_to_robot=True,
                )),
            )
        )
        start_thread.start()
        self.assertTrue(loop.preflight_entered.wait(timeout=1.0))

        stop_thread = threading.Thread(
            target=lambda: responses.setdefault(
                "stop",
                handler.handle(SimpleNamespace(command=CMD_STOP)),
            )
        )
        stop_thread.start()
        time.sleep(0.05)

        self.assertTrue(stop_thread.is_alive())
        self.assertEqual(loop.stop_count, 0)

        loop.preflight_release.set()
        start_thread.join(timeout=1.0)
        stop_thread.join(timeout=1.0)

        self.assertFalse(start_thread.is_alive())
        self.assertFalse(stop_thread.is_alive())
        self.assertTrue(responses["start"].success)
        self.assertTrue(responses["stop"].success)
        self.assertEqual(loop.stop_count, 1)
        self.assertFalse(session.running)

    def test_real_start_deadline_expiry_after_preflight_does_not_start(self) -> None:
        handler, session, loop = self._handler()
        handler.handle(SimpleNamespace(
            command=CMD_LOAD,
            model_path="/models/policy",
            robot_type="ffw",
            task_instruction="pick",
        ))

        with patch(
            "main_runtime.service_handler.time.monotonic",
            side_effect=[100.0, 101.0, 116.0],
        ):
            response = handler.handle(SimpleNamespace(
                command=CMD_START,
                publish_to_robot=True,
            ))

        self.assertFalse(response.success)
        self.assertIn("deadline expired", response.message)
        self.assertIn("publishing was not activated", response.message)
        self.assertEqual(loop.preflights[-1], (True, "pick", False))
        self.assertEqual(loop.starts, [])
        self.assertFalse(session.running)

    def test_real_resume_deadline_expiry_after_preflight_does_not_resume(self) -> None:
        handler, session, loop = self._handler()
        handler.handle(SimpleNamespace(
            command=CMD_LOAD,
            model_path="/models/policy",
            robot_type="ffw",
            task_instruction="pick",
        ))
        handler.handle(SimpleNamespace(command=CMD_START, publish_to_robot=False))
        handler.handle(SimpleNamespace(command=CMD_PAUSE))
        starts_before = list(loop.starts)

        with patch(
            "main_runtime.service_handler.time.monotonic",
            side_effect=[200.0, 201.0, 216.0],
        ):
            response = handler.handle(SimpleNamespace(
                command=CMD_RESUME,
                task_instruction="place",
                publish_to_robot=True,
            ))

        self.assertFalse(response.success)
        self.assertIn("deadline expired", response.message)
        self.assertEqual(loop.preflights[-1], (True, "place", True))
        self.assertEqual(loop.starts, starts_before)
        self.assertTrue(session.running)
        self.assertTrue(session.paused)
        self.assertEqual(session.task_instruction, "pick")

    def test_queued_real_start_expiry_skips_preflight(self) -> None:
        handler, session, loop = self._handler()
        handler.handle(SimpleNamespace(
            command=CMD_LOAD,
            model_path="/models/policy",
            robot_type="ffw",
            task_instruction="pick",
        ))

        with patch(
            "main_runtime.service_handler.time.monotonic",
            side_effect=[300.0, 316.0],
        ):
            response = handler.handle(SimpleNamespace(
                command=CMD_START,
                publish_to_robot=True,
            ))

        self.assertFalse(response.success)
        self.assertEqual(loop.preflights, [])
        self.assertEqual(loop.starts, [])
        self.assertFalse(session.running)

    def test_deadline_does_not_change_simulation_start(self) -> None:
        handler, session, loop = self._handler()
        handler.handle(SimpleNamespace(
            command=CMD_LOAD,
            model_path="/models/policy",
            robot_type="ffw",
            task_instruction="pick",
        ))

        with patch(
            "main_runtime.service_handler.time.monotonic",
            return_value=400.0,
        ):
            response = handler.handle(SimpleNamespace(
                command=CMD_START,
                publish_to_robot=False,
            ))

        self.assertTrue(response.success)
        self.assertEqual(loop.starts[-1], False)
        self.assertTrue(session.running)

    def test_real_start_deadline_must_be_positive(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            ServiceHandler(
                SessionState(),
                FakeRequester(),
                FakeControlLoop(),
                make_response,
                real_start_deadline_s=0.0,
            )


if __name__ == "__main__":
    unittest.main()
