#!/usr/bin/env python3

from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np


RUNTIME_ROOT = Path(__file__).resolve().parents[1]
if str(RUNTIME_ROOT) not in sys.path:
    sys.path.insert(0, str(RUNTIME_ROOT))

robot_client_stub = types.ModuleType("robot_client")
robot_client_stub.RobotClient = object
sys.modules.setdefault("robot_client", robot_client_stub)

from main_runtime.control_loop import (  # noqa: E402
    ACTION_REQUEST_MODE_ASYNC_ORDERED,
    ACTION_REQUEST_MODE_SYNC_STEP,
    ControlLoop,
    normalize_action_request_mode,
)


class FakeProcessor:
    output_hz = 100.0

    def __init__(self, actions=None, buffer_size=100) -> None:
        self._actions = list(actions or [])
        self.buffer_size = buffer_size
        self.clear_count = 0
        self.pushed_chunks = []
        self.scheduled_delays = []
        self.align_flags = []
        self.deferred_actions = []

    def pop_action(self):
        if self._actions:
            return self._actions.pop(0)
        return None

    def clear(self) -> None:
        self.clear_count += 1
        self._actions.clear()
        self.buffer_size = 0

    def push_actions(self, chunk, scheduled_start_delay_s=None, align=True):
        data = np.asarray(chunk, dtype=np.float64)
        self.pushed_chunks.append(data.copy())
        self.scheduled_delays.append(scheduled_start_delay_s)
        self.align_flags.append(bool(align))
        self.buffer_size += len(data)
        return len(data)

    def defer_action(self, action, *, published_action=None) -> None:
        desired = np.asarray(action, dtype=np.float64).copy()
        published = (
            None
            if published_action is None
            else np.asarray(published_action, dtype=np.float64).copy()
        )
        self._actions.insert(0, desired)
        self.deferred_actions.append((desired, published))


class FakeRobot:
    def __init__(self) -> None:
        self.commands = []
        self.previews = []
        self.idles = []
        self.action_keys = ["arm"]
        self.safety_calls = []
        self.safety_error = None
        self.contract_calls = []
        self.contract_error = None
        self.safety_outputs = []

    def action_dimension(self, _action_keys) -> int:
        return 2

    def apply_real_action_safety(self, chunk, action_keys, **kwargs):
        self.safety_calls.append((np.asarray(chunk).copy(), list(action_keys), kwargs))
        if self.safety_error is not None:
            raise ValueError(self.safety_error)
        if self.safety_outputs:
            return np.asarray(
                self.safety_outputs.pop(0),
                dtype=np.float64,
            ).copy()
        return np.asarray(chunk, dtype=np.float64).copy()

    def validate_real_action_contract(self, action_keys) -> None:
        self.contract_calls.append(list(action_keys))
        if self.contract_error is not None:
            raise ValueError(self.contract_error)
        if list(action_keys) != ["arm"]:
            raise ValueError("real action key contract mismatch")

    def publish_action(self, action, action_keys) -> None:
        self.commands.append((np.asarray(action).copy(), list(action_keys)))

    def publish_action_preview(self, action, action_keys) -> None:
        self.previews.append((np.asarray(action).copy(), list(action_keys)))

    def publish_idle_action(self, action_keys) -> None:
        self.idles.append(list(action_keys))

    def close(self) -> None:
        pass


class FakeRequester:
    def __init__(self, response) -> None:
        self.response = response
        self.calls = []

    def get_action(self, task_instruction):
        self.calls.append(task_instruction)
        return self.response


class RaisingRequester:
    def __init__(self, error: Exception) -> None:
        self.error = error
        self.calls = []

    def get_action(self, task_instruction):
        self.calls.append(task_instruction)
        raise self.error


class ControlLoopTimingTests(unittest.TestCase):
    def test_requested_timing_reaches_processor_and_ensemble(self):
        loop = ControlLoop(
            requester=Mock(), inference_hz=30, control_hz=100,
            temporal_ensemble_coeff=0.01,
        )
        with patch("main_runtime.control_loop.RobotClient", return_value=FakeRobot()), \
                patch("main_runtime.control_loop.ActionChunkProcessor") as processor:
            loop.configure(
                "ffw_sh5_rev1", action_keys=["arm"],
                inference_hz=20, control_hz=80, chunk_align_window_s=0.6,
                action_request_mode="async_ordered", source_chunk_limit=4,
            )
            settings = processor.call_args.kwargs
            self.assertEqual(settings["inference_hz"], 20)
            self.assertEqual(settings["control_hz"], 80)
            self.assertEqual(settings["chunk_align_window_s"], 0.6)
            self.assertEqual(settings["source_chunk_limit"], 4)
            self.assertTrue(settings["sequential"])
            self.assertEqual(loop._control_hz, 80)
            self.assertEqual(loop._temporal_ensemble.source_hz, 20)
            self.assertEqual(loop._temporal_ensemble.coefficient, 0.01)
            loop._temporal_ensemble.update(np.zeros((4, 2)), 1.0)
            loop.configure("ffw_sh5_rev1", action_keys=["arm"])
            self.assertEqual(loop._temporal_ensemble.source_hz, 30)
            self.assertEqual(loop._temporal_ensemble._plans, [])
            self.assertEqual(processor.call_args.kwargs["control_hz"], 100)
            self.assertEqual(processor.call_args.kwargs["chunk_align_window_s"], 0.3)

    def test_zero_missing_and_invalid_timing_use_environment_defaults(self):
        loop = ControlLoop(requester=Mock(), inference_hz=30, control_hz=100)
        with patch("main_runtime.control_loop.RobotClient", return_value=FakeRobot()), \
                patch("main_runtime.control_loop.ActionChunkProcessor") as processor:
            for value in (None, 0, -1, float("nan"), float("inf"), "invalid"):
                with self.subTest(value=value):
                    loop.configure(
                        "ffw_sh5_rev1", action_keys=["arm"],
                        inference_hz=value, control_hz=value, chunk_align_window_s=value,
                    )
                    settings = processor.call_args.kwargs
                    self.assertEqual(settings["inference_hz"], 30)
                    self.assertEqual(settings["control_hz"], 100)
                    self.assertEqual(settings["chunk_align_window_s"], 0.3)


class ControlLoopSafetyTests(unittest.TestCase):
    def _simulation_hand_sync_loop(self):
        response = SimpleNamespace(success=True, message="ok", chunk_size=1,
                                   action_dim=2, action_list=[-1.5, 0.0])
        robot = FakeRobot()
        robot._robot_type = "ffw_sh5_rev1"
        robot.validate_real_action_contract = Mock()
        robot.publish_initial_pose_sync = Mock()
        loop = ControlLoop(
            requester=FakeRequester(response), real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
            real_first_action_max_delta_by_key={"hand_right": 0.10},
            real_source_step_max_delta_rad=0.03, real_state_max_age_s=0.5,
            real_tracking_max_delta_by_key={"hand_right": 0.70},
            real_warm_start_enabled=True,
            real_warm_start_max_total_delta_by_key={"hand_right": 0.70},
            real_resume_warm_start_max_total_delta_by_key={"hand_right": 0.70},
            simulation_initial_pose_sync_hand_max_delta_by_key={"hand_right": 1.55},
        )
        loop._robot = robot
        loop._processor = FakeProcessor(buffer_size=0)
        loop._action_keys = ["hand_right"]
        loop._initial_pose_sync_enabled = True
        return loop, robot

    def test_simulation_hand_envelope_applies_only_to_initial_sync(self):
        loop, robot = self._simulation_hand_sync_loop()
        self.assertTrue(loop.start(publish_to_robot=True, continuation=True))
        limits = robot.safety_calls[-1][2]
        self.assertEqual(limits["warm_start_max_total_delta_by_key"], {"hand_right": 1.55})
        self.assertEqual(limits["source_step_max_delta_rad"], 0.03)
        self.assertEqual(limits["first_action_max_delta_by_key"], {"hand_right": 0.10})
        self.assertEqual(limits["state_max_age_s"], 0.5)
        robot.publish_initial_pose_sync.assert_called_once()
        loop.preflight_start(True, continuation=True)
        self.assertEqual(robot.safety_calls[-1][2]["warm_start_max_total_delta_by_key"],
                         {"hand_right": 0.70})

    def test_simulation_hand_sync_rejects_fast_duration_other_robot_and_joint_errors(self):
        for failure in ("duration", "robot", "joint_limits"):
            with self.subTest(failure=failure):
                loop, robot = self._simulation_hand_sync_loop()
                if failure == "duration":
                    loop._initial_pose_sync_duration_s = 1.0
                elif failure == "robot":
                    robot._robot_type = "ffw_sg2_rev1"
                else:
                    robot.safety_error = "target violates official joint limits"
                with self.assertRaises(ValueError):
                    loop.start(publish_to_robot=True)
                robot.publish_initial_pose_sync.assert_not_called()

    def test_simulation_hand_sync_configuration_cannot_change_arm_or_disable_guards(self):
        for limits in ({"arm_right": 1.55}, {"hand_right": 1.56},
                       {"hand_right": float("nan")}, {"hand_right": None}):
            with self.subTest(limits=limits):
                with self.assertRaises(ValueError):
                    ControlLoop(requester=Mock(), real_action_safety_enabled=True,
                                real_warm_start_enabled=True,
                                real_warm_start_max_total_delta_by_key={"hand_right": 0.7},
                                simulation_initial_pose_sync_hand_max_delta_by_key=limits)
        with self.assertRaisesRegex(ValueError, "requires action safety"):
            ControlLoop(requester=Mock(),
                        simulation_initial_pose_sync_hand_max_delta_by_key={"hand_right": 1.55})

    def test_initial_sync_checks_original_safety_before_publishing(self):
        response = SimpleNamespace(success=True, message="ok", chunk_size=1,
                                   action_dim=2, action_list=[0.1, 0.2])
        robot = FakeRobot()
        robot.publish_initial_pose_sync = Mock()
        loop = ControlLoop(
            requester=FakeRequester(response),
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
            real_state_max_age_s=0.5,
            real_warm_start_enabled=True,
            real_warm_start_max_total_delta_by_key={"arm": 0.10},
            real_resume_warm_start_max_total_delta_by_key={"arm": 0.08},
        )
        loop._robot = robot
        loop._processor = FakeProcessor(buffer_size=0)
        loop._action_keys = ["arm"]
        loop._initial_pose_sync_enabled = True
        robot.safety_error = "current state is stale"

        with self.assertRaisesRegex(ValueError, "current state is stale"):
            loop.start(publish_to_robot=True)

        robot.publish_initial_pose_sync.assert_not_called()
        self.assertFalse(loop._running)
        robot.safety_error = None
        self.assertTrue(loop.start(publish_to_robot=True, continuation=True))
        robot.publish_initial_pose_sync.assert_called_once()
        kwargs = robot.safety_calls[-1][2]
        self.assertEqual(kwargs["first_action_max_delta_rad"], 0.03)
        self.assertEqual(kwargs["state_max_age_s"], 0.5)
        self.assertEqual(kwargs["warm_start_max_total_delta_by_key"], {"arm": 0.08})

    def test_publish_trace_records_follower_group_and_tracking_decision(self):
        loop = ControlLoop(requester=object())
        rows = []
        loop._diagnostic_trace = SimpleNamespace(
            enabled=True, record=lambda event, **fields: rows.append((event, fields)))
        robot = SimpleNamespace(get_joint_position_snapshot=lambda group: (
            np.array([0.5, 0.0]) if group == 'follower_arm' else np.array([]), 123.0))
        loop._trace_publish(robot, ['arm'], np.array([0.565, 0.0]),
                            np.array([0.559, 0.0]), True, 42)
        self.assertEqual(rows[0][0], 'publish')
        self.assertEqual(rows[0][1]['measured']['arm']['positions'], [0.5, 0.0])
        self.assertTrue(rows[0][1]['deferred'])
        self.assertEqual(rows[0][1]['buffer_size'], 42)

    def _make_loop(self, processor: FakeProcessor, robot: FakeRobot) -> ControlLoop:
        loop = ControlLoop(requester=object())
        loop._running = True
        loop._robot = robot
        loop._processor = processor
        loop._action_keys = ["arm"]
        return loop

    def test_real_clamp_mode_rejects_zero_tolerance_at_startup(self) -> None:
        with self.assertRaisesRegex(ValueError, "positive in clamp mode"):
            ControlLoop(
                requester=object(),
                real_action_safety_enabled=True,
                real_joint_limit_mode="clamp",
                real_joint_limit_tolerance_rad=0.0,
            )

    def test_joint_specific_limit_tolerance_requires_clamp_mode(self) -> None:
        with self.assertRaisesRegex(ValueError, "requires clamp mode"):
            ControlLoop(
                requester=object(),
                real_action_safety_enabled=True,
                real_joint_limit_mode="reject",
                real_joint_limit_tolerance_by_joint={"joint_a": 0.18},
            )

    def test_warm_start_requires_safety_and_total_limits(self) -> None:
        with self.assertRaisesRegex(ValueError, "requires real action safety"):
            ControlLoop(
                requester=object(),
                real_warm_start_enabled=True,
                real_warm_start_max_total_delta_by_key={"arm": 0.10},
            )
        with self.assertRaisesRegex(ValueError, "requires total-delta limits"):
            ControlLoop(
                requester=object(),
                real_action_safety_enabled=True,
                real_warm_start_enabled=True,
            )

    def test_source_step_bridge_requires_safety_and_raw_limits(self) -> None:
        with self.assertRaisesRegex(ValueError, "requires real action safety"):
            ControlLoop(
                requester=object(),
                real_source_step_bridge_enabled=True,
                real_source_step_bridge_max_raw_delta_by_key={"arm": 0.05},
            )
        with self.assertRaisesRegex(ValueError, "requires raw-delta limits"):
            ControlLoop(
                requester=object(),
                real_action_safety_enabled=True,
                real_source_step_bridge_enabled=True,
            )

    def test_tracking_bridge_requires_real_safety(self) -> None:
        with self.assertRaisesRegex(ValueError, "requires real action safety"):
            ControlLoop(
                requester=object(),
                real_tracking_bridge_max_total_delta_by_key={"arm": 0.30},
            )

    def test_dry_run_publishes_preview_without_robot_command(self) -> None:
        action = np.asarray([0.1, 0.2], dtype=np.float64)
        processor = FakeProcessor(actions=[action])
        robot = FakeRobot()
        loop = self._make_loop(processor, robot)

        loop.set_publish_to_robot(False)
        loop.tick()

        self.assertEqual(len(robot.commands), 0)
        self.assertEqual(len(robot.previews), 1)
        np.testing.assert_allclose(robot.previews[0][0], action)

    def test_robot_mode_publishes_preview_and_robot_command(self) -> None:
        action = np.asarray([0.3, 0.4], dtype=np.float64)
        processor = FakeProcessor(actions=[action])
        robot = FakeRobot()
        loop = self._make_loop(processor, robot)
        loop._publish_to_robot = True

        loop.tick()

        self.assertEqual(len(robot.commands), 1)
        self.assertEqual(len(robot.previews), 1)
        np.testing.assert_allclose(robot.commands[0][0], action)
        np.testing.assert_allclose(robot.previews[0][0], action)

    def test_robot_publish_error_does_not_crash_tick(self) -> None:
        class FailingRobot(FakeRobot):
            def publish_action(self, action, action_keys) -> None:
                raise RuntimeError("publish failed")

        processor = FakeProcessor(actions=[np.asarray([0.5], dtype=np.float64)])
        robot = FailingRobot()
        loop = self._make_loop(processor, robot)
        loop._publish_to_robot = True

        loop.tick()

        self.assertEqual(len(robot.previews), 1)

    def test_real_robot_publish_error_stops_and_clears_buffer(self) -> None:
        class FailingRobot(FakeRobot):
            def publish_action(self, action, action_keys) -> None:
                raise RuntimeError("publisher disconnected")

        processor = FakeProcessor(
            actions=[np.asarray([0.1, 0.2], dtype=np.float64)]
        )
        robot = FailingRobot()
        loop = ControlLoop(
            requester=object(),
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
        )
        loop._running = True
        loop._robot = robot
        loop._processor = processor
        loop._action_keys = ["arm"]
        loop._publish_to_robot = True
        loop._request_thread = SimpleNamespace(is_alive=lambda: True)

        loop.tick()

        self.assertFalse(loop._running)
        self.assertEqual(processor.clear_count, 1)
        self.assertEqual(robot.commands, [])

    def test_real_fail_stop_reports_async_reason_once(self) -> None:
        reasons = []
        processor = FakeProcessor(
            actions=[np.asarray([0.1, 0.2], dtype=np.float64)]
        )
        robot = FakeRobot()
        robot.safety_error = "tracking lag exceeded"
        loop = ControlLoop(
            requester=object(),
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
            real_action_blocked_callback=reasons.append,
        )
        loop._running = True
        loop._robot = robot
        loop._processor = processor
        loop._action_keys = ["arm"]
        loop._publish_to_robot = True
        loop._request_thread = SimpleNamespace(is_alive=lambda: True)

        loop.tick()
        loop.tick()

        self.assertEqual(
            reasons,
            ["publish-time tracking revalidation failed: tracking lag exceeded"],
        )

    def test_robot_mode_publishes_idle_when_action_buffer_is_empty(self) -> None:
        processor = FakeProcessor(actions=[], buffer_size=100)
        robot = FakeRobot()
        loop = self._make_loop(processor, robot)
        loop._publish_to_robot = True
        loop._action_keys = ["mobile"]

        loop.tick()

        self.assertEqual(robot.idles, [["mobile"]])
        self.assertEqual(len(robot.commands), 0)
        self.assertEqual(len(robot.previews), 0)

    def test_dry_run_does_not_publish_idle_when_action_buffer_is_empty(self) -> None:
        processor = FakeProcessor(actions=[], buffer_size=100)
        robot = FakeRobot()
        loop = self._make_loop(processor, robot)
        loop._publish_to_robot = False
        loop._action_keys = ["mobile"]

        loop.tick()

        self.assertEqual(robot.idles, [])

    def test_mode_change_clears_buffer(self) -> None:
        processor = FakeProcessor()
        robot = FakeRobot()
        loop = self._make_loop(processor, robot)

        loop.set_publish_to_robot(True)

        self.assertEqual(processor.clear_count, 1)

    def test_pause_clears_buffer(self) -> None:
        processor = FakeProcessor()
        robot = FakeRobot()
        loop = self._make_loop(processor, robot)

        loop.pause()

        self.assertEqual(processor.clear_count, 1)

    def test_refill_threshold_includes_observed_request_latency(self) -> None:
        processor = FakeProcessor()
        robot = FakeRobot()
        loop = self._make_loop(processor, robot)
        loop._refill_margin_s = 0.25
        loop._request_latency_ema_s = 0.25

        self.assertEqual(loop._refill_threshold(processor), 50)

    def test_initial_latency_sample_is_ignored_for_warmup(self) -> None:
        processor = FakeProcessor()
        robot = FakeRobot()
        loop = self._make_loop(processor, robot)
        loop._latency_warmup_remaining = 1

        loop._record_request_latency(5.0)
        self.assertIsNone(loop._request_latency_ema_s)

        loop._record_request_latency(0.25)
        self.assertEqual(loop._request_latency_ema_s, 0.25)

    def test_refill_latency_outlier_is_ignored(self) -> None:
        processor = FakeProcessor()
        robot = FakeRobot()
        loop = self._make_loop(processor, robot)
        loop._latency_warmup_remaining = 0
        loop._max_refill_latency_s = 1.0

        loop._record_request_latency(0.2)
        loop._record_request_latency(5.0)

        self.assertEqual(loop._request_latency_ema_s, 0.2)

    def test_async_mode_requests_before_buffer_is_empty(self) -> None:
        processor = FakeProcessor(buffer_size=10)
        robot = FakeRobot()
        loop = self._make_loop(processor, robot)
        loop._action_request_mode = "async"
        loop._refill_margin_s = 0.2
        loop._request_latency_ema_s = None

        self.assertTrue(loop._should_request_actions(processor))

        processor.buffer_size = 30
        self.assertFalse(loop._should_request_actions(processor))

    def test_sync_mode_prefetches_inside_bounded_refill_window(self) -> None:
        processor = FakeProcessor(buffer_size=10)
        robot = FakeRobot()
        loop = self._make_loop(processor, robot)
        loop._action_request_mode = "sync"
        loop._refill_margin_s = 0.06
        loop._request_latency_ema_s = 0.04

        self.assertTrue(loop._should_request_actions(processor))

        processor.buffer_size = 11
        self.assertFalse(loop._should_request_actions(processor))

    def test_ordered_async_mode_uses_near_tail_refill_window(self) -> None:
        processor = FakeProcessor(buffer_size=8)
        robot = FakeRobot()
        loop = self._make_loop(processor, robot)
        loop._action_request_mode = ACTION_REQUEST_MODE_ASYNC_ORDERED
        loop._refill_margin_s = 1.0
        loop._ordered_async_refill_margin_s = 0.03
        loop._request_latency_ema_s = 0.05

        self.assertTrue(loop._should_request_actions(processor))

        processor.buffer_size = 9
        self.assertFalse(loop._should_request_actions(processor))

    def test_ordered_async_mode_normalizes_alias(self) -> None:
        self.assertEqual(
            normalize_action_request_mode("ordered_async"),
            ACTION_REQUEST_MODE_ASYNC_ORDERED,
        )

    def test_step_sync_mode_normalizes_alias(self) -> None:
        self.assertEqual(
            normalize_action_request_mode("step_sync"),
            ACTION_REQUEST_MODE_SYNC_STEP,
        )

    def test_step_sync_waits_for_buffer_to_drain(self) -> None:
        processor = FakeProcessor(buffer_size=1)
        robot = FakeRobot()
        loop = self._make_loop(processor, robot)
        loop._action_request_mode = ACTION_REQUEST_MODE_SYNC_STEP

        self.assertFalse(loop._should_request_actions(processor))

        processor.buffer_size = 0
        self.assertTrue(loop._should_request_actions(processor))

    def test_sync_mode_buffers_chunk_without_scheduled_skip(self) -> None:
        response = SimpleNamespace(
            success=True,
            message="ok",
            chunk_size=2,
            action_dim=2,
            action_list=[0.1, 0.2, 0.3, 0.4],
        )
        processor = FakeProcessor(buffer_size=0)
        loop = ControlLoop(requester=FakeRequester(response))
        loop._running = True
        loop._processor = processor

        loop._request_and_buffer("pick", loop._generation, "sync")

        self.assertEqual(len(processor.pushed_chunks), 1)
        self.assertIsNone(processor.scheduled_delays[-1])
        self.assertEqual(processor.align_flags[-1], False)

    def test_async_mode_buffers_chunk_with_latency_and_buffer_delay(self) -> None:
        response = SimpleNamespace(
            success=True,
            message="ok",
            chunk_size=2,
            action_dim=2,
            action_list=[0.1, 0.2, 0.3, 0.4],
        )
        processor = FakeProcessor(buffer_size=50)
        loop = ControlLoop(requester=FakeRequester(response))
        loop._running = True
        loop._processor = processor

        loop._request_and_buffer("pick", loop._generation, "async")

        self.assertEqual(len(processor.pushed_chunks), 1)
        self.assertIsNotNone(processor.scheduled_delays[-1])
        self.assertGreaterEqual(processor.scheduled_delays[-1], 0.5)
        self.assertEqual(processor.align_flags[-1], True)

    def test_raw_and_ensemble_are_both_checked_and_reset_on_pause(self) -> None:
        response = SimpleNamespace(success=True, chunk_size=3, action_dim=2,
                                   action_list=[1.0] * 6)
        processor = FakeProcessor(buffer_size=0)
        robot = FakeRobot()
        loop = ControlLoop(requester=FakeRequester(response), inference_hz=10,
                           temporal_ensemble_coeff=0, temporal_ensemble_tail_fade_s=0,
                           real_action_safety_enabled=True)
        loop._running = True
        loop._publish_to_robot = True
        loop._real_first_action_pending = False
        loop._robot = robot
        loop._action_keys = ['arm']
        loop._processor = processor
        loop._temporal_ensemble.update(np.zeros((3, 2)), 1.0)
        with patch('main_runtime.control_loop.time.monotonic', side_effect=[1.1, 1.15]):
            loop._request_and_buffer('', loop._generation, 'async')
        np.testing.assert_allclose(robot.safety_calls[0][0], np.ones((3, 2)))
        np.testing.assert_allclose(robot.safety_calls[1][0], [[.5,.5],[.5,.5],[1,1]])
        np.testing.assert_allclose(processor.pushed_chunks[0], robot.safety_calls[1][0])
        loop.pause()
        assert not loop._temporal_ensemble._plans

    def test_raw_failure_is_not_hidden_by_ensemble_or_source_limiter(self):
        response = SimpleNamespace(success=True, chunk_size=3, action_dim=2,
                                   action_list=[0,0,.2,0,.2,0])
        robot = FakeRobot()
        robot.safety_error = ValueError('unsafe raw source-step delta')
        loop = ControlLoop(requester=FakeRequester(response), temporal_ensemble_coeff=.01,
                           real_action_safety_enabled=True, real_source_step_max_delta_rad=.03)
        loop._running = loop._publish_to_robot = True
        loop._processor = FakeProcessor(buffer_size=0)
        loop._robot = robot
        loop._action_keys = ['arm']
        loop._request_and_buffer('', loop._generation)
        self.assertFalse(loop._running)
        self.assertFalse(loop._temporal_ensemble._plans)
        self.assertEqual(len(robot.safety_calls), 1)
        self.assertFalse(loop._processor.pushed_chunks)

    def test_ensemble_generated_step_is_bounded_before_final_safety(self):
        response = SimpleNamespace(success=True, chunk_size=100, action_dim=2,
                                   action_list=[0.0] * 200)
        robot = FakeRobot()
        loop = ControlLoop(requester=FakeRequester(response), inference_hz=30,
                           temporal_ensemble_coeff=.01, real_action_safety_enabled=True,
                           real_source_step_max_delta_rad=.03)
        loop._running = loop._publish_to_robot = True
        loop._real_first_action_pending = False
        loop._processor = FakeProcessor(buffer_size=0)
        loop._robot = robot
        loop._action_keys = ['arm']
        # Both individual predictions are constant and valid, but their
        # disagreement at the old horizon's fade edge causes >0.12 rad steps.
        loop._temporal_ensemble.update(np.full((100,2),2.0), 1.0)
        with patch('main_runtime.control_loop.time.monotonic', side_effect=[1.5, 1.65]):
            loop._request_and_buffer('', loop._generation)
        final = robot.safety_calls[1][0]
        self.assertLessEqual(np.max(np.abs(np.diff(final,axis=0))), .03 + 1e-12)
        self.assertGreater(loop._ensemble_info_locked()['unlimited_max_step_rad'], .12)
        self.assertGreater(loop._ensemble_info_locked()['limited_values'], 0)
        np.testing.assert_array_equal(loop._temporal_ensemble._plans[-1][1], np.zeros((100,2)))

    def test_late_response_after_pause_cannot_repopulate_ensemble(self) -> None:
        response = SimpleNamespace(success=True, chunk_size=3, action_dim=2,
                                   action_list=[1.0] * 6)
        loop = ControlLoop(requester=FakeRequester(response), temporal_ensemble_coeff=.01)
        loop._processor = FakeProcessor(buffer_size=0)
        loop._running = True
        old_generation = loop._generation
        loop.pause()
        loop._running = True
        loop._request_and_buffer('', old_generation, 'async')
        assert not loop._temporal_ensemble._plans
        assert not loop._processor.pushed_chunks

    def test_stop_mode_change_and_deconfigure_clear_ensemble(self) -> None:
        loop = ControlLoop(requester=object(), temporal_ensemble_coeff=.01)
        for operation in [loop.stop, lambda: loop.set_publish_to_robot(True), loop.deconfigure]:
            loop._temporal_ensemble.update(np.ones((100, 2)), 1)
            operation()
            assert not loop._temporal_ensemble._plans

    def test_ordered_async_buffers_full_chunk_without_scheduled_skip(self) -> None:
        response = SimpleNamespace(
            success=True,
            message="ok",
            chunk_size=2,
            action_dim=2,
            action_list=[0.1, 0.2, 0.3, 0.4],
        )
        processor = FakeProcessor(buffer_size=5)
        loop = ControlLoop(requester=FakeRequester(response))
        loop._running = True
        loop._processor = processor

        loop._request_and_buffer(
            "pick",
            loop._generation,
            ACTION_REQUEST_MODE_ASYNC_ORDERED,
        )

        self.assertEqual(len(processor.pushed_chunks), 1)
        self.assertIsNone(processor.scheduled_delays[-1])
        self.assertEqual(processor.align_flags[-1], False)

    def test_step_sync_buffers_without_scheduled_skip(self) -> None:
        response = SimpleNamespace(
            success=True,
            message="ok",
            chunk_size=2,
            action_dim=2,
            action_list=[0.1, 0.2, 0.1, 0.2],
        )
        processor = FakeProcessor(buffer_size=0)
        loop = ControlLoop(requester=FakeRequester(response))
        loop._running = True
        loop._processor = processor

        loop._request_and_buffer(
            "pick",
            loop._generation,
            ACTION_REQUEST_MODE_SYNC_STEP,
        )

        self.assertEqual(len(processor.pushed_chunks), 1)
        self.assertIsNone(processor.scheduled_delays[-1])
        self.assertEqual(processor.align_flags[-1], False)

    def test_nonfinite_chunk_is_rejected_before_buffering(self) -> None:
        response = SimpleNamespace(
            success=True,
            message="ok",
            chunk_size=1,
            action_dim=2,
            action_list=[np.nan, 0.0],
        )
        processor = FakeProcessor(buffer_size=0)
        loop = ControlLoop(requester=FakeRequester(response))
        loop._running = True
        loop._processor = processor
        loop._robot = FakeRobot()
        loop._action_keys = ["arm"]

        loop._request_and_buffer("pick", loop._generation, "sync")

        self.assertEqual(processor.pushed_chunks, [])

    def test_model_robot_dimension_mismatch_is_rejected(self) -> None:
        response = SimpleNamespace(
            success=True,
            message="ok",
            chunk_size=1,
            action_dim=1,
            action_list=[0.0],
        )
        processor = FakeProcessor(buffer_size=0)
        loop = ControlLoop(requester=FakeRequester(response))
        loop._running = True
        loop._processor = processor
        loop._robot = FakeRobot()
        loop._action_keys = ["arm"]

        loop._request_and_buffer("pick", loop._generation, "sync")

        self.assertEqual(processor.pushed_chunks, [])

    def test_get_action_exception_stops_real_loop_and_clears_buffer(self) -> None:
        processor = FakeProcessor(buffer_size=0)
        loop = ControlLoop(
            requester=RaisingRequester(RuntimeError("transport lost")),
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
        )
        loop._running = True
        loop._processor = processor
        loop._robot = FakeRobot()
        loop._action_keys = ["arm"]
        loop._publish_to_robot = True

        loop._request_and_buffer("pick", loop._generation, "sync")

        self.assertFalse(loop._running)
        self.assertEqual(processor.clear_count, 1)
        self.assertEqual(processor.pushed_chunks, [])

    def test_get_action_exception_does_not_stop_simulation_loop(self) -> None:
        processor = FakeProcessor(buffer_size=0)
        loop = ControlLoop(
            requester=RaisingRequester(RuntimeError("transport lost")),
            real_action_safety_enabled=True,
            real_action_safety_required=True,
            real_first_action_max_delta_rad=0.03,
        )
        loop._running = True
        loop._processor = processor
        loop._robot = FakeRobot()
        loop._action_keys = ["arm"]
        loop._publish_to_robot = False

        loop._request_and_buffer("pick", loop._generation, "sync")

        self.assertTrue(loop._running)
        self.assertEqual(processor.clear_count, 0)

    def test_real_preflight_returns_safety_failure_without_publishing(self) -> None:
        response = SimpleNamespace(
            success=True,
            message="ok",
            chunk_size=1,
            action_dim=2,
            action_list=[0.1, 0.2],
        )
        robot = FakeRobot()
        robot.safety_error = "unsafe current-state to first-action delta"
        loop = ControlLoop(
            requester=FakeRequester(response),
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
        )
        loop._robot = robot
        loop._processor = FakeProcessor(buffer_size=0)
        loop._action_keys = ["arm"]

        with self.assertRaisesRegex(RuntimeError, "Real action preflight blocked"):
            loop.preflight_start(True, "pick")

        self.assertEqual(robot.commands, [])
        self.assertEqual(robot.previews, [])
        self.assertFalse(loop._running)

    def test_real_preflight_requests_bounded_warm_start(self) -> None:
        response = SimpleNamespace(
            success=True,
            message="ok",
            chunk_size=1,
            action_dim=2,
            action_list=[0.1, 0.2],
        )
        robot = FakeRobot()
        loop = ControlLoop(
            requester=FakeRequester(response),
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
            real_warm_start_enabled=True,
            real_warm_start_max_total_delta_by_key={"arm": 0.10},
            real_source_step_max_delta_rad=0.03,
            real_source_step_bridge_enabled=True,
            real_source_step_bridge_max_raw_delta_by_key={"arm": 0.05},
        )
        loop._robot = robot
        loop._processor = FakeProcessor(buffer_size=0)
        loop._action_keys = ["arm"]

        loop.preflight_start(True, "pick")

        self.assertEqual(len(robot.safety_calls), 1)
        kwargs = robot.safety_calls[0][2]
        self.assertEqual(
            kwargs["warm_start_max_total_delta_by_key"],
            {"arm": 0.10},
        )
        self.assertEqual(
            kwargs["source_step_bridge_max_raw_delta_by_key"],
            {"arm": 0.05},
        )
        self.assertEqual(robot.commands, [])
        self.assertEqual(robot.previews, [])
        self.assertEqual(len(loop._processor.pushed_chunks), 1)
        self.assertFalse(loop._processor.align_flags[-1])
        self.assertFalse(loop._real_first_action_pending)

    def test_real_resume_preflight_uses_tracking_envelope_for_arm_bridge(self) -> None:
        response = SimpleNamespace(
            success=True,
            message="ok",
            chunk_size=1,
            action_dim=2,
            action_list=[0.1, 0.2],
        )
        robot = FakeRobot()
        loop = ControlLoop(
            requester=FakeRequester(response),
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
            real_warm_start_enabled=True,
            real_warm_start_max_total_delta_by_key={
                "arm": 0.10,
                "hand": 0.12,
            },
            real_tracking_max_delta_by_key={
                "arm": 0.06,
                "hand": 1.20,
            },
            real_tracking_bridge_max_total_delta_by_key={
                "arm": 0.30,
            },
            real_resume_warm_start_max_total_delta_by_key={
                "arm": 0.55,
                "hand": 1.55,
            },
        )
        loop._robot = robot
        loop._processor = FakeProcessor(buffer_size=0)
        loop._action_keys = ["arm"]

        loop.preflight_start(True, "pick", continuation=True)

        kwargs = robot.safety_calls[0][2]
        self.assertEqual(
            kwargs["warm_start_max_total_delta_by_key"],
            {"arm": 0.55, "hand": 1.55},
        )

    def test_real_initial_preflight_keeps_strict_warm_start_limit(self) -> None:
        response = SimpleNamespace(
            success=True,
            message="ok",
            chunk_size=1,
            action_dim=2,
            action_list=[0.1, 0.2],
        )
        robot = FakeRobot()
        loop = ControlLoop(
            requester=FakeRequester(response),
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
            real_warm_start_enabled=True,
            real_warm_start_max_total_delta_by_key={"arm": 0.10},
            real_tracking_bridge_max_total_delta_by_key={"arm": 0.30},
        )
        loop._robot = robot
        loop._processor = FakeProcessor(buffer_size=0)
        loop._action_keys = ["arm"]

        loop.preflight_start(True, "pick")

        kwargs = robot.safety_calls[0][2]
        self.assertEqual(
            kwargs["warm_start_max_total_delta_by_key"],
            {"arm": 0.10},
        )

    def test_real_preflight_rejects_action_contract_before_inference(self) -> None:
        response = SimpleNamespace(
            success=True,
            message="ok",
            chunk_size=1,
            action_dim=2,
            action_list=[0.1, 0.2],
        )
        requester = FakeRequester(response)
        robot = FakeRobot()
        robot.contract_error = "publisher unavailable"
        loop = ControlLoop(
            requester=requester,
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
        )
        loop._robot = robot
        loop._processor = FakeProcessor(buffer_size=0)
        loop._action_keys = ["arm"]

        with self.assertRaisesRegex(RuntimeError, "publisher unavailable"):
            loop.preflight_start(True, "pick")

        self.assertEqual(requester.calls, [])
        self.assertFalse(loop._running)

    def test_empty_model_action_keys_keep_sim_fallback_but_fail_real_contract(
        self,
    ) -> None:
        response = SimpleNamespace(
            success=True,
            message="ok",
            chunk_size=1,
            action_dim=2,
            action_list=[0.1, 0.2],
        )
        requester = FakeRequester(response)
        robot = FakeRobot()
        processor = FakeProcessor(buffer_size=0)
        loop = ControlLoop(
            requester=requester,
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
        )

        with (
            patch(
                "main_runtime.control_loop.RobotClient",
                return_value=robot,
            ),
            patch(
                "main_runtime.control_loop.ActionChunkProcessor",
                return_value=processor,
            ),
        ):
            loop.configure("ffw", action_keys=[], publish_to_robot=False)

        self.assertEqual(loop._action_keys, ["arm"])
        self.assertEqual(loop._model_action_keys, [])
        with self.assertRaisesRegex(RuntimeError, "contract mismatch"):
            loop.preflight_start(True, "pick")

        self.assertEqual(robot.contract_calls, [[]])
        self.assertEqual(requester.calls, [])

    def test_required_safety_blocks_real_but_not_simulation_start(self) -> None:
        loop = ControlLoop(
            requester=object(),
            real_action_safety_enabled=False,
            real_action_safety_required=True,
        )

        loop.preflight_start(False, "pick")
        with self.assertRaisesRegex(RuntimeError, "required action safety"):
            loop.preflight_start(True, "pick")
        with self.assertRaisesRegex(RuntimeError, "required action safety"):
            loop.start(True)

    def test_first_real_publish_is_revalidated_and_blocked(self) -> None:
        action = np.asarray([0.1, 0.2], dtype=np.float64)
        processor = FakeProcessor(actions=[action])
        robot = FakeRobot()
        robot.safety_error = "live pose moved"
        loop = ControlLoop(
            requester=object(),
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
        )
        loop._running = True
        loop._robot = robot
        loop._processor = processor
        loop._action_keys = ["arm"]
        loop._publish_to_robot = True
        loop._request_thread = SimpleNamespace(is_alive=lambda: True)

        loop.tick()

        self.assertEqual(robot.commands, [])
        self.assertEqual(robot.previews, [])
        self.assertEqual(processor.clear_count, 1)

    def test_first_real_publish_uses_validated_action(self) -> None:
        action = np.asarray([0.1, 0.2], dtype=np.float64)
        processor = FakeProcessor(actions=[action])
        robot = FakeRobot()
        loop = ControlLoop(
            requester=object(),
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
        )
        loop._running = True
        loop._robot = robot
        loop._processor = processor
        loop._action_keys = ["arm"]
        loop._publish_to_robot = True
        loop._request_thread = SimpleNamespace(is_alive=lambda: True)

        loop.tick()

        self.assertEqual(len(robot.safety_calls), 1)
        self.assertEqual(len(robot.commands), 1)
        self.assertFalse(loop._real_first_publish_pending)

    def test_real_publish_forwards_joint_specific_limit_tolerance(self) -> None:
        action = np.asarray([0.1, 0.2], dtype=np.float64)
        processor = FakeProcessor(actions=[action])
        robot = FakeRobot()
        loop = ControlLoop(
            requester=object(),
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
            real_joint_limit_mode="clamp",
            real_joint_limit_tolerance_rad=0.02,
            real_joint_limit_tolerance_by_joint={"arm_l_joint2": 0.18},
        )
        loop._running = True
        loop._robot = robot
        loop._processor = processor
        loop._action_keys = ["arm"]
        loop._publish_to_robot = True
        loop._request_thread = SimpleNamespace(is_alive=lambda: True)

        loop.tick()

        self.assertEqual(
            robot.safety_calls[0][2]["joint_limit_tolerance_by_joint"],
            {"arm_l_joint2": 0.18},
        )

    def test_every_real_publish_is_revalidated_against_live_state(self) -> None:
        actions = [
            np.asarray([0.1, 0.2], dtype=np.float64),
            np.asarray([0.11, 0.21], dtype=np.float64),
        ]
        processor = FakeProcessor(actions=actions)
        robot = FakeRobot()
        loop = ControlLoop(
            requester=object(),
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
        )
        loop._running = True
        loop._robot = robot
        loop._processor = processor
        loop._action_keys = ["arm"]
        loop._publish_to_robot = True
        loop._request_thread = SimpleNamespace(is_alive=lambda: True)

        loop.tick()
        loop.tick()

        self.assertEqual(len(robot.safety_calls), 2)
        self.assertEqual(len(robot.commands), 2)
        for _chunk, _keys, kwargs in robot.safety_calls:
            self.assertEqual(kwargs["first_action_max_delta_rad"], 0.03)
            self.assertIsNone(kwargs["warm_start_max_total_delta_by_key"])
            self.assertIsNone(kwargs["source_step_bridge_max_raw_delta_by_key"])

    def test_subsequent_publish_uses_tracking_lag_gate(self) -> None:
        actions = [
            np.asarray([0.1, 0.2], dtype=np.float64),
            np.asarray([0.11, 0.21], dtype=np.float64),
        ]
        processor = FakeProcessor(actions=actions)
        robot = FakeRobot()
        loop = ControlLoop(
            requester=object(),
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
            real_first_action_max_delta_by_key={"arm": 0.03},
            real_tracking_max_delta_by_key={"arm": 0.06},
            real_tracking_bridge_max_total_delta_by_key={"arm": 0.30},
            real_source_step_max_delta_rad=0.03,
        )
        loop._running = True
        loop._robot = robot
        loop._processor = processor
        loop._action_keys = ["arm"]
        loop._publish_to_robot = True
        loop._request_thread = SimpleNamespace(is_alive=lambda: True)

        loop.tick()
        loop.tick()

        self.assertEqual(len(robot.safety_calls), 2)
        first_kwargs = robot.safety_calls[0][2]
        second_kwargs = robot.safety_calls[1][2]
        self.assertEqual(first_kwargs["first_action_max_delta_rad"], 0.03)
        self.assertEqual(
            first_kwargs["first_action_max_delta_by_key"],
            {"arm": 0.03},
        )
        self.assertIsNone(second_kwargs["first_action_max_delta_rad"])
        self.assertEqual(
            second_kwargs["first_action_max_delta_by_key"],
            {"arm": 0.06},
        )
        self.assertIsNone(first_kwargs["state_bridge_max_total_delta_by_key"])
        self.assertEqual(
            second_kwargs["state_bridge_max_total_delta_by_key"],
            {"arm": 0.30},
        )
        self.assertIsNone(first_kwargs["previous_published_action"])
        np.testing.assert_allclose(
            second_kwargs["previous_published_action"],
            actions[0],
        )

    def test_tracking_bridge_holds_desired_action_until_recovered(self) -> None:
        desired = np.asarray([0.20, 0.20], dtype=np.float64)
        following = np.asarray([0.30, 0.30], dtype=np.float64)
        processor = FakeProcessor(actions=[desired, following])
        robot = FakeRobot()
        robot.safety_outputs = [
            np.asarray([[0.03, 0.03]], dtype=np.float64),
            np.asarray([[0.08, 0.08]], dtype=np.float64),
            desired.reshape(1, -1),
        ]
        loop = ControlLoop(
            requester=object(),
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
            real_tracking_max_delta_by_key={"arm": 0.06},
            real_tracking_bridge_max_total_delta_by_key={"arm": 0.30},
            real_source_step_max_delta_rad=0.03,
        )
        loop._running = True
        loop._robot = robot
        loop._processor = processor
        loop._action_keys = ["arm"]
        loop._publish_to_robot = True
        loop._real_first_publish_pending = False
        loop._real_last_published_action = np.zeros(2, dtype=np.float64)
        loop._request_thread = SimpleNamespace(is_alive=lambda: True)

        loop.tick()
        loop.tick()

        self.assertEqual(len(processor._actions), 2)
        np.testing.assert_allclose(processor._actions[0], desired)
        np.testing.assert_allclose(robot.commands[0][0], [0.03, 0.03])
        np.testing.assert_allclose(robot.commands[1][0], [0.08, 0.08])
        self.assertEqual(len(processor.deferred_actions), 2)

        loop.tick()

        self.assertEqual(len(processor._actions), 1)
        np.testing.assert_allclose(processor._actions[0], following)
        np.testing.assert_allclose(robot.commands[2][0], desired)
        self.assertIsNone(loop._real_tracking_hold_action)

    def test_tracking_hold_stops_after_one_second_without_progress(self) -> None:
        reasons = []
        desired = np.asarray([0.20, 0.20], dtype=np.float64)
        processor = FakeProcessor(actions=[desired])
        robot = FakeRobot()
        robot.safety_outputs = [
            np.asarray([[0.03, 0.03]], dtype=np.float64),
            np.asarray([[0.03, 0.03]], dtype=np.float64),
        ]
        loop = ControlLoop(
            requester=object(),
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
            real_tracking_max_delta_by_key={"arm": 0.06},
            real_tracking_bridge_max_total_delta_by_key={"arm": 0.30},
            real_source_step_max_delta_rad=0.03,
            real_action_blocked_callback=reasons.append,
        )
        loop._running = True
        loop._robot = robot
        loop._processor = processor
        loop._action_keys = ["arm"]
        loop._publish_to_robot = True
        loop._real_first_publish_pending = False
        loop._real_last_published_action = np.zeros(2, dtype=np.float64)
        loop._request_thread = SimpleNamespace(is_alive=lambda: True)

        with patch(
            "main_runtime.control_loop.time.monotonic",
            side_effect=[10.0, 11.01],
        ):
            loop.tick()
            loop.tick()

        self.assertFalse(loop._running)
        self.assertEqual(len(robot.commands), 1)
        self.assertEqual(processor.clear_count, 1)
        self.assertEqual(len(reasons), 1)
        self.assertIn("made no progress", reasons[0])
        self.assertIn("tracking_diagnostics_unavailable=AttributeError", reasons[0])

    def test_tracking_stall_reports_joint_and_candidate_without_publishing_it(self) -> None:
        reasons = []
        desired = np.asarray([0.08, 0.20], dtype=np.float64)
        bounded = np.asarray([[0.03, 0.04]], dtype=np.float64)
        robot = FakeRobot()
        robot.safety_outputs = [bounded, bounded]
        robot.get_joint_names = lambda group: ["arm_r_joint6", "arm_r_joint7"]
        robot.get_joint_position_snapshot = lambda group: (
            np.asarray([0.01, 0.02]), 99.9,
        )
        processor = FakeProcessor(actions=[desired])
        loop = ControlLoop(
            requester=object(),
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
            real_action_blocked_callback=reasons.append,
        )
        loop._running = True
        loop._robot = robot
        loop._processor = processor
        loop._action_keys = ["arm_right"]
        loop._publish_to_robot = True
        loop._real_first_publish_pending = False
        loop._real_last_published_action = np.zeros(2)
        loop._request_thread = SimpleNamespace(is_alive=lambda: True)

        with patch("main_runtime.control_loop.time.monotonic", side_effect=[10.0, 11.01]), patch(
            "main_runtime.control_loop.time.time", return_value=100.0,
        ):
            loop.tick()
            loop.tick()

        self.assertFalse(loop._running)
        self.assertEqual(len(robot.commands), 1)
        self.assertEqual(processor.clear_count, 1)
        reason = reasons[0]
        self.assertIn("snapshot=after_guard; joint=arm_r_joint7 action_key=arm_right", reason)
        self.assertIn("desired=0.200000 bounded_candidate=0.040000 remaining=0.160000 rad", reason)
        self.assertIn("previous_published=0.040000 measured=0.020000", reason)
        self.assertIn("desired_state_delta=0.180000 candidate_state_delta=0.020000 rad", reason)
        self.assertIn("state_age=0.100s", reason)

    def test_tracking_diagnostics_reject_incomplete_joint_layout(self) -> None:
        loop = ControlLoop(requester=object())
        robot = FakeRobot()
        robot.get_joint_names = lambda group: ["wrong_width"]
        loop._robot = robot
        loop._action_keys = ["arm_right"]
        detail = loop._describe_real_tracking_hold_locked(np.ones(2), np.zeros(2))
        self.assertEqual(detail, "tracking_diagnostics_unavailable=ValueError")

    def test_subsequent_stale_state_blocks_before_second_publish(self) -> None:
        processor = FakeProcessor(actions=[
            np.asarray([0.1, 0.2], dtype=np.float64),
            np.asarray([0.11, 0.21], dtype=np.float64),
        ])
        robot = FakeRobot()
        loop = ControlLoop(
            requester=object(),
            real_action_safety_enabled=True,
            real_first_action_max_delta_rad=0.03,
        )
        loop._running = True
        loop._robot = robot
        loop._processor = processor
        loop._action_keys = ["arm"]
        loop._publish_to_robot = True
        loop._request_thread = SimpleNamespace(is_alive=lambda: True)

        loop.tick()
        robot.safety_error = "current state is stale"
        loop.tick()

        self.assertEqual(len(robot.commands), 1)
        self.assertFalse(loop._running)
        self.assertEqual(processor.clear_count, 1)

    def test_safety_profile_leaves_simulation_preview_unchanged(self) -> None:
        action = np.asarray([0.1, 0.2], dtype=np.float64)
        processor = FakeProcessor(actions=[action])
        robot = FakeRobot()
        loop = ControlLoop(
            requester=object(),
            real_action_safety_enabled=True,
            real_action_safety_required=True,
            real_first_action_max_delta_rad=0.03,
        )
        loop._running = True
        loop._robot = robot
        loop._processor = processor
        loop._action_keys = ["arm"]
        loop._publish_to_robot = False
        loop._request_thread = SimpleNamespace(is_alive=lambda: True)

        loop.tick()

        self.assertEqual(len(robot.previews), 1)
        self.assertEqual(robot.commands, [])
        self.assertEqual(robot.safety_calls, [])


if __name__ == "__main__":
    unittest.main()
