#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0

"""Robot-facing control loop owned by the Main process."""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Optional

import numpy as np


_parents = Path(__file__).resolve().parents
_default_acp = str(_parents[4] / "sdk" / "action_chunk_processing") if len(_parents) > 4 else ""
_ACTION_CHUNK_PATH = os.environ.get("ACTION_CHUNK_PROCESSING_SDK_PATH", _default_acp)
if os.path.exists(_ACTION_CHUNK_PATH) and _ACTION_CHUNK_PATH not in sys.path:
    sys.path.insert(0, _ACTION_CHUNK_PATH)

_default_rc = str(_parents[4] / "sdk" / "robot_client") if len(_parents) > 4 else ""
_ROBOT_CLIENT_PATH = os.environ.get("ROBOT_CLIENT_SDK_PATH", _default_rc)
if os.path.exists(_ROBOT_CLIENT_PATH) and _ROBOT_CLIENT_PATH not in sys.path:
    sys.path.insert(0, _ROBOT_CLIENT_PATH)

from action_chunk_processing import ActionChunkProcessor  # noqa: E402
from robot_client import RobotClient  # noqa: E402
from .diagnostic_trace import DiagnosticTrace  # noqa: E402
from .temporal_ensemble import TemporalActionEnsembler, limit_source_steps  # noqa: E402


try:  # pragma: no cover - SDK exists only in runtime container here.
    from zenoh_ros2_sdk import get_logger
except Exception:  # pragma: no cover
    import logging

    def get_logger(name: str):
        return logging.getLogger(name)


logger = get_logger("main_runtime.control_loop")


ACTION_REQUEST_MODE_ASYNC = "async"
ACTION_REQUEST_MODE_ASYNC_ORDERED = "async_ordered"
ACTION_REQUEST_MODE_SYNC = "sync"
ACTION_REQUEST_MODE_SYNC_STEP = "sync_step"
ACTION_REQUEST_MODES = {
    ACTION_REQUEST_MODE_ASYNC,
    ACTION_REQUEST_MODE_ASYNC_ORDERED,
    ACTION_REQUEST_MODE_SYNC,
    ACTION_REQUEST_MODE_SYNC_STEP,
}


def positive_finite_or_default(value: object, default: float) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError):
        return float(default)
    if not math.isfinite(parsed) or parsed <= 0.0:
        return float(default)
    return parsed


def normalize_action_request_mode(value: object) -> str:
    mode = str(value or "").strip().lower()
    if mode in {ACTION_REQUEST_MODE_ASYNC_ORDERED, "ordered_async"}:
        return ACTION_REQUEST_MODE_ASYNC_ORDERED
    if mode in {ACTION_REQUEST_MODE_SYNC_STEP, "step_sync"}:
        return ACTION_REQUEST_MODE_SYNC_STEP
    if mode == ACTION_REQUEST_MODE_SYNC:
        return ACTION_REQUEST_MODE_SYNC
    return ACTION_REQUEST_MODE_ASYNC


class ControlLoop:
    """Ticks RobotClient command publishing and refills action buffers."""

    _REAL_TRACKING_HOLD_PROGRESS_EPS_RAD = 0.001
    _REAL_TRACKING_HOLD_MAX_NO_PROGRESS_S = 1.0

    def __init__(
        self,
        requester,
        inference_hz: float = 15.0,
        control_hz: float = 100.0,
        chunk_align_window_s: float = 0.3,
        target_chunk_size: Optional[int] = None,
        postprocess_actions: bool = True,
        alignment_mode: str = "l2",
        refill_margin_s: float = 0.2,
        ordered_async_refill_margin_s: float = 0.03,
        latency_warmup_samples: int = 1,
        max_refill_latency_s: Optional[float] = 2.0,
        action_request_mode: str = ACTION_REQUEST_MODE_ASYNC,
        real_action_safety_enabled: bool = False,
        real_action_safety_required: bool = False,
        real_first_action_max_delta_rad: Optional[float] = None,
        real_first_action_max_delta_by_key: Optional[dict[str, float]] = None,
        real_tracking_max_delta_by_key: Optional[dict[str, float]] = None,
        real_tracking_bridge_max_total_delta_by_key: Optional[
            dict[str, float]
        ] = None,
        real_warm_start_enabled: bool = False,
        real_warm_start_max_total_delta_by_key: Optional[dict[str, float]] = None,
        real_resume_warm_start_max_total_delta_by_key: Optional[
            dict[str, float]
        ] = None,
        simulation_initial_pose_sync_hand_max_delta_by_key: Optional[
            dict[str, float]
        ] = None,
        real_source_step_max_delta_rad: Optional[float] = None,
        real_source_step_bridge_enabled: bool = False,
        real_source_step_bridge_max_raw_delta_by_key: Optional[
            dict[str, float]
        ] = None,
        real_joint_limit_mode: str = "off",
        real_joint_limit_tolerance_rad: float = 0.0,
        real_joint_limit_tolerance_by_joint: Optional[
            dict[str, float]
        ] = None,
        real_state_max_age_s: Optional[float] = 0.5,
        real_action_blocked_callback: Optional[Callable[[str], None]] = None,
        temporal_ensemble_coeff: Optional[float] = None,
        temporal_ensemble_tail_fade_s: float = 0.2,
    ) -> None:
        self._requester = requester
        self._inference_hz = float(inference_hz)
        self._control_hz = float(control_hz)
        self._chunk_align_window_s = float(chunk_align_window_s)
        self._default_inference_hz = self._inference_hz
        self._default_control_hz = self._control_hz
        self._default_chunk_align_window_s = self._chunk_align_window_s
        self._target_chunk_size = target_chunk_size
        self._postprocess_actions = bool(postprocess_actions)
        self._alignment_mode = alignment_mode
        self._temporal_ensemble = (
            None if temporal_ensemble_coeff is None else
            TemporalActionEnsembler(self._inference_hz, temporal_ensemble_coeff,
                                    tail_fade_s=temporal_ensemble_tail_fade_s)
        )
        self._refill_margin_s = float(refill_margin_s)
        self._default_refill_margin_s = self._refill_margin_s
        self._source_chunk_limit: Optional[int] = None
        self._ordered_async_refill_margin_s = float(
            ordered_async_refill_margin_s
        )
        if (
            not math.isfinite(self._ordered_async_refill_margin_s)
            or self._ordered_async_refill_margin_s <= 0.0
        ):
            raise ValueError(
                "ordered_async_refill_margin_s must be finite and positive"
            )
        self._request_latency_ema_s: Optional[float] = None
        self._request_latency_alpha = 0.2
        self._latency_warmup_samples = max(0, int(latency_warmup_samples))
        self._latency_warmup_remaining = self._latency_warmup_samples
        self._max_refill_latency_s = (
            None
            if max_refill_latency_s is None or max_refill_latency_s <= 0.0
            else float(max_refill_latency_s)
        )
        self._default_action_request_mode = normalize_action_request_mode(
            action_request_mode
        )
        self._action_request_mode = self._default_action_request_mode
        self._real_action_safety_enabled = bool(real_action_safety_enabled)
        self._real_action_safety_required = bool(real_action_safety_required)
        self._real_first_action_max_delta_rad = self._positive_optional_float(
            "real_first_action_max_delta_rad",
            real_first_action_max_delta_rad,
        )
        self._real_first_action_max_delta_by_key = {
            str(key): self._positive_optional_float(
                f"real_first_action_max_delta_by_key[{key}]",
                value,
            )
            for key, value in (real_first_action_max_delta_by_key or {}).items()
        }
        self._real_tracking_max_delta_by_key = {
            str(key): self._positive_optional_float(
                f"real_tracking_max_delta_by_key[{key}]",
                value,
            )
            for key, value in (real_tracking_max_delta_by_key or {}).items()
        }
        self._real_tracking_max_delta_rad = (
            None
            if self._real_tracking_max_delta_by_key
            else self._real_first_action_max_delta_rad
        )
        self._real_tracking_bridge_max_total_delta_by_key = {
            str(key): self._positive_optional_float(
                f"real_tracking_bridge_max_total_delta_by_key[{key}]",
                value,
            )
            for key, value in (
                real_tracking_bridge_max_total_delta_by_key or {}
            ).items()
        }
        if (
            self._real_tracking_bridge_max_total_delta_by_key
            and not self._real_action_safety_enabled
        ):
            raise ValueError(
                "real tracking bridge requires real action safety"
            )
        self._real_warm_start_enabled = bool(real_warm_start_enabled)
        self._real_warm_start_max_total_delta_by_key = {
            str(key): self._positive_optional_float(
                f"real_warm_start_max_total_delta_by_key[{key}]",
                value,
            )
            for key, value in (
                real_warm_start_max_total_delta_by_key or {}
            ).items()
        }
        if self._real_warm_start_enabled and not self._real_action_safety_enabled:
            raise ValueError(
                "real_warm_start_enabled requires real action safety"
            )
        if (
            self._real_warm_start_enabled
            and not self._real_warm_start_max_total_delta_by_key
        ):
            raise ValueError(
                "real_warm_start_enabled requires total-delta limits"
            )
        self._real_resume_warm_start_max_total_delta_by_key = {
            str(key): self._positive_optional_float(
                f"real_resume_warm_start_max_total_delta_by_key[{key}]",
                value,
            )
            for key, value in (
                real_resume_warm_start_max_total_delta_by_key or {}
            ).items()
        }
        if (
            self._real_resume_warm_start_max_total_delta_by_key
            and not self._real_warm_start_enabled
        ):
            raise ValueError(
                "real resume warm-start limits require warm-start to be enabled"
            )
        self._simulation_initial_pose_sync_hand_max_delta_by_key = {}
        for key, raw_value in (simulation_initial_pose_sync_hand_max_delta_by_key or {}).items():
            value = self._positive_optional_float(
                f"simulation_initial_pose_sync_hand_max_delta_by_key[{key}]", raw_value,
            )
            if key not in {"hand_left", "hand_right"} or value is None or value > 1.55:
                raise ValueError("simulation initial pose sync permits only hand limits up to 1.55 rad")
            self._simulation_initial_pose_sync_hand_max_delta_by_key[key] = value
        if self._simulation_initial_pose_sync_hand_max_delta_by_key and (
            not self._real_action_safety_enabled or not self._real_warm_start_enabled
        ):
            raise ValueError("simulation initial pose sync requires action safety and warm-start guards")
        self._real_source_step_max_delta_rad = self._positive_optional_float(
            "real_source_step_max_delta_rad",
            real_source_step_max_delta_rad,
        )
        self._real_source_step_bridge_enabled = bool(
            real_source_step_bridge_enabled
        )
        self._real_source_step_bridge_max_raw_delta_by_key = {
            str(key): self._positive_optional_float(
                f"real_source_step_bridge_max_raw_delta_by_key[{key}]",
                value,
            )
            for key, value in (
                real_source_step_bridge_max_raw_delta_by_key or {}
            ).items()
        }
        if (
            self._real_source_step_bridge_enabled
            and not self._real_action_safety_enabled
        ):
            raise ValueError(
                "real_source_step_bridge_enabled requires real action safety"
            )
        if (
            self._real_source_step_bridge_enabled
            and not self._real_source_step_bridge_max_raw_delta_by_key
        ):
            raise ValueError(
                "real_source_step_bridge_enabled requires raw-delta limits"
            )
        self._real_joint_limit_mode = str(
            real_joint_limit_mode or "off"
        ).strip().lower()
        if self._real_joint_limit_mode not in {"off", "reject", "clamp"}:
            raise ValueError(
                "real_joint_limit_mode must be one of: 'off', 'reject', 'clamp'"
            )
        self._real_joint_limit_tolerance_rad = float(
            real_joint_limit_tolerance_rad
        )
        if (
            not math.isfinite(self._real_joint_limit_tolerance_rad)
            or self._real_joint_limit_tolerance_rad < 0.0
        ):
            raise ValueError(
                "real_joint_limit_tolerance_rad must be finite and non-negative"
            )
        if (
            self._real_action_safety_enabled
            and self._real_joint_limit_mode == "clamp"
            and self._real_joint_limit_tolerance_rad <= 0.0
        ):
            raise ValueError(
                "real_joint_limit_tolerance_rad must be positive in clamp mode"
            )
        self._real_joint_limit_tolerance_by_joint = {
            str(name): self._positive_optional_float(
                f"real_joint_limit_tolerance_by_joint[{name}]",
                value,
            )
            for name, value in (
                real_joint_limit_tolerance_by_joint or {}
            ).items()
        }
        if (
            self._real_joint_limit_tolerance_by_joint
            and self._real_joint_limit_mode != "clamp"
        ):
            raise ValueError(
                "real_joint_limit_tolerance_by_joint requires clamp mode"
            )
        self._real_state_max_age_s = self._positive_optional_float(
            "real_state_max_age_s",
            real_state_max_age_s,
        )
        self._real_action_blocked_callback = real_action_blocked_callback

        self._lock = threading.RLock()
        self._robot: Optional[RobotClient] = None
        self._processor: Optional[ActionChunkProcessor] = None
        self._task_instruction = ""
        self._action_keys: list[str] = []
        self._model_action_keys: Optional[list[str]] = None
        self._publish_to_robot = False
        self._real_first_action_pending = True
        self._real_first_publish_pending = True
        self._real_last_published_action: Optional[np.ndarray] = None
        self._real_tracking_hold_action: Optional[np.ndarray] = None
        self._real_tracking_hold_best_remaining: Optional[float] = None
        self._real_tracking_hold_last_progress_at: Optional[float] = None
        self._running = False
        self._initial_pose_sync_enabled = False
        self._initial_pose_sync_duration_s = 5.0
        self._initial_pose_sync_in_progress = False
        self._initial_pose_sync_completed = False
        self._initial_pose_sync_deadline: Optional[float] = None
        self._initial_pose_sync_hold_pending = False
        self._generation = 0
        self._shutdown = threading.Event()
        self._request_thread: Optional[threading.Thread] = None
        self._thread: Optional[threading.Thread] = None
        self._diagnostic_trace = DiagnosticTrace()

    def configure(
        self,
        robot_type: str,
        task_instruction: str = "",
        action_keys: Optional[list[str]] = None,
        publish_to_robot: bool = False,
        action_request_mode: Optional[str] = None,
        refill_margin_s: Optional[float] = None,
        source_chunk_limit: Optional[int] = None,
        initial_pose_sync: bool = False,
        initial_pose_sync_duration_s: float = 5.0,
        control_hz: Optional[float] = None,
        inference_hz: Optional[float] = None,
        chunk_align_window_s: Optional[float] = None,
    ) -> None:
        duration_s = float(initial_pose_sync_duration_s)
        if not math.isfinite(duration_s) or not 1.0 <= duration_s <= 60.0:
            raise ValueError(
                "initial_pose_sync_duration_s must be between 1.0 and 60.0"
            )
        with self._lock:
            self.deconfigure()
            self._control_hz = positive_finite_or_default(
                control_hz, self._default_control_hz
            )
            self._inference_hz = positive_finite_or_default(
                inference_hz, self._default_inference_hz
            )
            self._chunk_align_window_s = positive_finite_or_default(
                chunk_align_window_s, self._default_chunk_align_window_s
            )
            if self._temporal_ensemble is not None:
                self._temporal_ensemble.source_hz = self._inference_hz
            self._action_request_mode = normalize_action_request_mode(
                action_request_mode
                if action_request_mode is not None
                else self._default_action_request_mode
            )
            self._refill_margin_s = (
                self._default_refill_margin_s
                if refill_margin_s is None
                else max(0.0, float(refill_margin_s))
            )
            self._source_chunk_limit = (
                None
                if source_chunk_limit is None
                else int(source_chunk_limit)
            )
            if (
                self._source_chunk_limit is not None
                and self._source_chunk_limit <= 0
            ):
                raise ValueError("source_chunk_limit must be positive")
            self._robot = RobotClient(
                robot_type,
                enable_command_publishers=True,
                enable_preview_publisher=True,
            )
            self._processor = ActionChunkProcessor(
                inference_hz=self._inference_hz,
                control_hz=self._control_hz,
                chunk_align_window_s=self._chunk_align_window_s,
                postprocess=self._postprocess_actions,
                target_chunk_size=self._target_chunk_size,
                source_chunk_limit=self._source_chunk_limit,
                alignment_mode=self._alignment_mode,
                sequential=(
                    self._action_request_mode
                    in {
                        ACTION_REQUEST_MODE_ASYNC_ORDERED,
                        ACTION_REQUEST_MODE_SYNC,
                    }
                ),
            )
            self._task_instruction = task_instruction or ""
            self._model_action_keys = (
                None if action_keys is None else list(action_keys)
            )
            # Preserve the legacy schema fallback for Simulation, but keep the
            # model-provided list separately so an empty Real contract cannot
            # be silently replaced by robot keys.
            self._action_keys = list(action_keys or self._robot.action_keys)
            self._publish_to_robot = bool(publish_to_robot)
            self._initial_pose_sync_enabled = bool(initial_pose_sync)
            self._initial_pose_sync_duration_s = duration_s
            self._initial_pose_sync_in_progress = False
            self._initial_pose_sync_completed = False
            self._initial_pose_sync_deadline = None
            self._initial_pose_sync_hold_pending = False
            self._real_first_action_pending = True
            self._real_first_publish_pending = True
            self._real_last_published_action = None
            self._reset_real_tracking_hold_locked()
            self._reset_request_latency_locked()
            self._generation += 1
            logger.info(
                "configured RobotClient command path for %s "
                "(publish_to_robot=%s action_request_mode=%s "
                "control_hz=%g inference_hz=%g chunk_align_window_s=%g "
                "initial_pose_sync=%s initial_pose_sync_duration_s=%g)",
                robot_type,
                self._publish_to_robot,
                self._action_request_mode,
                self._control_hz,
                self._inference_hz,
                self._chunk_align_window_s,
                self._initial_pose_sync_enabled,
                self._initial_pose_sync_duration_s,
            )

    def deconfigure(self) -> None:
        with self._lock:
            self._reset_temporal_ensemble_locked()
            self._running = False
            self._task_instruction = ""
            self._action_keys = []
            self._model_action_keys = None
            self._publish_to_robot = False
            self._initial_pose_sync_enabled = False
            self._initial_pose_sync_duration_s = 5.0
            self._initial_pose_sync_in_progress = False
            self._initial_pose_sync_completed = False
            self._initial_pose_sync_deadline = None
            self._initial_pose_sync_hold_pending = False
            self._refill_margin_s = self._default_refill_margin_s
            self._source_chunk_limit = None
            self._real_first_action_pending = True
            self._real_first_publish_pending = True
            self._real_last_published_action = None
            self._reset_real_tracking_hold_locked()
            self._action_request_mode = self._default_action_request_mode
            self._processor = None
            self._generation += 1
            if self._robot is not None:
                self._robot.close()
                self._robot = None
            self._reset_request_latency_locked()

    def initial_pose_sync_required(self, publish_to_robot: bool) -> bool:
        with self._lock:
            return (
                self._initial_pose_sync_enabled
                and bool(publish_to_robot)
                and not self._initial_pose_sync_completed
            )

    def initial_pose_sync_hold_required(self) -> bool:
        with self._lock:
            return (
                self._initial_pose_sync_in_progress
                or self._initial_pose_sync_hold_pending
            )

    def start(self, publish_to_robot: Optional[bool] = None, *, continuation: bool = False) -> bool:
        with self._lock:
            if self._initial_pose_sync_hold_pending:
                raise RuntimeError(
                    "initial pose sync hold is still pending - STOP again first"
                )
            target_publish_mode = (
                self._publish_to_robot
                if publish_to_robot is None
                else bool(publish_to_robot)
            )
            self._require_real_safety_if_needed(target_publish_mode)
            if target_publish_mode and self._real_action_safety_enabled:
                if self._robot is None or self._processor is None:
                    raise RuntimeError("control loop is not configured")
                self._validate_real_action_contract(
                    self._robot,
                    self._real_contract_action_keys_locked(),
                )
            if publish_to_robot is not None:
                self._set_publish_to_robot_locked(bool(publish_to_robot))
            should_sync = (
                self._initial_pose_sync_enabled
                and self._publish_to_robot
                and not self._initial_pose_sync_completed
            )
            if should_sync:
                if self._robot is None or self._processor is None:
                    raise RuntimeError("control loop is not configured")
                robot = self._robot
                processor = self._processor
                task_instruction = self._task_instruction
                action_keys = list(self._action_keys)
                duration_s = self._initial_pose_sync_duration_s
                generation = self._generation
                self._running = False
                self._initial_pose_sync_in_progress = False
                self._initial_pose_sync_deadline = None
            else:
                self._running = True
                self._diagnostic_trace.record(
                    'start', generation=self._generation,
                    publish_to_robot=self._publish_to_robot,
                    mode=self._action_request_mode,
                )
                return False

        started_at = time.monotonic()
        response = self._requester.get_action(task_instruction)
        self._record_request_latency(time.monotonic() - started_at)
        chunk = self._action_chunk_from_response(response, robot, action_keys)
        target = chunk[:1]
        if self._real_action_safety_enabled:
            warm_limits = None
            if self._real_warm_start_enabled:
                warm_limits = dict(self._real_warm_start_max_total_delta_by_key)
                if continuation:
                    warm_limits.update(self._real_tracking_max_delta_by_key)
                    warm_limits.update(self._real_tracking_bridge_max_total_delta_by_key)
                    warm_limits.update(self._real_resume_warm_start_max_total_delta_by_key)
                sim_hand_limits = self._simulation_initial_pose_sync_hand_max_delta_by_key
                if sim_hand_limits:
                    if getattr(robot, "_robot_type", None) != "ffw_sh5_rev1":
                        raise ValueError("simulation hand initial pose sync supports only ffw_sh5_rev1")
                    if duration_s < 5.0:
                        raise ValueError("simulation full-stroke hand initial pose sync requires at least 5 seconds")
                    warm_limits.update(sim_hand_limits)
            prepared = self._apply_real_safety(
                robot,
                target,
                action_keys,
                check_first_action=True,
                warm_start=warm_limits is not None,
                warm_start_max_total_delta_by_key_override=warm_limits,
                source_step_bridge=self._real_source_step_bridge_enabled,
            )
            target = prepared[-1:]

        with self._lock:
            if (
                generation != self._generation
                or robot is not self._robot
                or processor is not self._processor
            ):
                raise RuntimeError("initial pose sync cancelled")
            processor.clear()
            self._reset_temporal_ensemble_locked()
            try:
                robot.publish_initial_pose_sync(
                    target[0],
                    action_keys,
                    duration_s=duration_s,
                )
            except Exception:
                self._running = False
                self._initial_pose_sync_in_progress = False
                self._initial_pose_sync_deadline = None
                raise
            self._initial_pose_sync_in_progress = True
            self._initial_pose_sync_deadline = time.monotonic() + duration_s
            self._running = True
            self._diagnostic_trace.record(
                'initial_pose_sync', generation=self._generation,
                duration_s=duration_s,
            )
            logger.info(
                "initial pose sync target published: duration=%.3fs; "
                "discarded source chunk=%d",
                duration_s,
                len(chunk),
            )
            return True

    def preflight_start(
        self,
        publish_to_robot: bool,
        task_instruction: Optional[str] = None,
        *,
        continuation: bool = False,
    ) -> None:
        """Synchronously validate the first Real chunk without publishing it.

        The service handler calls this before acknowledging START/RESUME.  A
        validation error therefore becomes the service response shown by the
        UI, while Simulation and profiles with safety disabled remain fully
        asynchronous.
        """
        self._require_real_safety_if_needed(bool(publish_to_robot))
        if not publish_to_robot or not self._real_action_safety_enabled:
            return
        with self._lock:
            if self._robot is None or self._processor is None:
                raise RuntimeError("control loop is not configured")
            self._running = False
            self._set_publish_to_robot_locked(True)
            self._processor.clear()
            self._reset_temporal_ensemble_locked()
            self._real_first_action_pending = True
            self._real_first_publish_pending = True
            self._real_last_published_action = None
            self._reset_real_tracking_hold_locked()
            robot = self._robot
            preflight_generation = self._generation
            action_keys = list(self._action_keys)
            contract_action_keys = self._real_contract_action_keys_locked()
            instruction = (
                self._task_instruction
                if task_instruction is None
                else str(task_instruction or "")
            )
            warm_start_max_total_delta_by_key = (
                dict(self._real_warm_start_max_total_delta_by_key)
                if self._real_warm_start_enabled
                else None
            )
            if continuation and warm_start_max_total_delta_by_key is not None:
                # A resumed policy can legitimately be behind the follower by
                # the recorded command-to-state tracking envelope.  Reuse the
                # audited per-group tracking limits for total eligibility and
                # the wider arm-only state-bridge hard caps where present.
                # RobotClient still interpolates every emitted joint command
                # through the much smaller source-step limit.
                warm_start_max_total_delta_by_key.update(
                    self._real_tracking_max_delta_by_key
                )
                warm_start_max_total_delta_by_key.update(
                    self._real_tracking_bridge_max_total_delta_by_key
                )
                # Some approved follower bringup poses are intentionally
                # outside the recorded command-to-state tracking envelope
                # (notably an open hand versus a learned closed-hand first
                # action).  Keep that case separate from ordinary tracking:
                # only RESUME preflight may use this explicitly audited
                # envelope, and RobotClient still bounds every emitted step.
                warm_start_max_total_delta_by_key.update(
                    self._real_resume_warm_start_max_total_delta_by_key
                )

        try:
            self._validate_real_action_contract(robot, contract_action_keys)
            observed_at = time.monotonic()
            response = self._requester.get_action(instruction)
            chunk = self._action_chunk_from_response(
                response,
                robot,
                action_keys,
            )
            prepared = self._apply_real_safety(
                robot,
                chunk,
                action_keys,
                check_first_action=True,
                warm_start=(warm_start_max_total_delta_by_key is not None),
                warm_start_max_total_delta_by_key_override=(
                    warm_start_max_total_delta_by_key
                ),
                source_step_bridge=self._real_source_step_bridge_enabled,
            )
        except Exception as exc:
            with self._lock:
                self._reject_action_chunk_locked(str(exc))
            raise RuntimeError(f"Real action preflight blocked: {exc}") from exc
        with self._lock:
            if (self._robot is not robot or self._processor is None
                    or self._generation != preflight_generation):
                raise RuntimeError(
                    "Real action preflight invalidated by reconfiguration"
                )
            # The first plan is unchanged by averaging, and has already passed
            # safety. Seed history only after accepting this generation.
            self._ensemble_chunk_locked(chunk, observed_at)
            produced = self._processor.push_actions(
                prepared,
                scheduled_start_delay_s=None,
                align=False,
            )
            if self._diagnostic_trace.enabled:
                self._diagnostic_trace.record(
                    'chunk', generation=self._generation, preflight=True,
                    raw=chunk.tolist(), prepared=prepared.tolist(),
                    ensemble=self._ensemble_info_locked(), observed_at=observed_at,
                    selection=self._processor.last_push_info,
                    produced=produced, buffer_after=self._processor.buffer_size)
            if produced <= 0:
                self._reject_action_chunk_locked(
                    "preflight produced no buffered actions"
                )
                raise RuntimeError(
                    "Real action preflight blocked: preflight produced no "
                    "buffered actions"
                )
            # The exact chunk accepted above is now the first chunk that start
            # will consume.  Do not request and validate a second, potentially
            # different ACT sample before publishing begins.
            self._real_first_action_pending = False
        logger.info(
            "Real action preflight passed (no command published; "
            "continuation=%s source_steps=%d prepared_steps=%d "
            "buffered_steps=%d)",
            continuation,
            len(chunk),
            len(prepared),
            produced,
        )

    def pause(self) -> bool:
        robot = None
        action_keys: list[str] = []
        with self._lock:
            should_hold = (
                (
                    self._initial_pose_sync_in_progress
                    or self._initial_pose_sync_hold_pending
                )
                and self._publish_to_robot
                and self._robot is not None
            )
            if should_hold:
                robot = self._robot
                action_keys = list(self._action_keys)
            self._reset_temporal_ensemble_locked()
            self._running = False
            if self._processor is not None:
                self._processor.clear()
            self._real_first_action_pending = True
            self._real_first_publish_pending = True
            self._real_last_published_action = None
            self._reset_real_tracking_hold_locked()
            self._initial_pose_sync_deadline = None
            if should_hold:
                self._initial_pose_sync_hold_pending = True
            else:
                self._initial_pose_sync_in_progress = False
                self._initial_pose_sync_hold_pending = False
            self._generation += 1
        if robot is not None:
            try:
                robot.publish_current_pose_hold(action_keys, duration_s=0.1)
                logger.info(
                    "initial pose sync interrupted; current pose hold published"
                )
            except Exception as error:
                with self._lock:
                    if robot is self._robot:
                        self._initial_pose_sync_in_progress = True
                        self._initial_pose_sync_hold_pending = True
                logger.error(
                    "failed to hold current pose during sync pause: %s", error
                )
                return False
            with self._lock:
                if robot is not self._robot:
                    return False
                self._initial_pose_sync_in_progress = False
                self._initial_pose_sync_hold_pending = False
        return True

    def stop(self) -> bool:
        return self.pause()

    def set_publish_to_robot(self, publish_to_robot: bool) -> None:
        with self._lock:
            self._require_real_safety_if_needed(bool(publish_to_robot))
            self._set_publish_to_robot_locked(bool(publish_to_robot))

    def _set_publish_to_robot_locked(self, publish_to_robot: bool) -> None:
        if self._publish_to_robot == publish_to_robot:
            return
        self._publish_to_robot = publish_to_robot
        self._reset_temporal_ensemble_locked()
        if self._processor is not None:
            self._processor.clear()
        self._real_first_action_pending = True
        self._real_first_publish_pending = True
        self._real_last_published_action = None
        self._reset_real_tracking_hold_locked()
        self._generation += 1

    def set_task_instruction(self, task_instruction: str) -> None:
        with self._lock:
            self._task_instruction = task_instruction or ""

    def run_background(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self.run, daemon=True)
        self._thread.start()

    def run(self) -> None:
        next_t = time.monotonic()
        while not self._shutdown.is_set():
            period = self._tick_period()
            self.tick()
            next_t += period
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()

    def shutdown(self) -> None:
        self._shutdown.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self.deconfigure()
        self._diagnostic_trace.close()

    def tick(self) -> None:
        with self._lock:
            if not self._running or self._robot is None or self._processor is None:
                return
            robot = self._robot
            processor = self._processor
            task_instruction = self._task_instruction
            action_keys = list(self._action_keys)
            generation = self._generation
            publish_to_robot = self._publish_to_robot
            action_request_mode = self._action_request_mode

            if self._initial_pose_sync_in_progress:
                deadline = self._initial_pose_sync_deadline
                if deadline is not None and time.monotonic() < deadline:
                    if publish_to_robot:
                        try:
                            robot.publish_idle_action(action_keys)
                        except Exception as error:
                            logger.error(
                                "failed to publish idle action during pose sync: %s",
                                error,
                            )
                    return
                self._initial_pose_sync_in_progress = False
                self._initial_pose_sync_completed = True
                self._initial_pose_sync_deadline = None
                logger.info(
                    "initial pose sync complete; requesting a fresh action chunk"
                )

            action = processor.pop_action()
            if action is not None:
                desired_action = np.asarray(
                    action,
                    dtype=np.float64,
                ).copy()
                defer_desired_action = False
                real_publish_guard = (
                    publish_to_robot and self._real_action_safety_enabled
                )
                if real_publish_guard:
                    try:
                        first_publish_guard = self._real_first_publish_pending
                        action = self._apply_real_safety(
                            robot,
                            desired_action.reshape(1, -1),
                            action_keys,
                            check_first_action=first_publish_guard,
                            check_tracking_action=not first_publish_guard,
                            state_tracking_bridge=not first_publish_guard,
                            previous_published_action=(
                                None
                                if first_publish_guard
                                else self._real_last_published_action
                            ),
                        )[0]
                        defer_desired_action = (
                            not first_publish_guard
                            and not np.allclose(
                                action,
                                desired_action,
                                rtol=0.0,
                                atol=1e-12,
                            )
                        )
                        if defer_desired_action:
                            if not callable(getattr(processor, "defer_action", None)):
                                raise RuntimeError(
                                    "ActionChunkProcessor does not support "
                                    "tracking backpressure"
                                )
                            self._update_real_tracking_hold_locked(
                                desired_action,
                                np.asarray(action, dtype=np.float64),
                            )
                        else:
                            self._finish_real_tracking_hold_locked()
                    except Exception as exc:
                        self._reject_action_chunk_locked(
                            f"publish-time tracking revalidation failed: {exc}"
                        )
                        action = None
                if action is not None:
                    preview = getattr(robot, "publish_action_preview", None)
                    if callable(preview):
                        try:
                            preview(action, action_keys)
                        except Exception as e:
                            logger.warning("failed to publish action preview: %s", e)
                if action is not None and publish_to_robot:
                    try:
                        robot.publish_action(action, action_keys)
                        self._trace_publish(robot, action_keys, desired_action, action,
                                            defer_desired_action, processor.buffer_size)
                        if real_publish_guard:
                            self._real_last_published_action = np.asarray(
                                action,
                                dtype=np.float64,
                            ).copy()
                            if defer_desired_action:
                                processor.defer_action(
                                    desired_action,
                                    published_action=action,
                                )
                        if self._real_first_publish_pending:
                            self._real_first_publish_pending = False
                    except Exception as e:
                        if real_publish_guard:
                            self._reject_action_chunk_locked(
                                f"robot publish failed: {e}"
                            )
                        else:
                            logger.error("failed to publish robot action: %s", e)
            elif publish_to_robot:
                self._diagnostic_trace.record('buffer_empty', generation=generation)
                idle = getattr(robot, "publish_idle_action", None)
                if callable(idle):
                    try:
                        idle(action_keys)
                    except Exception as e:
                        if self._real_action_safety_enabled:
                            self._reject_action_chunk_locked(
                                f"idle robot publish failed: {e}"
                            )
                        else:
                            logger.error("failed to publish idle robot action: %s", e)

            should_request = self._running and self._should_request_actions(processor)

        if should_request:
            self._request_thread = threading.Thread(
                target=self._request_and_buffer,
                args=(task_instruction, generation, action_request_mode),
                daemon=True,
            )
            self._request_thread.start()

    def _request_and_buffer(
        self,
        task_instruction: str,
        generation: int,
        action_request_mode: str = ACTION_REQUEST_MODE_ASYNC,
    ) -> None:
        action_request_mode = normalize_action_request_mode(action_request_mode)
        started_at = time.monotonic()
        try:
            response = self._requester.get_action(task_instruction)
        except Exception as e:
            latency_s = time.monotonic() - started_at
            self._record_request_latency(latency_s)
            self._reject_action_chunk(f"get_action raised: {e}", generation)
            return
        latency_s = time.monotonic() - started_at
        self._record_request_latency(latency_s)
        try:
            with self._lock:
                robot = self._robot
                action_keys = list(self._action_keys)
                contract_action_keys = self._real_contract_action_keys_locked()
            chunk = self._action_chunk_from_response(
                response,
                robot,
                action_keys,
            )
        except Exception as exc:
            self._reject_action_chunk(str(exc), generation)
            return
        with self._lock:
            if (
                generation == self._generation
                and self._running
                and self._processor is not None
            ):
                robot = self._robot
                action_keys = list(self._action_keys)

                real_safety_active = (
                    self._publish_to_robot and self._real_action_safety_enabled
                )
                first_action_guard = (
                    real_safety_active and self._real_first_action_pending
                )
                raw_trace_chunk = chunk.tolist() if self._diagnostic_trace.enabled else None
                raw_chunk = chunk.copy()
                # A postprocessing limit may smooth ensemble-generated edges,
                # but must never conceal an unsafe raw model prediction. This
                # validation uses the existing raw-step caps and joint limits;
                # keep the raw time grid, not its expanded safety bridge, for
                # time-aligned averaging. Preflight already validates its raw
                # first plan before seeding ensemble history.
                if real_safety_active and self._temporal_ensemble is not None:
                    try:
                        if robot is None:
                            raise ValueError('RobotClient is unavailable for real action safety')
                        self._apply_real_safety(
                            robot, raw_chunk, action_keys, check_first_action=False,
                            source_step_bridge=self._real_source_step_bridge_enabled,
                        )
                    except Exception as exc:
                        self._reject_prediction_chunk_locked(
                            str(exc), raw_chunk, None, 'raw_model', started_at)
                        return
                try:
                    chunk = self._ensemble_chunk_locked(chunk, started_at)
                except ValueError as exc:
                    self._reject_prediction_chunk_locked(
                        str(exc), raw_chunk, None, 'ensemble', started_at)
                    return
                ensemble_trace_chunk = (
                    chunk.tolist() if self._diagnostic_trace.enabled
                    and self._temporal_ensemble is not None else None
                )
                if real_safety_active:
                    if robot is None:
                        self._reject_action_chunk_locked(
                            "RobotClient is unavailable for real action safety"
                        )
                        return
                    try:
                        if first_action_guard:
                            self._validate_real_action_contract(
                                robot,
                                contract_action_keys,
                            )
                        chunk = self._apply_real_safety(
                            robot,
                            chunk,
                            action_keys,
                            check_first_action=first_action_guard,
                            warm_start=(
                                first_action_guard
                                and self._real_warm_start_enabled
                            ),
                            source_step_bridge=(
                                self._real_source_step_bridge_enabled
                            ),
                        )
                    except Exception as exc:
                        self._reject_prediction_chunk_locked(
                            str(exc), raw_chunk, chunk, 'post_ensemble_safety', started_at)
                        return

                buffer_before = self._processor.buffer_size
                buffer_delay_s = buffer_before / max(
                    1.0,
                    self._processor.output_hz,
                )
                scheduled_start_delay_s = (
                    None
                    if action_request_mode
                    in {
                        ACTION_REQUEST_MODE_ASYNC_ORDERED,
                        ACTION_REQUEST_MODE_SYNC,
                        ACTION_REQUEST_MODE_SYNC_STEP,
                    }
                    else latency_s + buffer_delay_s
                )
                produced = self._processor.push_actions(
                    chunk,
                    scheduled_start_delay_s=scheduled_start_delay_s,
                    align=(action_request_mode == ACTION_REQUEST_MODE_ASYNC),
                )
                buffer_after = self._processor.buffer_size
                if self._diagnostic_trace.enabled:
                    self._diagnostic_trace.record(
                        'chunk', generation=generation, preflight=False,
                        latency_s=latency_s, raw=raw_trace_chunk, prepared=chunk.tolist(),
                        ensembled=ensemble_trace_chunk,
                        ensemble=self._ensemble_info_locked(), observed_at=started_at,
                        selection=getattr(self._processor, 'last_push_info', {}),
                        produced=produced, buffer_before=buffer_before, buffer_after=buffer_after)
                if produced > 0 and first_action_guard:
                    self._real_first_action_pending = False
                scheduled_start_text = (
                    "none"
                    if scheduled_start_delay_s is None
                    else f"{scheduled_start_delay_s:.3f}s"
                )
                log_buffered = (
                    logger.info
                    if action_request_mode
                    in {
                        ACTION_REQUEST_MODE_ASYNC_ORDERED,
                        ACTION_REQUEST_MODE_SYNC_STEP,
                    }
                    else logger.debug
                )
                log_buffered(
                    "buffered action chunk: source=%d produced=%d "
                    "buffer=%d->%d mode=%s latency=%.3fs buffer_delay=%.3fs "
                    "scheduled_start=%s",
                    response.chunk_size,
                    produced,
                    buffer_before,
                    buffer_after,
                    action_request_mode,
                    latency_s,
                    buffer_delay_s,
                    scheduled_start_text,
                )

    def _reset_temporal_ensemble_locked(self):
        if self._temporal_ensemble is not None:
            self._temporal_ensemble.reset()

    def _ensemble_info_locked(self):
        return ({'enabled': False} if self._temporal_ensemble is None
                else dict(self._temporal_ensemble.last_info))

    def _ensemble_chunk_locked(self, chunk, observed_at):
        if self._temporal_ensemble is None:
            return chunk
        result = self._temporal_ensemble.update(chunk, observed_at)
        info = self._temporal_ensemble.last_info
        info['step_limit_rad'] = self._real_source_step_max_delta_rad
        info['limited_values'] = 0
        if info['active_plans'] > 1 and self._real_source_step_max_delta_rad is not None:
            limited = limit_source_steps(result, self._real_source_step_max_delta_rad)
            info['limited_values'] = int(np.count_nonzero(np.abs(limited - result) > 1e-12))
            info['unlimited_max_step_rad'] = float(np.max(np.abs(np.diff(result, axis=0)))) if len(result)>1 else 0.0
            result = limited
        logger.info('Temporal ensemble: plans=%d overlap=%d..%d coefficient=%g limited_values=%d',
                    info['active_plans'], info['overlap_min'],
                    info['overlap_max'], info['coefficient'], info['limited_values'])
        return result

    def _reject_prediction_chunk_locked(self, reason, raw, ensembled, stage, observed_at):
        info = self._ensemble_info_locked()
        # Stop publishing before writing the bounded failure snapshot. Keep
        # this independent of the high-rate trace's 100 MB recording limit.
        self._reject_action_chunk_locked(reason)
        self._diagnostic_trace.record_rejection(
            reason=reason, stage=stage, observed_at=observed_at,
            raw=raw.tolist(), ensembled=None if ensembled is None else ensembled.tolist(),
            ensemble=info,
        )

    def _trace_publish(self, robot, action_keys, desired, published, deferred, buffer_size):
        if not self._diagnostic_trace.enabled:
            return
        try:
            published_at = time.monotonic()
            states = {}
            for key in action_keys:
                position, timestamp = robot.get_joint_position_snapshot(f'follower_{key}')
                states[key] = {'positions': position.tolist(), 'received_at': timestamp}
            self._diagnostic_trace.record(
                'publish', generation=self._generation, desired=desired.tolist(),
                published=published.tolist(), deferred=bool(deferred),
                buffer_size=buffer_size, measured=states, published_at=published_at)
        except Exception:
            # Observation for diagnostics must not prevent a valid command.
            logger.debug('Could not capture publish diagnostic', exc_info=True)

    @staticmethod
    def _action_chunk_from_response(response, robot, action_keys: list[str]) -> np.ndarray:
        if not bool(getattr(response, "success", False)):
            raise ValueError(
                f"get_action failed: {getattr(response, 'message', '')}"
            )
        try:
            chunk_size = int(response.chunk_size)
            action_dim = int(response.action_dim)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"invalid action shape metadata: {exc}") from exc
        if chunk_size <= 0 or action_dim <= 0:
            raise ValueError("get_action returned an empty chunk")
        try:
            data = np.asarray(response.action_list, dtype=np.float64)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(
                f"action list is not a numeric float array: {exc}"
            ) from exc
        expected_size = chunk_size * action_dim
        if data.size != expected_size:
            raise ValueError(
                f"action list size mismatch: {data.size} != "
                f"{chunk_size} * {action_dim}"
            )
        chunk = data.reshape(chunk_size, action_dim)
        if not np.isfinite(chunk).all():
            raise ValueError("action chunk contains NaN or Inf")

        expected_dim_fn = getattr(robot, "action_dimension", None)
        if callable(expected_dim_fn):
            try:
                expected_action_dim = int(expected_dim_fn(action_keys))
            except Exception as exc:
                raise ValueError(
                    f"failed to resolve robot action dimension: {exc}"
                ) from exc
            if expected_action_dim <= 0 or action_dim != expected_action_dim:
                raise ValueError(
                    "model/robot action dimension mismatch: "
                    f"model={action_dim}, robot={expected_action_dim}"
                )
        return chunk

    def _apply_real_safety(
        self,
        robot,
        chunk: np.ndarray,
        action_keys: list[str],
        *,
        check_first_action: bool,
        check_tracking_action: bool = False,
        warm_start: bool = False,
        warm_start_max_total_delta_by_key_override: Optional[
            dict[str, float]
        ] = None,
        source_step_bridge: bool = False,
        state_tracking_bridge: bool = False,
        previous_published_action: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        apply_safety = getattr(robot, "apply_real_action_safety", None)
        if not callable(apply_safety):
            raise ValueError("RobotClient does not provide real action safety")
        return apply_safety(
            chunk,
            action_keys,
            first_action_max_delta_rad=(
                self._real_first_action_max_delta_rad
                if check_first_action
                else (
                    self._real_tracking_max_delta_rad
                    if check_tracking_action
                    else None
                )
            ),
            first_action_max_delta_by_key=(
                self._real_first_action_max_delta_by_key
                if check_first_action
                else (
                    self._real_tracking_max_delta_by_key
                    if check_tracking_action
                    else None
                )
            ),
            warm_start_max_total_delta_by_key=(
                (
                    warm_start_max_total_delta_by_key_override
                    if warm_start_max_total_delta_by_key_override is not None
                    else self._real_warm_start_max_total_delta_by_key
                )
                if warm_start
                else None
            ),
            state_bridge_max_total_delta_by_key=(
                self._real_tracking_bridge_max_total_delta_by_key
                if state_tracking_bridge
                else None
            ),
            previous_published_action=previous_published_action,
            source_step_max_delta_rad=self._real_source_step_max_delta_rad,
            source_step_bridge_max_raw_delta_by_key=(
                self._real_source_step_bridge_max_raw_delta_by_key
                if source_step_bridge
                else None
            ),
            state_max_age_s=self._real_state_max_age_s,
            joint_limit_mode=self._real_joint_limit_mode,
            joint_limit_tolerance_rad=self._real_joint_limit_tolerance_rad,
            joint_limit_tolerance_by_joint=(
                self._real_joint_limit_tolerance_by_joint
            ),
        )

    @staticmethod
    def _validate_real_action_contract(robot, action_keys: list[str]) -> None:
        validate = getattr(robot, "validate_real_action_contract", None)
        if not callable(validate):
            raise ValueError(
                "RobotClient does not provide real action contract validation"
            )
        validate(action_keys)

    def _require_real_safety_if_needed(self, publish_to_robot: bool) -> None:
        if (
            publish_to_robot
            and self._real_action_safety_required
            and not self._real_action_safety_enabled
        ):
            raise RuntimeError(
                "Real robot deploy is disabled because required action safety "
                "is not enabled"
            )

    def _real_contract_action_keys_locked(self) -> list[str]:
        if self._model_action_keys is not None:
            return list(self._model_action_keys)
        return list(self._action_keys)

    def _reject_action_chunk(self, reason: str, generation: int) -> None:
        with self._lock:
            if generation != self._generation:
                return
            self._reject_action_chunk_locked(reason)

    def _reject_action_chunk_locked(self, reason: str) -> None:
        self._reset_temporal_ensemble_locked()
        if self._publish_to_robot and self._real_action_safety_enabled:
            was_running = self._running
            self._running = False
            if self._processor is not None:
                self._processor.clear()
            self._real_first_action_pending = True
            self._real_first_publish_pending = True
            self._real_last_published_action = None
            self._reset_real_tracking_hold_locked()
            self._generation += 1
            logger.error(
                "REAL ACTION BLOCKED (loop stopped, buffer cleared): %s",
                reason,
            )
            if was_running and self._real_action_blocked_callback is not None:
                try:
                    self._real_action_blocked_callback(reason)
                except Exception as exc:
                    logger.error(
                        "failed to report real action block: %s",
                        exc,
                    )
            return
        logger.warning("action chunk rejected: %s", reason)

    def _update_real_tracking_hold_locked(
        self,
        desired_action: np.ndarray,
        published_action: np.ndarray,
    ) -> None:
        desired = np.asarray(desired_action, dtype=np.float64).reshape(-1)
        published = np.asarray(published_action, dtype=np.float64).reshape(-1)
        remaining = float(np.max(np.abs(desired - published)))
        now = time.monotonic()
        same_target = (
            self._real_tracking_hold_action is not None
            and self._real_tracking_hold_action.shape == desired.shape
            and np.array_equal(self._real_tracking_hold_action, desired)
        )
        if not same_target:
            self._real_tracking_hold_action = desired.copy()
            self._real_tracking_hold_best_remaining = remaining
            self._real_tracking_hold_last_progress_at = now
            logger.debug(
                "Holding action buffer for follower tracking "
                "(remaining=%.6f rad)",
                remaining,
            )
            return

        best = self._real_tracking_hold_best_remaining
        if (
            best is None
            or remaining
            <= best - self._REAL_TRACKING_HOLD_PROGRESS_EPS_RAD
        ):
            self._real_tracking_hold_best_remaining = remaining
            self._real_tracking_hold_last_progress_at = now
            return

        last_progress_at = self._real_tracking_hold_last_progress_at
        if (
            last_progress_at is not None
            and now - last_progress_at
            > self._REAL_TRACKING_HOLD_MAX_NO_PROGRESS_S
        ):
            raise ValueError(
                "state-tracking hold made no progress for "
                f"{now - last_progress_at:.3f}s "
                f"(remaining={remaining:.6f} rad, "
                f"best_remaining={best:.6f} rad, "
                f"progress_eps={self._REAL_TRACKING_HOLD_PROGRESS_EPS_RAD:.6f} rad); "
                + self._describe_real_tracking_hold_locked(desired, published)
            )

    def _describe_real_tracking_hold_locked(
        self,
        desired: np.ndarray,
        bounded: np.ndarray,
    ) -> str:
        """Describe the blocked candidate without changing the safety decision.

        This runs only after the stall deadline. Cached follower snapshots are
        read after the guard, so they need not be the samples used by the guard.
        The bounded candidate on this failing tick has NOT been published.
        Diagnostic failures must never replace the original stall reason.
        """
        try:
            robot = self._robot
            layout = []
            for key in self._action_keys:
                width = robot.action_dimension([key])
                group = f"follower_{key}"
                names = list(robot.get_joint_names(group))
                if width <= 0 or len(names) != width:
                    raise ValueError("tracking diagnostic joint layout mismatch")
                layout.extend((key, group, name, j) for j, name in enumerate(names))
            if len(layout) != desired.size or bounded.shape != desired.shape:
                raise ValueError("tracking diagnostic action width mismatch")

            differences = np.abs(desired - bounded)
            indices = np.argsort(-differences, kind="stable")[:3]
            snapshots = {}
            descriptions = []
            for index in indices:
                if differences[index] <= 1e-12:
                    continue
                key, group, name, joint_index = layout[index]
                if group not in snapshots:
                    positions, timestamp = robot.get_joint_position_snapshot(group)
                    snapshots[group] = (
                        np.asarray(positions, dtype=np.float64).reshape(-1),
                        timestamp,
                    )
                positions, timestamp = snapshots[group]
                detail = (
                    f"joint={name} action_key={key} "
                    f"desired={desired[index]:.6f} "
                    f"bounded_candidate={bounded[index]:.6f} "
                    f"remaining={differences[index]:.6f} rad"
                )
                previous = self._real_last_published_action
                if previous is not None and previous.shape == desired.shape:
                    detail += f" previous_published={previous[index]:.6f}"
                if joint_index < positions.size and np.isfinite(positions[joint_index]):
                    measured = float(positions[joint_index])
                    detail += (
                        f" measured={measured:.6f} "
                        f"desired_state_delta={abs(desired[index] - measured):.6f} "
                        f"candidate_state_delta={abs(bounded[index] - measured):.6f} rad"
                    )
                else:
                    detail += " measured=unavailable"
                if timestamp is not None and math.isfinite(float(timestamp)):
                    detail += f" state_age={time.time() - float(timestamp):.3f}s"
                descriptions.append(detail)
            return "snapshot=after_guard; " + "; ".join(descriptions)
        except Exception as exc:
            return f"tracking_diagnostics_unavailable={type(exc).__name__}"

    def _finish_real_tracking_hold_locked(self) -> None:
        if self._real_tracking_hold_action is not None:
            logger.debug("Follower tracking recovered; advancing action buffer")
        self._reset_real_tracking_hold_locked()

    def _reset_real_tracking_hold_locked(self) -> None:
        self._real_tracking_hold_action = None
        self._real_tracking_hold_best_remaining = None
        self._real_tracking_hold_last_progress_at = None

    def _should_request_actions(self, processor: ActionChunkProcessor) -> bool:
        if self._request_thread is not None and self._request_thread.is_alive():
            return False
        if self._action_request_mode == ACTION_REQUEST_MODE_ASYNC_ORDERED:
            # Keep exactly one coherent decoder trajectory active. Start the
            # next CUDA request near the current tail, but append its complete
            # ordered result without latency/L2 source-row skipping. This
            # avoids both stale multi-chunk look-ahead and high-rate replans.
            return processor.buffer_size <= self._refill_threshold(
                processor,
                margin_s=self._ordered_async_refill_margin_s,
            )
        if self._action_request_mode == ACTION_REQUEST_MODE_SYNC_STEP:
            # Compatibility path for SH5 TactileACT's proven smooth run.
            # Sample the next observation only after the selected source
            # action has drained.  The request latency intentionally becomes
            # a hold at the last reference instead of a stale prefetch; this
            # preserves the slow, damped physical loop that previously
            # completed task motion without 0.13-second boundary chatter.
            return processor.buffer_size <= 0
        if self._action_request_mode == ACTION_REQUEST_MODE_SYNC:
            # Sequential mode keeps chunk order and disables time/L2 skipping,
            # but it must not wait for an empty 100 Hz output buffer before
            # starting a CUDA request.  ACT inference can exceed one 30 Hz
            # source period.  A small bounded look-ahead hides that latency;
            # append order remains deterministic under the single in-flight
            # request guard above.
            return processor.buffer_size <= self._refill_threshold(processor)
        return processor.buffer_size < self._refill_threshold(processor)

    def _refill_threshold(
        self,
        processor: ActionChunkProcessor,
        *,
        margin_s: Optional[float] = None,
    ) -> int:
        threshold_s = max(
            0.0,
            self._refill_margin_s if margin_s is None else float(margin_s),
        )
        if self._request_latency_ema_s is not None:
            threshold_s += max(0.0, self._request_latency_ema_s)
        return max(1, int(math.ceil(threshold_s * processor.output_hz)))

    def _record_request_latency(self, latency_s: float) -> None:
        latency_s = max(0.0, float(latency_s))
        with self._lock:
            if self._latency_warmup_remaining > 0:
                self._latency_warmup_remaining -= 1
                return
            if (
                self._max_refill_latency_s is not None
                and latency_s > self._max_refill_latency_s
            ):
                logger.debug(
                    "ignoring GET_ACTION latency sample %.3fs above %.3fs",
                    latency_s,
                    self._max_refill_latency_s,
                )
                return
            if self._request_latency_ema_s is None:
                self._request_latency_ema_s = latency_s
            else:
                alpha = self._request_latency_alpha
                self._request_latency_ema_s = (
                    alpha * latency_s
                    + (1.0 - alpha) * self._request_latency_ema_s
                )

    def _reset_request_latency_locked(self) -> None:
        self._request_latency_ema_s = None
        self._latency_warmup_remaining = self._latency_warmup_samples

    def _tick_period(self) -> float:
        with self._lock:
            if self._processor is None:
                hz = self._control_hz
            else:
                hz = self._processor.output_hz
        return 1.0 / max(1.0, hz)

    @staticmethod
    def _positive_optional_float(
        name: str,
        value: Optional[float],
    ) -> Optional[float]:
        if value is None:
            return None
        result = float(value)
        if not math.isfinite(result) or result <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return result
