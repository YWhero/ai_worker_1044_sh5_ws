#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0

"""External InferenceCommand handler for the Main process."""

from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path
from typing import List, Optional


CMD_LOAD, CMD_START, CMD_PAUSE, CMD_RESUME, CMD_STOP, CMD_UNLOAD = 0, 1, 2, 3, 4, 5
CMD_UPDATE_INSTRUCTION = 6
CMD_RESET_CYCLE = 7
CMD_SWITCH_POLICY = 8
CMD_PRELOAD_POLICY = 9
CMD_CLEAR_PRELOAD = 10
CMD_SWITCH_PRELOADED = 11


class ServiceHandler:
    def __init__(
        self,
        session,
        requester,
        control_loop,
        response_factory,
        real_start_deadline_s: float = 15.0,
    ):
        self._session = session
        self._requester = requester
        self._control_loop = control_loop
        self._response_factory = response_factory
        self._real_start_deadline_s = float(real_start_deadline_s)
        if (
            not math.isfinite(self._real_start_deadline_s)
            or self._real_start_deadline_s <= 0.0
        ):
            raise ValueError("real_start_deadline_s must be finite and positive")
        self._lifecycle_lock = threading.RLock()
        self._switch_failed = False

    def handle(self, request):
        # Capture before waiting for the lifecycle lock so a queued Real
        # request cannot become valid again after its caller has timed out.
        received_monotonic = time.monotonic()
        with self._lifecycle_lock:
            return self._handle_locked(request, received_monotonic)

    def _handle_locked(self, request, received_monotonic: float):
        cmd = int(request.command)
        try:
            if cmd == CMD_LOAD:
                return self._load(request)
            if cmd == CMD_START:
                return self._start(request, received_monotonic)
            if cmd == CMD_PAUSE:
                return self._pause()
            if cmd == CMD_RESUME:
                return self._resume(request, received_monotonic)
            if cmd == CMD_STOP:
                return self._stop()
            if cmd == CMD_UNLOAD:
                return self._unload()
            if cmd == CMD_UPDATE_INSTRUCTION:
                return self._update_instruction(request)
            if cmd == CMD_RESET_CYCLE:
                return self._reset_cycle(request, received_monotonic)
            if cmd in {CMD_SWITCH_POLICY, CMD_SWITCH_PRELOADED}:
                return self._switch_policy(request)
            if cmd in {CMD_PRELOAD_POLICY, CMD_CLEAR_PRELOAD}:
                return self._prepare_resident_policy(request)
            return self._make_response(False, f"Unknown command: {cmd}")
        except Exception as e:
            return self._make_response(False, str(e))

    def _load(self, request):
        if self._session.loaded:
            return self._make_response(False, "policy already loaded - UNLOAD first")
        if not request.model_path:
            return self._make_response(False, "model_path is required")
        if not request.robot_type:
            return self._make_response(False, "robot_type is required")

        response = self._requester.load_policy(request)
        if not response.success:
            return self._make_response(False, response.message)

        action_keys = list(response.action_keys)
        self._session.mark_loaded(
            robot_type=request.robot_type,
            task_instruction=request.task_instruction or "",
            action_keys=action_keys,
        )
        self._control_loop.configure(
            robot_type=request.robot_type,
            task_instruction=self._session.task_instruction,
            action_keys=action_keys,
            publish_to_robot=bool(getattr(request, "publish_to_robot", False)),
            action_request_mode=self._effective_action_request_mode(request),
            refill_margin_s=self._effective_refill_margin_s(request),
            source_chunk_limit=self._effective_source_chunk_limit(request),
            control_hz=getattr(request, "control_hz", 0),
            inference_hz=getattr(request, "inference_hz", 0),
            chunk_align_window_s=getattr(request, "chunk_align_window_s", 0.0),
            initial_pose_sync=bool(
                getattr(request, "initial_pose_sync", False)
            ),
            initial_pose_sync_duration_s=float(
                getattr(request, "initial_pose_sync_duration_s", 5.0) or 5.0
            ),
        )
        self._switch_failed = False
        return self._make_response(True, response.message or "loaded", action_keys)

    def _prepare_resident_policy(self, request):
        if self._session.running and not self._session.paused:
            raise RuntimeError("Pause inference before preloading or releasing a model")
        if self._session.loaded and request.robot_type != self._session.robot_type:
            raise RuntimeError("Cannot change robot type within an episode")
        operation = (self._requester.preload_policy if request.command == CMD_PRELOAD_POLICY
                     else self._requester.clear_preload)
        response = operation(request)
        return self._make_response(response.success, response.message, response.action_keys)

    def _switch_policy(self, request):
        if not self._session.running or not self._session.paused:
            raise RuntimeError("SWITCH_POLICY requires PAUSED")
        if not request.model_path:
            raise RuntimeError("model_path is required")
        if request.robot_type != self._session.robot_type:
            raise RuntimeError("Cannot change robot type within an episode")
        self._control_loop.pause()  # Discard chunks and invalidate late responses.
        self._switch_failed = True
        operation = (self._requester.switch_preloaded_policy
                     if request.command == CMD_SWITCH_PRELOADED else self._requester.switch_policy)
        response = operation(request)
        if not response.success:
            raise RuntimeError(response.message or "Model switch failed")
        if list(response.action_keys) != self._session.action_keys:
            raise RuntimeError("Switched policy action layout differs; Clear and reload")
        self._switch_failed = False
        return self._make_response(True, response.message, response.action_keys)

    @staticmethod
    def _effective_action_request_mode(request) -> str:
        """Select artifact-safe scheduling even for a stale UI bundle.

        SH5 TactileACT returns its saved four-step execution horizon with
        time-aligned overlap ensembling.  Ordered asynchronous refill keeps
        100 Hz output continuous without legacy L2 row skipping or a stale
        one-second queue.
        """
        requested = str(
            getattr(request, "action_request_mode", "async") or "async"
        ).strip().lower()
        forced = os.environ.get(
            "POLICY_ACTION_REQUEST_MODE_OVERRIDE",
            "",
        ).strip().lower()
        if forced:
            return forced

        root = Path(str(getattr(request, "model_path", "") or ""))
        config_path = root / "config.json"
        nested = root / "pretrained_model" / "config.json"
        if not config_path.is_file() and nested.is_file():
            config_path = nested
        if not config_path.is_file():
            return requested or "async"
        try:
            payload = json.loads(config_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError):
            return requested or "async"
        if str(payload.get("type", "")).strip().lower() == "tactile_act":
            return "async_ordered"
        return requested or "async"

    @staticmethod
    def _effective_refill_margin_s(request) -> float | None:
        del request
        raw = os.environ.get("POLICY_REFILL_MARGIN_S_OVERRIDE", "").strip()
        if not raw:
            return None
        value = float(raw)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(
                "POLICY_REFILL_MARGIN_S_OVERRIDE must be finite and non-negative"
            )
        return value

    @staticmethod
    def _effective_source_chunk_limit(request) -> int | None:
        del request
        raw = os.environ.get("POLICY_SOURCE_CHUNK_LIMIT", "").strip()
        if not raw:
            return None
        value = int(raw)
        if value <= 0:
            raise ValueError("POLICY_SOURCE_CHUNK_LIMIT must be positive")
        return value

    def _start(self, request, received_monotonic: float):
        if self._switch_failed:
            raise RuntimeError("Model switch failed; retry switching or Clear and reload")
        if not self._session.loaded:
            raise RuntimeError("LOAD first")
        publish_to_robot = bool(getattr(request, "publish_to_robot", False))
        self._check_real_start_deadline(
            publish_to_robot,
            received_monotonic,
            phase="before preflight",
        )
        sync_required = getattr(
            self._control_loop, "initial_pose_sync_required", None
        )
        should_sync = bool(
            callable(sync_required) and sync_required(publish_to_robot)
        )
        if not should_sync:
            self._control_loop.preflight_start(
                publish_to_robot,
                self._session.task_instruction,
            )
        self._check_real_start_deadline(
            publish_to_robot,
            received_monotonic,
            phase="before activation",
        )
        self._session.mark_running()
        sync_started = self._control_loop.start(
            publish_to_robot=publish_to_robot
        )
        return self._make_response(
            True,
            "initial pose sync started" if sync_started else "running",
        )

    def _pause(self):
        self._session.mark_paused()
        if self._control_loop.pause() is False:
            raise RuntimeError("failed to hold the robot while pausing")
        return self._make_response(True, "paused")

    def _resume(self, request, received_monotonic: float):
        if self._switch_failed:
            raise RuntimeError("Model switch failed; retry switching or Clear and reload")
        if not self._session.running:
            raise RuntimeError("not running")
        publish_to_robot = bool(getattr(request, "publish_to_robot", False))
        task_instruction = request.task_instruction or self._session.task_instruction
        self._check_real_start_deadline(
            publish_to_robot,
            received_monotonic,
            phase="before preflight",
        )
        sync_required = getattr(
            self._control_loop, "initial_pose_sync_required", None
        )
        should_sync = bool(
            callable(sync_required) and sync_required(publish_to_robot)
        )
        if not should_sync:
            self._control_loop.preflight_start(
                publish_to_robot,
                task_instruction,
                continuation=True,
            )
        self._check_real_start_deadline(
            publish_to_robot,
            received_monotonic,
            phase="before activation",
        )
        self._session.mark_resumed(request.task_instruction or "")
        self._control_loop.set_task_instruction(self._session.task_instruction)
        if should_sync:
            sync_started = self._control_loop.start(
                publish_to_robot=publish_to_robot, continuation=True
            )
        else:
            sync_started = self._control_loop.start(
                publish_to_robot=publish_to_robot
            )
        return self._make_response(
            True,
            "initial pose sync started" if sync_started else "resumed",
        )

    def _reset_cycle(self, request, received_monotonic: float):
        """Begin a fresh episode from PAUSED without reloading weights."""
        if self._switch_failed:
            raise RuntimeError("Model switch failed; retry switching or Clear and reload")
        if not self._session.running:
            raise RuntimeError("not running")
        if not self._session.paused:
            raise RuntimeError("RESET_CYCLE requires PAUSED")

        publish_to_robot = bool(getattr(request, "publish_to_robot", False))
        task_instruction = request.task_instruction or self._session.task_instruction
        self._check_real_start_deadline(
            publish_to_robot,
            received_monotonic,
            phase="before cycle reset",
        )
        # PAUSE already stopped the control loop and invalidated any pending
        # action generation. Reset Engine-side episode state while the command
        # publishers remain inactive.
        response = self._requester.reset_policy_cycle()
        if not response.success:
            raise RuntimeError(response.message or "policy cycle reset failed")

        self._check_real_start_deadline(
            publish_to_robot,
            received_monotonic,
            phase="before preflight",
        )
        self._control_loop.preflight_start(
            publish_to_robot,
            task_instruction,
            continuation=False,
        )
        self._check_real_start_deadline(
            publish_to_robot,
            received_monotonic,
            phase="before activation",
        )
        self._session.mark_resumed(request.task_instruction or "")
        self._control_loop.set_task_instruction(self._session.task_instruction)
        self._control_loop.start(publish_to_robot=publish_to_robot)
        return self._make_response(True, response.message or "cycle reset and resumed")

    def _stop(self):
        self._session.mark_stopped()
        self._control_loop.stop()
        return self._make_response(True, "stopped")

    def _unload(self):
        self._control_loop.deconfigure()
        response = self._requester.unload_policy()
        self._session.mark_unloaded()
        if not response.success:
            return self._make_response(False, response.message)
        return self._make_response(True, response.message or "unloaded")

    def _update_instruction(self, request):
        if not self._session.loaded:
            return self._make_response(False, "LOAD first")
        if not self._session.running:
            return self._make_response(False, "not running - START first")
        new_instruction = (request.task_instruction or "").strip()
        if not new_instruction:
            return self._make_response(False, "task_instruction must be non-empty")
        self._session.task_instruction = new_instruction
        self._control_loop.set_task_instruction(new_instruction)
        return self._make_response(True, f'instruction updated: "{new_instruction}"')

    def _check_real_start_deadline(
        self,
        publish_to_robot: bool,
        received_monotonic: float,
        *,
        phase: str,
    ) -> None:
        if not publish_to_robot:
            return
        elapsed_s = time.monotonic() - float(received_monotonic)
        if elapsed_s >= self._real_start_deadline_s:
            raise RuntimeError(
                "Real START/RESUME deadline expired "
                f"{phase}: elapsed={elapsed_s:.3f}s, "
                f"limit={self._real_start_deadline_s:.3f}s; robot publishing "
                "was not activated"
            )

    def _make_response(
        self,
        success: bool,
        message: str = "",
        action_keys: Optional[List[str]] = None,
    ):
        return self._response_factory(
            success=bool(success),
            message=str(message),
            action_keys=list(action_keys) if action_keys else [],
        )
