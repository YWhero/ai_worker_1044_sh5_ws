#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0

"""Client-side EngineCommand helper used by the Main process."""

from __future__ import annotations

import threading
from typing import Any

from engine_process.protocol import (
    CMD_GET_ACTION,
    CMD_LOAD_POLICY,
    CMD_RESET_POLICY,
    CMD_SWITCH_POLICY,
    CMD_PRELOAD_POLICY,
    CMD_CLEAR_PRELOAD,
    CMD_SWITCH_PRELOADED,
    CMD_UNLOAD_POLICY,
    EngineCommandRequest,
    EngineCommandResponse,
    response_from_message,
)


DEFAULT_LOAD_POLICY_TIMEOUT_S = 7200.0


class InferenceRequester:
    """Synchronous Engine process requester with seq_id stale-response guard."""

    def __init__(
        self,
        client: Any,
        get_action_timeout_s: float = 5.0,
        load_policy_timeout_s: float = DEFAULT_LOAD_POLICY_TIMEOUT_S,
    ) -> None:
        self._client = client
        self._get_action_timeout_s = float(get_action_timeout_s)
        self._load_policy_timeout_s = float(load_policy_timeout_s)
        self._seq_id = 0
        self._lock = threading.Lock()
        self._get_action_in_flight = False
        self._action_idle = threading.Event()
        self._action_idle.set()

    def has_pending_get_action(self) -> bool:
        with self._lock:
            return self._get_action_in_flight

    def load_policy(self, request: Any, timeout_s: float | None = None) -> EngineCommandResponse:
        seq_id = self._next_seq_id()
        engine_request = EngineCommandRequest(
            command=CMD_LOAD_POLICY,
            seq_id=seq_id,
            model_path=str(getattr(request, "model_path", "") or ""),
            embodiment_tag=str(getattr(request, "embodiment_tag", "") or ""),
            robot_type=str(getattr(request, "robot_type", "") or ""),
            task_instruction=str(getattr(request, "task_instruction", "") or ""),
            acceleration_mode=str(getattr(request, "acceleration_mode", "") or ""),
            acceleration_engine_path=str(
                getattr(request, "acceleration_engine_path", "") or ""
            ),
        )
        return self._call(
            engine_request,
            self._load_policy_timeout_s if timeout_s is None else timeout_s,
        )

    def get_action(self, task_instruction: str, timeout_s: float | None = None) -> EngineCommandResponse:
        with self._lock:
            if self._get_action_in_flight:
                return EngineCommandResponse(
                    success=False,
                    message="get_action already in flight",
                )
            self._get_action_in_flight = True
            self._action_idle.clear()
            seq_id = self._next_seq_id_locked()

        request = EngineCommandRequest(
            command=CMD_GET_ACTION,
            seq_id=seq_id,
            task_instruction=task_instruction or "",
        )
        try:
            return self._call(
                request,
                self._get_action_timeout_s if timeout_s is None else timeout_s,
            )
        finally:
            with self._lock:
                self._get_action_in_flight = False
                self._action_idle.set()

    def preload_policy(self, request: Any) -> EngineCommandResponse:
        return self._resident_policy_command(request, CMD_PRELOAD_POLICY)

    def clear_preload(self, request: Any) -> EngineCommandResponse:
        return self._resident_policy_command(request, CMD_CLEAR_PRELOAD)

    def switch_preloaded_policy(self, request: Any) -> EngineCommandResponse:
        return self._resident_policy_command(request, CMD_SWITCH_PRELOADED)

    def switch_policy(self, request: Any) -> EngineCommandResponse:
        return self._resident_policy_command(request, CMD_SWITCH_POLICY)

    def _resident_policy_command(self, request: Any, command: int) -> EngineCommandResponse:
        # PAUSE has stopped new requests. Drain the last caller before using
        # the shared service client for the long-running weight replacement.
        if not self._action_idle.wait(timeout=self._get_action_timeout_s):
            return EngineCommandResponse(
                success=False,
                message="Action request still in flight; retry switching after it completes",
            )
        return self._call(
            EngineCommandRequest(
                command=command,
                seq_id=self._next_seq_id(),
                model_path=str(getattr(request, "model_path", "") or ""),
                robot_type=str(getattr(request, "robot_type", "") or ""),
            ),
            self._load_policy_timeout_s,
        )

    def unload_policy(self, timeout_s: float | None = None) -> EngineCommandResponse:
        seq_id = self._next_seq_id()
        request = EngineCommandRequest(command=CMD_UNLOAD_POLICY, seq_id=seq_id)
        return self._call(
            request,
            self._load_policy_timeout_s if timeout_s is None else timeout_s,
        )

    def reset_policy_cycle(
        self,
        timeout_s: float | None = None,
    ) -> EngineCommandResponse:
        """Reset one episode's runtime state while retaining model weights."""
        with self._lock:
            if self._get_action_in_flight:
                return EngineCommandResponse(
                    success=False,
                    message=(
                        "cannot reset cycle while an action request is still in flight; "
                        "wait briefly and retry"
                    ),
                )
            seq_id = self._next_seq_id_locked()
        request = EngineCommandRequest(command=CMD_RESET_POLICY, seq_id=seq_id)
        return self._call(
            request,
            self._load_policy_timeout_s if timeout_s is None else timeout_s,
        )

    def _next_seq_id(self) -> int:
        with self._lock:
            return self._next_seq_id_locked()

    def _next_seq_id_locked(self) -> int:
        self._seq_id += 1
        return self._seq_id

    def _call(self, request: EngineCommandRequest, timeout_s: float) -> EngineCommandResponse:
        try:
            response = self._client.call(request, timeout_s=timeout_s)
        except TimeoutError:
            return EngineCommandResponse(
                success=False,
                seq_id=request.seq_id,
                message=f"engine request seq={request.seq_id} timed out after {timeout_s:.1f}s",
            )
        except Exception as e:
            return EngineCommandResponse(
                success=False,
                seq_id=request.seq_id,
                message=str(e),
            )

        if not isinstance(response, EngineCommandResponse):
            response = response_from_message(response)
        if response.seq_id != request.seq_id:
            return EngineCommandResponse(
                success=False,
                seq_id=request.seq_id,
                message=(
                    f"stale engine response discarded: expected seq={request.seq_id}, "
                    f"got seq={response.seq_id}"
                ),
            )
        return response
