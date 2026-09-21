#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0

"""Main process entrypoint.

Hosts the external ``/<backend>/inference_command`` service and a local control
loop. Heavy policy imports and sensor reads stay isolated in the Engine process.
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from pathlib import Path


_ZENOH_SDK_PATH = os.environ.get("ZENOH_SDK_PATH", "/zenoh_sdk")
if os.path.exists(_ZENOH_SDK_PATH) and _ZENOH_SDK_PATH not in sys.path:
    sys.path.insert(0, _ZENOH_SDK_PATH)

_parents = Path(__file__).resolve().parents
_default_rc = str(_parents[4] / "sdk" / "robot_client") if len(_parents) > 4 else ""
_ROBOT_CLIENT_PATH = os.environ.get("ROBOT_CLIENT_SDK_PATH", _default_rc)
if os.path.exists(_ROBOT_CLIENT_PATH) and _ROBOT_CLIENT_PATH not in sys.path:
    sys.path.insert(0, _ROBOT_CLIENT_PATH)

from zenoh_ros2_sdk import ROS2Publisher, ROS2ServiceServer, get_logger  # noqa: E402
from robot_client.messages import (  # noqa: E402
    INFERENCE_COMMAND_REQUEST_DEF,
    INFERENCE_COMMAND_RESPONSE_DEF,
    INFERENCE_STATUS_DEF,
)
from policy_logging import configure_policy_runtime_logging  # noqa: E402

from .control_loop import ControlLoop  # noqa: E402
from .inference_requester import (  # noqa: E402
    DEFAULT_LOAD_POLICY_TIMEOUT_S,
    InferenceRequester,
)
from .service_handler import ServiceHandler  # noqa: E402
from .session_state import SessionState  # noqa: E402
from .zenoh_client import ZenohEngineCommandClient  # noqa: E402


logger = get_logger("main_runtime")

_HAND_ACT_ACTION_KEYS = {
    "arm_left",
    "arm_right",
    "hand_left",
    "hand_right",
}
_HAND_ACT_FIRST_ACTION_MAX_DELTA_RAD = 0.03
_HAND_ACT_FIRST_ACTION_MAX_DELTA_BY_KEY = {
    "arm_left": 0.03,
    "arm_right": 0.03,
    "hand_left": 0.10,
    "hand_right": 0.10,
}
_HAND_ACT_TRACKING_MAX_DELTA_BY_KEY = {
    "arm_left": 0.06,
    "arm_right": 0.06,
    "hand_left": 1.20,
    "hand_right": 0.70,
}
_HAND_ACT_TRACKING_BRIDGE_MAX_TOTAL_DELTA_BY_KEY = {
    "arm_left": 0.30,
    "arm_right": 0.61,
}
_HAND_ACT_WARM_START_MAX_TOTAL_DELTA_BY_KEY = {
    "arm_left": 0.55,
    "arm_right": 0.61,
    "hand_left": 1.55,
    "hand_right": 0.70,
}
_HAND_ACT_RESUME_WARM_START_MAX_TOTAL_DELTA_BY_KEY = {
    "arm_left": 0.55,
    "arm_right": 0.61,
    "hand_left": 1.55,
    "hand_right": 0.70,
}
_HAND_ACT_SOURCE_STEP_MAX_DELTA_RAD = 0.03
_HAND_ACT_SOURCE_STEP_BRIDGE_MAX_RAW_DELTA_BY_KEY = {
    "arm_left": 0.09,
    "arm_right": 0.12,
    "hand_left": 0.30,
    "hand_right": 0.28,
}
_HAND_ACT_JOINT_LIMIT_MODE = "clamp"
_HAND_ACT_JOINT_LIMIT_TOLERANCE_RAD = 0.02
_HAND_ACT_JOINT_LIMIT_TOLERANCE_BY_JOINT = {
    "arm_l_joint2": 0.18,
    "arm_r_joint2": 0.26,
    "finger_l_joint6": 0.05,
    "finger_l_joint10": 0.04,
    "finger_l_joint14": 0.04,
    "finger_l_joint18": 0.04,
}
_HAND_ACT_STATE_MAX_AGE_S = 0.5
_HAND_ACT_REAL_START_DEADLINE_S = 15.0
_HAND_ACT_REFILL_MARGIN_S = 1.0
_HAND_ACT_ORDERED_ASYNC_REFILL_MARGIN_S = 0.03


class MainRuntime:
    def __init__(
        self,
        backend: str,
        router_ip: str,
        router_port: int,
        domain_id: int,
        namespace: str = "/",
    ) -> None:
        self._backend = backend
        self._router_ip = router_ip
        self._router_port = router_port
        self._domain_id = domain_id
        self._namespace = namespace
        self._node_name = f"{backend}_main_process"

        get_action_timeout_s = float(os.environ.get("GET_ACTION_TIMEOUT_S", "5.0"))
        inference_hz = float(os.environ.get("INFERENCE_HZ", "15.0"))
        control_hz = float(os.environ.get("CONTROL_HZ", "100.0"))
        target_chunk_size = self._target_chunk_size_from_env()
        refill_margin_s = float(os.environ.get("REFILL_MARGIN_S", "0.2"))
        ordered_async_refill_margin_s = float(
            os.environ.get("ORDERED_ASYNC_REFILL_MARGIN_S", "0.03")
        )
        real_start_deadline_s = self._strict_positive_float_env(
            "REAL_START_DEADLINE_S",
            "15.0",
        )
        real_safety = self._real_safety_settings_from_env()
        real_action_safety_required = real_safety["required"]
        real_action_safety_enabled = real_safety["enabled"]
        real_first_action_max_delta_rad = real_safety["first_action_max_delta_rad"]
        real_first_action_max_delta_by_key = real_safety[
            "first_action_max_delta_by_key"
        ]
        real_tracking_max_delta_by_key = real_safety[
            "tracking_max_delta_by_key"
        ]
        real_tracking_bridge_max_total_delta_by_key = real_safety[
            "tracking_bridge_max_total_delta_by_key"
        ]
        real_warm_start_enabled = real_safety["warm_start_enabled"]
        real_warm_start_max_total_delta_by_key = real_safety[
            "warm_start_max_total_delta_by_key"
        ]
        real_resume_warm_start_max_total_delta_by_key = real_safety[
            "resume_warm_start_max_total_delta_by_key"
        ]
        real_source_step_max_delta_rad = real_safety["source_step_max_delta_rad"]
        real_source_step_bridge_enabled = real_safety[
            "source_step_bridge_enabled"
        ]
        real_source_step_bridge_max_raw_delta_by_key = real_safety[
            "source_step_bridge_max_raw_delta_by_key"
        ]
        real_joint_limit_mode = real_safety["joint_limit_mode"]
        real_joint_limit_tolerance_rad = real_safety[
            "joint_limit_tolerance_rad"
        ]
        real_joint_limit_tolerance_by_joint = real_safety[
            "joint_limit_tolerance_by_joint"
        ]
        real_state_max_age_s = real_safety["state_max_age_s"]
        self._validate_hand_act_runtime_settings(
            inference_hz=inference_hz,
            control_hz=control_hz,
            target_chunk_size=target_chunk_size,
            refill_margin_s=refill_margin_s,
            ordered_async_refill_margin_s=ordered_async_refill_margin_s,
            real_start_deadline_s=real_start_deadline_s,
        )
        self._real_start_deadline_s = real_start_deadline_s

        engine_client = ZenohEngineCommandClient(
            service_name=f"/{backend}/engine_command",
            router_ip=router_ip,
            router_port=router_port,
            domain_id=domain_id,
            node_name=f"{backend}_engine_client",
            namespace=namespace,
        )
        self._requester = InferenceRequester(
            engine_client,
            get_action_timeout_s=get_action_timeout_s,
            load_policy_timeout_s=float(
                os.environ.get(
                    "LOAD_POLICY_TIMEOUT_S",
                    str(DEFAULT_LOAD_POLICY_TIMEOUT_S),
                )
            ),
        )
        self._session = SessionState()
        self._status_publisher = None
        self._control_loop = ControlLoop(
            self._requester,
            inference_hz=inference_hz,
            control_hz=control_hz,
            chunk_align_window_s=float(os.environ.get("CHUNK_ALIGN_WINDOW_S", "0.3")),
            target_chunk_size=target_chunk_size,
            postprocess_actions=self._bool_env("POSTPROCESS_ACTIONS", True),
            alignment_mode=os.environ.get("ACTION_ALIGNMENT_MODE", "l2"),
            refill_margin_s=refill_margin_s,
            ordered_async_refill_margin_s=ordered_async_refill_margin_s,
            latency_warmup_samples=int(
                os.environ.get("REFILL_LATENCY_WARMUP_SAMPLES", "1")
            ),
            max_refill_latency_s=self._optional_float_env(
                "REFILL_LATENCY_SAMPLE_MAX_S", "2.0"
            ),
            action_request_mode=os.environ.get("ACTION_REQUEST_MODE", "async"),
            real_action_safety_enabled=real_action_safety_enabled,
            real_action_safety_required=real_action_safety_required,
            real_first_action_max_delta_rad=real_first_action_max_delta_rad,
            real_first_action_max_delta_by_key=(
                real_first_action_max_delta_by_key
            ),
            real_tracking_max_delta_by_key=(
                real_tracking_max_delta_by_key
            ),
            real_tracking_bridge_max_total_delta_by_key=(
                real_tracking_bridge_max_total_delta_by_key
            ),
            real_warm_start_enabled=real_warm_start_enabled,
            real_warm_start_max_total_delta_by_key=(
                real_warm_start_max_total_delta_by_key
            ),
            real_resume_warm_start_max_total_delta_by_key=(
                real_resume_warm_start_max_total_delta_by_key
            ),
            simulation_initial_pose_sync_hand_max_delta_by_key=(
                self._simulation_initial_pose_sync_hand_limits_env()
            ),
            real_source_step_max_delta_rad=real_source_step_max_delta_rad,
            real_source_step_bridge_enabled=real_source_step_bridge_enabled,
            real_source_step_bridge_max_raw_delta_by_key=(
                real_source_step_bridge_max_raw_delta_by_key
            ),
            real_joint_limit_mode=real_joint_limit_mode,
            real_joint_limit_tolerance_rad=real_joint_limit_tolerance_rad,
            real_joint_limit_tolerance_by_joint=(
                real_joint_limit_tolerance_by_joint
            ),
            real_state_max_age_s=real_state_max_age_s,
            real_action_blocked_callback=self._handle_real_action_blocked,
            temporal_ensemble_coeff=self._temporal_ensemble_coefficient_env(),
            temporal_ensemble_tail_fade_s=float(os.environ.get(
                'POLICY_TEMPORAL_ENSEMBLE_TAIL_FADE_S', '0.2')),
        )
        logger.info('Policy temporal ensemble: coefficient=%s (none=disabled) tail_fade_s=%s',
                    os.environ.get('POLICY_TEMPORAL_ENSEMBLE_COEFF', 'none'),
                    os.environ.get('POLICY_TEMPORAL_ENSEMBLE_TAIL_FADE_S', '0.2'))
        logger.info(
            "Policy runtime config: INFERENCE_HZ=%s CONTROL_HZ=%s "
            "TARGET_CHUNK_SIZE=%s REFILL_MARGIN_S=%s "
            "ORDERED_ASYNC_REFILL_MARGIN_S=%s GET_ACTION_TIMEOUT_S=%s "
            "REAL_ACTION_SAFETY_REQUIRED=%s REAL_ACTION_SAFETY_ENABLED=%s "
            "REAL_FIRST_ACTION_MAX_DELTA_RAD=%s "
            "REAL_FIRST_ACTION_MAX_DELTA_BY_KEY=%s "
            "REAL_TRACKING_MAX_DELTA_BY_KEY=%s "
            "REAL_TRACKING_BRIDGE_MAX_TOTAL_DELTA_BY_KEY=%s "
            "REAL_WARM_START_ENABLED=%s "
            "REAL_WARM_START_MAX_TOTAL_DELTA_BY_KEY=%s "
            "REAL_RESUME_WARM_START_MAX_TOTAL_DELTA_BY_KEY=%s "
            "REAL_SOURCE_STEP_MAX_DELTA_RAD=%s "
            "REAL_SOURCE_STEP_BRIDGE_ENABLED=%s "
            "REAL_SOURCE_STEP_BRIDGE_MAX_RAW_DELTA_BY_KEY=%s "
            "REAL_JOINT_LIMIT_MODE=%s "
            "REAL_JOINT_LIMIT_TOLERANCE_RAD=%s "
            "REAL_JOINT_LIMIT_TOLERANCE_BY_JOINT=%s "
            "REAL_STATE_MAX_AGE_S=%s "
            "REAL_START_DEADLINE_S=%s",
            inference_hz,
            control_hz,
            "none" if target_chunk_size is None else target_chunk_size,
            refill_margin_s,
            ordered_async_refill_margin_s,
            get_action_timeout_s,
            real_action_safety_required,
            real_action_safety_enabled,
            real_first_action_max_delta_rad,
            real_first_action_max_delta_by_key,
            real_tracking_max_delta_by_key,
            real_tracking_bridge_max_total_delta_by_key,
            real_warm_start_enabled,
            real_warm_start_max_total_delta_by_key,
            real_resume_warm_start_max_total_delta_by_key,
            real_source_step_max_delta_rad,
            real_source_step_bridge_enabled,
            real_source_step_bridge_max_raw_delta_by_key,
            real_joint_limit_mode,
            real_joint_limit_tolerance_rad,
            real_joint_limit_tolerance_by_joint,
            real_state_max_age_s,
            real_start_deadline_s,
        )
        self._engine_client = engine_client
        self._command_srv = None
        self._shutdown = threading.Event()

    def start(self) -> None:
        self._wait_for_engine_ready()
        self._status_publisher = ROS2Publisher(
            topic="/task/inference_status",
            msg_type="interfaces/msg/InferenceStatus",
            msg_definition=INFERENCE_STATUS_DEF,
            router_ip=self._router_ip,
            router_port=self._router_port,
            domain_id=self._domain_id,
            node_name=f"{self._backend}_runtime_status",
            namespace=self._namespace,
        )
        self._control_loop.run_background()

        def _response_factory(**kwargs):
            ResponseClass = self._command_srv.response_msg_class
            return ResponseClass(**kwargs)

        handler = ServiceHandler(
            self._session,
            self._requester,
            self._control_loop,
            _response_factory,
            real_start_deadline_s=self._real_start_deadline_s,
        )
        self._command_srv = ROS2ServiceServer(
            service_name=f"/{self._backend}/inference_command",
            srv_type="interfaces/srv/InferenceCommand",
            callback=handler.handle,
            request_definition=INFERENCE_COMMAND_REQUEST_DEF,
            response_definition=INFERENCE_COMMAND_RESPONSE_DEF,
            router_ip=self._router_ip,
            router_port=self._router_port,
            domain_id=self._domain_id,
            node_name=self._node_name,
            namespace=self._namespace,
        )
        logger.info("InferenceCommand service up at /%s/inference_command", self._backend)
        logger.info("ZENOH_SUB_READY")
        while not self._shutdown.is_set():
            self._shutdown.wait(timeout=1.0)

    def _wait_for_engine_ready(self) -> None:
        timeout_s = float(os.environ.get("ENGINE_READY_TIMEOUT_S", "120.0"))
        ping_timeout_s = float(os.environ.get("ENGINE_READY_PING_TIMEOUT_S", "1.0"))
        deadline = time.monotonic() + timeout_s
        last_error = "not ready"
        while time.monotonic() < deadline:
            try:
                if self._engine_client.ping(timeout_s=ping_timeout_s):
                    logger.info(
                        "EngineCommand service ready at /%s/engine_command",
                        self._backend,
                    )
                    return
            except Exception as e:
                last_error = str(e)
                try:
                    self._engine_client.reconnect()
                except Exception as reconnect_error:
                    last_error = f"{last_error}; reconnect failed: {reconnect_error}"
            time.sleep(0.5)
        raise RuntimeError(
            f"EngineCommand service not ready after {timeout_s:.1f}s: {last_error}"
        )

    def shutdown(self) -> None:
        self._shutdown.set()
        self._control_loop.shutdown()
        if self._command_srv is not None:
            try:
                self._command_srv.close()
            except Exception:
                pass
            self._command_srv = None
        if self._status_publisher is not None:
            try:
                self._status_publisher.close()
            except Exception:
                pass
            self._status_publisher = None
        self._engine_client.close()

    def _handle_real_action_blocked(self, reason: str) -> None:
        """Expose an asynchronous Real fail-stop to the UI as PAUSED."""
        if not self._session.running:
            return
        self._session.paused = True
        publisher = self._status_publisher
        if publisher is None:
            logger.error(
                "Real inference paused but status publisher is unavailable: %s",
                reason,
            )
            return
        publisher.publish(
            robot_type=self._session.robot_type,
            inference_phase=3,
            error=f"Real inference paused by safety gate: {reason}",
        )

    @staticmethod
    def _bool_env(name: str, default: bool) -> bool:
        value = os.environ.get(name)
        if value is None:
            return default
        return value.strip().lower() in {"1", "true", "yes", "on"}

    @classmethod
    def _simulation_initial_pose_sync_hand_limits_env(cls) -> dict[str, float]:
        limits = cls._float_map_env("CYCLO_SIM_INITIAL_POSE_SYNC_HAND_MAX_DELTA_BY_KEY")
        if not limits:
            return {}
        if os.environ.get("CYCLO_INITIAL_POSE_SYNC_MODE", "hardware") != "simulation":
            raise ValueError("simulation hand initial pose sync limits require CYCLO_INITIAL_POSE_SYNC_MODE=simulation")
        for key, value in limits.items():
            if key not in {"hand_left", "hand_right"} or value > 1.55:
                raise ValueError("simulation initial pose sync permits only hand limits up to 1.55 rad")
        logger.warning("Explicit simulator initial pose sync hand envelope=%s; ordinary warm-start and tracking unchanged", limits)
        return limits

    @staticmethod
    def _strict_bool_env(name: str, default: bool) -> bool:
        value = os.environ.get(name)
        if value is None:
            return default
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
        raise ValueError(
            f"{name} must be one of: true/false, 1/0, yes/no, on/off"
        )

    @classmethod
    def _real_safety_env_config(cls) -> tuple[bool, bool]:
        runtime_profile = os.environ.get(
            "CYCLO_LEROBOT_RUNTIME_PROFILE", "default"
        ).strip().lower()
        if runtime_profile not in {"default", "hand-act"}:
            raise ValueError(
                "CYCLO_LEROBOT_RUNTIME_PROFILE must be 'default' or 'hand-act'"
            )
        hand_act_profile = runtime_profile == "hand-act"
        required = cls._strict_bool_env(
            "REAL_ACTION_SAFETY_REQUIRED",
            hand_act_profile,
        )
        enabled = cls._strict_bool_env(
            "REAL_ACTION_SAFETY_ENABLED",
            False,
        )
        if hand_act_profile and not required:
            raise ValueError(
                "REAL_ACTION_SAFETY_REQUIRED cannot be false for the hand-act "
                "runtime profile"
            )
        if hand_act_profile and not enabled:
            raise ValueError(
                "REAL_ACTION_SAFETY_ENABLED must be true for the hand-act "
                "runtime profile"
            )
        return required, enabled

    @classmethod
    def _real_safety_settings_from_env(cls) -> dict[str, object]:
        required, enabled = cls._real_safety_env_config()
        first_action_max_delta_rad = cls._strict_optional_positive_float_env(
            "REAL_FIRST_ACTION_MAX_DELTA_RAD",
            "none",
        )
        first_action_max_delta_by_key = cls._float_map_env(
            "REAL_FIRST_ACTION_MAX_DELTA_BY_KEY"
        )
        tracking_max_delta_by_key = cls._float_map_env(
            "REAL_TRACKING_MAX_DELTA_BY_KEY"
        )
        tracking_bridge_max_total_delta_by_key = cls._float_map_env(
            "REAL_TRACKING_BRIDGE_MAX_TOTAL_DELTA_BY_KEY"
        )
        warm_start_enabled = cls._strict_bool_env(
            "REAL_WARM_START_ENABLED",
            False,
        )
        warm_start_max_total_delta_by_key = cls._float_map_env(
            "REAL_WARM_START_MAX_TOTAL_DELTA_BY_KEY"
        )
        resume_warm_start_max_total_delta_by_key = cls._float_map_env(
            "REAL_RESUME_WARM_START_MAX_TOTAL_DELTA_BY_KEY"
        )
        source_step_max_delta_rad = cls._strict_optional_positive_float_env(
            "REAL_SOURCE_STEP_MAX_DELTA_RAD",
            "none",
        )
        source_step_bridge_enabled = cls._strict_bool_env(
            "REAL_SOURCE_STEP_BRIDGE_ENABLED",
            False,
        )
        source_step_bridge_max_raw_delta_by_key = cls._float_map_env(
            "REAL_SOURCE_STEP_BRIDGE_MAX_RAW_DELTA_BY_KEY"
        )
        joint_limit_mode = os.environ.get(
            "REAL_JOINT_LIMIT_MODE",
            "off",
        ).strip().lower()
        if joint_limit_mode not in {"off", "reject", "clamp"}:
            raise ValueError(
                "REAL_JOINT_LIMIT_MODE must be one of: off, reject, clamp"
            )
        try:
            joint_limit_tolerance_rad = float(
                os.environ.get("REAL_JOINT_LIMIT_TOLERANCE_RAD", "0.0")
            )
        except ValueError as exc:
            raise ValueError(
                "REAL_JOINT_LIMIT_TOLERANCE_RAD must be a finite float"
            ) from exc
        if (
            not math.isfinite(joint_limit_tolerance_rad)
            or joint_limit_tolerance_rad < 0.0
        ):
            raise ValueError(
                "REAL_JOINT_LIMIT_TOLERANCE_RAD must be finite and non-negative"
            )
        if (
            enabled
            and joint_limit_mode == "clamp"
            and joint_limit_tolerance_rad <= 0.0
        ):
            raise ValueError(
                "REAL_JOINT_LIMIT_TOLERANCE_RAD must be positive in clamp mode"
            )
        joint_limit_tolerance_by_joint = cls._float_map_env(
            "REAL_JOINT_LIMIT_TOLERANCE_BY_JOINT"
        )
        if joint_limit_tolerance_by_joint and joint_limit_mode != "clamp":
            raise ValueError(
                "REAL_JOINT_LIMIT_MODE must be clamp when "
                "REAL_JOINT_LIMIT_TOLERANCE_BY_JOINT is set"
            )
        state_max_age_s = cls._strict_optional_positive_float_env(
            "REAL_STATE_MAX_AGE_S",
            "0.5",
        )

        runtime_profile = os.environ.get(
            "CYCLO_LEROBOT_RUNTIME_PROFILE",
            "default",
        ).strip().lower()
        if runtime_profile == "hand-act":
            if first_action_max_delta_rad is None:
                raise ValueError(
                    "REAL_FIRST_ACTION_MAX_DELTA_RAD must be positive for "
                    "the hand-act runtime profile"
                )
            actual_keys = set(first_action_max_delta_by_key)
            if actual_keys != _HAND_ACT_ACTION_KEYS:
                missing = sorted(_HAND_ACT_ACTION_KEYS - actual_keys)
                unexpected = sorted(actual_keys - _HAND_ACT_ACTION_KEYS)
                raise ValueError(
                    "REAL_FIRST_ACTION_MAX_DELTA_BY_KEY must contain exactly "
                    f"{sorted(_HAND_ACT_ACTION_KEYS)}; missing={missing}, "
                    f"unexpected={unexpected}"
                )
            actual_tracking_keys = set(tracking_max_delta_by_key)
            if actual_tracking_keys != _HAND_ACT_ACTION_KEYS:
                missing = sorted(_HAND_ACT_ACTION_KEYS - actual_tracking_keys)
                unexpected = sorted(
                    actual_tracking_keys - _HAND_ACT_ACTION_KEYS
                )
                raise ValueError(
                    "REAL_TRACKING_MAX_DELTA_BY_KEY must contain exactly "
                    f"{sorted(_HAND_ACT_ACTION_KEYS)}; missing={missing}, "
                    f"unexpected={unexpected}"
                )
            expected_tracking_bridge_keys = {"arm_left", "arm_right"}
            actual_tracking_bridge_keys = set(
                tracking_bridge_max_total_delta_by_key
            )
            if actual_tracking_bridge_keys != expected_tracking_bridge_keys:
                missing = sorted(
                    expected_tracking_bridge_keys - actual_tracking_bridge_keys
                )
                unexpected = sorted(
                    actual_tracking_bridge_keys - expected_tracking_bridge_keys
                )
                raise ValueError(
                    "REAL_TRACKING_BRIDGE_MAX_TOTAL_DELTA_BY_KEY must contain "
                    f"exactly {sorted(expected_tracking_bridge_keys)}; "
                    f"missing={missing}, unexpected={unexpected}"
                )
            if source_step_max_delta_rad is None:
                raise ValueError(
                    "REAL_SOURCE_STEP_MAX_DELTA_RAD must be positive for "
                    "the hand-act runtime profile"
                )
            if not source_step_bridge_enabled:
                raise ValueError(
                    "REAL_SOURCE_STEP_BRIDGE_ENABLED must be true for the "
                    "hand-act runtime profile"
                )
            actual_source_bridge_keys = set(
                source_step_bridge_max_raw_delta_by_key
            )
            if actual_source_bridge_keys != _HAND_ACT_ACTION_KEYS:
                missing = sorted(
                    _HAND_ACT_ACTION_KEYS - actual_source_bridge_keys
                )
                unexpected = sorted(
                    actual_source_bridge_keys - _HAND_ACT_ACTION_KEYS
                )
                raise ValueError(
                    "REAL_SOURCE_STEP_BRIDGE_MAX_RAW_DELTA_BY_KEY must "
                    f"contain exactly {sorted(_HAND_ACT_ACTION_KEYS)}; "
                    f"missing={missing}, unexpected={unexpected}"
                )
            if not warm_start_enabled:
                raise ValueError(
                    "REAL_WARM_START_ENABLED must be true for the hand-act "
                    "runtime profile"
                )
            actual_warm_start_keys = set(
                warm_start_max_total_delta_by_key
            )
            if actual_warm_start_keys != _HAND_ACT_ACTION_KEYS:
                missing = sorted(
                    _HAND_ACT_ACTION_KEYS - actual_warm_start_keys
                )
                unexpected = sorted(
                    actual_warm_start_keys - _HAND_ACT_ACTION_KEYS
                )
                raise ValueError(
                    "REAL_WARM_START_MAX_TOTAL_DELTA_BY_KEY must contain "
                    f"exactly {sorted(_HAND_ACT_ACTION_KEYS)}; "
                    f"missing={missing}, unexpected={unexpected}"
                )
            actual_resume_warm_start_keys = set(
                resume_warm_start_max_total_delta_by_key
            )
            if actual_resume_warm_start_keys != _HAND_ACT_ACTION_KEYS:
                missing = sorted(
                    _HAND_ACT_ACTION_KEYS - actual_resume_warm_start_keys
                )
                unexpected = sorted(
                    actual_resume_warm_start_keys - _HAND_ACT_ACTION_KEYS
                )
                raise ValueError(
                    "REAL_RESUME_WARM_START_MAX_TOTAL_DELTA_BY_KEY must "
                    f"contain exactly {sorted(_HAND_ACT_ACTION_KEYS)}; "
                    f"missing={missing}, unexpected={unexpected}"
                )
            if state_max_age_s is None:
                raise ValueError(
                    "REAL_STATE_MAX_AGE_S must be positive for the hand-act "
                    "runtime profile"
                )
            if joint_limit_mode != _HAND_ACT_JOINT_LIMIT_MODE:
                raise ValueError(
                    "REAL_JOINT_LIMIT_MODE must be clamp for the hand-act "
                    "runtime profile"
                )
            expected_joint_tolerance_keys = set(
                _HAND_ACT_JOINT_LIMIT_TOLERANCE_BY_JOINT
            )
            actual_joint_tolerance_keys = set(
                joint_limit_tolerance_by_joint
            )
            if actual_joint_tolerance_keys != expected_joint_tolerance_keys:
                missing = sorted(
                    expected_joint_tolerance_keys
                    - actual_joint_tolerance_keys
                )
                unexpected = sorted(
                    actual_joint_tolerance_keys
                    - expected_joint_tolerance_keys
                )
                raise ValueError(
                    "REAL_JOINT_LIMIT_TOLERANCE_BY_JOINT must contain "
                    f"exactly {sorted(expected_joint_tolerance_keys)}; "
                    f"missing={missing}, unexpected={unexpected}"
                )
            cls._require_hand_act_float(
                "REAL_FIRST_ACTION_MAX_DELTA_RAD",
                first_action_max_delta_rad,
                _HAND_ACT_FIRST_ACTION_MAX_DELTA_RAD,
            )
            for key, expected in _HAND_ACT_FIRST_ACTION_MAX_DELTA_BY_KEY.items():
                cls._require_hand_act_float(
                    f"REAL_FIRST_ACTION_MAX_DELTA_BY_KEY[{key}]",
                    first_action_max_delta_by_key[key],
                    expected,
                )
            for key, expected in _HAND_ACT_TRACKING_MAX_DELTA_BY_KEY.items():
                cls._require_hand_act_float(
                    f"REAL_TRACKING_MAX_DELTA_BY_KEY[{key}]",
                    tracking_max_delta_by_key[key],
                    expected,
                )
            for key, expected in (
                _HAND_ACT_TRACKING_BRIDGE_MAX_TOTAL_DELTA_BY_KEY.items()
            ):
                cls._require_hand_act_float(
                    f"REAL_TRACKING_BRIDGE_MAX_TOTAL_DELTA_BY_KEY[{key}]",
                    tracking_bridge_max_total_delta_by_key[key],
                    expected,
                )
            for key, expected in (
                _HAND_ACT_WARM_START_MAX_TOTAL_DELTA_BY_KEY.items()
            ):
                cls._require_hand_act_float(
                    f"REAL_WARM_START_MAX_TOTAL_DELTA_BY_KEY[{key}]",
                    warm_start_max_total_delta_by_key[key],
                    expected,
                )
            for key, expected in (
                _HAND_ACT_RESUME_WARM_START_MAX_TOTAL_DELTA_BY_KEY.items()
            ):
                cls._require_hand_act_float(
                    "REAL_RESUME_WARM_START_MAX_TOTAL_DELTA_BY_KEY"
                    f"[{key}]",
                    resume_warm_start_max_total_delta_by_key[key],
                    expected,
                )
            cls._require_hand_act_float(
                "REAL_SOURCE_STEP_MAX_DELTA_RAD",
                source_step_max_delta_rad,
                _HAND_ACT_SOURCE_STEP_MAX_DELTA_RAD,
            )
            for key, expected in (
                _HAND_ACT_SOURCE_STEP_BRIDGE_MAX_RAW_DELTA_BY_KEY.items()
            ):
                cls._require_hand_act_float(
                    f"REAL_SOURCE_STEP_BRIDGE_MAX_RAW_DELTA_BY_KEY[{key}]",
                    source_step_bridge_max_raw_delta_by_key[key],
                    expected,
                )
            cls._require_hand_act_float(
                "REAL_JOINT_LIMIT_TOLERANCE_RAD",
                joint_limit_tolerance_rad,
                _HAND_ACT_JOINT_LIMIT_TOLERANCE_RAD,
            )
            for name, expected in (
                _HAND_ACT_JOINT_LIMIT_TOLERANCE_BY_JOINT.items()
            ):
                cls._require_hand_act_float(
                    f"REAL_JOINT_LIMIT_TOLERANCE_BY_JOINT[{name}]",
                    joint_limit_tolerance_by_joint[name],
                    expected,
                )
            cls._require_hand_act_float(
                "REAL_STATE_MAX_AGE_S",
                state_max_age_s,
                _HAND_ACT_STATE_MAX_AGE_S,
            )

        return {
            "required": required,
            "enabled": enabled,
            "first_action_max_delta_rad": first_action_max_delta_rad,
            "first_action_max_delta_by_key": first_action_max_delta_by_key,
            "tracking_max_delta_by_key": tracking_max_delta_by_key,
            "tracking_bridge_max_total_delta_by_key": (
                tracking_bridge_max_total_delta_by_key
            ),
            "warm_start_enabled": warm_start_enabled,
            "warm_start_max_total_delta_by_key": (
                warm_start_max_total_delta_by_key
            ),
            "resume_warm_start_max_total_delta_by_key": (
                resume_warm_start_max_total_delta_by_key
            ),
            "source_step_max_delta_rad": source_step_max_delta_rad,
            "source_step_bridge_enabled": source_step_bridge_enabled,
            "source_step_bridge_max_raw_delta_by_key": (
                source_step_bridge_max_raw_delta_by_key
            ),
            "joint_limit_mode": joint_limit_mode,
            "joint_limit_tolerance_rad": joint_limit_tolerance_rad,
            "joint_limit_tolerance_by_joint": (
                joint_limit_tolerance_by_joint
            ),
            "state_max_age_s": state_max_age_s,
        }

    @staticmethod
    def _require_hand_act_float(
        name: str,
        actual: float,
        expected: float,
    ) -> None:
        if not math.isclose(
            float(actual),
            float(expected),
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError(
                f"{name} must be exactly {expected:g} for the hand-act "
                f"runtime profile; got {actual:g}"
            )

    @staticmethod
    def _validate_hand_act_runtime_settings(
        *,
        inference_hz: float,
        control_hz: float,
        target_chunk_size: int | None,
        refill_margin_s: float,
        ordered_async_refill_margin_s: float,
        real_start_deadline_s: float,
    ) -> None:
        runtime_profile = os.environ.get(
            "CYCLO_LEROBOT_RUNTIME_PROFILE",
            "default",
        ).strip().lower()
        if runtime_profile != "hand-act":
            return
        if not math.isclose(
            float(inference_hz),
            30.0,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ValueError(
                "INFERENCE_HZ must be exactly 30 for the hand-act runtime profile"
            )
        if not math.isclose(
            float(control_hz),
            100.0,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ValueError(
                "CONTROL_HZ must be exactly 100 for the hand-act runtime profile"
            )
        if target_chunk_size is not None:
            raise ValueError(
                "TARGET_CHUNK_SIZE must be none for the hand-act runtime profile"
            )
        if not math.isclose(
            float(refill_margin_s),
            _HAND_ACT_REFILL_MARGIN_S,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError(
                "REFILL_MARGIN_S must be exactly 1 for the hand-act runtime "
                f"profile; got {refill_margin_s:g}"
            )
        if not math.isclose(
            float(ordered_async_refill_margin_s),
            _HAND_ACT_ORDERED_ASYNC_REFILL_MARGIN_S,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError(
                "ORDERED_ASYNC_REFILL_MARGIN_S must be exactly 0.03 for the "
                "hand-act runtime profile; got "
                f"{ordered_async_refill_margin_s:g}"
            )
        if not math.isclose(
            float(real_start_deadline_s),
            _HAND_ACT_REAL_START_DEADLINE_S,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError(
                "REAL_START_DEADLINE_S must be exactly 15 for the hand-act "
                f"runtime profile; got {real_start_deadline_s:g}"
            )

    @staticmethod
    def _target_chunk_size_from_env() -> int | None:
        raw = os.environ.get("TARGET_CHUNK_SIZE", "none").strip().lower()
        if raw in {"", "none", "off", "0"}:
            return None
        return int(raw)

    @staticmethod
    def _temporal_ensemble_coefficient_env() -> float | None:
        raw = os.environ.get('POLICY_TEMPORAL_ENSEMBLE_COEFF', 'none').strip().lower()
        # Zero means uniform averaging, not disabled.
        return None if raw in {'', 'none', 'off'} else float(raw)

    @staticmethod
    def _optional_float_env(name: str, default: str) -> float | None:
        raw = os.environ.get(name, default).strip().lower()
        if raw in {"", "none", "off", "0"}:
            return None
        return float(raw)

    @staticmethod
    def _strict_optional_positive_float_env(
        name: str,
        default: str,
    ) -> float | None:
        raw = os.environ.get(name, default).strip().lower()
        if raw in {"", "none", "off"}:
            return None
        try:
            value = float(raw)
        except ValueError as exc:
            raise ValueError(f"{name} must be a positive float or none") from exc
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    @staticmethod
    def _strict_positive_float_env(name: str, default: str) -> float:
        raw = os.environ.get(name, default).strip().lower()
        try:
            value = float(raw)
        except ValueError as exc:
            raise ValueError(f"{name} must be a positive float") from exc
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    @staticmethod
    def _float_map_env(name: str) -> dict[str, float]:
        raw = os.environ.get(name, "").strip()
        if not raw:
            return {}
        result: dict[str, float] = {}
        for item in raw.split(","):
            key, separator, value = item.partition("=")
            key = key.strip()
            if not separator or not key or not value.strip():
                raise ValueError(
                    f"{name} must be comma-separated key=value entries"
                )
            if key in result:
                raise ValueError(f"{name} contains duplicate key: {key}")
            try:
                threshold = float(value)
            except ValueError as exc:
                raise ValueError(
                    f"{name}[{key}] must be a positive float"
                ) from exc
            if not math.isfinite(threshold) or threshold <= 0.0:
                raise ValueError(
                    f"{name}[{key}] must be finite and positive"
                )
            result[key] = threshold
        return result


def main() -> None:  # pragma: no cover - container entrypoint.
    configure_policy_runtime_logging()
    backend = os.environ.get("POLICY_BACKEND", "").strip()
    if not backend:
        raise RuntimeError("POLICY_BACKEND env var is required")
    runtime = MainRuntime(
        backend=backend,
        router_ip=os.environ.get("ZENOH_ROUTER_IP", "127.0.0.1"),
        router_port=int(os.environ.get("ZENOH_ROUTER_PORT", "7447")),
        domain_id=int(os.environ.get("ROS_DOMAIN_ID", "30")),
    )
    try:
        runtime.start()
    except KeyboardInterrupt:
        logger.info("shutdown via SIGINT")
    finally:
        runtime.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
