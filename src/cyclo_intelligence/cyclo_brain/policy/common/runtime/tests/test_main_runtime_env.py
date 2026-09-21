#!/usr/bin/env python3

from __future__ import annotations

import logging
import os
import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


RUNTIME_ROOT = Path(__file__).resolve().parents[1]
if str(RUNTIME_ROOT) not in sys.path:
    sys.path.insert(0, str(RUNTIME_ROOT))

# ``main_runtime.main`` only needs these SDK symbols at import time.  Runtime
# construction is deliberately not exercised by these pure env-parser tests.
zenoh_stub = sys.modules.setdefault(
    "zenoh_ros2_sdk",
    types.ModuleType("zenoh_ros2_sdk"),
)
zenoh_stub.ROS2ServiceServer = object
zenoh_stub.ROS2Publisher = object
zenoh_stub.ROS2Subscriber = object
zenoh_stub.get_message_class = lambda _name: object
zenoh_stub.get_logger = logging.getLogger

robot_client_stub = sys.modules.setdefault(
    "robot_client",
    types.ModuleType("robot_client"),
)
robot_client_stub.RobotClient = getattr(robot_client_stub, "RobotClient", object)
messages_stub = types.ModuleType("robot_client.messages")
messages_stub.INFERENCE_COMMAND_REQUEST_DEF = []
messages_stub.INFERENCE_COMMAND_RESPONSE_DEF = []
messages_stub.INFERENCE_STATUS_DEF = []
sys.modules.setdefault("robot_client.messages", messages_stub)

from main_runtime.main import MainRuntime  # noqa: E402


class MainRuntimeSafetyEnvTests(unittest.TestCase):
    def test_simulation_initial_sync_hand_override_requires_explicit_mode(self):
        read = MainRuntime._simulation_initial_pose_sync_hand_limits_env
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(read(), {})
        env = {"CYCLO_SIM_INITIAL_POSE_SYNC_HAND_MAX_DELTA_BY_KEY": "hand_right=1.55"}
        with patch.dict(os.environ, env, clear=True):
            with self.assertRaisesRegex(ValueError, "require.*simulation"):
                read()
        env["CYCLO_INITIAL_POSE_SYNC_MODE"] = "simulation"
        with patch.dict(os.environ, env, clear=True):
            self.assertEqual(read(), {"hand_right": 1.55})
        for value in ("arm_right=0.7", "hand_right=1.56", "hand_right=nan", "hand_right=0"):
            with self.subTest(value=value):
                env["CYCLO_SIM_INITIAL_POSE_SYNC_HAND_MAX_DELTA_BY_KEY"] = value
                with patch.dict(os.environ, env, clear=True):
                    with self.assertRaises(ValueError):
                        read()

    def test_ensemble_coefficient_zero_is_uniform_averaging(self):
        for raw, expected in [('0', 0.0), ('0.01', .01), ('none', None), ('off', None)]:
            with patch.dict(os.environ, {'POLICY_TEMPORAL_ENSEMBLE_COEFF': raw}):
                self.assertEqual(MainRuntime._temporal_ensemble_coefficient_env(), expected)

    @staticmethod
    def _complete_hand_act_env() -> dict[str, str]:
        return {
            "CYCLO_LEROBOT_RUNTIME_PROFILE": "hand-act",
            "REAL_ACTION_SAFETY_REQUIRED": "true",
            "REAL_ACTION_SAFETY_ENABLED": "true",
            "REAL_FIRST_ACTION_MAX_DELTA_RAD": "0.03",
            "REAL_FIRST_ACTION_MAX_DELTA_BY_KEY": (
                "arm_left=0.03,arm_right=0.03,"
                "hand_left=0.10,hand_right=0.10"
            ),
            "REAL_TRACKING_MAX_DELTA_BY_KEY": (
                "arm_left=0.06,arm_right=0.06,"
                "hand_left=1.20,hand_right=0.70"
            ),
            "REAL_TRACKING_BRIDGE_MAX_TOTAL_DELTA_BY_KEY": (
                "arm_left=0.30,arm_right=0.61"
            ),
            "REAL_WARM_START_ENABLED": "true",
            "REAL_WARM_START_MAX_TOTAL_DELTA_BY_KEY": (
                "arm_left=0.55,arm_right=0.61,"
                "hand_left=1.55,hand_right=0.70"
            ),
            "REAL_RESUME_WARM_START_MAX_TOTAL_DELTA_BY_KEY": (
                "arm_left=0.55,arm_right=0.61,"
                "hand_left=1.55,hand_right=0.70"
            ),
            "REAL_SOURCE_STEP_MAX_DELTA_RAD": "0.03",
            "REAL_SOURCE_STEP_BRIDGE_ENABLED": "true",
            "REAL_SOURCE_STEP_BRIDGE_MAX_RAW_DELTA_BY_KEY": (
                "arm_left=0.09,arm_right=0.12,"
                "hand_left=0.30,hand_right=0.28"
            ),
            "REAL_JOINT_LIMIT_MODE": "clamp",
            "REAL_JOINT_LIMIT_TOLERANCE_RAD": "0.02",
            "REAL_JOINT_LIMIT_TOLERANCE_BY_JOINT": (
                "arm_l_joint2=0.18,arm_r_joint2=0.26,"
                "finger_l_joint6=0.05,finger_l_joint10=0.04,"
                "finger_l_joint14=0.04,finger_l_joint18=0.04"
            ),
            "REAL_STATE_MAX_AGE_S": "0.5",
            "REAL_START_DEADLINE_S": "15.0",
        }

    def test_hand_act_forces_required_when_env_is_missing(self) -> None:
        with patch.dict(os.environ, {
            "CYCLO_LEROBOT_RUNTIME_PROFILE": "hand-act",
            "REAL_ACTION_SAFETY_ENABLED": "true",
        }, clear=True):
            self.assertEqual(
                MainRuntime._real_safety_env_config(),
                (True, True),
            )

    def test_hand_act_rejects_required_false(self) -> None:
        with patch.dict(os.environ, {
            "CYCLO_LEROBOT_RUNTIME_PROFILE": "hand-act",
            "REAL_ACTION_SAFETY_REQUIRED": "false",
            "REAL_ACTION_SAFETY_ENABLED": "true",
        }, clear=True):
            with self.assertRaisesRegex(ValueError, "cannot be false"):
                MainRuntime._real_safety_env_config()

    def test_hand_act_rejects_safety_disabled(self) -> None:
        with patch.dict(os.environ, {
            "CYCLO_LEROBOT_RUNTIME_PROFILE": "hand-act",
            "REAL_ACTION_SAFETY_REQUIRED": "true",
            "REAL_ACTION_SAFETY_ENABLED": "false",
        }, clear=True):
            with self.assertRaisesRegex(ValueError, "must be true"):
                MainRuntime._real_safety_env_config()

    def test_hand_act_rejects_missing_enabled_setting(self) -> None:
        with patch.dict(os.environ, {
            "CYCLO_LEROBOT_RUNTIME_PROFILE": "hand-act",
        }, clear=True):
            with self.assertRaisesRegex(ValueError, "must be true"):
                MainRuntime._real_safety_env_config()

    def test_safety_boolean_typo_is_rejected(self) -> None:
        with patch.dict(os.environ, {
            "REAL_ACTION_SAFETY_ENABLED": "ture",
        }, clear=True):
            with self.assertRaisesRegex(ValueError, "must be one of"):
                MainRuntime._real_safety_env_config()

    def test_runtime_profile_typo_is_rejected(self) -> None:
        with patch.dict(os.environ, {
            "CYCLO_LEROBOT_RUNTIME_PROFILE": "hand_act",
        }, clear=True):
            with self.assertRaisesRegex(ValueError, "default.*hand-act"):
                MainRuntime._real_safety_env_config()

    def test_zero_positive_threshold_is_rejected(self) -> None:
        with patch.dict(os.environ, {
            "REAL_STATE_MAX_AGE_S": "0",
        }, clear=True):
            with self.assertRaisesRegex(ValueError, "finite and positive"):
                MainRuntime._strict_optional_positive_float_env(
                    "REAL_STATE_MAX_AGE_S",
                    "0.5",
                )

    def test_duplicate_threshold_key_is_rejected(self) -> None:
        with patch.dict(os.environ, {
            "REAL_FIRST_ACTION_MAX_DELTA_BY_KEY": "arm=0.03,arm=0.04",
        }, clear=True):
            with self.assertRaisesRegex(ValueError, "duplicate key"):
                MainRuntime._float_map_env(
                    "REAL_FIRST_ACTION_MAX_DELTA_BY_KEY"
                )

    def test_complete_hand_act_safety_settings_are_accepted(self) -> None:
        with patch.dict(
            os.environ,
            self._complete_hand_act_env(),
            clear=True,
        ):
            settings = MainRuntime._real_safety_settings_from_env()

        self.assertEqual(settings["joint_limit_mode"], "clamp")
        self.assertEqual(
            set(settings["first_action_max_delta_by_key"]),
            {"arm_left", "arm_right", "hand_left", "hand_right"},
        )
        self.assertTrue(settings["warm_start_enabled"])
        self.assertEqual(settings["tracking_max_delta_by_key"]["arm_left"], 0.06)
        self.assertEqual(
            settings["tracking_bridge_max_total_delta_by_key"]["arm_right"],
            0.61,
        )
        self.assertEqual(
            settings["tracking_bridge_max_total_delta_by_key"]["arm_left"],
            0.30,
        )
        expected_warm_start = {
            "arm_left": 0.55,
            "arm_right": 0.61,
            "hand_left": 1.55,
            "hand_right": 0.70,
        }
        self.assertEqual(
            settings["warm_start_max_total_delta_by_key"],
            expected_warm_start,
        )
        self.assertEqual(
            settings["resume_warm_start_max_total_delta_by_key"],
            expected_warm_start,
        )
        self.assertTrue(settings["source_step_bridge_enabled"])
        self.assertEqual(
            settings["source_step_bridge_max_raw_delta_by_key"]["hand_left"],
            0.30,
        )
        self.assertEqual(
            settings["joint_limit_tolerance_by_joint"],
            {
                "arm_l_joint2": 0.18,
                "arm_r_joint2": 0.26,
                "finger_l_joint6": 0.05,
                "finger_l_joint10": 0.04,
                "finger_l_joint14": 0.04,
                "finger_l_joint18": 0.04,
            },
        )

    def test_async_real_block_is_published_as_paused_status(self) -> None:
        published = []
        runtime = MainRuntime.__new__(MainRuntime)
        runtime._session = SimpleNamespace(
            running=True,
            paused=False,
            robot_type="ffw_sh5_rev1",
        )
        runtime._status_publisher = SimpleNamespace(
            publish=lambda **kwargs: published.append(kwargs)
        )

        runtime._handle_real_action_blocked("tracking hard cap exceeded")

        self.assertTrue(runtime._session.paused)
        self.assertEqual(
            published,
            [{
                "robot_type": "ffw_sh5_rev1",
                "inference_phase": 3,
                "error": (
                    "Real inference paused by safety gate: "
                    "tracking hard cap exceeded"
                ),
            }],
        )

    def test_hand_act_rejects_missing_global_first_delta(self) -> None:
        env = self._complete_hand_act_env()
        env.pop("REAL_FIRST_ACTION_MAX_DELTA_RAD")
        with patch.dict(os.environ, env, clear=True):
            with self.assertRaisesRegex(
                ValueError,
                "REAL_FIRST_ACTION_MAX_DELTA_RAD",
            ):
                MainRuntime._real_safety_settings_from_env()

    def test_hand_act_rejects_incomplete_per_key_thresholds(self) -> None:
        env = self._complete_hand_act_env()
        env["REAL_FIRST_ACTION_MAX_DELTA_BY_KEY"] = (
            "arm_left=0.03,arm_right=0.03,hand_left=0.10"
        )
        with patch.dict(os.environ, env, clear=True):
            with self.assertRaisesRegex(ValueError, "hand_right"):
                MainRuntime._real_safety_settings_from_env()

    def test_hand_act_rejects_incomplete_tracking_thresholds(self) -> None:
        env = self._complete_hand_act_env()
        env["REAL_TRACKING_MAX_DELTA_BY_KEY"] = (
            "arm_left=0.06,arm_right=0.06,hand_left=0.12"
        )
        with patch.dict(os.environ, env, clear=True):
            with self.assertRaisesRegex(ValueError, "hand_right"):
                MainRuntime._real_safety_settings_from_env()

    def test_hand_act_rejects_incomplete_tracking_bridge_thresholds(self) -> None:
        env = self._complete_hand_act_env()
        env["REAL_TRACKING_BRIDGE_MAX_TOTAL_DELTA_BY_KEY"] = "arm_left=0.30"
        with patch.dict(os.environ, env, clear=True):
            with self.assertRaisesRegex(ValueError, "arm_right"):
                MainRuntime._real_safety_settings_from_env()

    def test_hand_act_rejects_disabled_source_step_gate(self) -> None:
        env = self._complete_hand_act_env()
        env["REAL_SOURCE_STEP_MAX_DELTA_RAD"] = "off"
        with patch.dict(os.environ, env, clear=True):
            with self.assertRaisesRegex(
                ValueError,
                "REAL_SOURCE_STEP_MAX_DELTA_RAD",
            ):
                MainRuntime._real_safety_settings_from_env()

    def test_hand_act_rejects_disabled_warm_start(self) -> None:
        env = self._complete_hand_act_env()
        env["REAL_WARM_START_ENABLED"] = "false"
        with patch.dict(os.environ, env, clear=True):
            with self.assertRaisesRegex(ValueError, "REAL_WARM_START_ENABLED"):
                MainRuntime._real_safety_settings_from_env()

    def test_hand_act_rejects_disabled_source_step_bridge(self) -> None:
        env = self._complete_hand_act_env()
        env["REAL_SOURCE_STEP_BRIDGE_ENABLED"] = "false"
        with patch.dict(os.environ, env, clear=True):
            with self.assertRaisesRegex(
                ValueError,
                "REAL_SOURCE_STEP_BRIDGE_ENABLED",
            ):
                MainRuntime._real_safety_settings_from_env()

    def test_hand_act_rejects_incomplete_source_step_bridge_limits(self) -> None:
        env = self._complete_hand_act_env()
        env["REAL_SOURCE_STEP_BRIDGE_MAX_RAW_DELTA_BY_KEY"] = (
            "arm_left=0.05,arm_right=0.05,hand_left=0.05"
        )
        with patch.dict(os.environ, env, clear=True):
            with self.assertRaisesRegex(ValueError, "hand_right"):
                MainRuntime._real_safety_settings_from_env()

    def test_hand_act_rejects_incomplete_warm_start_limits(self) -> None:
        env = self._complete_hand_act_env()
        env["REAL_WARM_START_MAX_TOTAL_DELTA_BY_KEY"] = (
            "arm_left=0.55,arm_right=0.61,hand_left=1.55"
        )
        with patch.dict(os.environ, env, clear=True):
            with self.assertRaisesRegex(ValueError, "hand_right"):
                MainRuntime._real_safety_settings_from_env()

    def test_hand_act_rejects_incomplete_resume_warm_start_limits(self) -> None:
        env = self._complete_hand_act_env()
        env["REAL_RESUME_WARM_START_MAX_TOTAL_DELTA_BY_KEY"] = (
            "arm_left=0.55,arm_right=0.61,hand_left=1.55"
        )
        with patch.dict(os.environ, env, clear=True):
            with self.assertRaisesRegex(ValueError, "hand_right"):
                MainRuntime._real_safety_settings_from_env()

    def test_hand_act_rejects_disabled_state_age_gate(self) -> None:
        env = self._complete_hand_act_env()
        env["REAL_STATE_MAX_AGE_S"] = "off"
        with patch.dict(os.environ, env, clear=True):
            with self.assertRaisesRegex(ValueError, "REAL_STATE_MAX_AGE_S"):
                MainRuntime._real_safety_settings_from_env()

    def test_hand_act_rejects_joint_limit_mode_off(self) -> None:
        env = self._complete_hand_act_env()
        env["REAL_JOINT_LIMIT_MODE"] = "off"
        with patch.dict(os.environ, env, clear=True):
            with self.assertRaisesRegex(ValueError, "must be clamp"):
                MainRuntime._real_safety_settings_from_env()

    def test_hand_act_rejects_missing_joint_limit_mode(self) -> None:
        env = self._complete_hand_act_env()
        env.pop("REAL_JOINT_LIMIT_MODE")
        with patch.dict(os.environ, env, clear=True):
            with self.assertRaisesRegex(ValueError, "must be clamp"):
                MainRuntime._real_safety_settings_from_env()

    def test_hand_act_rejects_zero_clamp_tolerance(self) -> None:
        env = self._complete_hand_act_env()
        env["REAL_JOINT_LIMIT_TOLERANCE_RAD"] = "0"
        with patch.dict(os.environ, env, clear=True):
            with self.assertRaisesRegex(ValueError, "positive in clamp mode"):
                MainRuntime._real_safety_settings_from_env()

    def test_hand_act_rejects_incomplete_joint_limit_tolerances(self) -> None:
        env = self._complete_hand_act_env()
        env["REAL_JOINT_LIMIT_TOLERANCE_BY_JOINT"] = (
            "arm_l_joint2=0.18"
        )
        with patch.dict(os.environ, env, clear=True):
            with self.assertRaisesRegex(ValueError, "arm_r_joint2"):
                MainRuntime._real_safety_settings_from_env()

    def test_hand_act_rejects_overly_permissive_safety_values(self) -> None:
        mutations = {
            "REAL_FIRST_ACTION_MAX_DELTA_RAD": "100",
            "REAL_FIRST_ACTION_MAX_DELTA_BY_KEY": (
                "arm_left=0.03,arm_right=0.03,"
                "hand_left=100,hand_right=0.10"
            ),
            "REAL_TRACKING_MAX_DELTA_BY_KEY": (
                "arm_left=100,arm_right=0.06,"
                "hand_left=1.20,hand_right=0.70"
            ),
            "REAL_TRACKING_BRIDGE_MAX_TOTAL_DELTA_BY_KEY": (
                "arm_left=100,arm_right=0.61"
            ),
            "REAL_SOURCE_STEP_MAX_DELTA_RAD": "100",
            "REAL_SOURCE_STEP_BRIDGE_MAX_RAW_DELTA_BY_KEY": (
                "arm_left=100,arm_right=0.12,"
                "hand_left=0.30,hand_right=0.28"
            ),
            "REAL_WARM_START_MAX_TOTAL_DELTA_BY_KEY": (
                "arm_left=100,arm_right=0.61,"
                "hand_left=1.55,hand_right=0.70"
            ),
            "REAL_RESUME_WARM_START_MAX_TOTAL_DELTA_BY_KEY": (
                "arm_left=100,arm_right=0.61,"
                "hand_left=1.55,hand_right=0.70"
            ),
            "REAL_JOINT_LIMIT_MODE": "reject",
            "REAL_JOINT_LIMIT_TOLERANCE_RAD": "100",
            "REAL_JOINT_LIMIT_TOLERANCE_BY_JOINT": (
                "arm_l_joint2=100,arm_r_joint2=0.26,"
                "finger_l_joint6=0.05,finger_l_joint10=0.04,"
                "finger_l_joint14=0.04,finger_l_joint18=0.04"
            ),
            "REAL_STATE_MAX_AGE_S": "100",
        }
        for name, value in mutations.items():
            with self.subTest(name=name):
                env = self._complete_hand_act_env()
                env[name] = value
                with patch.dict(os.environ, env, clear=True):
                    with self.assertRaisesRegex(ValueError, name.split("[")[0]):
                        MainRuntime._real_safety_settings_from_env()

    def test_hand_act_runtime_requires_act_rates_and_bounded_prefetch(self) -> None:
        with patch.dict(os.environ, {
            "CYCLO_LEROBOT_RUNTIME_PROFILE": "hand-act",
        }, clear=True):
            MainRuntime._validate_hand_act_runtime_settings(
                inference_hz=30.0,
                control_hz=100.0,
                target_chunk_size=None,
                refill_margin_s=1.0,
                ordered_async_refill_margin_s=0.03,
                real_start_deadline_s=15.0,
            )
            with self.assertRaisesRegex(ValueError, "INFERENCE_HZ"):
                MainRuntime._validate_hand_act_runtime_settings(
                    inference_hz=15.0,
                    control_hz=100.0,
                    target_chunk_size=None,
                    refill_margin_s=1.0,
                    ordered_async_refill_margin_s=0.03,
                    real_start_deadline_s=15.0,
                )
            with self.assertRaisesRegex(ValueError, "CONTROL_HZ"):
                MainRuntime._validate_hand_act_runtime_settings(
                    inference_hz=30.0,
                    control_hz=30.0,
                    target_chunk_size=None,
                    refill_margin_s=1.0,
                    ordered_async_refill_margin_s=0.03,
                    real_start_deadline_s=15.0,
                )
            with self.assertRaisesRegex(ValueError, "TARGET_CHUNK_SIZE"):
                MainRuntime._validate_hand_act_runtime_settings(
                    inference_hz=30.0,
                    control_hz=100.0,
                    target_chunk_size=100,
                    refill_margin_s=1.0,
                    ordered_async_refill_margin_s=0.03,
                    real_start_deadline_s=15.0,
                )
            with self.assertRaisesRegex(ValueError, "REFILL_MARGIN_S"):
                MainRuntime._validate_hand_act_runtime_settings(
                    inference_hz=30.0,
                    control_hz=100.0,
                    target_chunk_size=None,
                    refill_margin_s=0.06,
                    ordered_async_refill_margin_s=0.03,
                    real_start_deadline_s=15.0,
                )
            with self.assertRaisesRegex(
                ValueError,
                "ORDERED_ASYNC_REFILL_MARGIN_S",
            ):
                MainRuntime._validate_hand_act_runtime_settings(
                    inference_hz=30.0,
                    control_hz=100.0,
                    target_chunk_size=None,
                    refill_margin_s=1.0,
                    ordered_async_refill_margin_s=0.15,
                    real_start_deadline_s=15.0,
                )
            with self.assertRaisesRegex(ValueError, "REAL_START_DEADLINE_S"):
                MainRuntime._validate_hand_act_runtime_settings(
                    inference_hz=30.0,
                    control_hz=100.0,
                    target_chunk_size=None,
                    refill_margin_s=1.0,
                    ordered_async_refill_margin_s=0.03,
                    real_start_deadline_s=19.0,
                )

    def test_real_start_deadline_env_is_strictly_positive(self) -> None:
        for raw in ("0", "-1", "none", "nan", "typo"):
            with self.subTest(raw=raw):
                with patch.dict(os.environ, {
                    "REAL_START_DEADLINE_S": raw,
                }, clear=True):
                    with self.assertRaises(ValueError):
                        MainRuntime._strict_positive_float_env(
                            "REAL_START_DEADLINE_S",
                            "15.0",
                        )


if __name__ == "__main__":
    unittest.main()
