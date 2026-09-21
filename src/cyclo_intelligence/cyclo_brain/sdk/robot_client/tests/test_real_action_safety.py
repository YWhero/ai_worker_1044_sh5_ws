#!/usr/bin/env python3

from __future__ import annotations

import sys
import threading
import time
import types
import unittest
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[4]
ROBOT_CLIENT_ROOT = Path(__file__).resolve().parents[1]
SCHEMA_ROOT = REPO_ROOT / "shared" / "shared" / "robot_configs"
for path in (ROBOT_CLIENT_ROOT, SCHEMA_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

zenoh_stub = types.ModuleType("zenoh_ros2_sdk")
zenoh_stub.ROS2Publisher = object
zenoh_stub.ROS2Subscriber = object
zenoh_stub.ROS2ServiceServer = object
zenoh_stub.get_message_class = lambda _name: object
sys.modules.setdefault("zenoh_ros2_sdk", zenoh_stub)

cv2_stub = types.ModuleType("cv2")
cv2_stub.IMREAD_COLOR = 1
cv2_stub.COLOR_BGR2RGB = 4
sys.modules.setdefault("cv2", cv2_stub)

MODULE_PATH = ROBOT_CLIENT_ROOT / "robot_client" / "robot_client.py"
spec = importlib.util.spec_from_file_location(
    "robot_client_real_action_safety_module",
    MODULE_PATH,
)
robot_client_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(robot_client_module)
RobotClient = robot_client_module.RobotClient


class RealActionSafetyTests(unittest.TestCase):
    def _robot(self) -> RobotClient:
        robot = RobotClient.__new__(RobotClient)
        robot._closed = True
        robot._lock = threading.Lock()
        robot._action_keys = ["arm"]
        robot._recorded_action_keys = ["arm"]
        robot._action_groups = {
            "arm": {
                "msg_type": "trajectory_msgs/msg/JointTrajectory",
                "joint_names": ["joint_a", "joint_b"],
            }
        }
        robot._command_publishers = {"leader_arm": object()}
        robot._joint_position_limits = {
            "joint_a": (0.0, 1.0),
            "joint_b": (-1.0, 1.0),
        }
        robot._joint_positions = {
            "follower_arm": np.asarray([0.5, 0.0], dtype=np.float32)
        }
        robot._joint_timestamps = {"follower_arm": time.time()}
        robot._joint_velocities = {}
        robot._joint_efforts = {}
        robot._joint_children = {}
        robot._config = {
            "joint_groups": {
                "follower_arm": {
                    "joint_names": ["joint_a", "joint_b"],
                }
            }
        }
        return robot

    def test_small_urdf_violation_is_clamped_without_mutating_input(self) -> None:
        robot = self._robot()
        original = np.asarray([[0.5, 1.01], [0.6, -1.005]], dtype=np.float64)

        with self.assertLogs("robot_client", level="WARNING") as captured:
            safe = robot.apply_real_action_safety(
                original,
                joint_limit_mode="clamp",
                joint_limit_tolerance_rad=0.02,
            )

        np.testing.assert_allclose(safe, [[0.5, 1.0], [0.6, -1.0]])
        np.testing.assert_allclose(original, [[0.5, 1.01], [0.6, -1.005]])
        self.assertIn("joint=joint_b", captured.output[0])
        self.assertIn("violation=0.010000", captured.output[0])
        self.assertIn("tolerance=0.020000", captured.output[0])

    def test_large_urdf_violation_rejects_complete_chunk(self) -> None:
        robot = self._robot()

        with self.assertRaisesRegex(ValueError, "joint_b.*0.030000 rad"):
            robot.apply_real_action_safety(
                np.asarray([[0.5, 1.03]], dtype=np.float64),
                joint_limit_mode="clamp",
                joint_limit_tolerance_rad=0.02,
            )

    def test_joint_specific_urdf_tolerance_clamps_only_audited_joint(
        self,
    ) -> None:
        robot = self._robot()

        with self.assertLogs("robot_client", level="WARNING") as captured:
            safe = robot.apply_real_action_safety(
                np.asarray([[0.5, 1.03]], dtype=np.float64),
                joint_limit_mode="clamp",
                joint_limit_tolerance_rad=0.02,
                joint_limit_tolerance_by_joint={"joint_b": 0.04},
            )

        np.testing.assert_allclose(safe, [[0.5, 1.0]])
        self.assertIn("joint=joint_b", captured.output[0])
        self.assertIn("tolerance=0.040000", captured.output[0])

        with self.assertRaisesRegex(ValueError, "joint_a.*0.030000 rad"):
            robot.apply_real_action_safety(
                np.asarray([[1.03, 0.0]], dtype=np.float64),
                joint_limit_mode="clamp",
                joint_limit_tolerance_rad=0.02,
                joint_limit_tolerance_by_joint={"joint_b": 0.04},
            )

    def test_joint_specific_urdf_tolerance_keeps_its_own_hard_cap(self) -> None:
        robot = self._robot()

        with self.assertRaisesRegex(ValueError, "joint_b.*0.050000 rad"):
            robot.apply_real_action_safety(
                np.asarray([[0.5, 1.05]], dtype=np.float64),
                joint_limit_mode="clamp",
                joint_limit_tolerance_rad=0.02,
                joint_limit_tolerance_by_joint={"joint_b": 0.04},
            )

    def test_act0820_left_joint2_prediction_is_clamped_to_urdf_boundary(
        self,
    ) -> None:
        robot = self._robot()
        robot._action_groups["arm"]["joint_names"] = [
            "arm_l_joint1",
            "arm_l_joint2",
        ]
        robot._joint_position_limits = {
            "arm_l_joint1": (-3.14, 3.14),
            "arm_l_joint2": (0.0, 3.14),
        }
        robot._joint_positions["follower_arm"] = np.asarray(
            [0.5, 0.0],
            dtype=np.float32,
        )

        with self.assertLogs("robot_client", level="WARNING") as captured:
            safe = robot.apply_real_action_safety(
                np.asarray([[0.5, -0.028265]], dtype=np.float64),
                joint_limit_mode="clamp",
                joint_limit_tolerance_rad=0.02,
                joint_limit_tolerance_by_joint={"arm_l_joint2": 0.18},
            )

        np.testing.assert_allclose(safe, [[0.5, 0.0]])
        self.assertIn("joint=arm_l_joint2", captured.output[0])
        self.assertIn("violation=0.028265", captured.output[0])

        with self.assertRaisesRegex(
            ValueError,
            "arm_l_joint2.*0.180001 rad",
        ):
            robot.apply_real_action_safety(
                np.asarray([[0.5, -0.180001]], dtype=np.float64),
                joint_limit_mode="clamp",
                joint_limit_tolerance_rad=0.02,
                joint_limit_tolerance_by_joint={"arm_l_joint2": 0.18},
            )

    def test_tactile_act_left_finger6_negative_tail_is_bounded_and_clamped(
        self,
    ) -> None:
        robot = self._robot()
        robot._action_groups["arm"]["joint_names"] = [
            "finger_l_joint6",
            "other_finger",
        ]
        robot._joint_position_limits = {
            "finger_l_joint6": (0.0, 2.0),
            "other_finger": (0.0, 2.0),
        }

        with self.assertLogs("robot_client", level="WARNING") as captured:
            safe = robot.apply_real_action_safety(
                np.asarray([[-0.028154, 0.5]], dtype=np.float64),
                joint_limit_mode="clamp",
                joint_limit_tolerance_rad=0.02,
                joint_limit_tolerance_by_joint={"finger_l_joint6": 0.05},
            )

        np.testing.assert_allclose(safe, [[0.0, 0.5]])
        self.assertIn("joint=finger_l_joint6", captured.output[0])
        self.assertIn("violation=0.028154", captured.output[0])
        self.assertIn("tolerance=0.050000", captured.output[0])

        with self.assertRaisesRegex(
            ValueError,
            "finger_l_joint6.*0.050001 rad",
        ):
            robot.apply_real_action_safety(
                np.asarray([[-0.050001, 0.5]], dtype=np.float64),
                joint_limit_mode="clamp",
                joint_limit_tolerance_rad=0.02,
                joint_limit_tolerance_by_joint={"finger_l_joint6": 0.05},
            )

    def test_tactile_act_left_finger14_negative_tail_is_bounded_and_clamped(
        self,
    ) -> None:
        robot = self._robot()
        robot._action_groups["arm"]["joint_names"] = [
            "finger_l_joint14",
            "other_finger",
        ]
        robot._joint_position_limits = {
            "finger_l_joint14": (0.0, 2.0),
            "other_finger": (0.0, 2.0),
        }

        with self.assertLogs("robot_client", level="WARNING") as captured:
            safe = robot.apply_real_action_safety(
                np.asarray([[-0.026017, 0.5]], dtype=np.float64),
                joint_limit_mode="clamp",
                joint_limit_tolerance_rad=0.02,
                joint_limit_tolerance_by_joint={"finger_l_joint14": 0.04},
            )

        np.testing.assert_allclose(safe, [[0.0, 0.5]])
        self.assertIn("joint=finger_l_joint14", captured.output[0])
        self.assertIn("violation=0.026017", captured.output[0])
        self.assertIn("tolerance=0.040000", captured.output[0])

        with self.assertRaisesRegex(
            ValueError,
            "finger_l_joint14.*0.040001 rad",
        ):
            robot.apply_real_action_safety(
                np.asarray([[-0.040001, 0.5]], dtype=np.float64),
                joint_limit_mode="clamp",
                joint_limit_tolerance_rad=0.02,
                joint_limit_tolerance_by_joint={"finger_l_joint14": 0.04},
            )

    def test_tactile_act_left_finger10_final_cup_tail_is_bounded_and_clamped(
        self,
    ) -> None:
        robot = self._robot()
        robot._action_groups["arm"]["joint_names"] = [
            "finger_l_joint10",
            "other_finger",
        ]
        robot._joint_position_limits = {
            "finger_l_joint10": (0.0, 2.0),
            "other_finger": (0.0, 2.0),
        }

        with self.assertLogs("robot_client", level="WARNING") as captured:
            safe = robot.apply_real_action_safety(
                np.asarray([[-0.024089, 0.5]], dtype=np.float64),
                joint_limit_mode="clamp",
                joint_limit_tolerance_rad=0.02,
                joint_limit_tolerance_by_joint={"finger_l_joint10": 0.04},
            )

        np.testing.assert_allclose(safe, [[0.0, 0.5]])
        self.assertIn("joint=finger_l_joint10", captured.output[0])
        self.assertIn("violation=0.024089", captured.output[0])
        self.assertIn("tolerance=0.040000", captured.output[0])

        with self.assertRaisesRegex(
            ValueError,
            "finger_l_joint10.*0.040001 rad",
        ):
            robot.apply_real_action_safety(
                np.asarray([[-0.040001, 0.5]], dtype=np.float64),
                joint_limit_mode="clamp",
                joint_limit_tolerance_rad=0.02,
                joint_limit_tolerance_by_joint={"finger_l_joint10": 0.04},
            )

    def test_tactile_act_left_finger18_cycle_tail_is_bounded_and_clamped(
        self,
    ) -> None:
        robot = self._robot()
        robot._action_groups["arm"]["joint_names"] = [
            "finger_l_joint18",
            "other_finger",
        ]
        robot._joint_position_limits = {
            "finger_l_joint18": (0.0, 2.0),
            "other_finger": (0.0, 2.0),
        }

        with self.assertLogs("robot_client", level="WARNING") as captured:
            safe = robot.apply_real_action_safety(
                np.asarray([[-0.020227, 0.5]], dtype=np.float64),
                joint_limit_mode="clamp",
                joint_limit_tolerance_rad=0.02,
                joint_limit_tolerance_by_joint={"finger_l_joint18": 0.04},
            )

        np.testing.assert_allclose(safe, [[0.0, 0.5]])
        self.assertIn("joint=finger_l_joint18", captured.output[0])
        self.assertIn("violation=0.020227", captured.output[0])
        self.assertIn("tolerance=0.040000", captured.output[0])

        with self.assertRaisesRegex(
            ValueError,
            "finger_l_joint18.*0.040001 rad",
        ):
            robot.apply_real_action_safety(
                np.asarray([[-0.040001, 0.5]], dtype=np.float64),
                joint_limit_mode="clamp",
                joint_limit_tolerance_rad=0.02,
                joint_limit_tolerance_by_joint={"finger_l_joint18": 0.04},
            )

    def test_joint_specific_urdf_tolerance_rejects_unknown_joint(self) -> None:
        robot = self._robot()

        with self.assertRaisesRegex(ValueError, "unknown action joints: ghost"):
            robot.apply_real_action_safety(
                np.asarray([[0.5, 0.0]], dtype=np.float64),
                joint_limit_mode="clamp",
                joint_limit_tolerance_rad=0.02,
                joint_limit_tolerance_by_joint={"ghost": 0.04},
            )

    def test_joint_specific_urdf_tolerance_requires_clamp_mode(self) -> None:
        robot = self._robot()

        with self.assertRaisesRegex(ValueError, "requires clamp mode"):
            robot.apply_real_action_safety(
                np.asarray([[0.5, 0.0]], dtype=np.float64),
                joint_limit_mode="reject",
                joint_limit_tolerance_by_joint={"joint_b": 0.04},
            )

    def test_first_action_uses_fresh_live_state_and_rejects_jump(self) -> None:
        robot = self._robot()

        with self.assertRaisesRegex(ValueError, "joint_a=0.040000 rad"):
            robot.apply_real_action_safety(
                np.asarray([[0.54, 0.0]], dtype=np.float64),
                first_action_max_delta_rad=0.03,
                state_max_age_s=0.5,
                joint_limit_mode="reject",
            )

    def test_per_action_key_first_delta_overrides_scalar_default(self) -> None:
        robot = self._robot()

        safe = robot.apply_real_action_safety(
            np.asarray([[0.54, 0.0]], dtype=np.float64),
            first_action_max_delta_rad=0.03,
            first_action_max_delta_by_key={"arm": 0.10},
            state_max_age_s=0.5,
            joint_limit_mode="reject",
        )

        np.testing.assert_allclose(safe, [[0.54, 0.0]])

    def test_bounded_warm_start_inserts_source_safe_bridge(self) -> None:
        robot = self._robot()
        original = np.asarray(
            [[0.576, 0.09], [0.586, 0.10]],
            dtype=np.float64,
        )

        safe = robot.apply_real_action_safety(
            original,
            first_action_max_delta_rad=0.03,
            warm_start_max_total_delta_by_key={"arm": 0.10},
            source_step_max_delta_rad=0.03,
            state_max_age_s=0.5,
            joint_limit_mode="reject",
        )

        self.assertEqual(safe.shape, (4, 2))
        np.testing.assert_allclose(safe[-1], original[-1])
        self.assertLessEqual(float(np.max(np.abs(safe[0] - [0.5, 0.0]))), 0.03)
        self.assertLessEqual(float(np.max(np.abs(np.diff(safe, axis=0)))), 0.03)
        np.testing.assert_allclose(original, [[0.576, 0.09], [0.586, 0.10]])

    def test_warm_start_total_delta_still_rejects_large_mismatch(self) -> None:
        robot = self._robot()

        with self.assertRaisesRegex(
            ValueError,
            "unsafe warm-start total delta.*joint_a=0.120000",
        ):
            robot.apply_real_action_safety(
                np.asarray([[0.62, 0.0]], dtype=np.float64),
                first_action_max_delta_rad=0.03,
                warm_start_max_total_delta_by_key={"arm": 0.10},
                source_step_max_delta_rad=0.03,
                state_max_age_s=0.5,
                joint_limit_mode="reject",
            )

    def test_warm_start_requires_every_position_group_total_limit(self) -> None:
        robot = self._robot()

        with self.assertRaisesRegex(ValueError, "limit missing.*arm"):
            robot.apply_real_action_safety(
                np.asarray([[0.51, 0.0]], dtype=np.float64),
                first_action_max_delta_rad=0.03,
                warm_start_max_total_delta_by_key={"other": 0.10},
                source_step_max_delta_rad=0.03,
                state_max_age_s=0.5,
                joint_limit_mode="reject",
            )

    def test_warm_start_only_interpolates_groups_over_their_first_gate(self) -> None:
        robot = self._robot()
        robot._action_keys = ["arm", "hand"]
        robot._recorded_action_keys = ["arm", "hand"]
        robot._action_groups["hand"] = {
            "msg_type": "trajectory_msgs/msg/JointTrajectory",
            "joint_names": ["finger"],
        }
        robot._joint_position_limits["finger"] = (0.0, 1.0)
        robot._joint_positions["follower_hand"] = np.asarray(
            [-0.09],
            dtype=np.float32,
        )
        robot._joint_timestamps["follower_hand"] = time.time()

        safe = robot.apply_real_action_safety(
            np.asarray([[0.576, 0.0, 0.0]], dtype=np.float64),
            action_keys=["arm", "hand"],
            first_action_max_delta_rad=0.03,
            first_action_max_delta_by_key={"arm": 0.03, "hand": 0.10},
            warm_start_max_total_delta_by_key={"arm": 0.10, "hand": 0.12},
            source_step_max_delta_rad=0.03,
            state_max_age_s=0.5,
            joint_limit_mode="reject",
        )

        self.assertEqual(safe.shape, (3, 3))
        self.assertLessEqual(float(np.max(np.abs(safe[0, :2] - [0.5, 0.0]))), 0.03)
        # The hand's target is already within its 0.10-rad first-action gate,
        # so every bridge row uses the valid zero command instead of copying
        # the out-of-range measured encoder offset.
        np.testing.assert_allclose(safe[:, 2], 0.0)

    def test_warm_start_clamps_small_out_of_range_state_before_bridge(self) -> None:
        robot = self._robot()
        robot._action_keys = ["hand"]
        robot._recorded_action_keys = ["hand"]
        robot._action_groups["hand"] = {
            "msg_type": "trajectory_msgs/msg/JointTrajectory",
            "joint_names": ["finger"],
        }
        robot._joint_position_limits["finger"] = (0.0, 1.0)
        robot._joint_positions["follower_hand"] = np.asarray(
            [-0.06],
            dtype=np.float32,
        )
        robot._joint_timestamps["follower_hand"] = time.time()

        safe = robot.apply_real_action_safety(
            np.asarray([[0.45]], dtype=np.float64),
            action_keys=["hand"],
            first_action_max_delta_by_key={"hand": 0.10},
            warm_start_max_total_delta_by_key={"hand": 0.70},
            source_step_max_delta_rad=0.03,
            state_max_age_s=0.5,
            joint_limit_mode="clamp",
            joint_limit_tolerance_rad=0.02,
        )

        np.testing.assert_allclose(safe[0], [0.0])
        self.assertLessEqual(abs(float(safe[0, 0]) - (-0.06)), 0.10)
        np.testing.assert_allclose(safe[-1], [0.45])
        self.assertLessEqual(
            float(np.max(np.abs(np.diff(safe[:, 0])))),
            0.03,
        )

    def test_tracking_bridge_bounds_arm_without_interpolating_hand_offset(self) -> None:
        robot = self._robot()
        robot._action_keys = ["arm", "hand"]
        robot._recorded_action_keys = ["arm", "hand"]
        robot._action_groups["hand"] = {
            "msg_type": "trajectory_msgs/msg/JointTrajectory",
            "joint_names": ["finger"],
        }
        robot._joint_position_limits["finger"] = (0.0, 1.0)
        robot._joint_positions["follower_hand"] = np.asarray(
            [-0.30],
            dtype=np.float32,
        )
        robot._joint_timestamps["follower_hand"] = time.time()

        safe = robot.apply_real_action_safety(
            np.asarray([[0.560864, 0.0, 0.40]], dtype=np.float64),
            action_keys=["arm", "hand"],
            first_action_max_delta_by_key={"arm": 0.06, "hand": 1.20},
            state_bridge_max_total_delta_by_key={"arm": 0.30},
            source_step_max_delta_rad=0.03,
            state_max_age_s=0.5,
            joint_limit_mode="reject",
        )

        self.assertEqual(safe.shape, (3, 3))
        self.assertLessEqual(float(np.max(np.abs(safe[0, :2] - [0.5, 0.0]))), 0.03)
        np.testing.assert_allclose(safe[-1, :2], [0.560864, 0.0])
        # Hand encoder zero and command zero are not interchangeable. The hand
        # stays on the model command while only the arm slice is state-bridged.
        np.testing.assert_allclose(safe[:, 2], 0.40)

    def test_tracking_bridge_rejects_arm_lag_above_dataset_hard_cap(self) -> None:
        robot = self._robot()

        with self.assertRaisesRegex(
            ValueError,
            "unsafe state-tracking total delta.*joint_a=0.310000",
        ):
            robot.apply_real_action_safety(
                np.asarray([[0.81, 0.0]], dtype=np.float64),
                first_action_max_delta_by_key={"arm": 0.06},
                state_bridge_max_total_delta_by_key={"arm": 0.30},
                source_step_max_delta_rad=0.03,
                state_max_age_s=0.5,
                joint_limit_mode="reject",
            )

    def test_previous_command_caps_tracking_bridge_output_step(self) -> None:
        robot = self._robot()
        previous = np.asarray([0.484448, 0.0], dtype=np.float64)

        safe = robot.apply_real_action_safety(
            np.asarray([[0.560864, 0.0]], dtype=np.float64),
            first_action_max_delta_by_key={"arm": 0.06},
            state_bridge_max_total_delta_by_key={"arm": 0.30},
            previous_published_action=previous,
            source_step_max_delta_rad=0.03,
            state_max_age_s=0.5,
            joint_limit_mode="reject",
        )

        self.assertLessEqual(
            float(np.max(np.abs(safe[0] - previous))),
            0.03 + 1e-12,
        )
        np.testing.assert_allclose(safe[0], [0.514448, 0.0])

    def test_previous_command_gate_rejects_conflict_with_state_hard_cap(self) -> None:
        robot = self._robot()

        with self.assertRaisesRegex(
            ValueError,
            "previous-command step limiting cannot satisfy state hard cap",
        ):
            robot.apply_real_action_safety(
                np.asarray([[0.50, 0.0]], dtype=np.float64),
                first_action_max_delta_by_key={"arm": 0.06},
                state_bridge_max_total_delta_by_key={"arm": 0.30},
                previous_published_action=np.asarray([0.0, 0.0]),
                source_step_max_delta_rad=0.03,
                state_max_age_s=0.5,
                joint_limit_mode="reject",
            )

    def test_tracking_lag_does_not_reverse_an_already_published_command(self) -> None:
        for direction in (1, -1):
            with self.subTest(direction=direction):
                robot = self._robot()
                previous = np.asarray([0.5 + direction * 0.059, 0.0])
                desired = np.asarray([[0.5 + direction * 0.065, 0.0]])
                safe = robot.apply_real_action_safety(
                    desired, first_action_max_delta_by_key={'arm': 0.06},
                    state_bridge_max_total_delta_by_key={'arm': 0.30},
                    previous_published_action=previous,
                    source_step_max_delta_rad=0.03,
                )
                np.testing.assert_allclose(safe[0], previous)
                np.testing.assert_allclose(safe[-1], desired[0])
                self.assertTrue(np.all(direction * np.diff(safe[:, 0]) >= -1e-12))

    def test_tracking_bridge_preserves_intentional_model_reversal(self) -> None:
        robot = self._robot()
        previous = np.asarray([0.60, 0.0])
        safe = robot.apply_real_action_safety(
            np.asarray([[0.565, 0.0]]),
            first_action_max_delta_by_key={'arm': 0.06},
            state_bridge_max_total_delta_by_key={'arm': 0.30},
            previous_published_action=previous, source_step_max_delta_rad=0.03,
        )
        self.assertLess(safe[0, 0], previous[0])
        self.assertGreaterEqual(safe[0, 0], 0.565)
        self.assertLessEqual(previous[0] - safe[0, 0], 0.03 + 1e-12)

    def test_warm_start_and_tracking_bridge_cannot_mix(self) -> None:
        robot = self._robot()

        with self.assertRaisesRegex(ValueError, "cannot be enabled"):
            robot.apply_real_action_safety(
                np.asarray([[0.51, 0.0]], dtype=np.float64),
                first_action_max_delta_rad=0.03,
                warm_start_max_total_delta_by_key={"arm": 0.10},
                state_bridge_max_total_delta_by_key={"arm": 0.30},
                source_step_max_delta_rad=0.03,
                joint_limit_mode="reject",
            )

    def test_stale_live_state_fails_closed(self) -> None:
        robot = self._robot()
        robot._joint_timestamps["follower_arm"] = time.time() - 1.0

        with self.assertRaisesRegex(ValueError, "current state is stale"):
            robot.apply_real_action_safety(
                np.asarray([[0.5, 0.0]], dtype=np.float64),
                first_action_max_delta_rad=0.03,
                state_max_age_s=0.5,
                joint_limit_mode="reject",
            )

    def test_source_step_delta_rejects_complete_chunk(self) -> None:
        robot = self._robot()

        with self.assertRaisesRegex(ValueError, "unsafe source-step delta"):
            robot.apply_real_action_safety(
                np.asarray([[0.5, 0.0], [0.54, 0.0]], dtype=np.float64),
                source_step_max_delta_rad=0.03,
                joint_limit_mode="reject",
            )

    def test_bounded_source_step_bridge_subdivides_small_discontinuity(self) -> None:
        robot = self._robot()
        original = np.asarray(
            [[0.50, 0.0], [0.537, 0.0], [0.547, 0.0]],
            dtype=np.float64,
        )

        safe = robot.apply_real_action_safety(
            original,
            source_step_max_delta_rad=0.03,
            source_step_bridge_max_raw_delta_by_key={"arm": 0.05},
            joint_limit_mode="reject",
        )

        self.assertEqual(safe.shape, (4, 2))
        np.testing.assert_allclose(safe[[0, -1]], original[[0, -1]])
        self.assertLessEqual(float(np.max(np.abs(np.diff(safe, axis=0)))), 0.03)
        np.testing.assert_allclose(original, [[0.50, 0.0], [0.537, 0.0], [0.547, 0.0]])

    def test_source_step_bridge_rejects_raw_jump_above_hard_cap(self) -> None:
        robot = self._robot()

        with self.assertRaisesRegex(
            ValueError,
            "unsafe raw source-step delta.*0.051000 rad.*0.050000 rad",
        ):
            robot.apply_real_action_safety(
                np.asarray([[0.50, 0.0], [0.551, 0.0]], dtype=np.float64),
                source_step_max_delta_rad=0.03,
                source_step_bridge_max_raw_delta_by_key={"arm": 0.05},
                joint_limit_mode="reject",
            )

    def test_source_step_bridge_requires_each_position_group_limit(self) -> None:
        robot = self._robot()

        with self.assertRaisesRegex(ValueError, "limit missing.*arm"):
            robot.apply_real_action_safety(
                np.asarray([[0.50, 0.0], [0.51, 0.0]], dtype=np.float64),
                source_step_max_delta_rad=0.03,
                source_step_bridge_max_raw_delta_by_key={"other": 0.05},
                joint_limit_mode="reject",
            )

    def test_nonfinite_and_wrong_dimension_are_rejected(self) -> None:
        robot = self._robot()

        with self.assertRaisesRegex(ValueError, "non-finite"):
            robot.apply_real_action_safety(
                np.asarray([[np.nan, 0.0]], dtype=np.float64)
            )
        with self.assertRaisesRegex(ValueError, "dimension mismatch"):
            robot.apply_real_action_safety(
                np.asarray([[0.0]], dtype=np.float64)
            )

    def test_real_action_contract_rejects_duplicate_keys(self) -> None:
        robot = self._robot()

        with self.assertRaisesRegex(ValueError, "duplicates: arm"):
            robot.validate_real_action_contract(["arm", "arm"])

    def test_real_action_contract_rejects_permuted_keys(self) -> None:
        robot = self._robot()
        robot._action_keys = ["arm", "hand"]
        robot._recorded_action_keys = ["arm", "hand"]
        robot._action_groups["hand"] = {
            "msg_type": "trajectory_msgs/msg/JointTrajectory",
            "joint_names": ["finger"],
        }
        robot._command_publishers["leader_hand"] = object()

        with self.assertRaisesRegex(ValueError, "expected ordered keys"):
            robot.validate_real_action_contract(["hand", "arm"])

    def test_real_action_contract_rejects_missing_publisher(self) -> None:
        robot = self._robot()
        robot._command_publishers.clear()

        with self.assertRaisesRegex(ValueError, "leader_arm"):
            robot.validate_real_action_contract(["arm"])

    def test_actual_sh5_urdf_limits_are_loaded(self) -> None:
        urdf = SCHEMA_ROOT / "urdf" / "ffw_sh5_follower.urdf"

        limits = RobotClient._load_joint_position_limits(str(urdf))

        self.assertEqual(limits["arm_l_joint2"], (0.0, 3.14))
        self.assertEqual(limits["arm_r_joint2"], (-3.14, 0.0))
        self.assertEqual(len([name for name in limits if name.startswith("finger_")]), 40)

    def test_joint_position_snapshot_copies_position_and_timestamp_atomically(self) -> None:
        robot = self._robot()
        expected_timestamp = robot._joint_timestamps["follower_arm"]

        positions, timestamp = robot.get_joint_position_snapshot("follower_arm")
        positions[0] = 999.0

        self.assertEqual(timestamp, expected_timestamp)
        self.assertEqual(robot._joint_positions["follower_arm"][0], 0.5)

    def test_invalid_position_slice_does_not_refresh_position_timestamp(self) -> None:
        robot = self._robot()
        old_timestamp = robot._joint_timestamps["follower_arm"]
        msg = SimpleNamespace(
            name=["joint_a", "joint_b"],
            position=[],
            velocity=[1.0, 2.0],
            effort=[],
        )

        robot._update_joint("follower_arm", msg)

        self.assertEqual(robot._joint_timestamps["follower_arm"], old_timestamp)
        np.testing.assert_allclose(robot._joint_velocities["follower_arm"], [1.0, 2.0])

    def test_invalid_synthetic_position_slice_has_no_timestamp(self) -> None:
        robot = self._robot()
        robot._config["joint_groups"]["follower_child"] = {
            "joint_names": ["joint_a", "joint_b"],
        }
        robot._joint_children = {"follower_arm": ["follower_child"]}
        msg = SimpleNamespace(
            name=["joint_a", "joint_b"],
            position=[0.1],
            velocity=[],
            effort=[],
        )

        robot._update_joint("follower_arm", msg)

        self.assertNotIn("follower_child", robot._joint_positions)
        self.assertNotIn("follower_child", robot._joint_timestamps)


if __name__ == "__main__":
    unittest.main()
