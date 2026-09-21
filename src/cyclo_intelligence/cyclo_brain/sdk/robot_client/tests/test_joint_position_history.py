#!/usr/bin/env python3

from __future__ import annotations

from collections import deque
import importlib.util
import os
from pathlib import Path
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch

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
    "robot_client_joint_history_module",
    MODULE_PATH,
)
robot_client_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(robot_client_module)
RobotClient = robot_client_module.RobotClient


class JointPositionHistoryTests(unittest.TestCase):
    def _robot(self, samples) -> RobotClient:
        robot = RobotClient.__new__(RobotClient)
        robot._lock = threading.Lock()
        # These tests exercise a data-only object without live subscriptions.
        robot._closed = True
        robot._config = {
            "joint_groups": {
                "follower_upper_body": {
                    "joint_names": ["joint_a", "joint_b", "joint_c"],
                },
            },
        }
        robot._joint_history_samples = {
            "follower_upper_body": deque(samples, maxlen=512),
        }
        return robot

    def test_resamples_causally_and_preserves_requested_joint_order(self) -> None:
        now = time.monotonic()
        samples = [
            (
                now - 0.1 * (5 - index),
                np.asarray([index, index + 10, index + 20], dtype=np.float32),
            )
            for index in range(6)
        ]
        robot = self._robot(samples)

        history = robot.get_joint_position_history(
            ["joint_c", "joint_a"],
            history_size=6,
            sample_hz=10.0,
        )

        np.testing.assert_allclose(
            history,
            np.asarray([[20, 0], [21, 1], [22, 2], [23, 3], [24, 4], [25, 5]]),
        )
        self.assertEqual(history.dtype, np.float32)

    def test_rejects_incomplete_history_window(self) -> None:
        now = time.monotonic()
        robot = self._robot(
            [
                (now - 0.1, np.asarray([0, 1, 2], dtype=np.float32)),
                (now, np.asarray([3, 4, 5], dtype=np.float32)),
            ]
        )

        with self.assertRaisesRegex(RuntimeError, "warmup incomplete"):
            robot.get_joint_position_history(
                ["joint_a"],
                history_size=6,
                sample_hz=10.0,
            )

    def test_rejects_stale_latest_sample(self) -> None:
        now = time.monotonic()
        robot = self._robot(
            [
                (
                    now - 1.0 + index * 0.1,
                    np.asarray([index, index, index], dtype=np.float32),
                )
                for index in range(6)
            ]
        )

        with self.assertRaisesRegex(RuntimeError, "stale"):
            robot.get_joint_position_history(
                ["joint_a"],
                history_size=6,
                sample_hz=10.0,
            )


class TactileHistoryTimingTests(unittest.TestCase):
    def _robot(self, timestamps, allowance=None):
        robot = RobotClient.__new__(RobotClient)
        robot._lock = threading.Lock()
        robot._closed = True
        robot._tactile_history_max_age_s = allowance
        robot._tactile_history_samples = {
            "left_hand": deque(
                (stamp, np.full((5, 3, 3), index + 1, dtype=np.float32))
                for index, stamp in enumerate(timestamps)
            ),
        }
        return robot

    def test_hardware_keeps_strict_single_period_gap_limit(self):
        robot = self._robot([9.90, 9.92, 9.99, 10.0])
        with patch.object(robot_client_module.time, "monotonic", return_value=10.0):
            with self.assertRaisesRegex(RuntimeError, "stale resampling gap.*limit=0.033"):
                robot.get_tactile_taxel_history("left_hand", 3, 30.0)

    def test_explicit_simulation_allowance_keeps_causal_values_and_grid(self):
        robot = self._robot([9.90, 9.92, 9.99, 10.0], allowance=0.075)
        with patch.object(robot_client_module.time, "monotonic", return_value=10.0):
            history = robot.get_tactile_taxel_history("left_hand", 3, 30.0)
        self.assertEqual(history.shape, (3, 5, 3, 3))
        # 30 Hz targets choose 9.92, 9.92, 10.0, never the future 9.99 frame.
        np.testing.assert_array_equal(history[:, 0, 0, 0], [2, 2, 4])

    def test_simulation_still_rejects_older_latest_frame(self):
        robot = self._robot([9.90, 9.92, 9.99, 10.0], allowance=0.075)
        with patch.object(robot_client_module.time, "monotonic", return_value=10.08):
            with self.assertRaisesRegex(RuntimeError, "history is stale.*limit=0.075"):
                robot.get_tactile_taxel_history("left_hand", 3, 30.0)

    def test_simulation_still_rejects_large_resampling_gap(self):
        robot = self._robot([9.70, 9.80, 10.0], allowance=0.075)
        with patch.object(robot_client_module.time, "monotonic", return_value=10.0):
            with self.assertRaisesRegex(RuntimeError, "stale resampling gap.*limit=0.075"):
                robot.get_tactile_taxel_history("left_hand", 3, 30.0)

    def test_simulation_still_rejects_missing_window_and_invalid_timestamps(self):
        cases = [([9.98, 10.0], "warmup incomplete"),
                 ([9.90, 9.90, 10.0], "non-monotonic"),
                 ([float("nan"), 10.0], "non-finite")]
        with patch.object(robot_client_module.time, "monotonic", return_value=10.0):
            for timestamps, failure in cases:
                with self.subTest(failure=failure):
                    with self.assertRaisesRegex(RuntimeError, failure):
                        self._robot(timestamps, allowance=0.075).get_tactile_taxel_history(
                            "left_hand", 3, 30.0,
                        )

    def test_simulation_allowance_requires_explicit_mode_and_is_bounded(self):
        read = robot_client_module._simulation_tactile_history_max_age
        with patch.dict(os.environ, {}, clear=True):
            self.assertIsNone(read())
        with patch.dict(os.environ, {"CYCLO_SIM_TACTILE_HISTORY_MAX_AGE_S": "0.075"}, clear=True):
            with self.assertRaisesRegex(ValueError, "requires.*simulation"):
                read()
        for raw in ("invalid", "nan", "inf", "0", "-1", "0.101"):
            with self.subTest(raw=raw):
                with patch.dict(os.environ, {"CYCLO_SENSOR_HISTORY_MODE": "simulation",
                                           "CYCLO_SIM_TACTILE_HISTORY_MAX_AGE_S": raw}, clear=True):
                    with self.assertRaises(ValueError):
                        read()
        with patch.dict(os.environ, {"CYCLO_SENSOR_HISTORY_MODE": "simulation",
                                   "CYCLO_SIM_TACTILE_HISTORY_MAX_AGE_S": "0.075"}, clear=True):
            self.assertEqual(read(), 0.075)


if __name__ == "__main__":
    unittest.main()
