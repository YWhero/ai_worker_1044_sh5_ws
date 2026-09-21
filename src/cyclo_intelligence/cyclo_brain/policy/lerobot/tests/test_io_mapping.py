#!/usr/bin/env python3

import sys
import types
import unittest
import importlib.util
from pathlib import Path

import numpy as np


robot_client_stub = types.ModuleType("robot_client")
robot_client_stub.RobotClient = object
sys.modules.setdefault("robot_client", robot_client_stub)

ENGINE_DIR = Path(__file__).resolve().parents[1] / "lerobot_engine"
package = types.ModuleType("lerobot_engine")
package.__path__ = [str(ENGINE_DIR)]
sys.modules.setdefault("lerobot_engine", package)

spec = importlib.util.spec_from_file_location(
    "lerobot_engine.io_mapping",
    ENGINE_DIR / "io_mapping.py",
)
io_mapping = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = io_mapping
spec.loader.exec_module(io_mapping)
IoMappingMixin = io_mapping.IoMappingMixin
from lerobot_engine.tactile_runtime import (
    TACTILE_MODE_BOTH_EPISODE_BASELINE,
    TACTILE_MODE_LEFT_ZERO_RIGHT_BASELINE,
    TACTILE_RUNTIME_MODE_FIELD,
)


class IoMappingCameraAliasTest(unittest.TestCase):
    def test_maps_rgb_prefixed_cameras_to_policy_keys(self):
        robot_cameras = [
            "rgb.cam_left_head",
            "rgb.cam_right_head",
            "rgb.cam_left_wrist",
            "rgb.cam_right_wrist",
        ]
        policy_keys = {
            "observation.images.cam_left_head",
            "observation.images.cam_right_head",
            "observation.images.cam_left_wrist",
            "observation.images.cam_right_wrist",
        }

        self.assertEqual(
            IoMappingMixin._resolve_camera_mappings(robot_cameras, policy_keys),
            {
                "rgb.cam_left_head": "observation.images.cam_left_head",
                "rgb.cam_right_head": "observation.images.cam_right_head",
                "rgb.cam_left_wrist": "observation.images.cam_left_wrist",
                "rgb.cam_right_wrist": "observation.images.cam_right_wrist",
            },
        )

    def test_keeps_exact_camera_key_preferred(self):
        robot_cameras = ["rgb.cam_left_head"]
        policy_keys = {
            "observation.images.rgb.cam_left_head",
            "observation.images.cam_left_head",
        }

        with self.assertRaisesRegex(RuntimeError, "Missing camera mappings"):
            IoMappingMixin._resolve_camera_mappings(robot_cameras, policy_keys)

        self.assertEqual(
            IoMappingMixin._resolve_camera_mappings(
                robot_cameras,
                {"observation.images.rgb.cam_left_head"},
            ),
            {"rgb.cam_left_head": "observation.images.rgb.cam_left_head"},
        )

    def test_maps_legacy_single_head_policy_key_to_left_head_camera(self):
        self.assertEqual(
            IoMappingMixin._resolve_camera_mappings(
                [
                    "cam_left_head",
                    "cam_left_wrist",
                    "cam_right_wrist",
                ],
                {
                    "observation.images.rgb.cam_head",
                    "observation.images.cam_wrist_left",
                    "observation.images.cam_wrist_right",
                },
            ),
            {
                "cam_left_head": "observation.images.rgb.cam_head",
                "cam_left_wrist": "observation.images.cam_wrist_left",
                "cam_right_wrist": "observation.images.cam_wrist_right",
            },
        )

    def test_maps_scene_and_unprefixed_wrist_keys(self):
        self.assertEqual(
            IoMappingMixin._resolve_camera_mappings(
                ["rgb.cam_left_head", "rgb.cam_left_wrist", "rgb.cam_right_wrist"],
                {
                    "observation.images.scene",
                    "observation.images.wrist_left",
                    "observation.images.wrist_right",
                },
            ),
            {
                "rgb.cam_left_head": "observation.images.scene",
                "rgb.cam_left_wrist": "observation.images.wrist_left",
                "rgb.cam_right_wrist": "observation.images.wrist_right",
            },
        )

    @staticmethod
    def _tactile_mapper(mode):
        mapper = IoMappingMixin()
        config = types.SimpleNamespace()
        setattr(config, TACTILE_RUNTIME_MODE_FIELD, mode)
        mapper._policy = types.SimpleNamespace(config=config)
        mapper._tactile_inputs = {
            "observation.tactile.left": "tactile_left",
            "observation.tactile.right": "tactile_right",
        }
        samples = {
            "tactile_left": np.tile(
                np.asarray([[[2.0, 4.0]]], dtype=np.float32),
                (20, 1, 1, 1),
            ),
            "tactile_right": np.tile(
                np.asarray([[[1.0, 3.0]]], dtype=np.float32),
                (20, 1, 1, 1),
            ),
        }
        mapper._robot = types.SimpleNamespace(
            wait_for_tactile_samples=lambda sensor_name, **_kwargs: samples[
                sensor_name
            ]
        )
        return mapper

    def test_tactile_act_calibrates_both_hands(self):
        mapper = self._tactile_mapper(
            TACTILE_MODE_BOTH_EPISODE_BASELINE
        )

        baselines = mapper._calibrate_tactile_inputs()

        self.assertEqual(set(baselines), set(mapper._tactile_inputs))
        np.testing.assert_allclose(
            baselines["observation.tactile.left"],
            [[[2.0, 4.0]]],
        )

    def test_legacy_mode_skips_left_hand_calibration(self):
        mapper = self._tactile_mapper(
            TACTILE_MODE_LEFT_ZERO_RIGHT_BASELINE
        )

        baselines = mapper._calibrate_tactile_inputs()

        self.assertEqual(
            set(baselines),
            {"observation.tactile.right"},
        )


if __name__ == "__main__":
    unittest.main()
