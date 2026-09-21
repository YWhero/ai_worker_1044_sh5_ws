#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys
import types
import unittest

import numpy as np
import torch


ENGINE_DIR = Path(__file__).resolve().parents[1] / "lerobot_engine"
package = types.ModuleType("lerobot_engine")
package.__path__ = [str(ENGINE_DIR)]
sys.modules.setdefault("lerobot_engine", package)

constants_spec = importlib.util.spec_from_file_location(
    "lerobot_engine.constants",
    ENGINE_DIR / "constants.py",
)
constants = importlib.util.module_from_spec(constants_spec)
sys.modules[constants_spec.name] = constants
constants_spec.loader.exec_module(constants)

image_spec = importlib.util.spec_from_file_location(
    "lerobot_engine.image_preprocessing",
    ENGINE_DIR / "image_preprocessing.py",
)
image_preprocessing = importlib.util.module_from_spec(image_spec)
sys.modules[image_spec.name] = image_preprocessing
image_spec.loader.exec_module(image_preprocessing)

preprocessing_spec = importlib.util.spec_from_file_location(
    "lerobot_engine.preprocessing",
    ENGINE_DIR / "preprocessing.py",
)
preprocessing = importlib.util.module_from_spec(preprocessing_spec)
sys.modules[preprocessing_spec.name] = preprocessing
preprocessing_spec.loader.exec_module(preprocessing)

PreprocessingMixin = preprocessing.PreprocessingMixin
STATE_KEY = constants.STATE_KEY
from lerobot_engine.tactile_runtime import (
    TACTILE_MODE_BOTH_EPISODE_BASELINE,
    TACTILE_MODE_LEFT_ZERO_RIGHT_BASELINE,
    TACTILE_RUNTIME_MODE_FIELD,
)


class FakeRobot:
    _config = {"cameras": {}}

    def __init__(self, positions):
        self._positions = positions

    def get_images(self, format="rgb"):
        return {"unused": np.zeros((2, 2, 3), dtype=np.uint8)}

    def get_joint_positions(self):
        return {"follower_arm": self._positions}


class Preprocessor(PreprocessingMixin):
    def __init__(self, positions, expected):
        self._robot = FakeRobot(positions)
        self._cameras = {}
        self._state_modalities = ["arm"]
        self._image_resize = {}
        self._device = torch.device("cpu")
        feature = SimpleNamespace(shape=(expected,))
        config = SimpleNamespace(input_features={STATE_KEY: feature})
        self._policy = SimpleNamespace(config=config)

    def _fail(self, message):
        return {"error": message}


class FakeTactileRobot:
    def __init__(self, values):
        self._values = values

    def get_tactile_taxels(self, sensor_name):
        return np.asarray(self._values[sensor_name], dtype=np.float32)


class TactilePreprocessor(PreprocessingMixin):
    def __init__(self, mode):
        self._robot = FakeTactileRobot({
            "tactile_left": [[[5.0, 9.0]]],
            "tactile_right": [[[7.0, 3.0]]],
        })
        self._tactile_inputs = {
            "observation.tactile.left": "tactile_left",
            "observation.tactile.right": "tactile_right",
        }
        self._tactile_baselines = {
            "observation.tactile.left": np.asarray(
                [[[2.0, 10.0]]], dtype=np.float32
            ),
            "observation.tactile.right": np.asarray(
                [[[1.0, 1.0]]], dtype=np.float32
            ),
        }
        self._device = torch.device("cpu")
        config = SimpleNamespace(
            input_features={
                key: SimpleNamespace(shape=(1, 1, 2))
                for key in self._tactile_inputs
            }
        )
        setattr(config, TACTILE_RUNTIME_MODE_FIELD, mode)
        self._policy = SimpleNamespace(config=config)


class PreprocessingTest(unittest.TestCase):
    def test_pads_short_state_to_policy_shape(self):
        preprocessor = Preprocessor([1.0, 2.0], expected=4)

        batch = preprocessor._build_observation("task")

        np.testing.assert_allclose(
            batch[STATE_KEY].numpy(),
            np.asarray([[1.0, 2.0, 0.0, 0.0]], dtype=np.float32),
        )

    def test_truncates_long_state_to_policy_shape(self):
        preprocessor = Preprocessor([1.0, 2.0, 3.0, 4.0], expected=2)

        batch = preprocessor._build_observation("task")

        np.testing.assert_allclose(
            batch[STATE_KEY].numpy(),
            np.asarray([[1.0, 2.0]], dtype=np.float32),
        )

    def test_tactile_act_baseline_corrects_both_live_hands(self):
        preprocessor = TactilePreprocessor(
            TACTILE_MODE_BOTH_EPISODE_BASELINE
        )
        batch = {}

        preprocessor._add_tactile_observations(batch)

        np.testing.assert_allclose(
            batch["observation.tactile.left"].numpy(),
            [[[[3.0, 0.0]]]],
        )
        np.testing.assert_allclose(
            batch["observation.tactile.right"].numpy(),
            [[[[6.0, 2.0]]]],
        )

    def test_legacy_corrected_mode_keeps_left_zero_contract(self):
        preprocessor = TactilePreprocessor(
            TACTILE_MODE_LEFT_ZERO_RIGHT_BASELINE
        )
        del preprocessor._tactile_baselines[
            "observation.tactile.left"
        ]
        batch = {}

        preprocessor._add_tactile_observations(batch)

        np.testing.assert_allclose(
            batch["observation.tactile.left"].numpy(),
            [[[[0.0, 0.0]]]],
        )
        np.testing.assert_allclose(
            batch["observation.tactile.right"].numpy(),
            [[[[6.0, 2.0]]]],
        )

if __name__ == "__main__":
    unittest.main()
