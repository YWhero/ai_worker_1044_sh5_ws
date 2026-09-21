#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys
import types
from unittest.mock import Mock

import numpy as np
import pytest
import torch


ENGINE_DIR = Path(__file__).resolve().parents[1] / "vitacformer_engine"

robot_client_stub = types.ModuleType("robot_client")
robot_client_stub.RobotClient = object
sys.modules.setdefault("robot_client", robot_client_stub)

package = types.ModuleType("vitacformer_engine")
package.__path__ = [str(ENGINE_DIR)]
sys.modules.setdefault("vitacformer_engine", package)


def _load_module(name: str):
    qualified = f"vitacformer_engine.{name}"
    spec = importlib.util.spec_from_file_location(
        qualified,
        ENGINE_DIR / f"{name}.py",
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[qualified] = module
    spec.loader.exec_module(module)
    return module


constants = _load_module("constants")
io_mapping = _load_module("io_mapping")
preprocessing = _load_module("preprocessing")
prediction = _load_module("prediction")


class _FakeRobot:
    _config = {"cameras": {constants.CAMERA_NAME: {"rotation_deg": 0}}}

    def get_images(self, format="rgb"):
        assert format == "rgb"
        return {
            constants.CAMERA_NAME: np.full(
                (188, 336, 3),
                255,
                dtype=np.uint8,
            )
        }

    def get_joint_position_history(self, joint_names, history_size, sample_hz):
        assert tuple(joint_names) == constants.JOINT_NAMES
        assert history_size == 6
        assert sample_hz == 10.0
        return np.arange(6 * 54, dtype=np.float32).reshape(6, 54)

    def get_tactile_taxel_history(self, sensor_name, history_size, sample_hz):
        assert history_size == 18
        assert sample_hz == 30.0
        start = 3.0 if sensor_name == "left_hand_pressure" else 5.0
        frames = np.arange(18, dtype=np.float32)[:, None, None, None] + start
        return np.broadcast_to(frames, (18, 5, 3, 3)).copy()


class _Preprocessor(preprocessing.PreprocessingMixin):
    def __init__(self):
        self._robot = _FakeRobot()
        self._policy = object()
        self._device = torch.device("cpu")
        self._tactile_inputs = {
            "left": "left_hand_pressure",
            "right": "right_hand_pressure",
        }
        self._tactile_baselines = {
            "left": np.full((5, 3, 3), 2.0, dtype=np.float32),
            "right": np.full((5, 3, 3), 4.0, dtype=np.float32),
        }


def test_preprocessing_builds_strict_vitacformer_shapes_and_representation():
    batch = _Preprocessor()._build_observation()

    assert tuple(batch[constants.IMAGE_KEY].shape) == (1, 3, 188, 336)
    assert tuple(batch[constants.STATE_KEY].shape) == (1, 6, 54)
    assert tuple(batch[constants.TACTILE_BATCH_KEY].shape) == (1, 18, 180)
    assert torch.all(batch[constants.IMAGE_KEY] == 1.0)

    tactile = batch[constants.TACTILE_BATCH_KEY][0].numpy()
    # Both hands begin one count above their episode baseline.
    np.testing.assert_allclose(tactile[0, :90], 1.0)
    np.testing.assert_allclose(tactile[0, 90:], 0.0)
    # The second half is each raw channel relative to the oldest frame.
    np.testing.assert_allclose(tactile[-1, 90:], 17.0)


def test_tactile_sensor_resolution_is_fail_closed():
    resolve = io_mapping.IoMappingMixin._resolve_tactile_inputs
    assert resolve(["left_hand_pressure", "right_hand_pressure"]) == {
        "left": "left_hand_pressure",
        "right": "right_hand_pressure",
    }
    with pytest.raises(RuntimeError, match="one left tactile sensor"):
        resolve(["right_hand_pressure"])


@pytest.mark.parametrize("chunk_size", [100, 200])
def test_prediction_returns_only_the_configured_finite_chunk(chunk_size):
    class _Policy:
        config = SimpleNamespace(chunk_size=chunk_size)

        @staticmethod
        def parameters():
            return iter([torch.zeros(1)])

        @staticmethod
        def predict_action_chunk(_batch):
            return torch.zeros((1, chunk_size, 54), dtype=torch.float32)

    runner = prediction.PredictionMixin()
    runner._policy = _Policy()
    result = runner._predict_chunk({})
    assert result.shape == (chunk_size, 54)
    assert result.dtype == np.float64

    runner._policy = SimpleNamespace(
        config=SimpleNamespace(chunk_size=chunk_size),
        parameters=lambda: iter([torch.zeros(1)]),
        predict_action_chunk=lambda _batch: torch.zeros((1, 99, 54))
    )
    with pytest.raises(RuntimeError, match=f"1, {chunk_size}, 54"):
        runner._predict_chunk({})


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf")])
def test_prediction_rejects_nonfinite_h200_output(bad_value):
    chunk = torch.zeros(1, 200, 54)
    chunk[0, -1, -1] = bad_value
    policy = SimpleNamespace(
        config=SimpleNamespace(chunk_size=200),
        parameters=lambda: iter([torch.zeros(1)]),
        predict_action_chunk=lambda _batch: chunk,
    )
    with pytest.raises(RuntimeError, match="NaN or Inf"):
        prediction.PredictionMixin._predict_policy_chunk(policy, {})


def _switch_engine():
    runtime = ENGINE_DIR.parents[1] / 'common' / 'runtime'
    sys.path.insert(0, str(runtime))
    module = _load_module('engine')
    runner = module.ViTacFormerEngine()
    runner._robot = Mock()
    runner._policy = Mock()
    runner._loaded_robot_type = 'ffw_sh5_rev1'
    runner._loaded_model_path = '/models/part1'
    runner._action_keys = list(constants.ACTION_KEYS)
    runner._tactile_baselines = {'left': np.ones((5, 3, 3)), 'right': np.ones((5, 3, 3))}
    runner._init_robot = Mock()
    return runner


def test_weight_switch_keeps_sensor_connection_and_original_tactile_baseline():
    runner = _switch_engine()
    robot, baselines = runner._robot, runner._tactile_baselines
    candidate = Mock(config=SimpleNamespace(model_action_keys=list(constants.ACTION_KEYS)))
    runner._load_policy_assets = Mock(return_value=candidate)
    result = runner.switch_policy(SimpleNamespace(
        model_path='/models/part2', robot_type='ffw_sh5_rev1',
    ))
    assert result['success']
    assert runner._policy is candidate
    assert runner._loaded_model_path == '/models/part2'
    candidate.reset.assert_called_once_with()
    assert runner._robot is robot
    assert runner._tactile_baselines is baselines
    runner._init_robot.assert_not_called()
    robot.close.assert_not_called()


def test_cycle_home_start_recreates_policy_and_episode_sensor_state():
    runner = _switch_engine()
    old = runner._policy
    runner._device = torch.device('cpu')
    candidate = Mock()
    runner._load_policy_assets = Mock(return_value=candidate)
    runner._teardown_robot = Mock()

    result = runner.reset_cycle()

    assert result['success']
    runner._load_policy_assets.assert_called_once_with('/models/part1', torch.device('cpu'))
    assert runner._policy is candidate and runner._policy is not old
    runner._teardown_robot.assert_called_once_with()
    runner._init_robot.assert_called_once_with('ffw_sh5_rev1')
    candidate.reset.assert_called_once_with()
    old.reset.assert_not_called()


def test_failed_cycle_reload_does_not_report_a_new_session():
    runner = _switch_engine()
    runner._device = torch.device('cpu')
    runner._load_policy_assets = Mock(side_effect=RuntimeError('load failed'))
    runner._teardown_robot = Mock()
    result = runner.reset_cycle()
    assert not result['success'] and 'load failed' in result['message']
    runner._init_robot.assert_not_called()


@pytest.mark.parametrize('failure', ['missing', 'layout'])
def test_failed_weight_switch_preserves_old_policy_and_episode(failure):
    runner = _switch_engine()
    old_policy, baselines = runner._policy, runner._tactile_baselines
    if failure == 'missing':
        runner._load_policy_assets = Mock(side_effect=FileNotFoundError('missing checkpoint'))
    else:
        runner._load_policy_assets = Mock(return_value=Mock(config=SimpleNamespace(model_action_keys=['wrong'])))
    result = runner.switch_policy(SimpleNamespace(
        model_path='/models/part2', robot_type='ffw_sh5_rev1',
    ))
    assert not result['success']
    assert runner._policy is old_policy
    assert runner._loaded_model_path == '/models/part1'
    assert runner._tactile_baselines is baselines
    runner._robot.close.assert_not_called()


def _preload_candidate(runner):
    candidate = Mock(config=SimpleNamespace(model_action_keys=list(constants.ACTION_KEYS)))
    runner._load_policy_assets = Mock(return_value=candidate)
    runner._predict_policy_chunk = Mock(return_value=np.zeros((100, 54)))
    runner._device = torch.device('cpu')
    return candidate


def test_preload_warms_candidate_without_sensor_reads_or_changing_active_history():
    runner = _switch_engine()
    old, robot, baseline = runner._policy, runner._robot, runner._tactile_baselines
    candidate = _preload_candidate(runner)
    request = SimpleNamespace(model_path='/models/part2', robot_type='ffw_sh5_rev1')
    assert runner.preload_policy(request)['success']
    assert runner._policy is old and runner._robot is robot
    assert runner._tactile_baselines is baseline
    old.reset.assert_not_called()
    runner._init_robot.assert_not_called()
    batch = runner._predict_policy_chunk.call_args.args[1]
    assert batch[constants.IMAGE_KEY].shape == (1, 3, 188, 336)
    assert batch[constants.STATE_KEY].shape == (1, 6, 54)
    assert batch[constants.TACTILE_BATCH_KEY].shape == (1, 18, 180)
    assert candidate.reset.call_count == 2  # synthetic history discarded
    assert runner.preload_policy(request)['success']
    runner._load_policy_assets.assert_called_once()


def test_cached_switch_and_switch_back_never_load_or_reconnect():
    runner = _switch_engine()
    old, robot, baseline = runner._policy, runner._robot, runner._tactile_baselines
    old.config.model_action_keys = list(constants.ACTION_KEYS)
    candidate = _preload_candidate(runner)
    request = SimpleNamespace(model_path='/models/part2', robot_type='ffw_sh5_rev1')
    assert runner.preload_policy(request)['success']
    runner._load_policy_assets.reset_mock()
    assert runner.switch_preloaded_policy(request)['success']
    assert runner._policy is candidate and runner._preloaded_policy is old
    request.model_path = '/models/part1'
    assert runner.switch_preloaded_policy(request)['success']
    assert runner._policy is old and runner._preloaded_policy is candidate
    runner._load_policy_assets.assert_not_called()
    runner._init_robot.assert_not_called()
    assert runner._robot is robot and runner._tactile_baselines is baseline


@pytest.mark.parametrize('mismatch', ['missing', 'path', 'robot'])
def test_stale_preload_fails_without_disk_fallback_or_policy_change(mismatch):
    runner = _switch_engine()
    old = runner._policy
    _preload_candidate(runner)
    request = SimpleNamespace(model_path='/models/part2', robot_type='ffw_sh5_rev1')
    if mismatch != 'missing':
        assert runner.preload_policy(request)['success']
    if mismatch == 'path':
        request.model_path = '/models/part3'
    elif mismatch == 'robot':
        request.robot_type = 'ffw_sg2_rev1'
    runner._load_policy_assets.reset_mock()
    assert not runner.switch_preloaded_policy(request)['success']
    runner._load_policy_assets.assert_not_called()
    assert runner._policy is old


def test_preload_before_first_load_survives_start_setup_and_cleanup_frees_both():
    runner = _switch_engine()
    runner._policy = runner._robot = runner._loaded_model_path = runner._loaded_robot_type = None
    candidate = _preload_candidate(runner)
    request = SimpleNamespace(model_path='/models/part2', robot_type='ffw_sh5_rev1')
    assert runner.preload_policy(request)['success']
    assert not runner.is_ready and runner._robot is None
    runner._init_robot.assert_not_called()
    first = Mock()
    runner._load_policy_assets.return_value = first
    request.model_path = '/models/part1'
    assert runner.load_policy(request)['success']
    assert runner._policy is first and runner._preloaded_policy is candidate
    runner.cleanup()
    assert runner._policy is None and runner._preloaded_policy is None


def test_failed_warmup_keeps_active_model_and_invalidates_spare():
    runner = _switch_engine()
    old, baseline = runner._policy, runner._tactile_baselines
    _preload_candidate(runner)
    runner._predict_policy_chunk.side_effect = RuntimeError('out of memory')
    result = runner.preload_policy(SimpleNamespace(model_path='/models/part2', robot_type='ffw_sh5_rev1'))
    assert not result['success'] and 'out of memory' in result['message']
    assert runner._policy is old and runner._tactile_baselines is baseline
    assert runner._preloaded_policy is None
