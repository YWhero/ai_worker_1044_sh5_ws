#!/usr/bin/env python3

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import torch


_PREDICTION_PATH = (
    Path(__file__).resolve().parents[1] / "lerobot_engine" / "prediction.py"
)
_SPEC = importlib.util.spec_from_file_location(
    "lerobot_prediction_under_test", _PREDICTION_PATH
)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
PredictionMixin = _MODULE.PredictionMixin


class _PredictionHarness(PredictionMixin):
    def __init__(self, policy):
        self._policy = policy
        self._device = torch.device("cpu")


class _ActPolicy:
    def __init__(self):
        self.config = SimpleNamespace(
            type="act",
            temporal_ensemble_coeff=0.01,
        )
        self.select_calls = 0

    def select_action(self, _batch):
        self.select_calls += 1
        return torch.tensor([[1.0, 2.0, 3.0]])

    def predict_action_chunk(self, _batch):
        raise AssertionError("ACT raw chunk path must not be used")


class _QueuedActPolicy:
    def __init__(self):
        self.config = SimpleNamespace(
            type="act",
            temporal_ensemble_coeff=None,
            n_action_steps=4,
        )
        self.predict_calls = 0

    def select_action(self, _batch):
        raise AssertionError("non-ensemble ACT must return its horizon at once")

    def predict_action_chunk(self, _batch):
        self.predict_calls += 1
        return torch.arange(18, dtype=torch.float32).reshape(1, 6, 3)


class _OverlapQueuedActPolicy(_QueuedActPolicy):
    def __init__(self):
        super().__init__()
        self.config.n_action_steps = 2
        self._cyclo_overlap_action_ensemble = True
        self._cyclo_overlap_action_ensemble_coeff = -0.01
        self._cyclo_overlap_action_smoothing_alpha = 0.5
        self.predict_calls = 0

    def predict_action_chunk(self, _batch):
        start = 100.0 * self.predict_calls
        self.predict_calls += 1
        return (
            torch.arange(18, dtype=torch.float32).reshape(1, 6, 3)
            + start
        )


class _ChunkPolicy:
    def __init__(self):
        self.config = SimpleNamespace(type="diffusion")

    def select_action(self, _batch):
        raise AssertionError("non-ACT select_action path must not be used")

    def predict_action_chunk(self, _batch):
        return torch.arange(12, dtype=torch.float32).reshape(1, 4, 3)


def _causal_ema(actions, alpha=0.5, previous=None):
    expected = actions.clone()
    for step in range(expected.shape[1]):
        current = expected[:, step]
        if previous is None:
            previous = current
        else:
            previous = previous + alpha * (current - previous)
            expected[:, step] = previous
    return expected, previous


def test_temporal_ensemble_act_returns_one_selected_action():
    policy = _ActPolicy()
    result = _PredictionHarness(policy)._predict_chunk({})

    assert policy.select_calls == 1
    assert result.shape == (1, 1, 3)
    torch.testing.assert_close(
        result[:, 0], torch.tensor([[1.0, 2.0, 3.0]])
    )


def test_non_ensemble_act_returns_configured_execution_horizon():
    policy = _QueuedActPolicy()
    result = _PredictionHarness(policy)._predict_chunk({})

    assert policy.predict_calls == 1
    assert result.shape == (1, 4, 3)
    torch.testing.assert_close(
        result,
        torch.arange(18, dtype=torch.float32).reshape(1, 6, 3)[:, :4],
    )


def test_tactile_act_ensembles_time_aligned_decoder_tails():
    policy = _OverlapQueuedActPolicy()
    harness = _PredictionHarness(policy)
    first = harness._predict_chunk({})
    second = harness._predict_chunk({})

    assert policy.predict_calls == 2
    assert first.shape == (1, 2, 3)
    first_raw = torch.arange(18, dtype=torch.float32).reshape(1, 6, 3)[:, :2]
    expected_first, previous = _causal_ema(first_raw)
    torch.testing.assert_close(first, expected_first)
    old_aligned = (
        torch.arange(18, dtype=torch.float32).reshape(1, 6, 3)[:, 2:4]
    )
    new_aligned = (
        torch.arange(18, dtype=torch.float32).reshape(1, 6, 3)[:, :2]
        + 100.0
    )
    weights = torch.exp(torch.tensor([0.0, 0.01]))
    weights /= weights.sum()
    expected_raw = old_aligned * weights[0] + new_aligned * weights[1]
    expected, _ = _causal_ema(expected_raw, previous=previous)
    torch.testing.assert_close(second, expected)


def test_overlap_runtime_reset_drops_old_decoder_tails():
    policy = _OverlapQueuedActPolicy()
    harness = _PredictionHarness(policy)
    harness._predict_chunk({})
    harness._reset_prediction_runtime_state()
    result = harness._predict_chunk({})

    expected = (
        torch.arange(18, dtype=torch.float32).reshape(1, 6, 3)[:, :2]
        + 100.0
    )
    expected, _ = _causal_ema(expected)
    torch.testing.assert_close(result, expected)


def test_overlap_runtime_rejects_invalid_smoothing_alpha():
    policy = _OverlapQueuedActPolicy()
    policy._cyclo_overlap_action_smoothing_alpha = 0.0

    try:
        _PredictionHarness(policy)._predict_chunk({})
    except ValueError as error:
        assert "smoothing alpha" in str(error)
    else:
        raise AssertionError("invalid smoothing alpha must fail")


def test_non_act_keeps_native_predicted_chunk():
    result = _PredictionHarness(_ChunkPolicy())._predict_chunk({})

    assert result.shape == (1, 4, 3)
    torch.testing.assert_close(
        result,
        torch.arange(12, dtype=torch.float32).reshape(1, 4, 3),
    )
