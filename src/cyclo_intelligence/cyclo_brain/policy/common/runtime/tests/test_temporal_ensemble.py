"""Numerical checks against a simple time-indexed reference ensemble."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from main_runtime.temporal_ensemble import TemporalActionEnsembler, limit_source_steps


def test_integer_offset_matches_original_exponential_prediction_weights():
    ensemble = TemporalActionEnsembler(30, .01)
    first = np.arange(12, dtype=float).reshape(6, 2)
    second = first + 20
    np.testing.assert_array_equal(ensemble.update(first, 0), first)
    result = ensemble.update(second, 2 / 30)
    weight = np.exp(-.01)
    np.testing.assert_allclose(result[:4], (second[:4] + weight * first[2:]) / (1 + weight))
    np.testing.assert_array_equal(result[4:], second[4:])
    assert ensemble.last_info['overlap_max'] == 2


def test_three_plans_use_raw_predictions_and_newer_plan_has_greater_weight():
    ensemble = TemporalActionEnsembler(10, .01)
    for stamp, value in [(0, 2), (.1, 8)]:
        ensemble.update(np.full((5, 1), value), stamp)
    result = ensemble.update(np.full((5, 1), 20), .2)
    weights = np.exp(-.01 * np.arange(3))
    np.testing.assert_allclose(result[:3], np.dot(weights, [20, 8, 2]) / weights.sum())
    assert result[0, 0] > 10  # uniform mean would be exactly 10


def test_fractional_request_interval_interpolates_and_does_not_extrapolate():
    ensemble = TemporalActionEnsembler(10, 0)
    first = np.arange(4, dtype=float)[:, None]
    ensemble.update(first, 100)
    first[:] = 999  # history must own its copy
    result = ensemble.update(np.zeros((4, 1)), 100.05)
    np.testing.assert_allclose(result[:, 0], [.25, .75, 1.25, 0], atol=1e-12)


def test_expired_predictions_and_reset_do_not_leak_into_new_episode():
    ensemble = TemporalActionEnsembler(30, .01)
    ensemble.update(np.zeros((100, 54)), 0)
    next_chunk = np.ones((100, 54))
    np.testing.assert_array_equal(ensemble.update(next_chunk, 4), next_chunk)
    assert ensemble.last_info['active_plans'] == 1
    ensemble.reset()
    np.testing.assert_array_equal(ensemble.update(next_chunk, 0), next_chunk)


@pytest.mark.parametrize('coefficient', [-1, float('nan'), float('inf')])
def test_invalid_coefficient_is_rejected(coefficient):
    with pytest.raises(ValueError):
        TemporalActionEnsembler(30, coefficient)


def test_invalid_or_out_of_order_input_does_not_corrupt_history():
    ensemble = TemporalActionEnsembler(30, .01)
    ensemble.update(np.zeros((5, 2)), 1)
    for chunk, timestamp in [(np.ones((5, 2)), 1), (np.ones((5, 3)), 2),
                             (np.full((5, 2), np.nan), 2)]:
        with pytest.raises(ValueError):
            ensemble.update(chunk, timestamp)
    assert len(ensemble._plans) == 1


def test_long_horizons_and_history_cap_remain_bounded():
    ensemble = TemporalActionEnsembler(30, .01, max_plans=3)
    for i in range(10):
        result = ensemble.update(np.full((200, 54), i), i / 30)
    assert result.shape == (200, 54)
    assert np.isfinite(result).all()
    assert ensemble.last_info['active_plans'] == 3


def test_expiring_prediction_fades_without_extrapolating_past_its_horizon():
    ensemble = TemporalActionEnsembler(10, 0, tail_fade_s=.2)
    ensemble.update(np.ones((6, 1)), 0)
    result = ensemble.update(np.zeros((6, 1)), .2)
    np.testing.assert_allclose(result[:, 0], [.5, .5, 1/3, 0, 0, 0])
    assert np.max(np.abs(np.diff(result[:, 0]))) < .5


def test_source_limiter_preserves_first_row_input_bounds_and_shape():
    raw = np.array([[.5,-.5],[1,-1],[0,0],[0,0]])
    original = raw.copy()
    result = limit_source_steps(raw, .03)
    np.testing.assert_array_equal(result[0], raw[0])
    np.testing.assert_array_equal(raw, original)
    assert result.shape == raw.shape
    assert np.max(np.abs(np.diff(result,axis=0))) <= .03 + 1e-12
    assert np.all(result >= raw.min(axis=0))
    assert np.all(result <= raw.max(axis=0))
