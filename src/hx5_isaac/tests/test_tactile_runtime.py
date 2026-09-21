"""Physical tensor validity and zero-contact fast path, without launching Kit."""
from pathlib import Path
import sys
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulator import PhysicsRuntime


def runtime_with_data(data):
    runtime = PhysicsRuntime.__new__(PhysicsRuntime)
    runtime.np, runtime.dt, runtime.sequence = np, 1/120, 1
    runtime.stats = {'valid_tip_samples': 0, 'invalid_tip_samples': 0}
    view = Mock()
    view.get_contact_force_data.return_value = data
    runtime.tip_views = {f'finger_{side}_sensor{finger}': (view, {})
                         for side in ('l', 'r') for finger in range(1, 6)}
    runtime.tip_pose = Mock(return_value=([0., 0., 0.], [0., 0., 0., 1.]))
    return runtime, view


def tensor(counts, starts):
    return (np.array([[1.5]]), np.array([[0., 0., 0.]]),
            np.array([[0., 0., 1.]]), np.array([[-.0002]]),
            np.asarray(counts), np.asarray(starts))


def test_initialized_zero_contacts_keep_sampling_and_skip_pose_transforms():
    runtime, view = runtime_with_data(tensor([[0, 0]], [[0, 0]]))
    samples = runtime.contacts()
    assert view.get_contact_force_data.call_count == 10
    runtime.tip_pose.assert_not_called()
    for side in samples.values():
        for sample in side.values():
            assert sample == {'valid': True, 'forces': [0.]*9, 'penetration_depths': [0.]*9}
    assert runtime.stats == {'valid_tip_samples': 10, 'invalid_tip_samples': 0}


def test_positive_contacts_still_use_measured_pose_force_and_depth():
    runtime, _ = runtime_with_data(tensor([[1]], [[0]]))
    samples = runtime.contacts()
    assert runtime.tip_pose.call_count == 10
    for side in samples.values():
        for sample in side.values():
            assert sample['valid']
            assert sum(sample['forces']) == pytest.approx(1.5)
            assert max(sample['penetration_depths']) == pytest.approx(.0002)


@pytest.mark.parametrize('data', [None, (None,)*6, tensor([], []),
    tensor([[0]], [[0, 0]]), tensor([[float('nan')]], [[0]]),
    tensor([[-1]], [[0]]), tensor([[.5]], [[0]]),
    tensor([[0]], [[-.5]]), tensor([[2]], [[0]])])
def test_unavailable_or_invalid_tensors_never_become_valid_zero_samples(data):
    runtime, _ = runtime_with_data(data)
    samples = runtime.contacts()
    assert all(sample == {'valid': False} for side in samples.values() for sample in side.values())
    assert runtime.stats == {'valid_tip_samples': 0, 'invalid_tip_samples': 10}
