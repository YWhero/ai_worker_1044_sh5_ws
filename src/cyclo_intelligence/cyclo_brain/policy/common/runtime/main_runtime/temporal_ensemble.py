"""Time-aligned, non-recursive averaging of absolute action predictions.

ViTacFormer's reference inference.py uses exp(-0.01 * prediction_rank),
favoring newer overlapping predictions. We retain that weighting, but use
monotonic request times instead of assuming a fixed inference interval.
Call under the control-loop lock, before safety bridging changes source time.
"""

import math

import numpy as np


def limit_source_steps(chunk: np.ndarray, max_delta: float) -> np.ndarray:
    """Bound a postprocessed trajectory without changing its source time grid.

    Validate raw model predictions separately before using this function; it
    must not hide model failures. The first row stays unchanged and subsequent
    targets move at most max_delta per joint/source tick. Convex movement toward
    each target preserves joint bounds. Input and ensemble history are untouched.
    """
    if not math.isfinite(max_delta) or max_delta <= 0:
        raise ValueError('source step limit must be finite and positive')
    result = np.array(chunk, dtype=np.float64, copy=True)
    if result.ndim != 2 or min(result.shape) < 1 or not np.isfinite(result).all():
        raise ValueError('source step limiter requires a finite nonempty chunk')
    for i in range(1, len(result)):
        result[i] = result[i-1] + np.clip(result[i] - result[i-1], -max_delta, max_delta)
    return result


class TemporalActionEnsembler:
    def __init__(self, source_hz: float, coefficient: float, max_plans: int = 256,
                 tail_fade_s: float = 0.0):
        if not math.isfinite(source_hz) or source_hz <= 0:
            raise ValueError('ensemble source_hz must be finite and positive')
        if not math.isfinite(coefficient) or coefficient < 0:
            raise ValueError('ensemble coefficient must be finite and non-negative')
        if max_plans < 1:
            raise ValueError('ensemble max_plans must be positive')
        if not math.isfinite(tail_fade_s) or tail_fade_s < 0:
            raise ValueError('ensemble tail_fade_s must be finite and non-negative')
        self.source_hz = float(source_hz)
        self.coefficient = float(coefficient)
        self.max_plans = int(max_plans)
        self.tail_fade_s = float(tail_fade_s)
        self.reset()

    def reset(self):
        self._plans = []
        self.last_info = {}

    def update(self, chunk: np.ndarray, observed_at: float) -> np.ndarray:
        raw = np.asarray(chunk, dtype=np.float64)
        if raw.ndim != 2 or min(raw.shape) < 1 or not np.isfinite(raw).all():
            raise ValueError('ensemble requires a finite nonempty (T, D) chunk')
        if not math.isfinite(observed_at):
            raise ValueError('ensemble observation time must be finite')
        if self._plans:
            if observed_at <= self._plans[-1][0]:
                raise ValueError('ensemble observation times must increase')
            if raw.shape[1] != self._plans[-1][1].shape[1]:
                raise ValueError('ensemble action dimension changed without reset')
        # A past plan must still predict at least the first requested instant.
        plans = [(stamp, plan) for stamp, plan in self._plans
                 if (observed_at - stamp) * self.source_hz <= len(plan) - 1 + 1e-9]
        plans = plans[-(self.max_plans - 1):] if self.max_plans > 1 else []
        total = raw.copy()
        weight_sum = np.ones(len(raw), dtype=np.float64)
        counts = np.ones(len(raw), dtype=np.int64)
        for stamp, plan in reversed(plans):
            positions = (observed_at - stamp) * self.source_hz + np.arange(len(raw))
            valid = (positions >= 0) & (positions <= len(plan) - 1 + 1e-9)
            indices = np.flatnonzero(valid)
            position = np.clip(positions[valid], 0, len(plan) - 1)
            lower = np.floor(position).astype(np.int64)
            upper = np.minimum(lower + 1, len(plan) - 1)
            alpha = (position - lower)[:, None]
            values = plan[lower] * (1.0 - alpha) + plan[upper] * alpha
            weights = np.exp(-self.coefficient * counts[indices])
            if self.tail_fade_s:
                # In asynchronous full-horizon output, a previous prediction
                # can expire inside this chunk. Fade its contribution before
                # that edge rather than abruptly changing the denominator.
                # Zero disables this adaptation and gives reference weights.
                weights *= np.clip((len(plan) - 1 - position)
                                   / (self.tail_fade_s * self.source_hz), 0, 1)
            total[indices] += values * weights[:, None]
            weight_sum[indices] += weights
            counts[indices] += weights > 0
        result = total / weight_sum[:, None]
        # Keep raw predictions, never previous ensemble outputs. Otherwise
        # old predictions would receive their weight repeatedly.
        self._plans = plans + [(float(observed_at), raw.copy())]
        self.last_info = dict(
            enabled=True, coefficient=self.coefficient, source_hz=self.source_hz,
            tail_fade_s=self.tail_fade_s,
            observed_at=float(observed_at), active_plans=len(self._plans),
            overlap_min=int(counts.min()), overlap_max=int(counts.max()),
        )
        return result
