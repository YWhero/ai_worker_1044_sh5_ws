"""ViTacFormer inference and action-chunk conversion."""

from __future__ import annotations

from typing import Dict

import numpy as np
import torch


class PredictionMixin:
    def _predict_chunk(self, batch: Dict[str, torch.Tensor]) -> np.ndarray:
        assert self._policy is not None
        return self._predict_policy_chunk(self._policy, batch)

    @staticmethod
    def _predict_policy_chunk(policy, batch: Dict[str, torch.Tensor]) -> np.ndarray:
        with torch.inference_mode():
            device_type = next(policy.parameters()).device.type
            with torch.autocast(
                device_type=device_type,
                dtype=torch.bfloat16,
                enabled=device_type == "cuda",
            ):
                action = policy.predict_action_chunk(batch)
        if action.dim() != 3 or action.shape[0] != 1:
            raise RuntimeError(
                "ViTacFormer output must be (1, T, 54), got "
                f"{tuple(action.shape)}"
            )
        expected_chunk_size = policy.config.chunk_size
        if action.shape[1:] != (expected_chunk_size, 54):
            raise RuntimeError(
                f"ViTacFormer output must be (1, {expected_chunk_size}, 54), got "
                f"{tuple(action.shape)}"
            )
        chunk = action[0].detach().to(torch.float64).cpu().numpy()
        if not np.isfinite(chunk).all():
            raise RuntimeError("ViTacFormer produced NaN or Inf actions")
        return np.ascontiguousarray(chunk)
