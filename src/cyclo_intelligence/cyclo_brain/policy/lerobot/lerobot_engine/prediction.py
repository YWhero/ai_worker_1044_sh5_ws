#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0

"""LeRobot prediction helpers."""

from __future__ import annotations

import logging
from contextlib import nullcontext
from typing import Dict

import numpy as np
import torch


logger = logging.getLogger("lerobot_engine")


class PredictionMixin:
    """Policy input batch -> action chunk."""

    def _reset_prediction_runtime_state(self) -> None:
        """Clear cross-request prediction state on every logical LOAD."""
        self._act_overlap_tails: list[torch.Tensor] = []
        self._act_overlap_last_action: torch.Tensor | None = None

    def _predict_overlap_ensembled_act(
        self,
        batch: Dict[str, torch.Tensor],
    ) -> torch.Tensor:
        """Return the saved ACT horizon with time-aligned decoder overlap.

        TactileACT predicts 16 rows but is configured to execute four before
        observing again.  At a replan boundary, rows 4..15 from older decoder
        calls describe the same absolute times as the new chunk's leading
        rows.  Averaging those aligned estimates reduces boundary chatter
        while still refreshing observation/state every four source actions.
        """
        assert self._policy is not None
        actions = self._policy.predict_action_chunk(batch)
        if actions.dim() != 3:
            raise ValueError(
                "TactileACT predict_action_chunk must return (B, T, A), got "
                f"{tuple(actions.shape)}"
            )
        execution_steps = int(
            getattr(self._policy.config, "n_action_steps", 0)
        )
        if execution_steps <= 0 or actions.shape[1] < execution_steps:
            raise ValueError(
                "TactileACT overlap ensemble requires 0 < n_action_steps <= "
                f"decoder horizon, got {execution_steps} and {actions.shape[1]}"
            )

        tails = list(getattr(self, "_act_overlap_tails", []) or [])
        candidates = [
            tail[:, :execution_steps]
            for tail in tails
            if tail.shape[1] >= execution_steps
        ]
        candidates.append(actions[:, :execution_steps])
        stacked = torch.stack(candidates, dim=0)
        coefficient = float(
            getattr(
                self._policy,
                "_cyclo_overlap_action_ensemble_coeff",
                -0.01,
            )
        )
        weights = torch.exp(
            -coefficient
            * torch.arange(
                len(candidates),
                dtype=stacked.dtype,
                device=stacked.device,
            )
        )
        weights = weights / weights.sum()
        ensembled = torch.sum(
            stacked * weights[:, None, None, None],
            dim=0,
        )

        advanced_tails = [
            tail[:, execution_steps:].detach()
            for tail in tails
            if tail.shape[1] > execution_steps
        ]
        if actions.shape[1] > execution_steps:
            advanced_tails.append(actions[:, execution_steps:].detach())
        self._act_overlap_tails = advanced_tails

        # The overlap removes most replan-boundary discontinuity, but each
        # four-row response can still contain a one-frame decoder spike.  A
        # causal EMA in action space suppresses that spike without looking
        # ahead or changing the 30 Hz source cadence.  Keep the very first
        # action unchanged so real preflight still evaluates the policy's
        # actual starting target; subsequent actions continue from the last
        # filtered target across inference requests.
        alpha = float(
            getattr(
                self._policy,
                "_cyclo_overlap_action_smoothing_alpha",
                1.0,
            )
        )
        if not 0.0 < alpha <= 1.0:
            raise ValueError(
                "TactileACT overlap smoothing alpha must be in (0, 1], got "
                f"{alpha}"
            )
        filtered = ensembled.clone()
        previous = getattr(self, "_act_overlap_last_action", None)
        for step in range(filtered.shape[1]):
            current = filtered[:, step]
            if previous is None:
                previous = current
            else:
                previous = previous + alpha * (current - previous)
                filtered[:, step] = previous
        self._act_overlap_last_action = previous.detach()
        return filtered

    @staticmethod
    def _is_act_policy(policy: object) -> bool:
        """Return whether ``policy`` uses LeRobot ACT execution semantics.

        Custom TactileACT checkpoints are reconstructed with an upstream ACT
        config (``type=act``), so this deliberately covers both vanilla ACT
        and TactileACT without importing their concrete classes here.
        """
        config = getattr(policy, "config", None)
        return str(getattr(config, "type", "")).strip().lower() == "act"

    def _prediction_autocast_context(self):
        """Return the precision context required by the loaded policy."""
        assert self._policy is not None
        config = getattr(self._policy, "config", None)
        if (
            getattr(config, "architecture_version", None)
            not in {"sh5_right_v2", "sh5_right_v3_absolute"}
            or not bool(getattr(config, "use_amp", False))
        ):
            return nullcontext()

        device = getattr(self, "_device", None)
        device_type = getattr(device, "type", None)
        if device_type is None and device is not None:
            device_type = str(device).split(":", 1)[0]
        if device_type != "cuda":
            raise RuntimeError(
                "SH5 T-Rex requires a CUDA device with bf16 autocast; "
                f"loaded device is {device!r}"
            )
        bf16_supported = getattr(torch.cuda, "is_bf16_supported", None)
        if callable(bf16_supported) and not bf16_supported():
            raise RuntimeError(
                "SH5 T-Rex requires CUDA hardware with bfloat16 support"
            )
        return torch.autocast(
            device_type="cuda",
            dtype=torch.bfloat16,
        )

    def _predict_chunk(self, batch: Dict[str, torch.Tensor]) -> torch.Tensor:
        """Return a chunk tensor of shape (1, T, A)."""
        assert self._policy is not None
        with self._prediction_autocast_context():
            if self._is_act_policy(self._policy):
                config = self._policy.config
                if bool(
                    getattr(
                        self._policy,
                        "_cyclo_overlap_action_ensemble",
                        False,
                    )
                ):
                    return self._predict_overlap_ensembled_act(batch)
                if getattr(config, "temporal_ensemble_coeff", None) is None:
                    # Return the configured execution horizon in one response
                    # so the 100 Hz publisher can prefetch the next inference.
                    # ``chunk_size`` is the supervised decoder horizon;
                    # ``n_action_steps`` is the closed-loop execution contract.
                    # Executing decoder tail rows beyond that contract lets
                    # prediction errors accumulate before the next observation.
                    actions = self._policy.predict_action_chunk(batch)
                    if actions.dim() != 3:
                        raise ValueError(
                            "ACT predict_action_chunk must return (B, T, A), "
                            f"got {tuple(actions.shape)}"
                        )
                    n_action_steps = int(
                        getattr(config, "n_action_steps", 1)
                    )
                    if n_action_steps <= 0:
                        raise ValueError(
                            "ACT n_action_steps must be positive, got "
                            f"{n_action_steps}"
                        )
                    return actions[:, :n_action_steps]

                # Temporal-ensemble ACT needs a fresh observation for every
                # selected action because select_action() updates the saved
                # ensembler.  Keep that one-action path for vanilla ACT.
                action = self._policy.select_action(batch)
                if action.dim() == 1:
                    action = action.unsqueeze(0)
                if action.dim() != 2:
                    raise ValueError(
                        "ACT select_action must return (B, A), got "
                        f"{tuple(action.shape)}"
                    )

                # Sequential ActionChunkProcessor owns source-period timing,
                # including the final-action hold.  Return exactly the single
                # action selected by the temporal ensembler.
                return action.unsqueeze(1)
            try:
                action = self._policy.predict_action_chunk(batch)
                if action.dim() == 2:
                    action = action.unsqueeze(1)
                return action
            except (NotImplementedError, AttributeError):
                logger.debug(
                    "predict_action_chunk unavailable; falling back to select_action"
                )
                action = self._policy.select_action(batch)
                if action.dim() == 1:
                    action = action.unsqueeze(0)
                return action.unsqueeze(1)

    @staticmethod
    def _to_numpy_chunk(action: torch.Tensor) -> np.ndarray:
        """(B, T, A) or (B, A) tensor -> (T, A) float64 numpy."""
        chunk = action.detach().cpu()
        if chunk.dim() == 3:
            chunk = chunk[0]
        elif chunk.dim() == 2:
            pass
        elif chunk.dim() == 1:
            chunk = chunk.unsqueeze(0)
        else:
            raise ValueError(
                f"Unexpected action tensor shape: {tuple(chunk.shape)}"
            )
        return chunk.to(torch.float64).numpy()
