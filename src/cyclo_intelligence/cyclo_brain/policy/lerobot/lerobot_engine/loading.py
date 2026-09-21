#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""LeRobot engine loading helpers (LoadingMixin).

Extracted from ``engine.py`` to keep the core ``LeRobotEngine`` class
focused on the ``InferenceEngine`` API. Mixed into the engine via
multiple inheritance; bind-mounted into the policy container as part
of the ``/app/lerobot_engine/`` package.

Owns:
- ``_resolve_model_dir``: auto-descend lerobot training-output roots.
- ``_load_policy_assets``: load weights + stored pre/post processors.
- ``_infer_image_resize``: read per-input-image shape hints off the policy.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Dict, Tuple

import torch

from .image_preprocessing import infer_image_resize_targets

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies import get_policy_class, make_pre_post_processors
from lerobot.policies.pretrained import PreTrainedPolicy


logger = logging.getLogger("lerobot_engine")
_NATIVE_TREX_ARCHITECTURE_VERSIONS = frozenset(
    {"sh5_right_v2", "sh5_right_v3_absolute"}
)


class LoadingMixin:
    """Policy load helpers — weights, processors, resize hint."""

    def _reset_policy_runtime_state(self) -> None:
        """Reset policy and preprocessor state on every logical LOAD."""
        for label, component in (
            ("policy", getattr(self, "_policy", None)),
            ("preprocessor", getattr(self, "_preprocessor", None)),
        ):
            reset = getattr(component, "reset", None)
            if not callable(reset):
                continue
            reset()
            logger.info("Reset LeRobot %s runtime state", label)
        reset_prediction = getattr(
            self,
            "_reset_prediction_runtime_state",
            None,
        )
        if callable(reset_prediction):
            reset_prediction()

    @staticmethod
    def _uses_native_trex_loader(
        policy_type: str,
        config_payload: Dict[str, Any],
    ) -> bool:
        """Select the audited SH5 policy without breaking legacy T-Rex artifacts.

        Older local T-Rex checkpoints also use ``type=trex`` but require the
        compatibility loader in :mod:`lerobot_engine.trex`.  New checkpoints
        identify the LeRobot-native implementation explicitly.  An unknown
        explicit version is rejected instead of being interpreted as either
        architecture.
        """
        if policy_type != "trex":
            return False
        architecture_version = config_payload.get("architecture_version")
        if architecture_version is None:
            return False
        if architecture_version not in _NATIVE_TREX_ARCHITECTURE_VERSIONS:
            raise ValueError(
                "Unsupported T-Rex architecture_version: "
                f"{architecture_version!r}; expected one of "
                f"{sorted(_NATIVE_TREX_ARCHITECTURE_VERSIONS)!r}"
            )
        return True


    @staticmethod
    def _resolve_model_dir(model_path: str) -> str:
        """Auto-descend lerobot training-output roots.

        Users frequently paste the training-output root which contains
        ``pretrained_model/`` next to ``training_state/`` or a collection of
        numeric checkpoints. Select the highest saved checkpoint
        deterministically so ``from_pretrained`` finds ``config.json``.
        """
        root = Path((model_path or "").strip())
        if (root / "train_config.json").exists():
            return str(root)
        if (root / "config.json").exists():
            return str(root)
        nested = root / "pretrained_model"
        if (nested / "config.json").exists():
            logger.info("Descending into pretrained_model: %s", nested)
            return str(nested)

        checkpoint_dirs = [
            candidate
            for pattern in (
                "*/pretrained_model",
                "checkpoints/*/pretrained_model",
            )
            for candidate in root.glob(pattern)
            if (candidate / "config.json").exists()
        ]
        if checkpoint_dirs:
            def _checkpoint_sort_key(candidate: Path) -> tuple[int, int, str]:
                checkpoint_name = candidate.parent.name
                if checkpoint_name.isdigit():
                    return (1, int(checkpoint_name), str(candidate))
                return (0, 0, str(candidate))

            selected = max(checkpoint_dirs, key=_checkpoint_sort_key)
            logger.info(
                "Selected latest checkpoint from training output: %s",
                selected,
            )
            return str(selected)
        return str(root)

    @staticmethod
    def _load_policy_assets(
        model_path: str, device: torch.device
    ) -> tuple[PreTrainedPolicy, Any, Any]:
        """Load policy weights + saved pre/post processors."""
        import json

        config_path = Path(model_path) / "config.json"
        config_payload = {}
        tactile_act_loaded = False
        if config_path.exists():
            with open(config_path) as f:
                config_payload = json.load(f)
            policy_type = config_payload.get("type", "act")
        else:
            # ACT was the original default; fall back to it for
            # checkpoints saved before ``type`` started being recorded.
            policy_type = "act"

        logger.info("Policy type: %s", policy_type)
        input_features = config_payload.get("input_features", {}) or {}
        tactile_input_keys = [
            key
            for key in input_features
            if key.startswith("observation.tactile.")
        ]
        has_tactile_metadata = (
            "tactile_expert_hidden_dim" in config_payload
        )
        if policy_type == "fastwam":
            # Encode text on CPU before selectively moving the denoisers/VAE
            # to the runtime device; moving the entire policy can exhaust VRAM.
            PolicyClass = get_policy_class(policy_type)
            policy_config = PreTrainedConfig.from_pretrained(model_path)
            policy_config.device = "cpu"
            policy = PolicyClass.from_pretrained(model_path, config=policy_config)
        elif LoadingMixin._uses_native_trex_loader(
            policy_type,
            config_payload,
        ):
            PolicyClass = get_policy_class(policy_type)
            logger.info(
                "Detected native SH5 T-Rex checkpoint: architecture=%s, "
                "source=%s, tactile_inputs=%s",
                config_payload["architecture_version"],
                config_payload.get("trex_repo_path"),
                tactile_input_keys,
            )
            policy = PolicyClass.from_pretrained(model_path)
        elif policy_type == "trex":
            from .trex import load_trex_policy

            logger.info(
                "Detected T-Rex checkpoint: source=%s, tactile_inputs=%s",
                config_payload.get("trex_repo_path"),
                tactile_input_keys,
            )
            policy = load_trex_policy(model_path, device)
        elif (
            policy_type in {"act", "tactile_act"}
            and has_tactile_metadata
            and tactile_input_keys
        ):
            from .tactile_act import load_tactile_act_policy

            logger.info(
                "Detected tactile ACT checkpoint: expert_hidden_dim=%s, inputs=%s",
                config_payload["tactile_expert_hidden_dim"],
                tactile_input_keys,
            )
            policy = load_tactile_act_policy(
                model_path,
                config_payload,
                device,
            )
            tactile_act_loaded = True
        elif policy_type == "tactile_act":
            raise ValueError(
                "tactile_act checkpoint requires tactile_expert_hidden_dim "
                "and observation.tactile.* input features"
            )
        elif policy_type == "act" and has_tactile_metadata:
            from .tactile_act import (
                load_standard_act_with_legacy_tactile_metadata,
            )

            logger.info(
                "Loading standard ACT checkpoint; ignoring unused legacy "
                "tactile_expert_hidden_dim metadata"
            )
            policy = load_standard_act_with_legacy_tactile_metadata(
                model_path,
                config_payload,
                device,
            )
        else:
            PolicyClass = get_policy_class(policy_type)

            # ``from_pretrained`` reads config.json, instantiates the policy
            # config, then loads safetensors. We don't pass ``config=`` — the
            # saved config is already what we want.
            policy = PolicyClass.from_pretrained(model_path)
        if policy_type == "fastwam":
            policy = policy.eval()
            logger.info("FastWAM weights loaded on CPU for selective offload")
        else:
            policy = policy.to(device).eval()
        if tactile_act_loaded:
            # Execute the saved four-step closed-loop horizon while retaining
            # the decoder tail solely as a time-aligned overlap estimate for
            # later replans.  This removes four-step boundary discontinuities
            # without turning the 16-step decoder horizon into open-loop
            # execution.  The marker is runtime-only; checkpoint files and the
            # saved ACT config remain untouched.
            setattr(policy, "_cyclo_overlap_action_ensemble", True)
            setattr(policy, "_cyclo_overlap_action_ensemble_coeff", -0.01)
            # Keep enough damping to avoid the previously observed stationary
            # chatter, while reducing the lag/attenuation of brief contact
            # motions used to separate nested paper cups. This is an
            # inference-only A/B profile; checkpoint weights and processors
            # remain unchanged.
            setattr(policy, "_cyclo_overlap_action_smoothing_alpha", 0.85)
            logger.info(
                "TactileACT execution profile: four-step overlap ensemble "
                "(n_action_steps=%s decoder chunk=%s coeff=%s ema_alpha=%s)",
                getattr(policy.config, "n_action_steps", "unknown"),
                getattr(policy.config, "chunk_size", "unknown"),
                getattr(
                    policy,
                    "_cyclo_overlap_action_ensemble_coeff",
                    "unknown",
                ),
                getattr(
                    policy,
                    "_cyclo_overlap_action_smoothing_alpha",
                    "unknown",
                ),
            )
        if policy_type != "fastwam":
            logger.info("Policy weights loaded on %s", device)

        # Stored processor pipelines include the dataset-time normalizer
        # stats and image transforms so we don't re-derive (and de-sync)
        # them. Falling through to the default factory here would wipe
        # those stats and produce garbage actions.
        processor_kwargs: Dict[str, Any] = {
            "preprocessor_overrides": {
                "device_processor": {"device": str(device)},
            },
        }
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=policy.config,
            pretrained_path=model_path,
            **processor_kwargs,
        )
        logger.info("Pre/post processors loaded")
        return policy, preprocessor, postprocessor

    def _infer_image_resize(self, policy: PreTrainedPolicy) -> Dict[str, Tuple[int, int]]:
        """Best-effort per-policy-key target ``(W, H)`` from config.

        Many lerobot policies advertise the expected image shape under
        ``input_features['observation.images.<cam>'].shape = (C, H, W)``.
        Pre-resizing on the host keeps mixed camera shapes aligned with the
        dataset metadata. Missing keys mean: leave that camera at native size.
        """
        try:
            features = getattr(policy.config, "input_features", {}) or {}
            return infer_image_resize_targets(features)
        except Exception:
            pass
        return {}
