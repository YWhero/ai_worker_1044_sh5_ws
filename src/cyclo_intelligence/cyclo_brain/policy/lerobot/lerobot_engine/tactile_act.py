#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0

"""ACT variant with one transformer token per tactile taxel.

The tactile ACT checkpoints trained by the internal pipeline add two pieces to
upstream LeRobot ACT:

* a small per-taxel MLP for every ``observation.tactile.*`` feature;
* a learned positional embedding for every taxel.

The policy image used by Cyclo contains upstream LeRobot, so those custom
layers are reconstructed here in the bind-mounted inference package. This
keeps ordinary ACT checkpoints on the upstream loading path.
"""

from __future__ import annotations

import copy
import json
import tempfile
from collections.abc import Mapping
from math import prod
from pathlib import Path
from typing import Any

import einops
import torch
from torch import Tensor, nn

from lerobot.configs import PreTrainedConfig
from lerobot.policies.act.modeling_act import ACT, ACTPolicy, ACTTemporalEnsembler
from lerobot.policies.pretrained import PreTrainedPolicy
from lerobot.utils.constants import ACTION, OBS_ENV_STATE, OBS_IMAGES, OBS_STATE

from .tactile_runtime import (
    TACTILE_MODE_BOTH_EPISODE_BASELINE,
    TACTILE_MODE_LEFT_ZERO_RIGHT_BASELINE,
    set_tactile_runtime_mode,
)


TACTILE_KEY_PREFIX = "observation.tactile."
TACTILE_HIDDEN_DIM_FIELD = "tactile_expert_hidden_dim"
TACTILE_TRAINING_ONLY_FIELDS = frozenset(
    {
        "tactile_optimizer_lr",
        "tactile_pos_embed_init_std",
        "tactile_noise_std",
        "tactile_dropout_prob",
        "supervise_terminal_hold",
        "scheduler_warmup_steps",
        "scheduler_decay_steps",
        "scheduler_decay_lr",
        "diagnostic_log_freq",
    }
)


def _safe_feature_key(feature_key: str) -> str:
    """Convert a dotted feature name into a valid ModuleDict key."""
    return feature_key.replace(".", "__")


def _feature_shape(feature: Any) -> tuple[int, ...]:
    shape = getattr(feature, "shape", None)
    if shape is None and isinstance(feature, Mapping):
        shape = feature.get("shape")
    if shape is None:
        raise ValueError(f"Feature has no shape: {feature!r}")
    return tuple(int(value) for value in shape)


def tactile_feature_shapes(config: PreTrainedConfig) -> dict[str, tuple[int, ...]]:
    features = getattr(config, "input_features", {}) or {}
    return {
        key: _feature_shape(feature)
        for key, feature in features.items()
        if key.startswith(TACTILE_KEY_PREFIX)
    }


class TactileACT(ACT):
    """Upstream ACT with learned per-taxel tactile encoder tokens."""

    def __init__(self, config: PreTrainedConfig):
        super().__init__(config)

        hidden_dim = int(getattr(config, TACTILE_HIDDEN_DIM_FIELD))
        if hidden_dim <= 0:
            raise ValueError(
                f"{TACTILE_HIDDEN_DIM_FIELD} must be positive, got {hidden_dim}"
            )

        shapes = tactile_feature_shapes(config)
        if not shapes:
            raise ValueError(
                f"{TACTILE_HIDDEN_DIM_FIELD} is set but no "
                f"{TACTILE_KEY_PREFIX}* input features were found"
            )

        self.tactile_feature_shapes = shapes
        self.tactile_experts = nn.ModuleDict()
        self.tactile_pos_embed = nn.ParameterDict()
        for feature_key, shape in shapes.items():
            safe_key = _safe_feature_key(feature_key)
            taxel_count = prod(shape)
            self.tactile_experts[safe_key] = nn.Sequential(
                nn.Linear(1, hidden_dim),
                nn.ReLU(),
                nn.Linear(hidden_dim, config.dim_model),
                nn.LayerNorm(config.dim_model),
            )
            self.tactile_pos_embed[safe_key] = nn.Parameter(
                torch.empty(taxel_count, config.dim_model)
            )

    def _tactile_encoder_inputs(
        self,
        batch: dict[str, Tensor],
        batch_size: int,
    ) -> tuple[list[Tensor], list[Tensor]]:
        tokens: list[Tensor] = []
        position_embeddings: list[Tensor] = []

        for feature_key, shape in self.tactile_feature_shapes.items():
            if feature_key not in batch:
                raise KeyError(f"Missing tactile policy input: {feature_key}")

            tactile = batch[feature_key]
            actual_shape = tuple(int(value) for value in tactile.shape[1:])
            if actual_shape != shape:
                raise ValueError(
                    f"{feature_key} has shape {actual_shape}, expected {shape}"
                )

            safe_key = _safe_feature_key(feature_key)
            taxel_values = tactile.reshape(batch_size, prod(shape), 1)
            projected = self.tactile_experts[safe_key](taxel_values)
            projected = einops.rearrange(projected, "b n c -> n b c")
            positions = self.tactile_pos_embed[safe_key].unsqueeze(1)

            tokens.extend(list(projected))
            position_embeddings.extend(list(positions))

        return tokens, position_embeddings

    def forward(
        self,
        batch: dict[str, Tensor],
    ) -> tuple[Tensor, tuple[Tensor, Tensor] | tuple[None, None]]:
        """Run ACT with tactile taxels appended to the encoder sequence."""
        if self.config.use_vae and self.training:
            assert ACTION in batch, (
                "actions must be provided when using the variational "
                "objective in training mode."
            )

        batch_size = (
            batch[OBS_IMAGES][0].shape[0]
            if OBS_IMAGES in batch
            else batch[OBS_ENV_STATE].shape[0]
        )

        if self.config.use_vae and ACTION in batch and self.training:
            cls_embed = einops.repeat(
                self.vae_encoder_cls_embed.weight,
                "1 d -> b 1 d",
                b=batch_size,
            )
            if self.config.robot_state_feature:
                robot_state_embed = self.vae_encoder_robot_state_input_proj(
                    batch[OBS_STATE]
                ).unsqueeze(1)
            action_embed = self.vae_encoder_action_input_proj(batch[ACTION])

            if self.config.robot_state_feature:
                vae_encoder_input = [
                    cls_embed,
                    robot_state_embed,
                    action_embed,
                ]
            else:
                vae_encoder_input = [cls_embed, action_embed]
            vae_encoder_input = torch.cat(vae_encoder_input, axis=1)

            pos_embed = self.vae_encoder_pos_enc.clone().detach()
            cls_joint_is_pad = torch.full(
                (
                    batch_size,
                    2 if self.config.robot_state_feature else 1,
                ),
                False,
                device=action_embed.device,
            )
            key_padding_mask = torch.cat(
                [cls_joint_is_pad, batch["action_is_pad"]],
                axis=1,
            )
            cls_token_out = self.vae_encoder(
                vae_encoder_input.permute(1, 0, 2),
                pos_embed=pos_embed.permute(1, 0, 2),
                key_padding_mask=key_padding_mask,
            )[0]
            latent_pdf_params = self.vae_encoder_latent_output_proj(cls_token_out)
            mu = latent_pdf_params[:, : self.config.latent_dim]
            log_sigma_x2 = latent_pdf_params[:, self.config.latent_dim :]
            latent_sample = (
                mu
                + log_sigma_x2.div(2).exp()
                * torch.randn_like(mu)
            )
        else:
            mu = log_sigma_x2 = None
            device = (
                batch[OBS_STATE].device
                if OBS_STATE in batch
                else next(self.parameters()).device
            )
            latent_sample = torch.zeros(
                [batch_size, self.config.latent_dim],
                dtype=torch.float32,
                device=device,
            )

        encoder_in_tokens = [self.encoder_latent_input_proj(latent_sample)]
        encoder_in_pos_embed = list(
            self.encoder_1d_feature_pos_embed.weight.unsqueeze(1)
        )

        if self.config.robot_state_feature:
            encoder_in_tokens.append(
                self.encoder_robot_state_input_proj(batch[OBS_STATE])
            )
        if self.config.env_state_feature:
            encoder_in_tokens.append(
                self.encoder_env_state_input_proj(batch[OBS_ENV_STATE])
            )

        tactile_tokens, tactile_positions = self._tactile_encoder_inputs(
            batch,
            batch_size,
        )
        encoder_in_tokens.extend(tactile_tokens)
        encoder_in_pos_embed.extend(tactile_positions)

        if self.config.image_features:
            for img in batch[OBS_IMAGES]:
                cam_features = self.backbone(img)["feature_map"]
                cam_pos_embed = self.encoder_cam_feat_pos_embed(
                    cam_features
                ).to(dtype=cam_features.dtype)
                cam_features = self.encoder_img_feat_input_proj(cam_features)

                cam_features = einops.rearrange(
                    cam_features,
                    "b c h w -> (h w) b c",
                )
                cam_pos_embed = einops.rearrange(
                    cam_pos_embed,
                    "b c h w -> (h w) b c",
                )
                encoder_in_tokens.extend(list(cam_features))
                encoder_in_pos_embed.extend(list(cam_pos_embed))

        encoder_in_tokens_tensor = torch.stack(encoder_in_tokens, axis=0)
        encoder_in_pos_embed_tensor = torch.stack(
            encoder_in_pos_embed,
            axis=0,
        )

        encoder_out = self.encoder(
            encoder_in_tokens_tensor,
            pos_embed=encoder_in_pos_embed_tensor,
        )
        decoder_in = torch.zeros(
            (
                self.config.chunk_size,
                batch_size,
                self.config.dim_model,
            ),
            dtype=encoder_in_pos_embed_tensor.dtype,
            device=encoder_in_pos_embed_tensor.device,
        )
        decoder_out = self.decoder(
            decoder_in,
            encoder_out,
            encoder_pos_embed=encoder_in_pos_embed_tensor,
            decoder_pos_embed=self.decoder_pos_embed.weight.unsqueeze(1),
        )
        actions = self.action_head(decoder_out.transpose(0, 1))
        return actions, (mu, log_sigma_x2)


class TactileACTPolicy(ACTPolicy):
    """ACTPolicy that instantiates :class:`TactileACT`."""

    def __init__(self, config: PreTrainedConfig, **kwargs: Any):
        PreTrainedPolicy.__init__(self, config)
        config.validate_features()
        self.config = config
        self.model = TactileACT(config)

        if config.temporal_ensemble_coeff is not None:
            self.temporal_ensembler = ACTTemporalEnsembler(
                config.temporal_ensemble_coeff,
                config.chunk_size,
            )
        self.reset()


def _load_act_config_compat(
    config_payload: Mapping[str, Any],
    device: torch.device,
) -> tuple[PreTrainedConfig, int | None]:
    """Parse ACT config while tolerating the internal tactile metadata field."""
    payload = copy.deepcopy(dict(config_payload))
    raw_hidden_dim = payload.pop(TACTILE_HIDDEN_DIM_FIELD, None)
    hidden_dim = int(raw_hidden_dim) if raw_hidden_dim is not None else None
    for field in TACTILE_TRAINING_ONLY_FIELDS:
        payload.pop(field, None)
    # ``tactile_act`` is an internal checkpoint discriminator rather than an
    # upstream LeRobot registry key.  Reconstruct the base ACT config, then
    # instantiate the custom policy class below.
    payload["type"] = "act"

    with tempfile.TemporaryDirectory(prefix="cyclo-tactile-act-config-") as tmp:
        config_path = Path(tmp) / "config.json"
        config_path.write_text(json.dumps(payload))
        config = PreTrainedConfig.from_pretrained(tmp)

    config.device = str(device)

    # Every backbone tensor is restored from model.safetensors. Avoid an
    # unnecessary ImageNet download during offline robot inference.
    config.pretrained_backbone_weights = None
    return config, hidden_dim


def load_tactile_act_policy(
    model_path: str | Path,
    config_payload: Mapping[str, Any],
    device: torch.device,
) -> TactileACTPolicy:
    """Load a tactile ACT checkpoint without mutating its saved config."""
    config, hidden_dim = _load_act_config_compat(config_payload, device)
    if hidden_dim is None:
        raise ValueError(f"Missing {TACTILE_HIDDEN_DIM_FIELD} in tactile ACT config")
    setattr(config, TACTILE_HIDDEN_DIM_FIELD, hidden_dim)
    saved_type = str(config_payload.get("type", "act")).strip().lower()
    runtime_mode = (
        TACTILE_MODE_BOTH_EPISODE_BASELINE
        if saved_type == "tactile_act"
        else TACTILE_MODE_LEFT_ZERO_RIGHT_BASELINE
    )
    set_tactile_runtime_mode(config, runtime_mode)

    policy = TactileACTPolicy.from_pretrained(
        model_path,
        config=config,
        strict=True,
    )
    return policy


def load_standard_act_with_legacy_tactile_metadata(
    model_path: str | Path,
    config_payload: Mapping[str, Any],
    device: torch.device,
) -> ACTPolicy:
    """Load ordinary ACT saved with an unused tactile config field.

    Some training runs serialized ``tactile_expert_hidden_dim`` even when the
    dataset exposed no tactile input.  The checkpoint itself is ordinary ACT;
    remove only that unsupported metadata field before strict weight loading.
    """
    config, _ = _load_act_config_compat(config_payload, device)
    return ACTPolicy.from_pretrained(
        model_path,
        config=config,
        strict=True,
    )
