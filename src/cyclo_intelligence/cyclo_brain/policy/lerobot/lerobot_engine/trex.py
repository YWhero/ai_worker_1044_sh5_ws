#!/usr/bin/env python3
"""LeRobot adapter for the locally trained pressure-reactive T-Rex policy."""

from __future__ import annotations

import sys
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from types import MethodType
from typing import Any

import numpy as np
import torch
from PIL import Image
from torch import nn

from lerobot.configs import PipelineFeatureType, PolicyFeature, PreTrainedConfig
from lerobot.policies.pretrained import PreTrainedPolicy
from lerobot.processor import ProcessorStep, ProcessorStepRegistry
from lerobot.types import EnvTransition, TransitionKey


_DEFAULT_TREX_REPO = "/workspace/T-Rex-SH5"
_PRESSURE_CURRENT_KEY = "_trex_pressure_current"
_PRESSURE_DEFORMATION_KEY = "_trex_pressure_deformation"
_PRESSURE_HISTORY_KEY = "_trex_pressure_history"
_QWEN_INPUT_IDS_KEY = "_trex_input_ids"
_QWEN_ATTENTION_MASK_KEY = "_trex_attention_mask"
_QWEN_PIXEL_VALUES_KEY = "_trex_pixel_values"
_QWEN_IMAGE_GRID_KEY = "_trex_image_grid_thw"


@PreTrainedConfig.register_subclass("trex")
@dataclass
class TRexConfig(PreTrainedConfig):
    """Serialized deployment subset of the local T-Rex training config."""

    trex_repo_path: str = _DEFAULT_TREX_REPO
    origin_model_path: str = "Qwen/Qwen3-VL-2B-Instruct"
    midtrain_checkpoint: str | None = None
    midtrain_revision: str | None = None
    load_midtrain_checkpoint: bool = False
    processor_name: str = ""
    processor_subfolder: str = "processor"
    image_min_pixels: int = 100352
    image_max_pixels: int = 401408
    chunk_size: int = 16
    n_action_steps: int = 16
    pressure_history_size: int = 16
    pressure_left_key: str = "observation.tactile.left"
    pressure_right_key: str = "observation.tactile.right"
    pressure_scale: float = 255.0
    pressure_baseline_mode: str = "zero"
    pressure_baseline_value: float = 0.0
    pressure_adapter_width: int = 256
    cascaded_total_steps: int = 10
    cascaded_split_step: int = 6
    tactile_loss_weight: float = 1.0
    action_loss_weight: float = 1.0
    tactile_dropout: float = 0.0
    time_sampling_beta_alpha: float = 1.5
    time_sampling_beta_beta: float = 1.0
    fast_refresh_offsets: list[int] | None = None
    tactile_intermediate_size: int = 1536
    checkpoint_action_dim: int = 62
    freeze_visual_encoder: bool = True
    freeze_language_backbone: bool = True
    train_action_expert: bool = False
    train_tactile_expert: bool = False
    dtype: str = "bfloat16"
    normalization_mapping: dict[str, str] | None = None
    optimizer_lr: float = 1e-4
    optimizer_betas: list[float] | None = None
    optimizer_eps: float = 1e-8
    optimizer_weight_decay: float = 0.01
    optimizer_grad_clip_norm: float = 1.0
    scheduler_warmup_steps: int = 500
    scheduler_decay_steps: int = 20000
    scheduler_decay_lr: float = 1e-5

    @property
    def observation_delta_indices(self) -> list[int] | None:
        return None

    @property
    def action_delta_indices(self) -> list[int] | None:
        return None

    @property
    def reward_delta_indices(self) -> list[int] | None:
        return None

    def get_optimizer_preset(self):
        raise NotImplementedError("TRexConfig is inference-only")

    def get_scheduler_preset(self):
        return None

    def validate_features(self) -> None:
        if not self.input_features or "observation.state" not in self.input_features:
            raise ValueError("T-Rex requires observation.state")
        if not self.output_features or "action" not in self.output_features:
            raise ValueError("T-Rex requires an action output")


def _register_processor(name: str):
    """Register a step while remaining safe under module reloads in tests."""

    def decorator(cls):
        if name not in ProcessorStepRegistry.list():
            return ProcessorStepRegistry.register(name)(cls)
        return cls

    return decorator


@_register_processor("trex_pressure_tactile_v1")
@dataclass
class TRexPressureProcessorStep(ProcessorStep):
    """Turn two 5x3x3 pressure grids into current/deform/history tensors."""

    left_key: str = "observation.tactile.left"
    right_key: str = "observation.tactile.right"
    history_size: int = 16
    pressure_scale: float = 255.0
    baseline_mode: str = "zero"
    baseline_value: float = 0.0

    def __post_init__(self) -> None:
        if self.history_size <= 0 or self.pressure_scale <= 0:
            raise ValueError("Invalid T-Rex pressure history/scale")
        self._history: deque[torch.Tensor] = deque(maxlen=self.history_size)

    @staticmethod
    def _batched_fingers(value: torch.Tensor) -> torch.Tensor:
        value = torch.as_tensor(value, dtype=torch.float32)
        while value.ndim > 4 and value.shape[0] == 1:
            value = value.squeeze(0)
        if value.ndim == 3:
            value = value.unsqueeze(0)
        if value.ndim != 4 or tuple(value.shape[-3:]) != (5, 3, 3):
            raise ValueError(
                "T-Rex tactile input must be [B,5,3,3], got "
                f"{tuple(value.shape)}"
            )
        return value.reshape(value.shape[0], 5, 9)

    def __call__(self, transition: EnvTransition) -> EnvTransition:
        observation = transition[TransitionKey.OBSERVATION]
        if observation is None:
            return transition
        left = self._batched_fingers(observation[self.left_key])
        right = self._batched_fingers(observation[self.right_key])
        current = torch.cat([left, right], dim=1) / self.pressure_scale
        if self.baseline_mode != "zero":
            raise ValueError(
                f"Unsupported T-Rex pressure baseline: {self.baseline_mode}"
            )
        deformation = current - self.baseline_value / self.pressure_scale
        self._history.append(current.detach())
        frames = list(self._history)
        frames = [frames[0]] * (self.history_size - len(frames)) + frames
        observation[_PRESSURE_CURRENT_KEY] = current
        observation[_PRESSURE_DEFORMATION_KEY] = deformation
        observation[_PRESSURE_HISTORY_KEY] = torch.stack(frames, dim=1)
        return transition

    def reset(self) -> None:
        self._history.clear()

    def get_config(self) -> dict[str, Any]:
        return {
            "left_key": self.left_key,
            "right_key": self.right_key,
            "history_size": self.history_size,
            "pressure_scale": self.pressure_scale,
            "baseline_mode": self.baseline_mode,
            "baseline_value": self.baseline_value,
        }

    def transform_features(
        self,
        features: dict[PipelineFeatureType, dict[str, PolicyFeature]],
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        return features


@_register_processor("trex_qwen3_vl_v1")
@dataclass
class TRexQwen3VLProcessorStep(ProcessorStep):
    """Apply the Qwen3-VL chat/image processor recorded by training."""

    processor_name: str = ""
    processor_subfolder: str = "processor"
    image_min_pixels: int = 100352
    image_max_pixels: int = 401408
    input_features: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        from transformers import AutoProcessor

        self._processor = AutoProcessor.from_pretrained(
            self.processor_name,
            subfolder=self.processor_subfolder or None,
            trust_remote_code=True,
            min_pixels=self.image_min_pixels,
            max_pixels=self.image_max_pixels,
        )
        self._image_keys = [
            key
            for key, feature in (self.input_features or {}).items()
            if str(
                feature.get("type")
                if isinstance(feature, dict)
                else getattr(feature, "type", "")
            ).upper().endswith("VISUAL")
        ]
        if not self._image_keys:
            raise ValueError("T-Rex processor has no visual input")

    @staticmethod
    def _to_pil_batch(value: torch.Tensor) -> list[Image.Image]:
        value = torch.as_tensor(value).detach().cpu()
        while value.ndim > 4 and value.shape[0] == 1:
            value = value.squeeze(0)
        if value.ndim == 3:
            value = value.unsqueeze(0)
        if value.ndim != 4:
            raise ValueError(f"T-Rex image must be BCHW: {tuple(value.shape)}")
        images = []
        for tensor in value:
            array = tensor.permute(1, 2, 0).float().numpy()
            if float(array.max()) <= 1.0:
                array *= 255.0
            images.append(
                Image.fromarray(
                    np.clip(array, 0, 255).astype(np.uint8), mode="RGB"
                )
            )
        return images

    def __call__(self, transition: EnvTransition) -> EnvTransition:
        observation = transition[TransitionKey.OBSERVATION]
        if observation is None:
            return transition
        complementary = transition[TransitionKey.COMPLEMENTARY_DATA] or {}
        task_value = complementary.get("task", "")
        tasks = (
            [str(value) for value in task_value]
            if isinstance(task_value, (list, tuple))
            else [str(task_value)]
        )
        image_batches = [
            self._to_pil_batch(observation[key]) for key in self._image_keys
        ]
        batch_size = len(image_batches[0])
        if len(tasks) == 1 and batch_size > 1:
            tasks *= batch_size
        texts, flat_images = [], []
        for index in range(batch_size):
            sample_images = [images[index] for images in image_batches]
            content = [{"type": "image"} for _ in sample_images]
            content.append({"type": "text", "text": tasks[index]})
            texts.append(
                self._processor.apply_chat_template(
                    [{"role": "user", "content": content}],
                    tokenize=False,
                    add_generation_prompt=True,
                )
            )
            flat_images.extend(sample_images)
        encoded = self._processor(
            text=texts, images=flat_images, return_tensors="pt", padding=True
        )
        observation[_QWEN_INPUT_IDS_KEY] = encoded.input_ids
        observation[_QWEN_ATTENTION_MASK_KEY] = encoded.attention_mask
        observation[_QWEN_PIXEL_VALUES_KEY] = encoded.pixel_values
        observation[_QWEN_IMAGE_GRID_KEY] = encoded.image_grid_thw
        return transition

    def get_config(self) -> dict[str, Any]:
        return {
            "processor_name": self.processor_name,
            "processor_subfolder": self.processor_subfolder,
            "image_min_pixels": self.image_min_pixels,
            "image_max_pixels": self.image_max_pixels,
            "input_features": self.input_features,
        }

    def transform_features(
        self,
        features: dict[PipelineFeatureType, dict[str, PolicyFeature]],
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        return features


class PressureTokenAdapter(nn.Module):
    """Checkpoint-compatible 3-modality, 10-finger pressure encoder."""

    def __init__(self, hidden_size: int, width: int) -> None:
        super().__init__()
        self.current_encoder = nn.Sequential(
            nn.Linear(9, width), nn.GELU(), nn.Linear(width, hidden_size),
            nn.LayerNorm(hidden_size),
        )
        self.deformation_encoder = nn.Sequential(
            nn.Linear(9, width), nn.GELU(), nn.Linear(width, hidden_size),
            nn.LayerNorm(hidden_size),
        )
        self.temporal_encoder = nn.Sequential(
            nn.Conv1d(9, width, 3, padding=1), nn.GELU(),
            nn.Conv1d(width, width, 3, padding=1),
        )
        self.temporal_projection = nn.Sequential(
            nn.Linear(width, hidden_size), nn.LayerNorm(hidden_size)
        )
        self.finger_embedding = nn.Parameter(torch.zeros(1, 10, hidden_size))
        self.modality_embedding = nn.Parameter(torch.zeros(3, 1, hidden_size))

    def forward(self, current, deformation, history) -> torch.Tensor:
        current_tokens = self.current_encoder(current)
        deformation_tokens = self.deformation_encoder(deformation)
        batch, steps, fingers, channels = history.shape
        temporal = history.permute(0, 2, 3, 1).reshape(
            batch * fingers, channels, steps
        )
        temporal = self.temporal_encoder(temporal).mean(dim=-1)
        temporal_tokens = self.temporal_projection(temporal).reshape(
            batch, fingers, -1
        )
        modalities = torch.stack(
            [current_tokens, deformation_tokens, temporal_tokens], dim=1
        )
        modalities = (
            modalities
            + self.finger_embedding[:, None, :, :]
            + self.modality_embedding[None, :, :, :]
        )
        return modalities.flatten(1, 2)


class TRexPolicy(PreTrainedPolicy):
    """LeRobot policy contract around the local Qwen3VLVLAModel."""

    config_class = TRexConfig
    name = "trex"

    def __init__(self, config: TRexConfig) -> None:
        super().__init__(config)
        repo_path = Path(config.trex_repo_path)
        if not repo_path.is_dir():
            raise FileNotFoundError(f"T-Rex source not found: {repo_path}")
        if str(repo_path) not in sys.path:
            sys.path.insert(0, str(repo_path))

        from qwen_vla.modeling_vla import Qwen3VLVLAModel
        from qwen_vla import modeling_qwen3vl_mot
        from transformers import AutoConfig
        from transformers.models.qwen3_vl.modeling_qwen3_vl import (
            Qwen3VLModel,
            Qwen3VLVisionModel,
        )

        full_config = AutoConfig.from_pretrained(
            config.origin_model_path, trust_remote_code=True
        )
        text_config = full_config.text_config
        # T-Rex was authored against transformers 4.57, whose Qwen3-VL
        # rotary module read ``rope_scaling``.  Transformers 5.5 reads the
        # equivalent ``rope_parameters`` directly from Qwen3VLTextConfig.
        # Keep the upstream checkout untouched and adapt only this runtime.
        if hasattr(text_config, "rope_parameters"):
            rotary_class = modeling_qwen3vl_mot.Qwen3VLRotaryEmbeddingWrapper

            def _transformers5_rotary_init(instance, rotary_config, device=None):
                nn.Module.__init__(instance)
                instance._rope = modeling_qwen3vl_mot._VLRotaryEmbedding(
                    config=rotary_config,
                    device=device,
                )

            rotary_class.__init__ = _transformers5_rotary_init
        action_dim = int((config.output_features or {})["action"].shape[0])
        previous_dtype = torch.get_default_dtype()
        torch.set_default_dtype(torch.bfloat16)
        try:
            self.trex_model = Qwen3VLVLAModel(
                config=text_config,
                action_dim=action_dim,
                action_chunk=config.chunk_size,
                use_tactile_deform=False,
                use_robot_state=False,
                image_token_id=int(
                    getattr(full_config, "image_token_id", 151655)
                ),
                tactile_intermediate_size=config.tactile_intermediate_size,
            )
            self.trex_model.visual = Qwen3VLVisionModel(
                full_config.vision_config
            )
            # transformers 5.x wraps the merged visual tokens in
            # BaseModelOutputWithDeepstackFeatures.  T-Rex 4.57 code expects
            # the merged tensor directly (the new ``pooler_output`` field).
            original_visual_forward = self.trex_model.visual.forward

            def _tensor_visual_forward(instance, *args, **kwargs):
                output = original_visual_forward(*args, **kwargs)
                return getattr(output, "pooler_output", output)

            self.trex_model.visual.forward = MethodType(
                _tensor_visual_forward,
                self.trex_model.visual,
            )
            self.pressure_adapter = PressureTokenAdapter(
                text_config.hidden_size, config.pressure_adapter_width
            )
        finally:
            torch.set_default_dtype(previous_dtype)

        class _RopeStub:
            def __init__(self, model_config):
                self.config = model_config

            def get_vision_position_ids(self, *args, **kwargs):
                return Qwen3VLModel.get_vision_position_ids(
                    self, *args, **kwargs
                )

            def get_rope_index(
                self, input_ids, image_grid_thw=None, attention_mask=None
            ):
                mm_token_type_ids = (
                    input_ids
                    == int(getattr(self.config, "image_token_id", 151655))
                ).to(torch.int32)
                return Qwen3VLModel.get_rope_index(
                    self,
                    input_ids=input_ids,
                    mm_token_type_ids=mm_token_type_ids,
                    image_grid_thw=image_grid_thw,
                    attention_mask=attention_mask,
                )

        object.__setattr__(
            self.trex_model,
            "_rope_index_fn",
            _RopeStub(full_config).get_rope_index,
        )

    def reset(self) -> None:
        return None

    def get_optim_params(self) -> dict:
        return {"params": self.parameters()}

    def forward(self, batch):
        raise NotImplementedError("TRexPolicy training is not supported")

    def select_action(self, batch, **kwargs):
        return self.predict_action_chunk(batch, **kwargs)[:, 0]

    @staticmethod
    def _required(batch, key):
        if key not in batch:
            raise KeyError(f"T-Rex preprocessor did not produce {key}")
        return batch[key]

    def _continue_with_pressure(
        self, cached_kv, positions, n_action, x_split, tau_split, tokens
    ):
        model = self.trex_model
        cache = model._clone_dynamic_cache(cached_kv)
        device, dtype = x_split.device, torch.bfloat16
        batch, chunk_size = x_split.shape[:2]
        tokens = tokens.to(device=device, dtype=dtype)
        sequence_size = tokens.shape[1] + 1 + chunk_size
        tactile_positions = model.model._extend_position_ids(
            positions, n_action, sequence_size
        )[..., -sequence_size:]
        dt = torch.tensor(
            -1.0 / self.config.cascaded_total_steps,
            dtype=dtype, device=device,
        )
        time = torch.tensor(tau_split, dtype=dtype, device=device)
        action = x_split.to(dtype)
        remaining = (
            self.config.cascaded_total_steps - self.config.cascaded_split_step
        )
        for step in range(remaining):
            embeddings = torch.cat(
                [
                    tokens,
                    model.t_embedder(time.expand(batch)).unsqueeze(1),
                    model.x_embedder(action),
                ],
                dim=1,
            )
            if step:
                cache.crop(-sequence_size)
            output = model.model(
                inputs_embeds=embeddings,
                position_ids=tactile_positions,
                past_key_values=cache,
                use_cache=True,
                latent_indexes=torch.arange(0, 0, device=device),
                action_indexes=torch.arange(0, 0, device=device),
                tactile_indexes=torch.arange(0, sequence_size, device=device),
            )
            velocity = model.final_layer_tactile(
                output.last_hidden_state[:, -chunk_size:, :]
            )
            action = action + dt * velocity
            time = time + dt
        return action

    @torch.no_grad()
    def predict_action_chunk(self, batch, **kwargs):
        model = self.trex_model
        input_ids = self._required(batch, _QWEN_INPUT_IDS_KEY)
        attention_mask = self._required(batch, _QWEN_ATTENTION_MASK_KEY)
        pixel_values = self._required(batch, _QWEN_PIXEL_VALUES_KEY)
        image_grid = self._required(batch, _QWEN_IMAGE_GRID_KEY)
        embeddings = model.prepare_inputs_embeds(
            input_ids=input_ids,
            pixel_values=pixel_values.to(torch.bfloat16),
            image_grid_thw=image_grid,
        )
        positions, _ = model.get_rope_index(
            input_ids=input_ids,
            image_grid_thw=image_grid,
            attention_mask=attention_mask,
        )
        positions = positions[:, :, : embeddings.shape[1]]
        noise = torch.randn(
            input_ids.shape[0],
            self.config.chunk_size,
            int((self.config.output_features or {})["action"].shape[0]),
            dtype=torch.bfloat16,
            device=input_ids.device,
        )
        x_split, cache, n_action, tau_split = (
            model.forward_flow_action_partial(
                inputs_embeds=embeddings,
                position_ids=positions,
                attention_mask=attention_mask,
                noise=noise,
                num_steps_total=self.config.cascaded_total_steps,
                split_step=self.config.cascaded_split_step,
                refresh_clean_kv=True,
            )
        )
        pressure_tokens = self.pressure_adapter(
            self._required(batch, _PRESSURE_CURRENT_KEY).to(torch.bfloat16),
            self._required(batch, _PRESSURE_DEFORMATION_KEY).to(
                torch.bfloat16
            ),
            self._required(batch, _PRESSURE_HISTORY_KEY).to(torch.bfloat16),
        )
        return self._continue_with_pressure(
            cache, positions, n_action, x_split, tau_split, pressure_tokens
        )


def load_trex_policy(model_path: str, device: torch.device) -> TRexPolicy:
    """Load a T-Rex checkpoint without allocating a duplicate state dict."""
    config = PreTrainedConfig.from_pretrained(model_path)
    if not isinstance(config, TRexConfig):
        raise TypeError(
            f"Expected TRexConfig, got {type(config).__name__}"
        )
    config.device = str(device)
    return TRexPolicy.from_pretrained(
        model_path, config=config, strict=True
    )
