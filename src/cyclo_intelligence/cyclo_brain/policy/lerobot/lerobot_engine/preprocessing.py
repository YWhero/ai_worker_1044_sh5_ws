#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0

"""LeRobot preprocessing helpers.

Builds a policy-ready batch from RobotClient sensor/state reads.
"""

from __future__ import annotations

import logging
import math
from typing import Any, Dict, List

import numpy as np
import torch

from .constants import STATE_KEY as _STATE_KEY
from .image_preprocessing import prepare_policy_image
from .tactile_runtime import (
    TACTILE_MODE_BOTH_EPISODE_BASELINE,
    tactile_runtime_mode,
)


logger = logging.getLogger("lerobot_engine")
_TACTILE_KEY_PREFIX = "observation.tactile."
_TACTILE_BASELINE_KEY_PREFIX = "observation.tactile_baseline."


class PreprocessingMixin:
    """RobotClient observation -> policy input batch."""

    def _build_observation(self, task_instruction: str) -> Dict[str, Any]:
        """Pull raw sensor data from RobotClient and build a policy batch."""
        assert self._robot is not None

        images = self._robot.get_images(format="rgb")
        if not images:
            return self._fail("No camera frames available")

        batch: Dict[str, Any] = {}

        for cam_name, policy_key in self._cameras.items():
            img = images.get(cam_name)
            if img is None:
                return self._fail(f"Missing camera frame: {cam_name}")
            cam_cfg = self._robot._config.get("cameras", {}).get(cam_name, {})
            try:
                img = prepare_policy_image(
                    img,
                    rotation_deg=cam_cfg.get("rotation_deg", 0),
                    target_size=self._image_resize.get(policy_key),
                )
            except Exception as exc:
                return self._fail(f"Camera preprocessing failed for {cam_name}: {exc}")
            tensor = torch.from_numpy(img.copy()).to(torch.float32) / 255.0
            tensor = tensor.permute(2, 0, 1).contiguous().unsqueeze(0)
            batch[policy_key] = tensor.to(self._device)

        target_joint_names = list(
            getattr(self, "_observation_state_joint_names", []) or []
        )
        joint_dict = self._robot.get_joint_positions()
        if not joint_dict:
            return self._fail("No joint positions available")

        state_parts: List[np.ndarray] = []
        state_joint_names: List[str] = []
        for modality in self._state_modalities:
            if modality == "mobile":
                odom = self._robot.get_odom()
                if odom is None:
                    return self._fail("Missing odom for mobile state")
                state_parts.append(
                    np.array(
                        [
                            float(odom["linear_velocity"][0]),
                            float(odom["linear_velocity"][1]),
                            float(odom["angular_velocity"][2]),
                        ],
                        dtype=np.float32,
                    )
                )
                state_joint_names.extend(
                    ("linear_x", "linear_y", "angular_z")
                )
                continue
            group = f"follower_{modality}"
            positions = joint_dict.get(group)
            if positions is None or len(positions) == 0:
                return self._fail(f"Missing joint group: {modality}")
            values = np.asarray(positions, dtype=np.float32)
            names = list(
                self._robot._config.get("joint_groups", {})
                .get(group, {})
                .get("joint_names", [])
            )
            if names and len(names) != len(values):
                return self._fail(
                    f"Joint-name/value mismatch for {modality}: "
                    f"{len(names)} != {len(values)}"
                )
            state_parts.append(values)
            if names:
                state_joint_names.extend(str(name) for name in names)
            else:
                state_joint_names.extend(
                    f"{group}[{index}]"
                    for index in range(len(values))
                )

        flat_state = np.concatenate(state_parts)
        if target_joint_names:
            name_to_value = dict(zip(state_joint_names, flat_state))
            missing = [
                name for name in target_joint_names if name not in name_to_value
            ]
            if missing:
                return self._fail(
                    "Cannot apply observation.state joint order; missing "
                    f"joints: {missing}"
                )
            flat_state = np.asarray(
                [name_to_value[name] for name in target_joint_names],
                dtype=np.float32,
            )

        try:
            expected = int(
                self._policy.config.input_features[_STATE_KEY].shape[0]
            )
        except Exception:
            expected = flat_state.size

        if target_joint_names:
            if flat_state.size != expected:
                return self._fail(
                    "Model-specific observation.state has "
                    f"{flat_state.size} values, policy expects {expected}; "
                    "refusing to pad or truncate named state"
                )
        else:
            # Newer SH5 policies append tactile finger means after the regular
            # joint/mobile state. Only request tactile when the policy shape
            # needs those extra dimensions so older policies remain compatible.
            if flat_state.size < expected:
                tactile_parts: List[np.ndarray] = []
                for tactile_name in (
                    getattr(self, "_tactile_modalities", []) or []
                ):
                    tactile = self._robot.get_tactile(tactile_name)
                    if tactile is None or len(tactile) == 0:
                        return self._fail(
                            f"Missing tactile sensor: {tactile_name}"
                        )
                    tactile_parts.append(
                        np.asarray(tactile, dtype=np.float32)
                    )
                if tactile_parts:
                    flat_state = np.concatenate(
                        [flat_state, *tactile_parts]
                    )

            # Legacy policies without named state metadata retain their old
            # zero-padding/truncation compatibility behavior.
            if flat_state.size < expected:
                pad = np.zeros(expected - flat_state.size, dtype=np.float32)
                logger.warning(
                    "state dim mismatch: got %d, policy expects %d - "
                    "padding %d zeros",
                    flat_state.size,
                    expected,
                    expected - flat_state.size,
                )
                flat_state = np.concatenate([flat_state, pad])
            elif flat_state.size > expected:
                logger.warning(
                    "state dim mismatch: got %d, policy expects %d - "
                    "truncating to %d",
                    flat_state.size,
                    expected,
                    expected,
                )
                flat_state = flat_state[:expected]
        batch[_STATE_KEY] = (
            torch.from_numpy(flat_state).unsqueeze(0).to(self._device)
        )

        self._add_tactile_observations(batch)
        batch["task"] = [task_instruction or ""]
        return batch

    def _add_tactile_observations(self, batch: Dict[str, Any]) -> None:
        """Add tactile tensors with the checkpoint's training-time semantics.

        New pressure-native checkpoints consume raw taxels and an explicit
        calibration baseline as separate features. Legacy checkpoints only
        advertise ``observation.tactile.<side>`` and continue to receive the
        already baseline-corrected values they were trained on.
        """
        mappings = getattr(self, "_tactile_inputs", {}) or {}
        config = self._policy.config
        input_features = config.input_features
        native_trex = (
            getattr(config, "architecture_version", None)
            in {"sh5_right_v2", "sh5_right_v3_absolute"}
        )
        history_size = getattr(config, "tactile_history_size", None)
        raw_history_hz = getattr(config, "tactile_history_hz", None)
        runtime_mode = tactile_runtime_mode(config)
        history_hz = None
        if native_trex:
            if (
                isinstance(history_size, bool)
                or not isinstance(history_size, int)
                or history_size <= 0
            ):
                raise RuntimeError(
                    "Native SH5 T-Rex requires a positive integer "
                    "tactile_history_size"
                )
            if isinstance(raw_history_hz, bool):
                raise RuntimeError(
                    "Native SH5 T-Rex requires finite positive "
                    "tactile_history_hz"
                )
            try:
                history_hz = float(raw_history_hz)
            except (TypeError, ValueError) as exc:
                raise RuntimeError(
                    "Native SH5 T-Rex requires finite positive "
                    "tactile_history_hz"
                ) from exc
            if not math.isfinite(history_hz) or history_hz <= 0.0:
                raise RuntimeError(
                    "Native SH5 T-Rex requires finite positive "
                    "tactile_history_hz"
                )

        for policy_key, sensor_name in mappings.items():
            feature = input_features[policy_key]
            expected_shape = tuple(int(value) for value in feature.shape)
            side = policy_key.rsplit(".", 1)[-1].lower()
            baseline_key = f"{_TACTILE_BASELINE_KEY_PREFIX}{side}"
            baseline_feature = input_features.get(baseline_key)

            if (
                side == "left"
                and runtime_mode != TACTILE_MODE_BOTH_EPISODE_BASELINE
            ):
                if native_trex:
                    raise RuntimeError(
                        "Native SH5 T-Rex cannot consume left tactile input"
                    )
                # The training dataset's complete left tactile stream was
                # zeroed. Live values must never leak into this feature.
                corrected = np.zeros(expected_shape, dtype=np.float32)
            else:
                if native_trex:
                    if expected_shape != (45,):
                        raise RuntimeError(
                            "Native SH5 T-Rex tactile feature must be flat "
                            f"45-D, got {expected_shape}"
                        )
                    get_history = getattr(
                        self._robot,
                        "get_tactile_taxel_history",
                        None,
                    )
                    if not callable(get_history):
                        raise RuntimeError(
                            "RobotClient does not provide timestamped tactile "
                            "history required by native SH5 T-Rex"
                        )
                    taxels = get_history(
                        sensor_name,
                        history_size=history_size,
                        sample_hz=history_hz,
                    )
                    expected_history_shape = (
                        history_size,
                        5,
                        3,
                        3,
                    )
                    if tuple(taxels.shape) != expected_history_shape:
                        raise RuntimeError(
                            f"{sensor_name} history must be "
                            f"{expected_history_shape}, got {tuple(taxels.shape)}"
                        )
                    tactile = np.asarray(
                        taxels,
                        dtype=np.float32,
                    ).reshape(history_size, 45)
                else:
                    taxels = self._robot.get_tactile_taxels(sensor_name)
                    if taxels is None or taxels.size == 0:
                        raise RuntimeError(
                            f"Missing tactile taxels: {sensor_name}"
                        )
                    if taxels.size != int(np.prod(expected_shape)):
                        raise RuntimeError(
                            f"{sensor_name} has {taxels.size} taxels, policy "
                            f"{policy_key} expects shape {expected_shape}"
                        )
                    tactile = np.asarray(
                        taxels,
                        dtype=np.float32,
                    ).reshape(expected_shape)
                if not np.isfinite(tactile).all():
                    raise RuntimeError(
                        f"{sensor_name} tactile input contains NaN or Inf"
                    )

                if (
                    side == "right"
                    or runtime_mode == TACTILE_MODE_BOTH_EPISODE_BASELINE
                ):
                    baseline = getattr(
                        self,
                        "_tactile_baselines",
                        {},
                    ).get(policy_key)
                    if baseline is None:
                        raise RuntimeError(
                            f"Missing right tactile baseline: {policy_key}"
                        )
                    baseline = np.asarray(
                        baseline,
                        dtype=np.float32,
                    ).reshape(expected_shape)
                    if not np.isfinite(baseline).all():
                        raise RuntimeError(
                            f"Right tactile baseline contains NaN or Inf: "
                            f"{policy_key}"
                        )
                    if native_trex and baseline_feature is None:
                        raise RuntimeError(
                            "Native SH5 T-Rex requires an explicit right "
                            "tactile baseline feature"
                        )
                    if baseline_feature is None:
                        corrected = np.maximum(tactile - baseline, 0.0)
                    else:
                        baseline_shape = tuple(
                            int(value) for value in baseline_feature.shape
                        )
                        if baseline_shape != expected_shape:
                            raise RuntimeError(
                                f"{baseline_key} shape {baseline_shape} does "
                                f"not match {policy_key} shape {expected_shape}"
                            )
                        corrected = tactile
                        batch[baseline_key] = (
                            torch.from_numpy(
                                np.ascontiguousarray(
                                    baseline,
                                    dtype=np.float32,
                                )
                            )
                            .unsqueeze(0)
                            .to(self._device)
                        )
                else:
                    corrected = tactile

            batch[policy_key] = (
                torch.from_numpy(
                    np.ascontiguousarray(corrected, dtype=np.float32)
                )
                .unsqueeze(0)
                .to(self._device)
            )
