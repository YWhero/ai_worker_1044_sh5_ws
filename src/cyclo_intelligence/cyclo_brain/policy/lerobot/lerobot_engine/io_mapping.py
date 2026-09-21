#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""LeRobot engine I/O mapping helpers (IoMappingMixin).

Extracted from ``engine.py`` to keep the core ``LeRobotEngine`` class
focused on the ``InferenceEngine`` API. Mixed into the engine via
multiple inheritance; bind-mounted into the policy container as part
of the ``/app/lerobot_engine/`` package.

Owns:
- ``_init_robot``: create RobotClient + resolve camera / state mappings.
- ``_teardown_robot``: release the RobotClient.
- ``_policy_image_keys``: read the policy's expected image input keys.
"""

from __future__ import annotations

import logging
import os
import re
import time
from typing import Dict, Iterable

import numpy as np

from .constants import IMAGE_KEY_PREFIX as _IMAGE_KEY_PREFIX
from .tactile_runtime import (
    TACTILE_MODE_BOTH_EPISODE_BASELINE,
    tactile_runtime_mode,
)

from robot_client import RobotClient


logger = logging.getLogger("lerobot_engine")


_CAMERA_SEMANTIC_RE = re.compile(
    r"^cam_(?P<a>left|right|head|wrist)_(?P<b>left|right|head|wrist)$"
)
_TACTILE_KEY_PREFIX = "observation.tactile."
_DEFAULT_TACTILE_BASELINE_SAMPLES = 20
_DEFAULT_TACTILE_BASELINE_TIMEOUT_S = 5.0
_DEFAULT_TACTILE_HISTORY_TIMEOUT_S = 2.0
_HEAD_CAMERA_PROFILE_TOPIC = "/ffw/head_camera_profile"
_HEAD_CAMERA_PROFILE_TIMEOUT_S = 1.0
_HEAD_CAMERA_NAMES = {"cam_left_head", "cam_right_head"}


class IoMappingMixin:
    """Robot wiring — camera / state modality resolution and teardown."""

    def _init_robot(self, robot_type: str) -> None:
        """Create RobotClient + resolve camera / state mappings."""
        policy_image_keys = self._policy_image_keys()
        requested_camera_names, task_head_camera = (
            self._requested_camera_names(policy_image_keys)
        )
        self._robot = RobotClient(
            robot_type,
            requested_camera_names=requested_camera_names,
        )

        # Cameras: only those that match a policy input key
        # ``observation.images.<cam>``. Cameras advertised by the robot
        # but not consumed by the policy are silently ignored — same
        # behavior as GR00TInference.
        try:
            active = self._resolve_camera_mappings(
                self._robot.camera_names,
                policy_image_keys,
            )
        except RuntimeError:
            active = self._resolve_single_task_head_camera(
                self._robot.camera_names,
                policy_image_keys,
                task_head_camera,
            )
            if active is None:
                raise
        if not active and policy_image_keys:
            raise RuntimeError(
                "No cameras match the policy's expected input keys: "
                f"policy needs {sorted(policy_image_keys)}, robot has "
                f"{self._robot.camera_names}"
            )
        self._cameras = active

        # State modalities: prefer the robot_config observation.state order
        # because it matches the LeRobot converter's observation.state concat
        # order. Older runtime configs did not expose this list, so keep a
        # follower-group fallback for compatibility. Synthetic per-modality
        # views (with ``parent``) win over their leaf physical group;
        # otherwise the leaf group is used directly.
        groups = self._robot._config.get("joint_groups", {})
        modalities = list(self._robot._config.get("state_modalities") or [])
        if not modalities:
            parents = {cfg.get("parent") for cfg in groups.values() if cfg.get("parent")}
            modality_groups = []
            for name, cfg in groups.items():
                if cfg.get("role") != "follower" or not name.startswith("follower_"):
                    continue
                if cfg.get("parent"):
                    modality_groups.append(name)
                elif name not in parents:
                    modality_groups.append(name)
            modalities = sorted(name[len("follower_"):] for name in modality_groups)
        if not modalities:
            raise RuntimeError(
                f"No follower joint groups in robot_type={robot_type}"
            )

        # Mobile is sourced from sensors["odom"] in the new schema —
        # bridge it into observation.state alongside the joint states so
        # policies trained on the legacy physical_ai_server pipeline
        # (with mobile as a 3-vector modality) still see it.
        sensors = self._robot._config.get("sensors", {})
        self._has_mobile_state = "odom" in sensors
        if self._has_mobile_state and "mobile" not in modalities:
            modalities = sorted(set(modalities) | {"mobile"})

        self._state_modalities = modalities
        self._tactile_modalities = list(
            self._robot._config.get("tactile_modalities") or []
        )
        self._tactile_inputs = self._resolve_tactile_mappings(
            self._policy_tactile_keys(),
            self._tactile_modalities,
        )
        self._observation_state_joint_names = (
            self._resolve_observation_state_joint_names(modalities)
        )
        self._action_keys = self._resolve_policy_action_keys(modalities)

        # Wait only for inputs consumed by this policy. A standard ACT
        # checkpoint must not depend on tactile topics merely because the
        # robot advertises tactile sensors. Which tactile sides are required
        # is an explicit checkpoint-scoped preprocessing contract.
        if not self._wait_for_policy_inputs(timeout=10.0):
            raise RuntimeError(
                f"Required policy inputs were not ready for robot_type={robot_type}"
            )
        self._tactile_baselines = self._calibrate_tactile_inputs()
        self._wait_for_native_trex_tactile_history()
        logger.info(
            "Robot ready: cameras=%s state_modalities=%s "
            "tactile_modalities=%s tactile_inputs=%s tactile_runtime_mode=%s",
            list(self._cameras.keys()),
            self._state_modalities,
            self._tactile_modalities,
            self._tactile_inputs,
            tactile_runtime_mode(self._policy),
        )

    def _resolve_observation_state_joint_names(
        self,
        modalities: Iterable[str],
    ) -> list[str]:
        """Read an optional model-specific observation.state joint order."""
        configured = getattr(
            getattr(self._policy, "config", None),
            "observation_state_joint_names",
            None,
        )
        if configured is None:
            return []
        if not isinstance(configured, list) or not configured:
            raise ValueError(
                "Policy observation_state_joint_names must be "
                "a non-empty list"
            )
        names = [str(name).strip() for name in configured]
        if any(not name for name in names):
            raise ValueError(
                "Policy observation_state_joint_names contains "
                "an empty name"
            )
        if len(set(names)) != len(names):
            raise ValueError(
                "Policy observation_state_joint_names contains "
                "duplicates"
            )

        available: set[str] = set()
        groups = self._robot._config.get("joint_groups", {})
        for modality in modalities:
            if modality == "mobile":
                available.update(("linear_x", "linear_y", "angular_z"))
                continue
            group = groups.get(f"follower_{modality}", {})
            available.update(str(name) for name in group.get("joint_names", []))
        missing = [name for name in names if name not in available]
        if missing:
            raise ValueError(
                "Policy observation_state_joint_names are not "
                f"available on this robot: {missing}"
            )

        try:
            expected = int(
                self._policy.config.input_features[
                    "observation.state"
                ].shape[0]
            )
        except Exception:
            expected = len(names)
        if len(names) != expected:
            raise ValueError(
                "Policy observation_state_joint_names has "
                f"{len(names)} entries, policy expects {expected}"
            )
        logger.info(
            "Using model-specific observation.state joint order (%d joints)",
            len(names),
        )
        return names

    def _wait_for_policy_inputs(self, timeout: float) -> bool:
        """Wait for exactly the camera/state/sensor streams this policy reads."""
        deadline = time.monotonic() + max(0.0, float(timeout))
        required_cameras = list(self._cameras)
        required_joints = [
            f"follower_{modality}"
            for modality in self._state_modalities
            if modality != "mobile"
        ]
        required_sensors = ["odom"] if self._has_mobile_state else []
        runtime_mode = tactile_runtime_mode(self._policy)
        required_sensors.extend(
            sensor_name
            for policy_key, sensor_name in self._tactile_inputs.items()
            if (
                runtime_mode == TACTILE_MODE_BOTH_EPISODE_BASELINE
                or policy_key.rsplit(".", 1)[-1].lower() != "left"
            )
        )

        def missing() -> list[str]:
            result = [
                f"camera:{name}"
                for name in required_cameras
                if not self._robot.is_image_ready(name)
            ]
            result.extend(
                f"joint:{name}"
                for name in required_joints
                if not self._robot.is_joint_ready(name)
            )
            result.extend(
                f"sensor:{name}"
                for name in required_sensors
                if not self._robot.is_sensor_ready(name)
            )
            return result

        while time.monotonic() < deadline:
            if not missing():
                return True
            time.sleep(0.05)
        logger.warning("Timeout waiting for policy inputs. Missing: %s", missing())
        return False

    def _policy_tactile_keys(self) -> set[str]:
        try:
            features = getattr(self._policy.config, "input_features", {}) or {}
            return {
                key
                for key in features
                if key.startswith(_TACTILE_KEY_PREFIX)
            }
        except Exception:
            return set()

    @staticmethod
    def _resolve_tactile_mappings(
        policy_keys: Iterable[str],
        sensor_names: Iterable[str],
    ) -> Dict[str, str]:
        """Map ``observation.tactile.left/right`` to runtime sensors."""
        sensors = list(sensor_names)
        mappings: Dict[str, str] = {}
        for policy_key in sorted(policy_keys):
            side = policy_key.rsplit(".", 1)[-1].lower()
            matches = [
                sensor
                for sensor in sensors
                if side in sensor.lower()
            ]
            if len(matches) != 1:
                raise RuntimeError(
                    f"Cannot uniquely map tactile input {policy_key}: "
                    f"matches={matches}, robot sensors={sensors}"
                )
            mappings[policy_key] = matches[0]
        return mappings

    def _calibrate_tactile_inputs(self) -> Dict[str, np.ndarray]:
        """Reproduce the checkpoint-scoped first-20-message correction."""
        if not self._tactile_inputs:
            return {}

        sample_count = int(
            os.environ.get(
                "LEROBOT_TACTILE_BASELINE_SAMPLES",
                _DEFAULT_TACTILE_BASELINE_SAMPLES,
            )
        )
        timeout = float(
            os.environ.get(
                "LEROBOT_TACTILE_BASELINE_TIMEOUT_S",
                _DEFAULT_TACTILE_BASELINE_TIMEOUT_S,
            )
        )
        baselines: Dict[str, np.ndarray] = {}
        runtime_mode = tactile_runtime_mode(self._policy)

        for policy_key, sensor_name in self._tactile_inputs.items():
            side = policy_key.rsplit(".", 1)[-1].lower()
            if (
                side == "left"
                and runtime_mode != TACTILE_MODE_BOTH_EPISODE_BASELINE
            ):
                # The corrected training set contains an all-zero left hand.
                # Do not read or calibrate live left tactile at inference.
                continue
            if side not in {"left", "right"}:
                continue

            samples = self._robot.wait_for_tactile_samples(
                sensor_name,
                sample_count=sample_count,
                timeout=timeout,
            )
            if samples.shape[0] < sample_count:
                raise RuntimeError(
                    f"Right tactile calibration needs {sample_count} samples "
                    f"from {sensor_name}, got {samples.shape[0]} in "
                    f"{timeout:.1f}s"
                )

            baseline = np.rint(np.median(samples, axis=0)).astype(np.float32)
            baselines[policy_key] = baseline
            logger.info(
                "Calibrated %s from first %d samples: shape=%s range=%.0f..%.0f",
                policy_key,
                sample_count,
                tuple(baseline.shape),
                float(baseline.min()),
                float(baseline.max()),
            )
        return baselines

    def _wait_for_native_trex_tactile_history(self) -> None:
        """Warm up the sensor-cadence history required by native SH5 T-Rex."""
        config = getattr(getattr(self, "_policy", None), "config", None)
        if getattr(config, "architecture_version", None) not in {
            "sh5_right_v2",
            "sh5_right_v3_absolute",
        }:
            return

        history_size = getattr(config, "tactile_history_size", None)
        history_hz = getattr(config, "tactile_history_hz", None)
        if (
            isinstance(history_size, bool)
            or not isinstance(history_size, int)
            or history_size <= 0
        ):
            raise RuntimeError(
                "Native SH5 T-Rex requires a positive integer "
                "tactile_history_size"
            )
        if isinstance(history_hz, bool):
            raise RuntimeError(
                "Native SH5 T-Rex requires finite positive tactile_history_hz"
            )
        try:
            history_hz = float(history_hz)
        except (TypeError, ValueError) as exc:
            raise RuntimeError(
                "Native SH5 T-Rex requires finite positive tactile_history_hz"
            ) from exc
        if not np.isfinite(history_hz) or history_hz <= 0.0:
            raise RuntimeError(
                "Native SH5 T-Rex requires finite positive tactile_history_hz"
            )

        right_inputs = [
            (policy_key, sensor_name)
            for policy_key, sensor_name in self._tactile_inputs.items()
            if policy_key.rsplit(".", 1)[-1].lower() == "right"
        ]
        if len(right_inputs) != 1:
            raise RuntimeError(
                "Native SH5 T-Rex requires exactly one right tactile input; "
                f"resolved {right_inputs}"
            )

        required_span = (history_size - 1) / history_hz
        default_timeout = max(
            _DEFAULT_TACTILE_HISTORY_TIMEOUT_S,
            required_span + 1.0,
        )
        raw_timeout = os.environ.get("LEROBOT_TACTILE_HISTORY_TIMEOUT_S")
        try:
            timeout = (
                default_timeout
                if raw_timeout is None or not raw_timeout.strip()
                else float(raw_timeout)
            )
        except (TypeError, ValueError) as exc:
            raise RuntimeError(
                "LEROBOT_TACTILE_HISTORY_TIMEOUT_S must be finite and positive"
            ) from exc
        if not np.isfinite(timeout) or timeout <= 0.0:
            raise RuntimeError(
                "LEROBOT_TACTILE_HISTORY_TIMEOUT_S must be finite and positive"
            )

        wait_for_history = getattr(
            self._robot,
            "wait_for_tactile_taxel_history",
            None,
        )
        if not callable(wait_for_history):
            raise RuntimeError(
                "RobotClient does not provide timestamped tactile history "
                "required by native SH5 T-Rex"
            )

        policy_key, sensor_name = right_inputs[0]
        history = wait_for_history(
            sensor_name,
            history_size=history_size,
            sample_hz=history_hz,
            timeout=timeout,
        )
        expected_shape = (history_size, 5, 3, 3)
        if tuple(history.shape) != expected_shape:
            raise RuntimeError(
                f"{sensor_name} history for {policy_key} must be "
                f"{expected_shape}, got {tuple(history.shape)}"
            )
        logger.info(
            "Native SH5 T-Rex tactile history ready: sensor=%s size=%d "
            "rate=%.3fHz span=%.3fs",
            sensor_name,
            history_size,
            history_hz,
            required_span,
        )

    def _resolve_policy_action_keys(self, fallback_modalities: Iterable[str]) -> list[str]:
        """Choose robot action keys whose flat width matches the policy output.

        SH5 right-only policies output ``arm_right + hand_right`` (27D) while
        older full-body checkpoints output either all non-mobile joints (57D)
        or the complete robot action layout (60D). The runtime must publish the
        same grouping that the model was trained to emit.
        """
        all_keys = list(getattr(self._robot, "action_keys", []) or [])
        if not all_keys:
            return list(fallback_modalities)

        action_dim = self._policy_action_dim()
        configured = getattr(
            getattr(self._policy, "config", None),
            "model_action_keys",
            None,
        )
        if configured is not None:
            if not isinstance(configured, list) or not configured:
                raise ValueError(
                    "Policy model_action_keys must be a non-empty "
                    "list"
                )
            model_keys = [str(key).strip() for key in configured]
            if any(not key for key in model_keys) or len(set(model_keys)) != len(
                model_keys
            ):
                raise ValueError(
                    "Policy model_action_keys must contain unique "
                    "non-empty names"
                )
            missing = [key for key in model_keys if key not in all_keys]
            if missing:
                raise ValueError(
                    "Policy model_action_keys are unavailable on "
                    f"this robot: {missing}; robot={all_keys}"
                )
            if action_dim is None:
                raise RuntimeError(
                    "Cannot map policy model_action_keys "
                    "because the policy action dimension is unavailable"
                )
            width = int(self._robot.action_dimension(model_keys))
            if width != action_dim:
                raise RuntimeError(
                    "Policy model_action_keys have flat width "
                    f"{width}, but policy action dimension is {action_dim}: "
                    f"{model_keys}"
                )
            logger.info(
                "Using model action layout: "
                "%s (%dD)",
                model_keys,
                width,
            )
            return model_keys

        if action_dim is None:
            return all_keys

        recorded_keys = list(getattr(self._robot, "recorded_action_keys", []) or [])
        non_mobile_keys = [
            key
            for key in all_keys
            if self._robot._action_groups.get(key, {}).get("msg_type")
            != "geometry_msgs/msg/Twist"
        ]
        candidates = [
            recorded_keys,
            non_mobile_keys,
            all_keys,
            list(fallback_modalities),
        ]
        seen: set[tuple[str, ...]] = set()
        for keys in candidates:
            candidate = tuple(keys)
            if not candidate or candidate in seen:
                continue
            seen.add(candidate)
            try:
                width = self._robot.action_dimension(list(candidate))
            except Exception:
                continue
            if width == action_dim:
                logger.info(
                    "Policy action dim %d mapped to action_keys=%s",
                    action_dim,
                    list(candidate),
                )
                return list(candidate)

        raise RuntimeError(
            "Policy action dim "
            f"{action_dim} did not match known robot action layouts: "
            f"recorded={recorded_keys}, non_mobile={non_mobile_keys}, "
            f"all={all_keys}, fallback={list(fallback_modalities)}"
        )

    def _policy_action_dim(self) -> int | None:
        try:
            features = getattr(self._policy.config, "output_features", {}) or {}
            feature = features.get("action") if hasattr(features, "get") else None
            shape = getattr(feature, "shape", None)
            if shape is None and isinstance(feature, dict):
                shape = feature.get("shape")
            if shape is None:
                return None
            return int(list(shape)[0])
        except Exception:
            return None

    def _teardown_robot(self) -> None:
        if self._robot is not None:
            try:
                self._robot.close()
            except Exception:
                pass
            self._robot = None

    def _policy_image_keys(self) -> set:
        try:
            features = getattr(self._policy.config, "input_features", {}) or {}
            return {k for k in features.keys() if k.startswith(_IMAGE_KEY_PREFIX)}
        except Exception:
            return set()

    def _policy_camera_names(self) -> set[str]:
        names = set()
        for key in self._policy_image_keys():
            suffix = key[len(_IMAGE_KEY_PREFIX):].rsplit(".", 1)[-1].strip()
            match = _CAMERA_SEMANTIC_RE.match(suffix)
            if match:
                first = match.group("a")
                second = match.group("b")
                side = first if first in {"left", "right"} else second
                part = first if first in {"head", "wrist"} else second
                if side in {"left", "right"} and part in {"head", "wrist"}:
                    suffix = f"cam_{side}_{part}"
            if suffix:
                names.add(suffix)
        return names

    def _requested_camera_names(
        self,
        policy_image_keys: set,
    ) -> tuple[set[str], str]:
        """Select camera sources; single-head policies follow ffw_bringup."""
        policy_camera_names = self._policy_camera_names()
        if len(policy_image_keys) != 1:
            return policy_camera_names, ""

        head_camera = self._read_head_camera_profile_once()
        if head_camera not in _HEAD_CAMERA_NAMES:
            return policy_camera_names, ""

        logger.info(
            "Using task head camera profile %s for single-image policy",
            head_camera,
        )
        return {head_camera}, head_camera

    @staticmethod
    def _read_head_camera_profile_once() -> str:
        """Read the latched head-camera profile published by ffw_bringup."""
        node = None
        context = None
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )
            from std_msgs.msg import String

            camera_name = ""

            def _callback(msg: String) -> None:
                nonlocal camera_name
                camera_name = str(msg.data or "").strip()

            context = Context()
            rclpy.init(args=None, context=context)
            node = rclpy.create_node(
                "lerobot_head_camera_profile_reader",
                context=context,
            )
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            node.create_subscription(
                String,
                _HEAD_CAMERA_PROFILE_TOPIC,
                _callback,
                qos,
            )

            deadline = time.monotonic() + _HEAD_CAMERA_PROFILE_TIMEOUT_S
            while context.ok() and not camera_name and time.monotonic() < deadline:
                timeout = min(0.05, max(0.0, deadline - time.monotonic()))
                rclpy.spin_once(node, timeout_sec=timeout)
            return camera_name
        except Exception as exc:
            logger.debug("Head camera profile unavailable: %s", exc)
            return ""
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                try:
                    if context is not None:
                        import rclpy

                        rclpy.shutdown(context=context)
                except Exception:
                    pass

    @staticmethod
    def _resolve_single_task_head_camera(
        robot_camera_names: Iterable[str],
        policy_image_keys: set,
        task_head_camera: str,
    ) -> Dict[str, str] | None:
        """Feed the one model image input from the task-selected head camera."""
        if task_head_camera not in _HEAD_CAMERA_NAMES:
            return None
        if len(policy_image_keys) != 1:
            return None
        if task_head_camera not in set(robot_camera_names):
            return None
        return {task_head_camera: next(iter(policy_image_keys))}

    @classmethod
    def _resolve_camera_mappings(
        cls,
        robot_camera_names: Iterable[str],
        policy_image_keys: set,
    ) -> Dict[str, str]:
        """Map RobotClient camera names to policy image feature keys.

        The canonical Cyclo camera names are ``cam_<side>_<part>`` such as
        ``cam_left_head``. Some runtime configs or checkpoints may expose
        ``rgb.`` prefixes. Older single-head checkpoints may use
        ``cam_head`` for the left head camera. Exact matches remain preferred.
        """
        camera_names = list(robot_camera_names)
        if not policy_image_keys:
            return {cam: f"{_IMAGE_KEY_PREFIX}{cam}" for cam in camera_names}

        active: Dict[str, str] = {}
        used_policy_keys = set()
        for cam in camera_names:
            exact = f"{_IMAGE_KEY_PREFIX}{cam}"
            candidates = cls._camera_policy_key_candidates(cam)
            matches = sorted(policy_image_keys & candidates)
            if not matches:
                continue

            if exact in matches:
                chosen = exact
            elif len(matches) == 1:
                chosen = matches[0]
            else:
                raise RuntimeError(
                    f"Ambiguous camera mapping for {cam}: matches {matches}"
                )

            if chosen in used_policy_keys:
                raise RuntimeError(
                    f"Policy camera key {chosen} matched multiple robot cameras"
                )
            active[cam] = chosen
            used_policy_keys.add(chosen)

        missing = sorted(policy_image_keys - used_policy_keys)
        if missing:
            raise RuntimeError(
                "Missing camera mappings for policy input keys: "
                f"{missing}; robot has {camera_names}; matched {active}"
            )
        return active

    @staticmethod
    def _camera_policy_key_candidates(camera_name: str) -> set:
        aliases = {camera_name}
        parts = camera_name.split(".")
        suffix = parts[-1]
        prefixes = parts[:-1]
        aliases.add(suffix)

        semantic_names = {suffix}
        if suffix == "cam_left_head":
            semantic_names.add("cam_head")
            semantic_names.add("scene")

        match = _CAMERA_SEMANTIC_RE.match(suffix)
        if match:
            first = match.group("a")
            second = match.group("b")
            side = first if first in {"left", "right"} else second
            part = first if first in {"head", "wrist"} else second
            if side in {"left", "right"} and part in {"head", "wrist"}:
                semantic_names.add(f"cam_{side}_{part}")
                semantic_names.add(f"cam_{part}_{side}")
                semantic_names.add(f"{part}_{side}")
                semantic_names.add(f"{side}_{part}")

        for name in semantic_names:
            aliases.add(name)
            aliases.add(f"rgb.{name}")
            if prefixes:
                aliases.add(".".join([*prefixes, name]))

        return {f"{_IMAGE_KEY_PREFIX}{alias}" for alias in aliases}
