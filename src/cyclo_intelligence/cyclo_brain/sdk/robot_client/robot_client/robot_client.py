#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Dongyun Kim

"""
RobotClient - High-level abstraction for robot sensor data and control.

Provides simple Python API over zenoh_ros2_sdk, hiding all Zenoh/ROS2 details.
Users only need to specify robot type to get automatic topic subscription.
"""
import os
import sys
import time
import threading
import logging
import math
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path
from typing import Iterable, Optional, Union

import numpy as np
import cv2

# Add zenoh_ros2_sdk to path if not already available
_SDK_PATH = os.environ.get("ZENOH_SDK_PATH", "")
if _SDK_PATH and _SDK_PATH not in sys.path:
    sys.path.insert(0, _SDK_PATH)

from zenoh_ros2_sdk import ROS2Publisher, ROS2Subscriber, get_message_class  # noqa: E402


# -- robot config schema helper -----------------------------------------------
# shared/robot_configs/ is bind-mounted into the policy container at
# /orchestrator_config/, so schema.py lands beside the per-robot yamls.
# The module is intentionally self-contained (no `shared` package
# imports) so it can be picked up as a standalone file from that mount.
_SCHEMA_DIR = os.environ.get("ORCHESTRATOR_CONFIG_PATH", "/orchestrator_config")
if os.path.isdir(_SCHEMA_DIR) and _SCHEMA_DIR not in sys.path:
    sys.path.insert(0, _SCHEMA_DIR)
try:
    import schema as robot_schema  # type: ignore[import-not-found]
except ImportError:
    _src = Path(__file__).resolve()
    for _parent in _src.parents:
        _cand = _parent / "shared" / "robot_configs"
        if _cand.is_dir():
            sys.path.insert(0, str(_cand))
            break
    import schema as robot_schema  # type: ignore[import-not-found]


logger = logging.getLogger("robot_client")

_TACTILE_HISTORY_MAX_SAMPLES = 512
_JOINT_HISTORY_MAX_SAMPLES = 512
_T_REX_TACTILE_WIDTH = 45
_SIM_TACTILE_HISTORY_MAX_AGE_CAP_S = 0.1


def _simulation_tactile_history_max_age() -> Optional[float]:
    """Read the explicit, bounded simulator receipt-jitter allowance.

    Hardware keeps the caller's single-period freshness limit. This setting
    changes only allowed source age, never the policy's causal sampling grid.
    """
    raw = os.environ.get("CYCLO_SIM_TACTILE_HISTORY_MAX_AGE_S")
    if raw is None or not raw.strip():
        return None
    if os.environ.get("CYCLO_SENSOR_HISTORY_MODE", "hardware") != "simulation":
        raise ValueError(
            "CYCLO_SIM_TACTILE_HISTORY_MAX_AGE_S requires "
            "CYCLO_SENSOR_HISTORY_MODE=simulation"
        )
    try:
        allowance = float(raw)
    except ValueError as exc:
        raise ValueError(
            "simulation tactile history max age must be finite and positive"
        ) from exc
    if (
        not math.isfinite(allowance)
        or not 0.0 < allowance <= _SIM_TACTILE_HISTORY_MAX_AGE_CAP_S
    ):
        raise ValueError(
            "simulation tactile history max age must be finite, positive, "
            f"and at most {_SIM_TACTILE_HISTORY_MAX_AGE_CAP_S}s"
        )
    logger.warning(
        "Explicit simulation tactile receipt-jitter allowance=%ss; "
        "causal sampling grid and missing-data checks remain enabled",
        allowance,
    )
    return allowance


def _float_env(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    try:
        return float(raw)
    except ValueError:
        logger.warning("Invalid %s=%r; using %s", name, raw, default)
        return default


def _deadband(value: float, threshold: float) -> float:
    return 0.0 if abs(value) < threshold else value


def _build_runtime_config(
    section: dict,
    requested_camera_names: Optional[Iterable[str]] = None,
) -> dict:
    """Translate the VLA-semantic schema into the cameras/joint_groups/
    sensors shape the RobotClient + inference engines consume.

    * ``observation.images``  → ``cameras``.
    * ``observation.state.<g>`` with JointState msg_type → physical
      follower joint group named ``follower_<g>``.
    * ``observation.state.<g>`` with Odometry msg_type   → ``sensors["odom"]``
      (treated as a sensor-backed state modality by GR00T).
    * ``observation.tactile.<g>`` with HandPressures msg_type →
      ``sensors["tactile_<g>"]`` as mean-pooled per-finger tactile values.
    * Each ``action.<modality>`` (excluding mobile/Twist) gets a SYNTHETIC
      ``follower_<modality>`` joint_group with ``parent`` pointing at the
      first physical follower; ``_update_joint`` slices the parent's
      message by the action's ``joint_names`` to populate it.
    """
    default_cameras = robot_schema.get_image_topics(section)
    requested_cameras = {
        str(name).strip()
        for name in (requested_camera_names or [])
        if str(name).strip()
    }
    if requested_cameras:
        cameras = {
            name: cfg
            for name, cfg in default_cameras.items()
            if name in requested_cameras
        }
    else:
        cameras = default_cameras
    state_groups = robot_schema.get_state_groups(section)
    tactile_groups = robot_schema.get_tactile_topics(section)
    action_groups = robot_schema.get_action_groups(section)

    joint_groups: dict = {}
    sensors: dict = {}
    source_state_modalities: list[str] = []
    tactile_modalities: list[str] = []
    physical_follower_name: Optional[str] = None

    for name, cfg in state_groups.items():
        msg_type = cfg["msg_type"]
        if msg_type == "sensor_msgs/msg/JointState":
            group_name = f"follower_{name}"
            joint_groups[group_name] = {
                "topic": cfg["topic"],
                "msg_type": msg_type,
                "role": "follower",
                "joint_names": list(cfg["joint_names"]),
            }
            if physical_follower_name is None:
                physical_follower_name = group_name
            source_state_modalities.append(name)
        elif msg_type == "nav_msgs/msg/Odometry":
            sensors["odom"] = {
                "topic": cfg["topic"],
                "msg_type": msg_type,
            }
            source_state_modalities.append(name)
        else:
            # Unknown state msg_type — keep it in joint_groups for
            # diagnostics; subscribers will pick it up via the generic
            # JointState callback unless a more specific shape lands.
            joint_groups[f"follower_{name}"] = {
                "topic": cfg["topic"],
                "msg_type": msg_type,
                "role": "follower",
                "joint_names": list(cfg["joint_names"]),
            }
            source_state_modalities.append(name)

    for name, cfg in tactile_groups.items():
        sensor_name = f"tactile_{name}"
        sensors[sensor_name] = {
            "topic": cfg["topic"],
            "msg_type": cfg["msg_type"],
            "kind": "tactile",
        }
        tactile_modalities.append(sensor_name)

    physical_joint_names = set(
        joint_groups.get(physical_follower_name, {}).get("joint_names", [])
    )
    if physical_follower_name is not None:
        for modality, cfg in action_groups.items():
            if cfg["msg_type"] == "geometry_msgs/msg/Twist":
                # action.mobile is command-only; observation RobotClient
                # instances stay read-only.
                continue
            action_joint_names = set(cfg.get("joint_names") or [])
            if physical_joint_names and not action_joint_names.issubset(
                physical_joint_names
            ):
                continue
            child_name = f"follower_{modality}"
            if child_name in joint_groups:
                # A physical follower already covers this modality.
                continue
            joint_groups[child_name] = {
                "parent": physical_follower_name,
                "role": "follower",
                "joint_names": list(cfg["joint_names"]),
            }

    state_modalities: list[str] = []
    for modality, cfg in action_groups.items():
        if cfg["msg_type"] == "geometry_msgs/msg/Twist":
            if "odom" in sensors:
                state_modalities.append(modality)
            continue
        if f"follower_{modality}" in joint_groups:
            state_modalities.append(modality)
    if not state_modalities:
        state_modalities = source_state_modalities

    return {
        "cameras": cameras,
        "joint_groups": joint_groups,
        "sensors": sensors,
        "state_modalities": state_modalities,
        "tactile_modalities": tactile_modalities,
    }


# Compatibility re-export for older engine code. Same VLA-semantic section in,
# same runtime-config dict out.
def derive_robot_config(section: dict) -> dict:
    return _build_runtime_config(section)


class RobotClient:
    """High-level robot interface over zenoh_ros2_sdk.

    Usage:
        robot = RobotClient("ffw_sg2_rev1")
        robot.wait_for_ready(timeout=10.0)
        images = robot.get_images()
        joints = robot.get_joint_positions()
    """

    def __init__(
        self,
        robot_type: str,
        sync_check: bool = False,
        sync_threshold_ms: float = 33.0,
        router_ip: str = "127.0.0.1",
        router_port: int = 7447,
        domain_id: Optional[int] = None,
        enable_command_publishers: bool = False,
        enable_preview_publisher: bool = False,
        requested_camera_names: Optional[Iterable[str]] = None,
    ):
        section = robot_schema.load_robot_section(robot_type)
        # Phase 4: yaml is VLA-semantic (observation.images / state +
        # action.<modality>). _build_runtime_config translates that into
        # the cameras / joint_groups / sensors shape RobotClient and the
        # downstream inference engines have always consumed.
        self._config = _build_runtime_config(
            section,
            requested_camera_names=requested_camera_names,
        )

        self._robot_type = robot_type
        self._sync_check = sync_check
        self._sync_threshold_ms = sync_threshold_ms
        self._router_ip = router_ip
        self._router_port = router_port
        self._domain_id = domain_id
        self._enable_command_publishers = bool(enable_command_publishers)
        self._enable_preview_publisher = bool(enable_preview_publisher)
        self._action_groups = robot_schema.get_action_groups(section)
        self._recorded_action_groups = robot_schema.get_recorded_action_groups(section)
        self._joint_position_limits = self._load_joint_position_limits(
            robot_schema.get_urdf_path(section)
        )

        # Thread-safe data stores
        self._lock = threading.Lock()
        self._images: dict[str, np.ndarray] = {}
        self._image_timestamps: dict[str, float] = {}
        self._joint_positions: dict[str, np.ndarray] = {}
        self._joint_velocities: dict[str, np.ndarray] = {}
        self._joint_efforts: dict[str, np.ndarray] = {}
        self._joint_timestamps: dict[str, float] = {}
        self._joint_positions_by_name: dict[str, float] = {}
        self._joint_position_timestamps_by_name: dict[str, float] = {}
        # Preserve physical joint-group callbacks at source cadence so
        # history-conditioned policies can reconstruct the same causal
        # sampling grid used by their training dataset.
        self._joint_history_samples: dict[
            str, deque[tuple[float, np.ndarray]]
        ] = {}
        self._sensors: dict[str, dict] = {}
        self._sensor_timestamps: dict[str, float] = {}
        # Keep the first raw tactile frames after subscription so an
        # inference policy can reproduce dataset-time startup calibration.
        self._tactile_calibration_samples: dict[str, list[np.ndarray]] = {}
        # Preserve raw pressure frames at sensor-callback cadence.  Policy
        # inference is chunked and therefore runs much more slowly than the
        # dataset sampling grid; building history in the policy preprocessor
        # would silently stretch a 1 s training window across many seconds.
        self._tactile_history_samples: dict[
            str, deque[tuple[float, np.ndarray]]
        ] = {}
        self._tactile_history_max_age_s = _simulation_tactile_history_max_age()
        self._task_instruction: str = ""

        self._subscribers: list = []
        self._command_publishers: dict[str, ROS2Publisher] = {}
        self._preview_publisher: Optional[ROS2Publisher] = None
        self._command_msg_types: dict[str, str] = {}
        self._command_joint_names: dict[str, list[str]] = {}
        self._action_keys = sorted(self._action_groups.keys())
        self._recorded_action_keys = [
            key for key in self._action_keys if key in self._recorded_action_groups
        ]
        self._cmd_vel_linear_deadband = max(
            0.0,
            _float_env("CMD_VEL_LINEAR_DEADBAND", 0.0),
        )
        self._cmd_vel_angular_deadband = max(
            0.0,
            _float_env("CMD_VEL_ANGULAR_DEADBAND", 0.0),
        )
        self._initial_pose_sync_state_max_age_s = _float_env(
            "INITIAL_POSE_SYNC_STATE_MAX_AGE_S",
            1.0,
        )
        if (
            not math.isfinite(self._initial_pose_sync_state_max_age_s)
            or self._initial_pose_sync_state_max_age_s <= 0.0
        ):
            logger.warning(
                "INITIAL_POSE_SYNC_STATE_MAX_AGE_S must be positive and finite; "
                "using 1.0"
            )
            self._initial_pose_sync_state_max_age_s = 1.0
        self._closed = False

        self._init_subscriptions()
        if self._enable_command_publishers:
            self._init_command_publishers()
            if self._cmd_vel_linear_deadband or self._cmd_vel_angular_deadband:
                logger.info(
                    "cmd_vel deadband enabled: linear=%s angular=%s",
                    self._cmd_vel_linear_deadband,
                    self._cmd_vel_angular_deadband,
                )
        if self._enable_preview_publisher:
            self._init_preview_publisher()
        logger.info(f"RobotClient initialized: {robot_type} "
                     f"({len(self._config.get('cameras', {}))} cameras, "
                     f"{len(self._config.get('joint_groups', {}))} joint groups)")

    # ------------------------------------------------------------------ #
    # Initialization
    # ------------------------------------------------------------------ #

    def _init_subscriptions(self):
        """Subscribe to all configured topics.

        Joint groups carrying a ``parent`` field have no physical topic of
        their own — they're synthetic per-modality views over a sibling
        group's data. ``_update_joint`` propagates from parent → children
        by name-based slicing inside the callback.
        """
        # Cameras
        for cam_name, cam_cfg in self._config.get("cameras", {}).items():
            sub = ROS2Subscriber(
                topic=cam_cfg["topic"],
                msg_type=cam_cfg["msg_type"],
                callback=lambda msg, name=cam_name: self._update_image(name, msg),
            )
            self._subscribers.append(sub)
            logger.debug(f"Subscribed camera: {cam_name} -> {cam_cfg['topic']}")

        # Index parent → list of child group names so the upper-body
        # callback knows which slices to populate per message.
        self._joint_children: dict[str, list[str]] = {}
        for child_name, child_cfg in self._config.get("joint_groups", {}).items():
            parent = child_cfg.get("parent")
            if parent:
                self._joint_children.setdefault(parent, []).append(child_name)

        # Joint groups — only those with their own physical topic.
        for group_name, group_cfg in self._config.get("joint_groups", {}).items():
            if group_cfg.get("parent"):
                logger.debug(
                    f"Skipped joint subscription: {group_name} "
                    f"(synthetic view of {group_cfg['parent']})"
                )
                continue
            sub = ROS2Subscriber(
                topic=group_cfg["topic"],
                msg_type=group_cfg["msg_type"],
                callback=lambda msg, name=group_name: self._update_joint(name, msg),
            )
            self._subscribers.append(sub)
            logger.debug(f"Subscribed joint: {group_name} -> {group_cfg['topic']}")

        # Additional sensors. ``sensor_cfg`` may carry an optional
        # ``type_hash`` override — escape hatch for messages where
        # zenoh_ros2_sdk's hash computation needs to be pinned to a known
        # wire hash. Default is auto-compute via the SDK.
        for sensor_name, sensor_cfg in self._config.get("sensors", {}).items():
            sub_kwargs = dict(
                topic=sensor_cfg["topic"],
                msg_type=sensor_cfg["msg_type"],
                callback=lambda msg, name=sensor_name: self._update_sensor(name, msg),
            )
            if sensor_cfg.get("type_hash"):
                sub_kwargs["type_hash"] = sensor_cfg["type_hash"]
            sub = ROS2Subscriber(**sub_kwargs)
            self._subscribers.append(sub)
            logger.debug(f"Subscribed sensor: {sensor_name} -> {sensor_cfg['topic']}")

    def _init_command_publishers(self):
        """Create publishers for configured action topics."""
        common = {
            "router_ip": self._router_ip,
            "router_port": self._router_port,
        }
        if self._domain_id is not None:
            common["domain_id"] = self._domain_id

        for action_key in self._action_keys:
            cfg = self._action_groups[action_key]
            publisher_key = f"leader_{action_key}"
            self._command_msg_types[publisher_key] = cfg["msg_type"]
            self._command_joint_names[publisher_key] = list(cfg.get("joint_names", []))
            self._command_publishers[publisher_key] = ROS2Publisher(
                topic=cfg["topic"],
                msg_type=cfg["msg_type"],
                **common,
            )
            logger.debug(
                "Command publisher: %s -> %s (%s)",
                publisher_key,
                cfg["topic"],
                cfg["msg_type"],
            )

    def _init_preview_publisher(self):
        """Create a unified trajectory preview publisher for the 3D viewer."""
        common = {
            "router_ip": self._router_ip,
            "router_port": self._router_port,
        }
        if self._domain_id is not None:
            common["domain_id"] = self._domain_id
        self._preview_publisher = ROS2Publisher(
            topic="/inference/trajectory_preview",
            msg_type="trajectory_msgs/msg/JointTrajectory",
            **common,
        )
        logger.debug("Action preview publisher: /inference/trajectory_preview")

    # ------------------------------------------------------------------ #
    # Callback handlers
    # ------------------------------------------------------------------ #

    def _update_image(self, cam_name: str, msg):
        """CompressedImage -> BGR numpy array."""
        try:
            data = msg.data
            if isinstance(data, (list, tuple)):
                data = bytes(data)
            buf = np.frombuffer(data, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if img is not None:
                with self._lock:
                    self._images[cam_name] = img
                    self._image_timestamps[cam_name] = time.time()
        except Exception as e:
            logger.warning(f"Failed to decode image from {cam_name}: {e}")

    def _update_joint(self, group_name: str, msg):
        """JointState -> np.ndarray(float32).

        Stores the full vector under ``group_name``. If the group has
        synthetic children (other yaml groups with ``parent: <group_name>``),
        slice each child's positions out of the message by joint name and
        store the slice under the child's group name too — so callers
        like ``get_joint_positions("follower_arm_left")`` see the same
        per-modality surface as before the upper-body collapse.
        """
        try:
            msg_names = list(msg.name) if hasattr(msg, 'name') else []
            position = list(msg.position) if hasattr(msg.position, '__iter__') else []
            velocity = list(msg.velocity) if hasattr(msg.velocity, '__iter__') else []
            effort = list(msg.effort) if hasattr(msg.effort, '__iter__') else []
            group_cfg = self._config.get("joint_groups", {}).get(group_name, {})
            wanted_names = list(group_cfg.get("joint_names") or [])
            name_to_idx = {n: i for i, n in enumerate(msg_names)} if msg_names else {}

            def _slice_values(values: list[float], label: str) -> Optional[np.ndarray]:
                if not values:
                    return None
                if wanted_names and name_to_idx:
                    try:
                        indices = [name_to_idx[n] for n in wanted_names]
                    except KeyError as missing:
                        logger.debug(
                            f"{group_name}: joint {missing} missing from "
                            f"{label} message"
                        )
                        return None
                    if max(indices, default=-1) >= len(values):
                        return None
                    return np.array([values[i] for i in indices], dtype=np.float32)
                return np.array(values, dtype=np.float32)

            now = time.time()
            received_monotonic = time.monotonic()
            with self._lock:
                if position and msg_names:
                    self._joint_positions_by_name.update(
                        {
                            name: float(value)
                            for name, value in zip(msg_names, position)
                        }
                    )
                    self._joint_position_timestamps_by_name.update(
                        {name: received_monotonic for name in msg_names}
                    )
                sliced_position = _slice_values(position, "position")
                sliced_velocity = _slice_values(velocity, "velocity")
                sliced_effort = _slice_values(effort, "effort")
                if sliced_position is not None:
                    self._joint_positions[group_name] = sliced_position
                    self._joint_timestamps[group_name] = now
                    history = self._joint_history_samples.setdefault(
                        group_name,
                        deque(maxlen=_JOINT_HISTORY_MAX_SAMPLES),
                    )
                    history.append(
                        (received_monotonic, sliced_position.copy())
                    )
                if sliced_velocity is not None:
                    self._joint_velocities[group_name] = sliced_velocity
                if sliced_effort is not None:
                    self._joint_efforts[group_name] = sliced_effort

                # Propagate to synthetic child views.
                children = getattr(self, "_joint_children", {}).get(group_name, [])
                if children and msg_names:
                    for child in children:
                        child_cfg = self._config["joint_groups"].get(child, {})
                        wanted = child_cfg.get("joint_names", [])
                        try:
                            indices = [name_to_idx[n] for n in wanted]
                        except KeyError as missing:
                            # First few callbacks may race ahead of full
                            # name list — skip this child until the parent
                            # message carries every joint we expect.
                            logger.debug(
                                f"{child}: joint {missing} missing from "
                                f"{group_name} message"
                            )
                            continue
                        if position and max(indices, default=-1) < len(position):
                            self._joint_positions[child] = np.array(
                                [position[i] for i in indices], dtype=np.float32
                            )
                            self._joint_timestamps[child] = now
                        if velocity and len(velocity) == len(msg_names):
                            self._joint_velocities[child] = np.array(
                                [velocity[i] for i in indices], dtype=np.float32
                            )
                        if effort and len(effort) == len(msg_names):
                            self._joint_efforts[child] = np.array(
                                [effort[i] for i in indices], dtype=np.float32
                            )
        except Exception as e:
            logger.warning(f"Failed to parse joint from {group_name}: {e}")

    def _update_sensor(self, sensor_name: str, msg):
        """Parse sensor messages (Odometry, Twist, etc.)."""
        try:
            data = {}
            sensor_cfg = self._config.get("sensors", {}).get(sensor_name, {})
            if sensor_name == "odom":
                pos = msg.pose.pose.position
                ori = msg.pose.pose.orientation
                lin = msg.twist.twist.linear
                ang = msg.twist.twist.angular
                data = {
                    "position": np.array([pos.x, pos.y, pos.z], dtype=np.float32),
                    "orientation": np.array([ori.x, ori.y, ori.z, ori.w], dtype=np.float32),
                    "linear_velocity": np.array([lin.x, lin.y, lin.z], dtype=np.float32),
                    "angular_velocity": np.array([ang.x, ang.y, ang.z], dtype=np.float32),
                }
            elif sensor_name == "cmd_vel":
                data = {
                    "linear": np.array([msg.linear.x, msg.linear.y, msg.linear.z], dtype=np.float32),
                    "angular": np.array([msg.angular.x, msg.angular.y, msg.angular.z], dtype=np.float32),
                }
            elif sensor_cfg.get("kind") == "tactile":
                taxels = self._hand_pressure_taxels(msg)
                data = {
                    # ``values`` remains the backwards-compatible per-finger
                    # mean view used by flat observation.state policies.
                    "values": taxels.mean(axis=(1, 2), dtype=np.float32),
                    # Custom tactile ACT consumes every 3x3 taxel.
                    "taxels": taxels,
                    "hand_name": str(getattr(msg, "hand_name", "") or ""),
                }
            else:
                data = {"raw": str(msg)}

            received_monotonic = time.monotonic()
            with self._lock:
                self._sensors[sensor_name] = data
                self._sensor_timestamps[sensor_name] = time.time()
                if sensor_cfg.get("kind") == "tactile":
                    history = self._tactile_history_samples.setdefault(
                        sensor_name,
                        deque(maxlen=_TACTILE_HISTORY_MAX_SAMPLES),
                    )
                    history.append(
                        (received_monotonic, data["taxels"].copy())
                    )
                    samples = self._tactile_calibration_samples.setdefault(
                        sensor_name,
                        [],
                    )
                    if len(samples) < 64:
                        samples.append(data["taxels"].copy())
        except Exception as e:
            logger.warning(f"Failed to parse sensor {sensor_name}: {e}")

    @staticmethod
    def _hand_pressure_taxels(msg) -> np.ndarray:
        """HandPressures -> float32 array shaped ``(fingers, 3, 3)``."""
        sensors = list(getattr(msg, "sensors", []) or [])
        if not sensors:
            raise ValueError("HandPressures message has no sensors")

        rows: list[np.ndarray] = []
        for sensor in sensors:
            raw_values = getattr(sensor, "pressure_values", None)
            if raw_values is None:
                arr = np.zeros(9, dtype=np.float32)
            elif isinstance(raw_values, (bytes, bytearray, memoryview)):
                arr = np.frombuffer(raw_values, dtype=np.uint8).astype(np.float32)
            else:
                arr = np.asarray(list(raw_values), dtype=np.float32)
            if arr.size != 9:
                raise ValueError(
                    "Expected 9 pressure taxels per finger, "
                    f"got {arr.size}"
                )
            rows.append(arr.reshape(3, 3))
        return np.stack(rows).astype(np.float32, copy=False)

    @classmethod
    def _mean_pool_hand_pressures(cls, msg) -> np.ndarray:
        """HandPressures -> one float32 mean value per finger sensor."""
        return cls._hand_pressure_taxels(msg).mean(
            axis=(1, 2),
            dtype=np.float32,
        )

    # ------------------------------------------------------------------ #
    # Image API
    # ------------------------------------------------------------------ #

    @property
    def camera_names(self) -> list[str]:
        return list(self._config.get("cameras", {}).keys())

    def get_images(
        self,
        resize: Optional[tuple[int, int]] = None,
        format: str = "bgr",
    ) -> dict[str, np.ndarray]:
        """Get all camera images.

        Args:
            resize: Optional (width, height) tuple. None = original size.
            format: "bgr" (default) or "rgb".
        """
        with self._lock:
            result = {k: v.copy() for k, v in self._images.items()}
        if resize:
            result = {k: cv2.resize(v, resize) for k, v in result.items()}
        if format == "rgb":
            result = {k: cv2.cvtColor(v, cv2.COLOR_BGR2RGB) for k, v in result.items()}
        return result

    def get_image(
        self,
        camera_name: str,
        resize: Optional[tuple[int, int]] = None,
        format: str = "bgr",
    ) -> Optional[np.ndarray]:
        """Get single camera image."""
        with self._lock:
            img = self._images.get(camera_name)
            if img is None:
                return None
            img = img.copy()
        if resize:
            img = cv2.resize(img, resize)
        if format == "rgb":
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    def is_image_ready(self, camera_name: str) -> bool:
        with self._lock:
            return camera_name in self._images

    def get_image_timestamp(self, camera_name: str) -> Optional[float]:
        with self._lock:
            return self._image_timestamps.get(camera_name)

    # ------------------------------------------------------------------ #
    # Joint API
    # ------------------------------------------------------------------ #

    @property
    def joint_group_names(self) -> list[str]:
        return list(self._config.get("joint_groups", {}).keys())

    @property
    def total_dof(self) -> int:
        return self._config.get("total_dof", 0)

    def get_joint_names(self, group_name: str) -> list[str]:
        cfg = self._config.get("joint_groups", {}).get(group_name, {})
        return cfg.get("joint_names", [])

    def get_dof(self, group_name: str) -> int:
        cfg = self._config.get("joint_groups", {}).get(group_name, {})
        return cfg.get("dof", 0)

    def get_joint_positions(
        self, group: Optional[str] = None
    ) -> Union[dict[str, np.ndarray], np.ndarray]:
        """Get joint positions. Returns dict if no group, or np.ndarray for specific group."""
        with self._lock:
            if group:
                arr = self._joint_positions.get(group)
                return arr.copy() if arr is not None else np.array([], dtype=np.float32)
            return {k: v.copy() for k, v in self._joint_positions.items()}

    def get_joint_position_snapshot(
        self,
        group_name: str,
    ) -> tuple[np.ndarray, Optional[float]]:
        """Atomically copy one joint-position vector and its receive time."""
        with self._lock:
            positions = self._joint_positions.get(group_name)
            timestamp = self._joint_timestamps.get(group_name)
            return (
                positions.copy()
                if positions is not None
                else np.array([], dtype=np.float32),
                timestamp,
            )

    def get_joint_position_history(
        self,
        joint_names: Iterable[str],
        history_size: int,
        sample_hz: float,
    ) -> np.ndarray:
        """Return a causal joint history in the caller's exact name order.

        The most recent physical JointState callback anchors a regular grid.
        Each grid point selects the latest source frame at or before it, which
        matches the converter's previous-value resampling semantics.
        """
        requested = [str(name).strip() for name in joint_names]
        if not requested or any(not name for name in requested):
            raise ValueError("joint_names must contain non-empty names")
        if len(set(requested)) != len(requested):
            raise ValueError("joint_names must not contain duplicates")
        if (
            isinstance(history_size, bool)
            or not isinstance(history_size, int)
            or history_size <= 0
        ):
            raise ValueError("history_size must be a positive integer")
        if isinstance(sample_hz, bool):
            raise ValueError("sample_hz must be finite and positive")
        try:
            sample_hz = float(sample_hz)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "sample_hz must be finite and positive"
            ) from exc
        if not np.isfinite(sample_hz) or sample_hz <= 0.0:
            raise ValueError("sample_hz must be finite and positive")

        with self._lock:
            candidates = []
            for group_name, group_samples in self._joint_history_samples.items():
                configured_names = list(
                    self._config.get("joint_groups", {})
                    .get(group_name, {})
                    .get("joint_names", [])
                )
                if not set(requested).issubset(configured_names):
                    continue
                candidates.append(
                    (
                        len(configured_names),
                        group_name,
                        configured_names,
                        list(group_samples),
                    )
                )
        if not candidates:
            raise RuntimeError(
                "Joint history is not ready for requested joints: "
                f"{requested}"
            )
        _, group_name, configured_names, samples = min(candidates)
        if not samples:
            raise RuntimeError(f"Joint history is not ready for {group_name}")

        timestamps = np.asarray(
            [timestamp for timestamp, _ in samples],
            dtype=np.float64,
        )
        if not np.isfinite(timestamps).all():
            raise RuntimeError(
                f"Joint history timestamps are non-finite for {group_name}"
            )
        if timestamps.size > 1 and np.any(np.diff(timestamps) <= 0.0):
            raise RuntimeError(
                f"Joint history timestamps are non-monotonic for {group_name}"
            )

        period = 1.0 / sample_hz
        latest = float(timestamps[-1])
        latest_age = time.monotonic() - latest
        tolerance = period + max(1e-9, period * 1e-6)
        if not np.isfinite(latest_age) or latest_age < 0.0:
            raise RuntimeError(
                f"Joint history clock is invalid for {group_name}"
            )
        if latest_age > tolerance:
            raise RuntimeError(
                f"Joint history is stale for {group_name}: "
                f"age={latest_age:.3f}s, limit={period:.3f}s"
            )

        targets = latest - (
            np.arange(history_size - 1, -1, -1, dtype=np.float64) * period
        )
        indices = np.searchsorted(timestamps, targets, side="right") - 1
        if np.any(indices < 0):
            covered = latest - float(timestamps[0])
            required = (history_size - 1) * period
            raise RuntimeError(
                f"Joint history warmup incomplete for {group_name}: "
                f"covered={covered:.3f}s, required={required:.3f}s"
            )
        selected_times = timestamps[indices]
        staleness = targets - selected_times
        if np.any(staleness < -1e-9) or np.any(staleness > tolerance):
            worst = float(staleness.max(initial=0.0))
            raise RuntimeError(
                f"Joint history has a stale resampling gap for {group_name}: "
                f"max_age={worst:.3f}s, limit={period:.3f}s"
            )

        name_to_index = {
            name: index for index, name in enumerate(configured_names)
        }
        requested_indices = [name_to_index[name] for name in requested]
        frames = []
        for index in indices:
            frame = np.asarray(samples[int(index)][1], dtype=np.float32)
            if frame.size != len(configured_names):
                raise RuntimeError(
                    f"Joint history frame for {group_name} has {frame.size} "
                    f"values; expected {len(configured_names)}"
                )
            frames.append(frame[requested_indices])
        return np.stack(frames).astype(np.float32, copy=False)

    def wait_for_joint_position_history(
        self,
        joint_names: Iterable[str],
        history_size: int,
        sample_hz: float,
        timeout: float,
    ) -> np.ndarray:
        """Wait for a valid causal joint history during policy LOAD."""
        requested = list(joint_names)
        if isinstance(timeout, bool):
            raise ValueError("timeout must be finite and positive")
        try:
            timeout = float(timeout)
        except (TypeError, ValueError) as exc:
            raise ValueError("timeout must be finite and positive") from exc
        if not np.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("timeout must be finite and positive")

        deadline = time.monotonic() + timeout
        last_error: RuntimeError | None = None
        while True:
            try:
                return self.get_joint_position_history(
                    requested,
                    history_size=history_size,
                    sample_hz=sample_hz,
                )
            except RuntimeError as exc:
                last_error = exc
            if self._closed or time.monotonic() >= deadline:
                detail = str(last_error) if last_error is not None else "unknown"
                raise RuntimeError(
                    "Timed out waiting for joint history after "
                    f"{timeout:.3f}s: {detail}"
                ) from last_error
            time.sleep(0.01)

    def get_joint_velocities(
        self, group: Optional[str] = None
    ) -> Union[dict[str, np.ndarray], np.ndarray]:
        with self._lock:
            if group:
                arr = self._joint_velocities.get(group)
                return arr.copy() if arr is not None else np.array([], dtype=np.float32)
            return {k: v.copy() for k, v in self._joint_velocities.items()}

    def get_joint_efforts(
        self, group: Optional[str] = None
    ) -> Union[dict[str, np.ndarray], np.ndarray]:
        with self._lock:
            if group:
                arr = self._joint_efforts.get(group)
                return arr.copy() if arr is not None else np.array([], dtype=np.float32)
            return {k: v.copy() for k, v in self._joint_efforts.items()}

    def is_joint_ready(self, group_name: str) -> bool:
        with self._lock:
            return group_name in self._joint_positions

    def get_joint_timestamp(self, group_name: str) -> Optional[float]:
        with self._lock:
            return self._joint_timestamps.get(group_name)

    # ------------------------------------------------------------------ #
    # Sensor API
    # ------------------------------------------------------------------ #

    def get_odom(self) -> Optional[dict]:
        with self._lock:
            return self._sensors.get("odom")

    def get_sensor(self, sensor_name: str) -> Optional[dict]:
        with self._lock:
            data = self._sensors.get(sensor_name)
            if data is None:
                return None
            return {
                key: value.copy() if isinstance(value, np.ndarray) else value
                for key, value in data.items()
            }

    def get_tactile(self, sensor_name: str) -> Optional[np.ndarray]:
        data = self.get_sensor(sensor_name)
        if not data:
            return None
        values = data.get("values")
        return values.copy() if isinstance(values, np.ndarray) else None

    def get_tactile_taxels(self, sensor_name: str) -> Optional[np.ndarray]:
        """Return the latest raw tactile matrix shaped ``(fingers, 3, 3)``."""
        data = self.get_sensor(sensor_name)
        if not data:
            return None
        taxels = data.get("taxels")
        return taxels.copy() if isinstance(taxels, np.ndarray) else None

    def get_tactile_taxel_history(
        self,
        sensor_name: str,
        history_size: int,
        sample_hz: float,
    ) -> np.ndarray:
        """Return an episode-style causal raw-taxel history for T-Rex.

        The latest callback timestamp anchors a regular ``sample_hz`` grid.
        Each grid point selects the most recent sensor frame at or before that
        point, matching the converter's causal previous-value resampling.
        Missing, stale, or non-monotonic history is reported to the caller.
        An explicitly configured simulation receipt-jitter allowance can widen
        the source-age limit; it does not change this causal sampling grid.
        """
        if (
            isinstance(history_size, bool)
            or not isinstance(history_size, int)
            or history_size <= 0
        ):
            raise ValueError("history_size must be a positive integer")
        if isinstance(sample_hz, bool):
            raise ValueError("sample_hz must be finite and positive")
        try:
            sample_hz = float(sample_hz)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "sample_hz must be finite and positive"
            ) from exc
        if not np.isfinite(sample_hz) or sample_hz <= 0.0:
            raise ValueError("sample_hz must be finite and positive")

        with self._lock:
            samples = list(
                self._tactile_history_samples.get(sensor_name, ())
            )
        if not samples:
            raise RuntimeError(
                f"Tactile history is not ready for {sensor_name}"
            )

        timestamps = np.asarray(
            [timestamp for timestamp, _ in samples],
            dtype=np.float64,
        )
        if not np.isfinite(timestamps).all():
            raise RuntimeError(
                f"Tactile history timestamps are non-finite for {sensor_name}"
            )
        if timestamps.size > 1 and np.any(np.diff(timestamps) <= 0.0):
            raise RuntimeError(
                f"Tactile history timestamps are non-monotonic for {sensor_name}"
            )

        period = 1.0 / sample_hz
        latest = float(timestamps[-1])
        latest_age = time.monotonic() - latest
        age_limit = max(
            period, getattr(self, "_tactile_history_max_age_s", None) or period,
        )
        tolerance = age_limit + max(1e-9, age_limit * 1e-6)
        if not np.isfinite(latest_age) or latest_age < 0.0:
            raise RuntimeError(
                f"Tactile history clock is invalid for {sensor_name}"
            )
        if latest_age > tolerance:
            raise RuntimeError(
                f"Tactile history is stale for {sensor_name}: "
                f"age={latest_age:.3f}s, limit={age_limit:.3f}s"
            )

        targets = latest - (
            np.arange(history_size - 1, -1, -1, dtype=np.float64) * period
        )
        indices = np.searchsorted(timestamps, targets, side="right") - 1
        if np.any(indices < 0):
            covered = latest - float(timestamps[0])
            required = (history_size - 1) * period
            raise RuntimeError(
                f"Tactile history warmup incomplete for {sensor_name}: "
                f"covered={covered:.3f}s, required={required:.3f}s"
            )

        selected_times = timestamps[indices]
        staleness = targets - selected_times
        if np.any(staleness < -1e-9) or np.any(staleness > tolerance):
            worst = float(staleness.max(initial=0.0))
            raise RuntimeError(
                f"Tactile history has a stale resampling gap for {sensor_name}: "
                f"max_age={worst:.3f}s, limit={age_limit:.3f}s"
            )

        frames: list[np.ndarray] = []
        for index in indices:
            frame = np.asarray(samples[int(index)][1], dtype=np.float32)
            if frame.size != _T_REX_TACTILE_WIDTH:
                raise RuntimeError(
                    f"Tactile history frame for {sensor_name} has "
                    f"{frame.size} taxels; expected {_T_REX_TACTILE_WIDTH}"
                )
            frames.append(frame.reshape(5, 3, 3))
        return np.stack(frames).astype(np.float32, copy=False)

    def wait_for_tactile_taxel_history(
        self,
        sensor_name: str,
        history_size: int,
        sample_hz: float,
        timeout: float,
    ) -> np.ndarray:
        """Wait for a valid causal history, then return it.

        This is intended for bounded policy-load warmup.  Validation remains
        centralized in :meth:`get_tactile_taxel_history`, so a timeout reports
        the last concrete coverage, staleness, clock, or shape failure.
        """
        if isinstance(timeout, bool):
            raise ValueError("timeout must be finite and positive")
        try:
            timeout = float(timeout)
        except (TypeError, ValueError) as exc:
            raise ValueError("timeout must be finite and positive") from exc
        if not np.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("timeout must be finite and positive")

        deadline = time.monotonic() + timeout
        last_error: RuntimeError | None = None
        while True:
            try:
                return self.get_tactile_taxel_history(
                    sensor_name,
                    history_size=history_size,
                    sample_hz=sample_hz,
                )
            except RuntimeError as exc:
                last_error = exc
            if self._closed or time.monotonic() >= deadline:
                detail = str(last_error) if last_error is not None else "unknown"
                raise RuntimeError(
                    f"Timed out waiting for tactile history from {sensor_name} "
                    f"after {timeout:.3f}s: {detail}"
                ) from last_error
            time.sleep(0.01)

    def wait_for_tactile_samples(
        self,
        sensor_name: str,
        sample_count: int,
        timeout: float,
    ) -> np.ndarray:
        """Return the first ``sample_count`` raw frames after subscription."""
        if sample_count <= 0:
            raise ValueError("sample_count must be positive")

        deadline = time.monotonic() + max(0.0, float(timeout))
        while True:
            with self._lock:
                samples = list(
                    self._tactile_calibration_samples.get(sensor_name, [])
                )
                closed = self._closed
            if len(samples) >= sample_count:
                return np.stack(samples[:sample_count]).astype(
                    np.float32,
                    copy=False,
                )
            if closed or time.monotonic() >= deadline:
                if not samples:
                    return np.empty((0,), dtype=np.float32)
                return np.stack(samples).astype(np.float32, copy=False)
            time.sleep(0.01)

    def get_tactile_values(self) -> dict[str, np.ndarray]:
        with self._lock:
            result = {}
            for name, data in self._sensors.items():
                if not name.startswith("tactile_"):
                    continue
                values = data.get("values")
                if isinstance(values, np.ndarray):
                    result[name] = values.copy()
            return result

    def is_sensor_ready(self, sensor_name: str) -> bool:
        with self._lock:
            return sensor_name in self._sensors

    # ------------------------------------------------------------------ #
    # Command API
    # ------------------------------------------------------------------ #

    @property
    def action_keys(self) -> list[str]:
        return list(self._action_keys)

    @property
    def recorded_action_keys(self) -> list[str]:
        return list(self._recorded_action_keys)

    def action_dimension(self, action_keys: Optional[list[str]] = None) -> int:
        """Return the flat action width for the requested action keys."""
        keys = list(action_keys) if action_keys else self._action_keys
        total = 0
        for action_key in keys:
            publish_key = self._resolve_action_key(action_key)
            cfg = self._action_groups.get(publish_key)
            if cfg is None:
                continue
            total += (
                3
                if cfg["msg_type"] == "geometry_msgs/msg/Twist"
                else len(cfg["joint_names"])
            )
        return total

    def validate_real_action_contract(self, action_keys: list[str]) -> None:
        """Fail closed unless Real action layout and publishers are exact.

        Model outputs are flat arrays, so merely matching the total width is
        insufficient: a permuted key list can route otherwise valid numbers to
        the wrong joints.  Real mode therefore requires the model keys to match
        the recorded action schema exactly, including order, and verifies every
        publisher before any command can be attempted.
        """
        keys = list(action_keys)
        if not keys or any(not isinstance(key, str) or not key for key in keys):
            raise ValueError("real action keys must be non-empty strings")

        duplicates = sorted({key for key in keys if keys.count(key) > 1})
        if duplicates:
            raise ValueError(
                "real action keys contain duplicates: " + ", ".join(duplicates)
            )

        expected = list(self._recorded_action_keys)
        if keys != expected:
            raise ValueError(
                "real action key contract mismatch: "
                f"expected ordered keys {expected}, got {keys}"
            )

        for action_key in expected:
            publish_key = self._resolve_action_key(action_key)
            cfg = self._action_groups.get(publish_key)
            if cfg is None:
                raise ValueError(
                    f"real action configuration is missing for {action_key}"
                )
            msg_type = cfg.get("msg_type")
            width = (
                3
                if msg_type == "geometry_msgs/msg/Twist"
                else len(cfg.get("joint_names") or [])
            )
            if width <= 0:
                raise ValueError(
                    f"real action width is invalid for {action_key}: {width}"
                )
            publisher_key = f"leader_{publish_key}"
            if self._command_publishers.get(publisher_key) is None:
                raise ValueError(
                    f"real action publisher is unavailable: {publisher_key}"
                )

    @staticmethod
    def _load_joint_position_limits(
        urdf_path: str,
    ) -> dict[str, tuple[float, float]]:
        """Load finite lower/upper position limits keyed by URDF joint name.

        Missing or malformed URDF data stays observable as a missing limit.  A
        caller that enables real-robot safety can then fail closed instead of
        silently inventing a permissive bound.
        """
        if not urdf_path:
            logger.warning("Robot config has no URDF path; position limits unavailable")
            return {}
        try:
            root = ET.parse(urdf_path).getroot()
        except (OSError, ET.ParseError) as exc:
            logger.warning("Failed to load URDF position limits from %s: %s", urdf_path, exc)
            return {}

        limits: dict[str, tuple[float, float]] = {}
        for joint in root.findall("joint"):
            name = str(joint.get("name") or "")
            joint_type = str(joint.get("type") or "")
            limit = joint.find("limit")
            if not name or joint_type in {"fixed", "continuous"} or limit is None:
                continue
            try:
                lower = float(limit.get("lower"))
                upper = float(limit.get("upper"))
            except (TypeError, ValueError):
                continue
            if math.isfinite(lower) and math.isfinite(upper) and lower <= upper:
                limits[name] = (lower, upper)
        return limits

    def apply_real_action_safety(
        self,
        chunk: np.ndarray,
        action_keys: Optional[list[str]] = None,
        *,
        first_action_max_delta_rad: Optional[float] = None,
        first_action_max_delta_by_key: Optional[dict[str, float]] = None,
        warm_start_max_total_delta_by_key: Optional[dict[str, float]] = None,
        state_bridge_max_total_delta_by_key: Optional[dict[str, float]] = None,
        previous_published_action: Optional[np.ndarray] = None,
        source_step_max_delta_rad: Optional[float] = None,
        source_step_bridge_max_raw_delta_by_key: Optional[dict[str, float]] = None,
        state_max_age_s: Optional[float] = 0.5,
        joint_limit_mode: str = "reject",
        joint_limit_tolerance_rad: float = 0.0,
        joint_limit_tolerance_by_joint: Optional[dict[str, float]] = None,
    ) -> np.ndarray:
        """Validate and, when configured, clamp a real-robot action chunk.

        This method never publishes.  It returns a validated copy or raises
        ``ValueError`` so the control loop can drop the complete chunk.  Only
        position action groups are compared with follower state and URDF
        limits; velocity-like Twist groups retain their normal semantics.
        """
        values = np.asarray(chunk, dtype=np.float64)
        if values.ndim != 2 or values.shape[0] <= 0:
            raise ValueError(f"action chunk must be non-empty 2D; got {values.shape}")
        if not np.isfinite(values).all():
            raise ValueError("action chunk contains non-finite values")

        keys = list(action_keys) if action_keys else self._action_keys
        expected_dim = self.action_dimension(keys)
        if expected_dim <= 0 or values.shape[1] != expected_dim:
            raise ValueError(
                f"action dimension mismatch: got {values.shape[1]}, expected {expected_dim}"
            )

        limit_mode = str(joint_limit_mode or "off").strip().lower()
        if limit_mode not in {"off", "reject", "clamp"}:
            raise ValueError(
                "joint_limit_mode must be one of: 'off', 'reject', 'clamp'"
            )
        tolerance = float(joint_limit_tolerance_rad)
        if not math.isfinite(tolerance) or tolerance < 0.0:
            raise ValueError("joint_limit_tolerance_rad must be finite and non-negative")
        tolerance_by_joint: dict[str, float] = {}
        for raw_name, raw_value in (
            joint_limit_tolerance_by_joint or {}
        ).items():
            name = str(raw_name).strip()
            joint_tolerance = float(raw_value)
            if (
                not name
                or not math.isfinite(joint_tolerance)
                or joint_tolerance < 0.0
            ):
                raise ValueError(
                    "joint_limit_tolerance_by_joint values must be finite "
                    "and non-negative"
                )
            tolerance_by_joint[name] = joint_tolerance
        if tolerance_by_joint and limit_mode != "clamp":
            raise ValueError(
                "joint_limit_tolerance_by_joint requires clamp mode"
            )
        max_delta = (
            None
            if first_action_max_delta_rad is None
            else float(first_action_max_delta_rad)
        )
        if max_delta is not None and (
            not math.isfinite(max_delta) or max_delta <= 0.0
        ):
            raise ValueError("first_action_max_delta_rad must be finite and positive")
        max_delta_by_key: dict[str, float] = {}
        for raw_key, raw_value in (first_action_max_delta_by_key or {}).items():
            key = str(raw_key).strip()
            threshold = float(raw_value)
            if not key or not math.isfinite(threshold) or threshold <= 0.0:
                raise ValueError(
                    "first_action_max_delta_by_key values must be finite and positive"
                )
            max_delta_by_key[key] = threshold
        warm_start_max_total_by_key: dict[str, float] = {}
        for raw_key, raw_value in (
            warm_start_max_total_delta_by_key or {}
        ).items():
            key = str(raw_key).strip()
            threshold = float(raw_value)
            if not key or not math.isfinite(threshold) or threshold <= 0.0:
                raise ValueError(
                    "warm_start_max_total_delta_by_key values must be finite "
                    "and positive"
                )
            warm_start_max_total_by_key[key] = threshold
        state_bridge_max_total_by_key: dict[str, float] = {}
        for raw_key, raw_value in (
            state_bridge_max_total_delta_by_key or {}
        ).items():
            key = str(raw_key).strip()
            threshold = float(raw_value)
            if not key or not math.isfinite(threshold) or threshold <= 0.0:
                raise ValueError(
                    "state_bridge_max_total_delta_by_key values must be "
                    "finite and positive"
                )
            state_bridge_max_total_by_key[key] = threshold
        if warm_start_max_total_by_key and state_bridge_max_total_by_key:
            raise ValueError(
                "warm-start and state-tracking bridge cannot be enabled "
                "in the same safety pass"
            )
        max_source_step = (
            None
            if source_step_max_delta_rad is None
            else float(source_step_max_delta_rad)
        )
        if max_source_step is not None and (
            not math.isfinite(max_source_step) or max_source_step <= 0.0
        ):
            raise ValueError("source_step_max_delta_rad must be finite and positive")
        source_step_bridge_max_raw_by_key: dict[str, float] = {}
        for raw_key, raw_value in (
            source_step_bridge_max_raw_delta_by_key or {}
        ).items():
            key = str(raw_key).strip()
            threshold = float(raw_value)
            if not key or not math.isfinite(threshold) or threshold <= 0.0:
                raise ValueError(
                    "source_step_bridge_max_raw_delta_by_key values must be "
                    "finite and positive"
                )
            source_step_bridge_max_raw_by_key[key] = threshold
        if source_step_bridge_max_raw_by_key and max_source_step is None:
            raise ValueError(
                "source-step bridge requires source_step_max_delta_rad"
            )
        previous_action = None
        if previous_published_action is not None:
            previous_action = np.asarray(
                previous_published_action,
                dtype=np.float64,
            ).reshape(-1)
            if previous_action.size != expected_dim:
                raise ValueError(
                    "previous published action dimension mismatch: "
                    f"got {previous_action.size}, expected {expected_dim}"
                )
            if not np.isfinite(previous_action).all():
                raise ValueError("previous published action contains non-finite values")
            if max_source_step is None:
                raise ValueError(
                    "previous published action gate requires "
                    "source_step_max_delta_rad"
                )
        max_age = None if state_max_age_s is None else float(state_max_age_s)
        if max_age is not None and (not math.isfinite(max_age) or max_age <= 0.0):
            raise ValueError("state_max_age_s must be finite and positive")

        safe = values.copy()
        offset = 0
        now = time.time()
        state_bridge_slices: list[tuple[int, int, np.ndarray]] = []
        state_bridge_steps = 1
        state_bridge_anchor_clamped = False
        position_slices: list[tuple[int, int]] = []
        position_joint_names: set[str] = set()
        post_step_state_checks: list[
            tuple[int, int, np.ndarray, float, str, list[str]]
        ] = []
        source_transition_steps = np.ones(
            max(0, len(safe) - 1),
            dtype=np.int64,
        )
        for action_key in keys:
            publish_key = self._resolve_action_key(action_key)
            cfg = self._action_groups.get(publish_key)
            if cfg is None:
                raise ValueError(f"unknown action key: {action_key}")
            msg_type = cfg["msg_type"]
            width = (
                3
                if msg_type == "geometry_msgs/msg/Twist"
                else len(cfg.get("joint_names") or [])
            )
            segment = safe[:, offset:offset + width]
            offset += width
            if msg_type == "geometry_msgs/msg/Twist":
                continue

            joint_names = list(cfg.get("joint_names") or [])
            if width <= 0 or len(joint_names) != width:
                raise ValueError(f"invalid joint layout for action key: {action_key}")
            position_slices.append((offset - width, offset))
            position_joint_names.update(joint_names)

            if limit_mode != "off":
                missing = [
                    name for name in joint_names if name not in self._joint_position_limits
                ]
                if missing:
                    raise ValueError(
                        "URDF position limits missing for: " + ", ".join(missing)
                    )
                lower = np.asarray(
                    [self._joint_position_limits[name][0] for name in joint_names],
                    dtype=np.float64,
                )
                upper = np.asarray(
                    [self._joint_position_limits[name][1] for name in joint_names],
                    dtype=np.float64,
                )
                violation = np.maximum(lower - segment, segment - upper)
                max_violation = float(np.max(violation))
                if max_violation > 0.0:
                    joint_tolerances = np.asarray(
                        [
                            tolerance_by_joint.get(name, tolerance)
                            for name in joint_names
                        ],
                        dtype=np.float64,
                    )
                    disallowed = violation > joint_tolerances[None, :] + 1e-12
                    if limit_mode == "reject":
                        disallowed = violation > 0.0
                    violation_for_reject = np.where(
                        disallowed,
                        violation,
                        -np.inf,
                    )
                    reject_t, reject_j = np.unravel_index(
                        int(np.argmax(violation_for_reject)),
                        violation.shape,
                    )
                    reject_violation = float(
                        violation_for_reject[reject_t, reject_j]
                    )
                    if math.isfinite(reject_violation):
                        name = joint_names[int(reject_j)]
                        raise ValueError(
                            "URDF joint limit exceeded: "
                            f"{name} at step {int(reject_t)} by "
                            f"{reject_violation:.6f} rad"
                        )
                    worst_t, worst_j = np.unravel_index(
                        int(np.argmax(violation)), violation.shape
                    )
                    name = joint_names[int(worst_j)]
                    applied_tolerance = float(joint_tolerances[int(worst_j)])
                    logger.warning(
                        "Clamping URDF joint limit: action_key=%s joint=%s "
                        "step=%d violation=%.6f rad tolerance=%.6f rad",
                        action_key,
                        name,
                        int(worst_t),
                        max_violation,
                        applied_tolerance,
                    )
                    segment[:] = np.clip(segment, lower, upper)

            if max_source_step is not None and len(segment) > 1:
                source_steps = np.abs(np.diff(segment, axis=0))
                worst_t, worst_j = np.unravel_index(
                    int(np.argmax(source_steps)), source_steps.shape
                )
                worst_step = float(source_steps[worst_t, worst_j])
                bridge_max_raw = source_step_bridge_max_raw_by_key.get(
                    action_key,
                    source_step_bridge_max_raw_by_key.get(publish_key),
                )
                if source_step_bridge_max_raw_by_key and bridge_max_raw is None:
                    raise ValueError(
                        "source-step bridge raw-delta limit missing for "
                        f"action_key={action_key}"
                    )
                if bridge_max_raw is not None:
                    if worst_step > bridge_max_raw:
                        raise ValueError(
                            "unsafe raw source-step delta: "
                            f"{joint_names[int(worst_j)]} "
                            f"step {int(worst_t)}->{int(worst_t) + 1}="
                            f"{worst_step:.6f} rad > {bridge_max_raw:.6f} rad "
                            f"(action_key={action_key})"
                        )
                    group_transition_steps = np.maximum(
                        1,
                        np.ceil(
                            np.max(source_steps, axis=1) / max_source_step
                        ).astype(np.int64),
                    )
                    source_transition_steps = np.maximum(
                        source_transition_steps,
                        group_transition_steps,
                    )
                elif worst_step > max_source_step:
                    raise ValueError(
                        "unsafe source-step delta: "
                        f"{joint_names[int(worst_j)]} "
                        f"step {int(worst_t)}->{int(worst_t) + 1}="
                        f"{worst_step:.6f} rad > {max_source_step:.6f} rad"
                    )

            group_max_delta = max_delta_by_key.get(
                action_key,
                max_delta_by_key.get(publish_key, max_delta),
            )
            warm_start_enabled = bool(warm_start_max_total_by_key)
            warm_start_max_total = warm_start_max_total_by_key.get(
                action_key,
                warm_start_max_total_by_key.get(publish_key),
            )
            tracking_bridge_max_total = state_bridge_max_total_by_key.get(
                action_key,
                state_bridge_max_total_by_key.get(publish_key),
            )
            if warm_start_enabled and warm_start_max_total is None:
                raise ValueError(
                    "warm-start total-delta limit missing for action_key="
                    f"{action_key}"
                )
            if warm_start_enabled and (
                max_source_step is None or group_max_delta is None
            ):
                raise ValueError(
                    "warm-start requires both source-step and first-action "
                    f"delta limits for action_key={action_key}"
                )
            if tracking_bridge_max_total is not None and (
                max_source_step is None or group_max_delta is None
            ):
                raise ValueError(
                    "state-tracking bridge requires both source-step and "
                    f"tracking delta limits for action_key={action_key}"
                )

            if (
                group_max_delta is not None
                or warm_start_enabled
                or tracking_bridge_max_total is not None
            ):
                state_group = f"follower_{publish_key}"
                current_raw, timestamp = self.get_joint_position_snapshot(
                    state_group
                )
                current = np.asarray(current_raw, dtype=np.float64).reshape(-1)
                if current.size != width:
                    raise ValueError(
                        f"current state unavailable for {state_group}: "
                        f"got {current.size}, expected {width}"
                    )
                if not np.isfinite(current).all():
                    raise ValueError(
                        f"current state contains non-finite values: {state_group}"
                    )
                if timestamp is None:
                    raise ValueError(f"current state timestamp unavailable: {state_group}")
                age_s = now - float(timestamp)
                if age_s < -0.1:
                    raise ValueError(
                        f"current state timestamp is in the future: {state_group}"
                    )
                if max_age is not None and age_s > max_age:
                    raise ValueError(
                        f"current state is stale for {state_group}: "
                        f"age={age_s:.3f}s, limit={max_age:.3f}s"
                    )
                deltas = np.abs(segment[0] - current)
                worst_idx = int(np.argmax(deltas))
                worst_delta = float(deltas[worst_idx])
                state_hard_limit = (
                    warm_start_max_total
                    if warm_start_enabled
                    else (
                        tracking_bridge_max_total
                        if tracking_bridge_max_total is not None
                        else group_max_delta
                    )
                )
                if state_hard_limit is not None:
                    post_step_state_checks.append(
                        (
                            offset - width,
                            offset,
                            current.copy(),
                            state_hard_limit,
                            action_key,
                            joint_names,
                        )
                    )
                if warm_start_enabled:
                    if worst_delta > warm_start_max_total:
                        raise ValueError(
                            "unsafe warm-start total delta: "
                            f"{joint_names[worst_idx]}={worst_delta:.6f} rad "
                            f"> {warm_start_max_total:.6f} rad "
                            f"(action_key={action_key})"
                        )
                    # A group already inside its own first-action limit needs
                    # no bridge.  This is important for calibrated hands:
                    # their measured encoder zero can be outside the command
                    # URDF range, while the verified first command is in
                    # range and intentionally has a wider 0.10-rad gate.
                    if worst_delta > group_max_delta:
                        per_step_limit = min(
                            max_source_step,
                            group_max_delta,
                        )
                        group_steps = max(
                            1,
                            int(math.ceil(worst_delta / per_step_limit)),
                        )
                        state_bridge_steps = max(
                            state_bridge_steps,
                            group_steps,
                        )
                        bridge_current = current.copy()
                        if limit_mode != "off":
                            clipped_current = np.clip(current, lower, upper)
                            clip_deltas = np.abs(clipped_current - current)
                            clip_worst_idx = int(np.argmax(clip_deltas))
                            clip_worst_delta = float(
                                clip_deltas[clip_worst_idx]
                            )
                            if clip_worst_delta > group_max_delta:
                                raise ValueError(
                                    "current state is too far outside command "
                                    "joint limits for a bounded warm-start: "
                                    f"{joint_names[clip_worst_idx]}="
                                    f"{clip_worst_delta:.6f} rad > "
                                    f"{group_max_delta:.6f} rad "
                                    f"(action_key={action_key})"
                                )
                            if clip_worst_delta > 0.0:
                                if limit_mode != "clamp":
                                    raise ValueError(
                                        "current state is outside command joint "
                                        "limits during warm-start: "
                                        f"{joint_names[clip_worst_idx]} by "
                                        f"{clip_worst_delta:.6f} rad "
                                        f"(action_key={action_key})"
                                    )
                                bridge_current = clipped_current
                                state_bridge_anchor_clamped = True
                                logger.warning(
                                    "Clamping warm-start state anchor: "
                                    "action_key=%s joint=%s violation=%.6f rad",
                                    action_key,
                                    joint_names[clip_worst_idx],
                                    clip_worst_delta,
                                )
                        state_bridge_slices.append(
                            (offset - width, offset, bridge_current)
                        )
                elif tracking_bridge_max_total is not None:
                    if worst_delta > tracking_bridge_max_total:
                        raise ValueError(
                            "unsafe state-tracking total delta: "
                            f"{joint_names[worst_idx]}={worst_delta:.6f} rad "
                            f"> {tracking_bridge_max_total:.6f} rad "
                            f"(action_key={action_key})"
                        )
                    if worst_delta > group_max_delta:
                        per_step_limit = min(
                            max_source_step,
                            group_max_delta,
                        )
                        group_steps = max(
                            1,
                            int(math.ceil(worst_delta / per_step_limit)),
                        )
                        state_bridge_steps = max(
                            state_bridge_steps,
                            group_steps,
                        )
                        state_bridge_slices.append(
                            (offset - width, offset, current.copy())
                        )
                elif worst_delta > group_max_delta:
                    raise ValueError(
                        "unsafe current-state to first-action delta: "
                        f"{joint_names[worst_idx]}={worst_delta:.6f} rad "
                        f"> {group_max_delta:.6f} rad "
                        f"(action_key={action_key})"
                    )

        if offset != values.shape[1]:
            raise ValueError(
                f"action layout consumed {offset} values, chunk has {values.shape[1]}"
            )
        unknown_tolerance_joints = sorted(
            set(tolerance_by_joint) - position_joint_names
        )
        if unknown_tolerance_joints:
            raise ValueError(
                "joint limit tolerance configured for unknown action joints: "
                + ", ".join(unknown_tolerance_joints)
            )
        if source_step_bridge_max_raw_by_key and len(safe) > 1:
            source_rows = [safe[0].copy()]
            for index, step_count in enumerate(source_transition_steps):
                start = safe[index]
                stop = safe[index + 1]
                for step in range(1, int(step_count) + 1):
                    alpha = step / float(step_count)
                    source_rows.append(start + alpha * (stop - start))
            source_safe = np.asarray(source_rows, dtype=np.float64)
            if len(source_safe) != len(safe):
                logger.info(
                    "Prepared bounded source-step bridge: source_steps=%d "
                    "prepared_steps=%d inserted_steps=%d",
                    len(safe),
                    len(source_safe),
                    len(source_safe) - len(safe),
                )
            safe = source_safe
        if warm_start_max_total_by_key or state_bridge_max_total_by_key:
            current_row = safe[0].copy()
            for start, stop, current in state_bridge_slices:
                current_row[start:stop] = current
            alphas = (
                np.arange(1, state_bridge_steps + 1, dtype=np.float64)
                / float(state_bridge_steps)
            )
            bridge = current_row + alphas[:, None] * (safe[0] - current_row)
            if state_bridge_max_total_by_key and previous_action is not None:
                # A lagging follower may be behind an already published goal.
                # Re-anchoring to its measured pose must not pull that goal
                # backwards on each threshold crossing. Hold the previous
                # command until the follower catches up, or advance toward the
                # desired command. Intentional model reversals remain valid.
                # The previous-command step gate and state hard caps below
                # still validate the actual first output.
                for start, stop, _ in state_bridge_slices:
                    previous = previous_action[start:stop]
                    desired = safe[0, start:stop]
                    bridge[:, start:stop] = np.clip(
                        bridge[:, start:stop],
                        np.minimum(previous, desired),
                        np.maximum(previous, desired),
                    )
            if state_bridge_anchor_clamped:
                # Emit the legal clipped anchor first.  This keeps the first
                # physical command within the same per-step bound even when a
                # calibrated encoder reports a small value beyond the URDF
                # command range.
                bridge = np.concatenate((current_row[None, :], bridge), axis=0)
            safe = np.concatenate((bridge, safe[1:]), axis=0)
            logger.info(
                "Prepared bounded real %s: bridge_steps=%d "
                "source_steps=%d prepared_steps=%d",
                (
                    "warm-start"
                    if warm_start_max_total_by_key
                    else "state-tracking bridge"
                ),
                state_bridge_steps,
                len(values),
                len(safe),
            )
        if previous_action is not None:
            limited_joints = 0
            for start, stop in position_slices:
                previous = previous_action[start:stop]
                desired = safe[0, start:stop]
                bounded = previous + np.clip(
                    desired - previous,
                    -max_source_step,
                    max_source_step,
                )
                limited_joints += int(
                    np.count_nonzero(np.abs(bounded - desired) > 1e-12)
                )
                safe[0, start:stop] = bounded
            for (
                start,
                stop,
                current,
                hard_limit,
                action_key,
                joint_names,
            ) in post_step_state_checks:
                deltas = np.abs(safe[0, start:stop] - current)
                worst_idx = int(np.argmax(deltas))
                worst_delta = float(deltas[worst_idx])
                if worst_delta > hard_limit:
                    raise ValueError(
                        "previous-command step limiting cannot satisfy state "
                        "hard cap: "
                        f"{joint_names[worst_idx]}={worst_delta:.6f} rad "
                        f"> {hard_limit:.6f} rad "
                        f"(action_key={action_key})"
                    )
            if limited_joints:
                logger.info(
                    "Bounded %d position joints against previous published "
                    "action (max_step=%.6f rad)",
                    limited_joints,
                    max_source_step,
                )
        return safe

    def publish_action(self, action: np.ndarray, action_keys: Optional[list[str]] = None) -> None:
        """Publish one flat action vector to the robot command topics.

        Main process control loops use this method. Engine process instances keep
        ``enable_command_publishers=False`` and remain read-only.
        """
        if not self._command_publishers:
            raise RuntimeError("RobotClient command publishers are not enabled")

        keys = list(action_keys) if action_keys else self._action_keys
        values = np.asarray(action, dtype=np.float64).reshape(-1)
        offset = 0
        for action_key in keys:
            publish_key = self._resolve_action_key(action_key)
            cfg = self._action_groups.get(publish_key)
            if cfg is None:
                continue
            publisher_key = f"leader_{publish_key}"
            msg_type = cfg["msg_type"]
            width = 3 if msg_type == "geometry_msgs/msg/Twist" else len(cfg["joint_names"])
            segment = values[offset:offset + width]
            offset += width

            publisher = self._command_publishers.get(publisher_key)
            if publisher is None:
                continue
            if msg_type == "geometry_msgs/msg/Twist":
                self._publish_twist(publisher, segment)
            else:
                self._publish_joint_trajectory(
                    publisher,
                    self._command_joint_names.get(publisher_key, []),
                    segment,
                )

    def publish_idle_action(self, action_keys: Optional[list[str]] = None) -> None:
        """Publish safe idle commands for velocity-like action topics.

        Position trajectory controllers hold their last target when the action
        buffer is empty. Twist command topics do not have that same semantics,
        so publish an explicit zero velocity for any commanded Twist modality.
        """
        if not self._command_publishers:
            raise RuntimeError("RobotClient command publishers are not enabled")

        keys = list(action_keys) if action_keys else self._action_keys
        for action_key in keys:
            publish_key = self._resolve_action_key(action_key)
            cfg = self._action_groups.get(publish_key)
            if cfg is None or cfg["msg_type"] != "geometry_msgs/msg/Twist":
                continue
            publisher = self._command_publishers.get(f"leader_{publish_key}")
            if publisher is None:
                continue
            self._publish_twist(publisher, np.zeros(3, dtype=np.float64))

    def publish_initial_pose_sync(
        self,
        action: np.ndarray,
        action_keys: Optional[list[str]] = None,
        duration_s: float = 5.0,
    ) -> None:
        """Publish one predicted pose as a slow position-only transition."""
        duration_s = float(duration_s)
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("duration_s must be a positive finite value")

        segments = self._build_action_segments(action, action_keys)
        hold_segments = self._build_current_position_segments(action_keys)
        if not hold_segments:
            raise ValueError("initial pose sync requires a position action group")

        try:
            for publisher, _joint_names, _values, msg_type in segments:
                if msg_type == "geometry_msgs/msg/Twist":
                    self._publish_twist(publisher, np.zeros(3, dtype=np.float64))
            for publisher, joint_names, values, msg_type in segments:
                if msg_type != "geometry_msgs/msg/Twist":
                    self._publish_joint_trajectory(
                        publisher,
                        joint_names,
                        values,
                        time_from_start_s=duration_s,
                    )
        except Exception:
            try:
                self._publish_position_segments(hold_segments, duration_s=0.1)
                self.publish_idle_action(action_keys)
            except Exception as hold_error:
                logger.error(
                    "failed to hold current pose after initial sync publish error: %s",
                    hold_error,
                )
            raise

    def publish_current_pose_hold(
        self,
        action_keys: Optional[list[str]] = None,
        duration_s: float = 0.1,
    ) -> None:
        """Replace active position trajectories with the latest joint pose."""
        duration_s = float(duration_s)
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("duration_s must be a positive finite value")
        segments = self._build_current_position_segments(action_keys)
        if not segments:
            raise ValueError("current pose hold requires a position action group")
        self.publish_idle_action(action_keys)
        self._publish_position_segments(segments, duration_s)

    def _build_action_segments(
        self,
        action: np.ndarray,
        action_keys: Optional[list[str]],
    ) -> list[tuple[ROS2Publisher, list[str], np.ndarray, str]]:
        if not self._command_publishers:
            raise RuntimeError("RobotClient command publishers are not enabled")

        keys = list(action_keys) if action_keys else self._action_keys
        values = np.asarray(action, dtype=np.float64).reshape(-1)
        if not np.all(np.isfinite(values)):
            raise ValueError("action contains non-finite values")

        segments = []
        offset = 0
        resolved_keys: set[str] = set()
        for action_key in keys:
            publish_key = self._resolve_action_key(action_key)
            if publish_key in resolved_keys:
                raise ValueError(f"duplicate action key: {action_key}")
            resolved_keys.add(publish_key)
            cfg = self._action_groups.get(publish_key)
            if cfg is None:
                raise ValueError(f"unknown action key: {action_key}")
            publisher_key = f"leader_{publish_key}"
            publisher = self._command_publishers.get(publisher_key)
            if publisher is None:
                raise RuntimeError(
                    f"publisher unavailable for action key: {action_key}"
                )
            msg_type = cfg["msg_type"]
            joint_names = list(self._command_joint_names.get(publisher_key, []))
            width = 3 if msg_type == "geometry_msgs/msg/Twist" else len(joint_names)
            if width <= 0:
                raise ValueError(
                    f"action key has no configured dimensions: {action_key}"
                )
            segment = values[offset:offset + width]
            if len(segment) != width:
                raise ValueError(
                    f"action dimension too small for {action_key}: "
                    f"expected {width}, got {len(segment)}"
                )
            offset += width
            segments.append((publisher, joint_names, segment.copy(), msg_type))

        if offset != len(values):
            raise ValueError(
                f"action dimension mismatch: expected {offset}, got {len(values)}"
            )
        return segments

    def _build_current_position_segments(
        self,
        action_keys: Optional[list[str]],
    ) -> list[tuple[ROS2Publisher, list[str], np.ndarray]]:
        keys = list(action_keys) if action_keys else self._action_keys
        planned = []
        missing = []
        stale = []
        now = time.monotonic()
        with self._lock:
            positions_by_name = dict(self._joint_positions_by_name)
            timestamps_by_name = dict(self._joint_position_timestamps_by_name)
            max_age_s = self._initial_pose_sync_state_max_age_s
        for action_key in keys:
            publish_key = self._resolve_action_key(action_key)
            cfg = self._action_groups.get(publish_key)
            if cfg is None:
                raise ValueError(f"unknown action key: {action_key}")
            if cfg["msg_type"] == "geometry_msgs/msg/Twist":
                continue
            publisher_key = f"leader_{publish_key}"
            publisher = self._command_publishers.get(publisher_key)
            if publisher is None:
                raise RuntimeError(
                    f"publisher unavailable for action key: {action_key}"
                )
            joint_names = list(self._command_joint_names.get(publisher_key, []))
            group_missing = [
                name
                for name in joint_names
                if name not in positions_by_name or name not in timestamps_by_name
            ]
            missing.extend(group_missing)
            if group_missing:
                continue
            group_stale = [
                (name, max(0.0, now - timestamps_by_name[name]))
                for name in joint_names
                if max(0.0, now - timestamps_by_name[name]) > max_age_s
            ]
            stale.extend(group_stale)
            if group_stale:
                continue
            planned.append((
                publisher,
                joint_names,
                np.asarray(
                    [positions_by_name[name] for name in joint_names],
                    dtype=np.float64,
                ),
            ))
        if missing:
            raise RuntimeError(
                "current joint state unavailable: "
                + ", ".join(sorted(set(missing)))
            )
        if stale:
            stale_by_name = {name: age for name, age in stale}
            details = ", ".join(
                f"{name}={age:.3f}s"
                for name, age in sorted(stale_by_name.items())
            )
            raise RuntimeError(
                f"current joint state stale: {details} (max {max_age_s:.3f}s)"
            )
        return planned

    def _publish_position_segments(self, segments, duration_s: float) -> None:
        for publisher, joint_names, values in segments:
            self._publish_joint_trajectory(
                publisher,
                joint_names,
                values,
                time_from_start_s=duration_s,
            )

    def build_action_preview(
        self,
        action: np.ndarray,
        action_keys: Optional[list[str]] = None,
    ) -> tuple[list[str], np.ndarray]:
        """Build a single joint trajectory point for preview-only consumers."""
        keys = list(action_keys) if action_keys else self._action_keys
        values = np.asarray(action, dtype=np.float64).reshape(-1)
        joint_names: list[str] = []
        positions: list[float] = []
        offset = 0
        for action_key in keys:
            publish_key = self._resolve_action_key(action_key)
            cfg = self._action_groups.get(publish_key)
            if cfg is None:
                continue
            msg_type = cfg["msg_type"]
            width = 3 if msg_type == "geometry_msgs/msg/Twist" else len(cfg["joint_names"])
            segment = values[offset:offset + width]
            offset += width
            if msg_type == "geometry_msgs/msg/Twist":
                continue
            names = list(cfg.get("joint_names", []))
            joint_names.extend(names)
            positions.extend(float(v) for v in segment[:len(names)])
        return joint_names, np.asarray(positions, dtype=np.float64)

    def publish_action_preview(
        self,
        action: np.ndarray,
        action_keys: Optional[list[str]] = None,
    ) -> None:
        """Publish preview-only action data for the 3D viewer."""
        if self._preview_publisher is None:
            return
        joint_names, positions = self.build_action_preview(action, action_keys)
        if not joint_names or len(joint_names) != len(positions):
            return
        self._publish_joint_trajectory(self._preview_publisher, joint_names, positions)

    def _resolve_action_key(self, action_key: str) -> str:
        if action_key in self._action_groups:
            return action_key
        if action_key == "odometry" and "mobile" in self._action_groups:
            return "mobile"
        return action_key

    def _publish_twist(self, publisher: ROS2Publisher, values: np.ndarray) -> None:
        Vector3 = get_message_class("geometry_msgs/msg/Vector3")
        linear_x = float(values[0]) if len(values) > 0 else 0.0
        linear_y = float(values[1]) if len(values) > 1 else 0.0
        angular_z = float(values[2]) if len(values) > 2 else 0.0
        filtered_linear_x = _deadband(linear_x, self._cmd_vel_linear_deadband)
        filtered_linear_y = _deadband(linear_y, self._cmd_vel_linear_deadband)
        filtered_angular_z = _deadband(angular_z, self._cmd_vel_angular_deadband)

        linear = Vector3(
            x=filtered_linear_x,
            y=filtered_linear_y,
            z=0.0,
        )
        angular = Vector3(
            x=0.0,
            y=0.0,
            z=filtered_angular_z,
        )
        publisher.publish(linear=linear, angular=angular)

    def _publish_joint_trajectory(
        self,
        publisher: ROS2Publisher,
        joint_names: list[str],
        values: np.ndarray,
        time_from_start_s: float = 0.0,
    ) -> None:
        Header = get_message_class("std_msgs/msg/Header")
        Time = get_message_class("builtin_interfaces/msg/Time")
        Duration = get_message_class("builtin_interfaces/msg/Duration")
        JointTrajectoryPoint = get_message_class(
            "trajectory_msgs/msg/JointTrajectoryPoint"
        )
        duration_s = max(0.0, float(time_from_start_s))
        duration_sec = int(duration_s)
        duration_nanosec = int(round((duration_s - duration_sec) * 1e9))
        if duration_nanosec >= 1_000_000_000:
            duration_sec += 1
            duration_nanosec -= 1_000_000_000
        point = JointTrajectoryPoint(
            positions=np.asarray(values, dtype=np.float64),
            velocities=np.zeros(0, dtype=np.float64),
            accelerations=np.zeros(0, dtype=np.float64),
            effort=np.zeros(0, dtype=np.float64),
            time_from_start=Duration(
                sec=duration_sec,
                nanosec=duration_nanosec,
            ),
        )
        publisher.publish(
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            joint_names=list(joint_names),
            points=[point],
        )

    # ------------------------------------------------------------------ #
    # Task instruction
    # ------------------------------------------------------------------ #

    def set_task_instruction(self, instruction: str):
        self._task_instruction = instruction

    @property
    def task_instruction(self) -> str:
        return self._task_instruction

    # ------------------------------------------------------------------ #
    # Observation
    # ------------------------------------------------------------------ #

    def get_observation(
        self,
        resize: Optional[tuple[int, int]] = None,
        format: str = "bgr",
    ) -> Optional[dict]:
        """Get full observation for inference.

        Returns:
            Dict with images, joint_positions, task_instruction.
            None if sync_check is enabled and data is out of sync.
        """
        if self._sync_check and not self._check_sync():
            return None
        return {
            "images": self.get_images(resize=resize, format=format),
            "joint_positions": self.get_joint_positions(),
            "task_instruction": self._task_instruction,
        }

    def _check_sync(self) -> bool:
        """Check if image and joint timestamps are within threshold."""
        threshold_s = self._sync_threshold_ms / 1000.0
        with self._lock:
            if not self._image_timestamps or not self._joint_timestamps:
                return False
            img_times = list(self._image_timestamps.values())
            jnt_times = list(self._joint_timestamps.values())

        latest_img = max(img_times) if img_times else 0
        latest_jnt = max(jnt_times) if jnt_times else 0
        return abs(latest_img - latest_jnt) < threshold_s

    # ------------------------------------------------------------------ #
    # Readiness / waiting
    # ------------------------------------------------------------------ #

    def wait_for_ready(self, timeout: float = 10.0) -> bool:
        """Wait until at least one frame from all sensors is received."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._all_ready():
                logger.info("All sensors ready")
                return True
            time.sleep(0.1)
        # Log what's missing
        missing = self._get_missing()
        logger.warning(f"Timeout waiting for sensors. Missing: {missing}")
        return False

    def wait_for_image(self, camera_name: str, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_image_ready(camera_name):
                return True
            time.sleep(0.1)
        return False

    def wait_for_joint(self, group_name: str, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_joint_ready(group_name):
                return True
            time.sleep(0.1)
        return False

    def _all_ready(self) -> bool:
        with self._lock:
            for cam in self._config.get("cameras", {}):
                if cam not in self._images:
                    return False
            for group in self._config.get("joint_groups", {}):
                if group not in self._joint_positions:
                    return False
            return True

    def _get_missing(self) -> list[str]:
        missing = []
        with self._lock:
            for cam in self._config.get("cameras", {}):
                if cam not in self._images:
                    missing.append(f"camera:{cam}")
            for group in self._config.get("joint_groups", {}):
                if group not in self._joint_positions:
                    missing.append(f"joint:{group}")
        return missing

    # ------------------------------------------------------------------ #
    # Info / diagnostics
    # ------------------------------------------------------------------ #

    def get_status(self) -> dict:
        """Get current status of all subscriptions."""
        with self._lock:
            return {
                "robot_type": self._robot_type,
                "cameras": {
                    name: {
                        "ready": name in self._images,
                        "shape": self._images[name].shape if name in self._images else None,
                        "timestamp": self._image_timestamps.get(name),
                    }
                    for name in self._config.get("cameras", {})
                },
                "joint_groups": {
                    name: {
                        "ready": name in self._joint_positions,
                        "dof": len(self._joint_positions[name]) if name in self._joint_positions else 0,
                        "timestamp": self._joint_timestamps.get(name),
                    }
                    for name in self._config.get("joint_groups", {})
                },
                "sensors": {
                    name: {
                        "ready": name in self._sensors,
                        "timestamp": self._sensor_timestamps.get(name),
                    }
                    for name in self._config.get("sensors", {})
                },
            }

    # ------------------------------------------------------------------ #
    # Cleanup
    # ------------------------------------------------------------------ #

    def close(self):
        """Close all subscriptions and command publishers."""
        if hasattr(self, '_closed') and self._closed:
            return
        self._closed = True
        for sub in self._subscribers:
            try:
                sub.close()
            except Exception as e:
                logger.debug(f"Error closing subscriber: {e}")
        self._subscribers.clear()
        for pub in self._command_publishers.values():
            try:
                pub.close()
            except Exception as e:
                logger.debug(f"Error closing command publisher: {e}")
        self._command_publishers.clear()
        if self._preview_publisher is not None:
            try:
                self._preview_publisher.close()
            except Exception as e:
                logger.debug(f"Error closing preview publisher: {e}")
            self._preview_publisher = None
        logger.info("RobotClient closed")

    def __del__(self):
        self.close()
