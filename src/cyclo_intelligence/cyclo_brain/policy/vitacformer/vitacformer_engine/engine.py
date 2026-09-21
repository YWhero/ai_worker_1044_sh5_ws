"""Dedicated ViTacFormer inference engine."""

from __future__ import annotations

import gc
import logging
import os
import sys
from typing import Any, Dict, Optional

import numpy as np


_ROBOT_CLIENT_PATH = os.environ.get("ROBOT_CLIENT_SDK_PATH", "/robot_client_sdk")
if os.path.exists(_ROBOT_CLIENT_PATH) and _ROBOT_CLIENT_PATH not in sys.path:
    sys.path.insert(0, _ROBOT_CLIENT_PATH)

from engine import InferenceEngine  # noqa: E402

import torch  # noqa: E402

from robot_client import RobotClient  # noqa: E402

from .io_mapping import IoMappingMixin  # noqa: E402
from .loading import LoadingMixin  # noqa: E402
from .model import ViTacFormerPolicy  # noqa: E402
from .prediction import PredictionMixin  # noqa: E402
from .preprocessing import PreprocessingMixin  # noqa: E402
from .constants import ACTION_KEYS, IMAGE_KEY, STATE_KEY, TACTILE_BATCH_KEY  # noqa: E402


logger = logging.getLogger("vitacformer_engine")


class ViTacFormerEngine(
    LoadingMixin,
    IoMappingMixin,
    PreprocessingMixin,
    PredictionMixin,
    InferenceEngine,
):
    def __init__(self) -> None:
        self._policy: Optional[ViTacFormerPolicy] = None
        self._robot: Optional[RobotClient] = None
        self._device: Optional[torch.device] = None
        self._loaded_model_path: Optional[str] = None
        self._loaded_robot_type: Optional[str] = None
        self._tactile_inputs: Dict[str, str] = {}
        self._tactile_baselines: Dict[str, np.ndarray] = {}
        self._action_keys: list[str] = []
        self._preloaded_policy: Optional[ViTacFormerPolicy] = None
        self._preloaded_model_path: Optional[str] = None
        self._preloaded_robot_type: Optional[str] = None

    @property
    def is_ready(self) -> bool:
        return self._policy is not None and self._robot is not None

    def load_policy(self, request: Any) -> Dict[str, Any]:
        if self.is_ready:
            return self._fail("ViTacFormer policy already loaded - UNLOAD first")
        model_path = str(getattr(request, "model_path", "") or "").strip()
        robot_type = str(getattr(request, "robot_type", "") or "").strip()
        try:
            self._device = torch.device(
                "cuda" if torch.cuda.is_available() else "cpu"
            )
            self._policy = self._load_policy_assets(model_path, self._device)
            self._loaded_model_path = model_path
            self._loaded_robot_type = robot_type
            self._policy.reset()
            self._init_robot(robot_type)
            return {
                "success": True,
                "message": f"loaded ViTacFormer {model_path}",
                "action_keys": list(self._action_keys),
            }
        except Exception as exc:
            logger.error("ViTacFormer load failed: %s", exc, exc_info=True)
            self.cleanup()
            return self._fail(str(exc))

    def get_action_chunk(self, request: Any) -> Dict[str, Any]:
        del request
        if not self.is_ready:
            return self._fail("ViTacFormer policy is not loaded")
        try:
            chunk = self._predict_chunk(self._build_observation())
            rows, action_dim = chunk.shape
            return {
                "success": True,
                "action_chunk": chunk.reshape(-1),
                "chunk_size": int(rows),
                "action_dim": int(action_dim),
            }
        except Exception as exc:
            logger.error("ViTacFormer inference failed: %s", exc, exc_info=True)
            return self._fail(str(exc))

    def reset_cycle(self) -> Dict[str, Any]:
        if self._policy is None or not self._loaded_robot_type:
            return self._fail("No loaded ViTacFormer policy to reset")
        try:
            # Cycle Home begins a new policy episode, including a new model
            # object. Do not carry any model-side caches across this boundary.
            candidate = self._load_policy_assets(self._loaded_model_path, self._device)
            self._teardown_robot()
            self._policy = candidate
            candidate.reset()
            self._init_robot(self._loaded_robot_type)
            logger.info("Fresh ViTacFormer cycle: reloaded policy and rebuilt observation histories/baselines")
            return {
                "success": True,
                "message": "Fresh ViTacFormer cycle; policy, histories and tactile baseline recreated",
                "action_keys": list(self._action_keys),
            }
        except Exception as exc:
            logger.error("ViTacFormer cycle reset failed: %s", exc, exc_info=True)
            self._teardown_robot()
            return self._fail(str(exc))

    def switch_policy(self, request: Any) -> Dict[str, Any]:
        """Replace weights and normalization, retaining the episode baseline.

        Loading the candidate must succeed before touching the active policy.
        Sensor histories keep advancing while command publishing is paused.
        In particular, do not zero tactile pressure while holding an object.
        """
        if not self.is_ready:
            return self._fail("No loaded ViTacFormer policy to switch")
        if str(getattr(request, "robot_type", "")) != self._loaded_robot_type:
            return self._fail("Cannot switch robot type within an episode")
        model_path = str(getattr(request, "model_path", "") or "").strip()
        if not model_path:
            return self._fail("model_path is required")
        try:
            # At most two sets of weights are retained. The explicit cached
            # switch below never enters this disk-loading path.
            self.clear_preload(request)
            candidate = self._load_policy_assets(model_path, self._device)
            candidate.reset()
            if list(candidate.config.model_action_keys) != self._action_keys:
                raise RuntimeError("Next model action layout differs from current model")
            self._policy = candidate
            self._loaded_model_path = model_path
            return {
                "success": True,
                "message": "ViTacFormer switched; episode tactile baseline retained",
                "action_keys": list(self._action_keys),
            }
        except Exception as exc:
            logger.error("ViTacFormer switch failed: %s", exc, exc_info=True)
            return self._fail(str(exc))

    def preload_policy(self, request: Any) -> Dict[str, Any]:
        """Prepare weights and kernels without creating a robot connection."""
        model_path = str(getattr(request, "model_path", "") or "").strip()
        robot_type = str(getattr(request, "robot_type", "") or "").strip()
        if robot_type != "ffw_sh5_rev1":
            return self._fail("Preloading requires SH5 ViTacFormer")
        if not model_path or model_path == self._loaded_model_path:
            return self._fail("Select a different next model path")
        if self._loaded_robot_type and robot_type != self._loaded_robot_type:
            return self._fail("Cannot change robot type within an episode")
        if (self._preloaded_policy is not None
                and model_path == self._preloaded_model_path
                and robot_type == self._preloaded_robot_type):
            return {"success": True, "message": "Next model is already preloaded and warmed up"}
        candidate = None
        try:
            self.clear_preload(request)
            device = self._device or torch.device("cuda" if torch.cuda.is_available() else "cpu")
            candidate = self._load_policy_assets(model_path, device)
            if list(candidate.config.model_action_keys) != list(ACTION_KEYS):
                raise RuntimeError("Next model action layout differs from SH5")
            candidate.reset()
            # Synthetic data only warms the identical prediction path. It is
            # never published and must not enter the episode's policy history.
            self._predict_policy_chunk(candidate, {
                IMAGE_KEY: torch.zeros((1, 3, 188, 336), device=device),
                STATE_KEY: torch.zeros((1, 6, 54), device=device),
                TACTILE_BATCH_KEY: torch.zeros((1, 18, 180), device=device),
            })
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            candidate.reset()
            self._preloaded_policy = candidate
            self._preloaded_model_path = model_path
            self._preloaded_robot_type = robot_type
            return {
                "success": True,
                "message": f"Next model preloaded on {device.type.upper()}; warmup complete",
                "action_keys": list(ACTION_KEYS),
            }
        except Exception as exc:
            logger.error("ViTacFormer preload failed: %s", exc, exc_info=True)
            candidate = None
            self.clear_preload(request)
            return self._fail(str(exc))

    def switch_preloaded_policy(self, request: Any) -> Dict[str, Any]:
        """Swap resident policies only; a stale cache never falls back to load."""
        if not self.is_ready:
            return self._fail("No loaded ViTacFormer policy to switch")
        model_path = str(getattr(request, "model_path", "") or "").strip()
        robot_type = str(getattr(request, "robot_type", "") or "")
        if (self._preloaded_policy is None or model_path != self._preloaded_model_path
                or robot_type != self._preloaded_robot_type
                or robot_type != self._loaded_robot_type):
            return self._fail("Next model is not preloaded; preload it again before switching")
        candidate = self._preloaded_policy
        if list(candidate.config.model_action_keys) != self._action_keys:
            return self._fail("Next model action layout differs from current model")
        candidate.reset()
        # Retain the previous policy as the spare: no GPU deallocation or disk
        # work on the handoff path. Clear/Release frees the spare separately.
        self._policy, self._preloaded_policy = candidate, self._policy
        self._loaded_model_path, self._preloaded_model_path = model_path, self._loaded_model_path
        return {
            "success": True,
            "message": "Preloaded ViTacFormer activated; episode tactile baseline retained",
            "action_keys": list(self._action_keys),
        }

    def clear_preload(self, request: Any = None) -> Dict[str, Any]:
        self._preloaded_policy = None
        self._preloaded_model_path = None
        self._preloaded_robot_type = None
        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        return {"success": True, "message": "Preloaded model released"}

    def cleanup(self) -> None:
        self._teardown_robot()
        had_policy = self._policy is not None or self._preloaded_policy is not None
        self._policy = None
        self._preloaded_policy = None
        self._preloaded_model_path = None
        self._preloaded_robot_type = None
        self._device = None
        self._loaded_model_path = None
        self._loaded_robot_type = None
        self._tactile_inputs = {}
        self._tactile_baselines = {}
        self._action_keys = []
        if had_policy:
            gc.collect()
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
                torch.cuda.ipc_collect()

    @staticmethod
    def _fail(message: str) -> Dict[str, Any]:
        return {"success": False, "message": message}


def create_engine() -> InferenceEngine:
    return ViTacFormerEngine()
