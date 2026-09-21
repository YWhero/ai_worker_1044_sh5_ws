"""Persistent joint-pose presets used by Action Canvas parameter editors."""

from __future__ import annotations

import json
import math
import os
import re
from pathlib import Path
from typing import Dict, List, Literal

from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel, Field


router = APIRouter(prefix="/bt/pose-presets", tags=["behavior-tree-pose-presets"])

POSE_PRESETS_DIR_ENV = "CYCLO_BT_POSE_PRESETS_DIR"
DEFAULT_POSE_PRESETS_DIR = "/workspace/bt/pose_presets"
_SAFE_NAME_RE = re.compile(r"^[\w-]+$")


def pose_presets_dir() -> Path:
    configured = os.environ.get(POSE_PRESETS_DIR_ENV, "").strip()
    return Path(configured or DEFAULT_POSE_PRESETS_DIR)


def normalize_component(value: str, label: str) -> str:
    candidate = str(value or "").strip()
    if not candidate:
        raise HTTPException(400, f"{label} is required")
    if not _SAFE_NAME_RE.fullmatch(candidate):
        raise HTTPException(400, f"Invalid {label}: {candidate!r}")
    return candidate


def robot_pose_dir(robot_type: str) -> Path:
    directory = pose_presets_dir() / normalize_component(robot_type, "robot_type")
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def preset_path(robot_type: str, name: str) -> Path:
    return robot_pose_dir(robot_type) / f"{normalize_component(name, 'name')}.json"


class JointPosePreset(BaseModel):
    schema_version: Literal["cyclo_joint_pose_v1"] = "cyclo_joint_pose_v1"
    name: str
    robot_type: str
    source_topic: str = "/joint_states"
    source_topics: List[str] = Field(default_factory=list)
    captured_at: str
    joints: Dict[str, float] = Field(default_factory=dict)


class JointPosePresetSaveRequest(JointPosePreset):
    overwrite: bool = False


class JointPosePresetListResponse(BaseModel):
    directory: str
    presets: List[JointPosePreset]


class JointPosePresetSaveResponse(BaseModel):
    ok: bool = True
    message: str
    name: str
    path: str


def validated_preset(request: JointPosePresetSaveRequest) -> JointPosePreset:
    name = normalize_component(request.name, "name")
    robot_type = normalize_component(request.robot_type, "robot_type")
    joints = {str(key).strip(): float(value) for key, value in request.joints.items()}
    if not joints:
        raise HTTPException(400, "At least one joint value is required")
    invalid = [key for key, value in joints.items() if not key or not math.isfinite(value)]
    if invalid:
        raise HTTPException(400, f"Invalid joint values: {', '.join(invalid)}")
    return JointPosePreset(
        name=name,
        robot_type=robot_type,
        source_topic=str(request.source_topic or "/joint_states").strip(),
        source_topics=[
            str(topic).strip()
            for topic in request.source_topics
            if str(topic).strip()
        ],
        captured_at=str(request.captured_at or "").strip(),
        joints=joints,
    )


def read_preset_file(path: Path) -> JointPosePreset:
    try:
        return JointPosePreset.model_validate_json(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise HTTPException(500, f"Invalid pose preset file: {path.name}") from exc


@router.get("", response_model=JointPosePresetListResponse)
async def list_pose_presets(
    robot_type: str = Query(..., min_length=1),
) -> JointPosePresetListResponse:
    directory = robot_pose_dir(robot_type)
    presets = [
        read_preset_file(path)
        for path in sorted(directory.glob("*.json"))
        if path.is_file()
    ]
    return JointPosePresetListResponse(directory=str(directory), presets=presets)


@router.post("", response_model=JointPosePresetSaveResponse)
async def save_pose_preset(
    request: JointPosePresetSaveRequest,
) -> JointPosePresetSaveResponse:
    preset = validated_preset(request)
    path = preset_path(preset.robot_type, preset.name)
    if path.exists() and not request.overwrite:
        raise HTTPException(
            409,
            {
                "code": "file_exists",
                "message": f"Pose preset already exists: {preset.name}",
                "name": preset.name,
                "path": str(path),
            },
        )

    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(preset.model_dump(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)
    return JointPosePresetSaveResponse(
        message=f"Saved pose preset: {preset.name}",
        name=preset.name,
        path=str(path),
    )


@router.delete("/{name}", response_model=JointPosePresetSaveResponse)
async def delete_pose_preset(
    name: str,
    robot_type: str = Query(..., min_length=1),
) -> JointPosePresetSaveResponse:
    path = preset_path(robot_type, name)
    if not path.is_file():
        raise HTTPException(404, f"Pose preset not found: {name}")
    path.unlink()
    return JointPosePresetSaveResponse(
        message=f"Deleted pose preset: {name}",
        name=normalize_component(name, "name"),
        path=str(path),
    )
