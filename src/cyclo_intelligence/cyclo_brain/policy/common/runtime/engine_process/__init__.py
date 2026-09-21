"""Engine process runtime.

The Engine process owns policy dependencies, RobotClient sensor reads, and one
synchronous inference call. The Main process talks to it through the internal
EngineCommand service.
"""

from .protocol import (
    CMD_GET_ACTION,
    CMD_LOAD_POLICY,
    CMD_RESET_POLICY,
    CMD_SWITCH_POLICY,
    CMD_PRELOAD_POLICY,
    CMD_CLEAR_PRELOAD,
    CMD_SWITCH_PRELOADED,
    CMD_UNLOAD_POLICY,
    EngineCommandRequest,
    EngineCommandResponse,
)
from .worker import EngineWorker

__all__ = [
    "CMD_GET_ACTION",
    "CMD_LOAD_POLICY",
    "CMD_RESET_POLICY",
    "CMD_SWITCH_POLICY",
    "CMD_PRELOAD_POLICY",
    "CMD_CLEAR_PRELOAD",
    "CMD_SWITCH_PRELOADED",
    "CMD_UNLOAD_POLICY",
    "EngineCommandRequest",
    "EngineCommandResponse",
    "EngineWorker",
]
