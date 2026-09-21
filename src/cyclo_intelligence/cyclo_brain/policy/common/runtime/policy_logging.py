#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0

"""Scoped logging configuration for policy runtime processes.

The Zenoh SDK installs a ``WARNING``-level handler by default, while backend
engines use regular Python loggers.  Policy lifecycle diagnostics need both
namespaces at ``INFO`` without changing the root logger (and therefore every
library in the process).
"""

from __future__ import annotations

import logging
import os
import threading
from typing import Final


POLICY_LOG_LEVEL_ENV: Final = "POLICY_LOG_LEVEL"
POLICY_LOGGER_NAMES: Final = (
    "zenoh_ros2_sdk",
    "lerobot_engine",
    "vitacformer_engine",
)

_DEFAULT_LOG_LEVEL: Final = "INFO"
_LOG_FORMAT: Final = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
_LOG_DATE_FORMAT: Final = "%Y-%m-%d %H:%M:%S"
_CONFIGURE_LOCK = threading.Lock()


def _resolve_log_level(value: str | int) -> int:
    if isinstance(value, int):
        return value

    normalized = value.strip().upper()
    resolved = getattr(logging, normalized, None)
    if not isinstance(resolved, int):
        raise ValueError(
            f"{POLICY_LOG_LEVEL_ENV} must be a valid logging level; got {value!r}"
        )
    return resolved


def configure_policy_runtime_logging(level: str | int | None = None) -> int:
    """Configure policy lifecycle loggers and return the resolved level.

    Configuration is deliberately limited to :data:`POLICY_LOGGER_NAMES`.
    Existing direct handlers (including the SDK's default handler) are reused;
    a stream handler is installed only when a namespace has no emitting
    handler.  Repeated calls therefore do not accumulate handlers.

    Args:
        level: Explicit logging level.  When omitted, ``POLICY_LOG_LEVEL`` is
            read from the environment and defaults to ``INFO``.
    """

    configured_level = _resolve_log_level(
        os.environ.get(POLICY_LOG_LEVEL_ENV, _DEFAULT_LOG_LEVEL)
        if level is None
        else level
    )
    formatter = logging.Formatter(_LOG_FORMAT, datefmt=_LOG_DATE_FORMAT)

    with _CONFIGURE_LOCK:
        for logger_name in POLICY_LOGGER_NAMES:
            namespace_logger = logging.getLogger(logger_name)
            namespace_logger.setLevel(configured_level)
            namespace_logger.propagate = False

            emitting_handlers = [
                handler
                for handler in namespace_logger.handlers
                if not isinstance(handler, logging.NullHandler)
            ]
            if not emitting_handlers:
                handler = logging.StreamHandler()
                handler.setFormatter(formatter)
                namespace_logger.addHandler(handler)
                emitting_handlers = [handler]

            for handler in emitting_handlers:
                handler.setLevel(configured_level)

    return configured_level
