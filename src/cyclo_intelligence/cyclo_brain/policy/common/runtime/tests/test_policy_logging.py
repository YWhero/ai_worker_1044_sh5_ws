#!/usr/bin/env python3

from __future__ import annotations

import io
import logging
import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


RUNTIME_ROOT = Path(__file__).resolve().parents[1]
if str(RUNTIME_ROOT) not in sys.path:
    sys.path.insert(0, str(RUNTIME_ROOT))

from policy_logging import (  # noqa: E402
    POLICY_LOGGER_NAMES,
    configure_policy_runtime_logging,
)


class PolicyLoggingTests(unittest.TestCase):
    def setUp(self) -> None:
        self._logger_state = {}
        for name in POLICY_LOGGER_NAMES:
            logger = logging.getLogger(name)
            handlers = list(logger.handlers)
            self._logger_state[name] = {
                "level": logger.level,
                "propagate": logger.propagate,
                "disabled": logger.disabled,
                "handlers": handlers,
                "handler_levels": {handler: handler.level for handler in handlers},
            }
            for handler in handlers:
                logger.removeHandler(handler)
            logger.setLevel(logging.NOTSET)
            logger.propagate = True
            logger.disabled = False

    def tearDown(self) -> None:
        for name, state in self._logger_state.items():
            logger = logging.getLogger(name)
            for handler in list(logger.handlers):
                logger.removeHandler(handler)
            for handler in state["handlers"]:
                handler.setLevel(state["handler_levels"][handler])
                logger.addHandler(handler)
            logger.setLevel(state["level"])
            logger.propagate = state["propagate"]
            logger.disabled = state["disabled"]

    def test_default_configuration_is_info_and_idempotent(self) -> None:
        root_logger = logging.getLogger()
        root_state = (root_logger.level, list(root_logger.handlers))

        with patch.dict(os.environ, {}, clear=False):
            os.environ.pop("POLICY_LOG_LEVEL", None)
            first_level = configure_policy_runtime_logging()
            first_handlers = {
                name: list(logging.getLogger(name).handlers)
                for name in POLICY_LOGGER_NAMES
            }
            second_level = configure_policy_runtime_logging()

        self.assertEqual(first_level, logging.INFO)
        self.assertEqual(second_level, logging.INFO)
        for name in POLICY_LOGGER_NAMES:
            logger = logging.getLogger(name)
            self.assertEqual(logger.level, logging.INFO)
            self.assertFalse(logger.propagate)
            self.assertEqual(logger.handlers, first_handlers[name])
            self.assertEqual(len(logger.handlers), 1)
            self.assertEqual(logger.handlers[0].level, logging.INFO)
        self.assertEqual((root_logger.level, list(root_logger.handlers)), root_state)

    def test_environment_level_reuses_handlers_and_applies_to_children(self) -> None:
        streams = {}
        handlers = {}
        for name in POLICY_LOGGER_NAMES:
            stream = io.StringIO()
            handler = logging.StreamHandler(stream)
            handler.setLevel(logging.ERROR)
            logging.getLogger(name).addHandler(handler)
            streams[name] = stream
            handlers[name] = handler

        with patch.dict(os.environ, {"POLICY_LOG_LEVEL": "debug"}):
            configured_level = configure_policy_runtime_logging()

        self.assertEqual(configured_level, logging.DEBUG)
        for name in POLICY_LOGGER_NAMES:
            logger = logging.getLogger(name)
            self.assertEqual(logger.handlers, [handlers[name]])
            self.assertEqual(handlers[name].level, logging.DEBUG)
            logging.getLogger(f"{name}.lifecycle").debug("lifecycle-visible")
            self.assertIn("lifecycle-visible", streams[name].getvalue())

    def test_invalid_environment_level_is_rejected(self) -> None:
        with patch.dict(os.environ, {"POLICY_LOG_LEVEL": "verbose"}):
            with self.assertRaisesRegex(ValueError, "POLICY_LOG_LEVEL"):
                configure_policy_runtime_logging()


if __name__ == "__main__":
    unittest.main()
