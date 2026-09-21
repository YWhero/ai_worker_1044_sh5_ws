#!/usr/bin/env python3

from __future__ import annotations

import os
import unittest
from unittest.mock import Mock, patch

from orchestrator.internal.communication.container_service_client import (
    ContainerServiceClient,
    ServiceResponse,
)


def _client_with_mock_call_service():
    client = ContainerServiceClient(node=None, service_prefix="/lerobot")
    client._call_service = Mock(return_value=ServiceResponse(
        success=True,
        message="ok",
        data={},
        request_id="",
    ))
    return client


class InferenceCommandTimeoutTests(unittest.TestCase):
    def test_start_and_resume_use_dedicated_20_second_timeout(self):
        client = _client_with_mock_call_service()

        client.inference_command(ContainerServiceClient.CMD_START)
        self.assertEqual(
            client._call_service.call_args.kwargs["timeout_sec"],
            20.0,
        )
        self.assertEqual(
            client._call_service.call_args.kwargs["availability_timeout_sec"],
            5.0,
        )

        client.inference_command(ContainerServiceClient.CMD_RESUME)
        self.assertEqual(
            client._call_service.call_args.kwargs["timeout_sec"],
            20.0,
        )
        self.assertEqual(
            client._call_service.call_args.kwargs["availability_timeout_sec"],
            5.0,
        )

        client.inference_command(ContainerServiceClient.CMD_RESET_CYCLE)
        self.assertEqual(
            client._call_service.call_args.kwargs["timeout_sec"],
            20.0,
        )
        self.assertEqual(
            client._call_service.call_args.kwargs["availability_timeout_sec"],
            5.0,
        )

    def test_non_lifecycle_command_keeps_10_second_timeout(self):
        client = _client_with_mock_call_service()

        client.inference_command(ContainerServiceClient.CMD_PAUSE)

        self.assertEqual(
            client._call_service.call_args.kwargs["timeout_sec"],
            10.0,
        )
        self.assertEqual(
            client._call_service.call_args.kwargs["availability_timeout_sec"],
            10.0,
        )

    def test_start_resume_timeout_can_be_overridden_by_environment(self):
        with patch.dict(
            os.environ,
            {"INFERENCE_START_RESUME_TIMEOUT_SEC": "24.5"},
        ):
            client = _client_with_mock_call_service()

        client.inference_command(ContainerServiceClient.CMD_RESUME)

        self.assertEqual(
            client._call_service.call_args.kwargs["timeout_sec"],
            24.5,
        )

    def test_per_call_timeout_still_has_highest_priority(self):
        client = _client_with_mock_call_service()

        client.inference_command(
            ContainerServiceClient.CMD_START,
            timeout_sec=27.0,
        )

        self.assertEqual(
            client._call_service.call_args.kwargs["timeout_sec"],
            27.0,
        )


if __name__ == "__main__":
    unittest.main()
