"""Cycle Home must confirm the matching physical return before Start."""
import json
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from orchestrator.internal.communication.communicator import Communicator


class CycleHomeStatusTests(unittest.TestCase):
    def setUp(self):
        self.communicator = object.__new__(Communicator)
        self.communicator._initial_pose_lock = threading.Lock()
        self.communicator._initial_pose_request_id = None
        self.communicator._initial_pose_state = 'idle'
        self.communicator._initial_pose_message = ''
        self.communicator._initial_pose_requested_at = 0.0
        self.communicator.initial_pose_trigger_publisher = Mock()

    def status(self, request_id, state):
        self.communicator._initial_pose_status_callback(SimpleNamespace(data=json.dumps({
            'request_id': request_id, 'state': state, 'message': 'test return',
        })))

    def test_start_requires_matching_home_completion(self):
        self.assertTrue(self.communicator.publish_initial_pose_return()[0])
        request_id = self.communicator._initial_pose_request_id
        published = self.communicator.initial_pose_trigger_publisher.publish.call_args.args[0]
        self.assertEqual(published.data, f'both_sticks_up:{request_id}')
        self.status('previous-home', 'succeeded')
        self.assertFalse(self.communicator.initial_pose_return_ready()[0])
        self.status(request_id, 'running')
        self.assertFalse(self.communicator.initial_pose_return_ready()[0])
        self.status(request_id, 'succeeded')
        self.assertTrue(self.communicator.initial_pose_return_ready()[0])

    def test_second_home_cannot_reuse_first_home_success(self):
        self.communicator.publish_initial_pose_return()
        old = self.communicator._initial_pose_request_id
        self.status(old, 'succeeded')
        self.communicator.publish_initial_pose_return()
        self.assertNotEqual(old, self.communicator._initial_pose_request_id)
        self.status(old, 'succeeded')
        self.assertFalse(self.communicator.initial_pose_return_ready()[0])

    def test_failed_or_unacknowledged_home_blocks_start(self):
        self.communicator.publish_initial_pose_return()
        request_id = self.communicator._initial_pose_request_id
        self.status(request_id, 'failed')
        self.status(request_id, 'succeeded')  # terminal failure cannot be undone
        self.assertFalse(self.communicator.initial_pose_return_ready()[0])
        self.communicator.publish_initial_pose_return()
        self.communicator._initial_pose_requested_at = time.monotonic() - 101
        self.assertIn('not confirmed', self.communicator.initial_pose_return_ready()[1])

    def test_malformed_status_does_not_enable_start(self):
        self.communicator.publish_initial_pose_return()
        for data in ('invalid', '[]', 'null', '{}'):
            self.communicator._initial_pose_status_callback(SimpleNamespace(data=data))
        self.assertFalse(self.communicator.initial_pose_return_ready()[0])


if __name__ == '__main__':
    unittest.main()
