import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from main_runtime.diagnostic_trace import DiagnosticTrace


class DiagnosticTraceTests(unittest.TestCase):
    def test_disabled_trace_does_not_start_a_writer(self):
        trace = DiagnosticTrace('')
        trace.record('publish', desired=[1.0])
        self.assertFalse(trace.enabled)
        self.assertIsNone(trace.path)
        self.assertTrue(trace._queue.empty())

    def test_trace_flushes_commands_and_defer_decisions_on_close(self):
        with tempfile.TemporaryDirectory() as directory:
            trace = DiagnosticTrace(directory)
            trace.record('publish', desired=[0.565], published=[0.559], deferred=True)
            trace.close()
            row = json.loads(trace.path.read_text())
            self.assertEqual(row['published'], [0.559])
            self.assertTrue(row['deferred'])
            self.assertEqual(row['dropped'], 0)

    def test_trace_size_limit_stops_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            trace = DiagnosticTrace(directory)
            trace.MAX_BYTES = 1
            trace.record('publish', published=[0.559])
            trace.close()
            self.assertEqual(json.loads(trace.path.read_text())['event'], 'trace_size_limit')

    def test_rejection_survives_capture_limit_and_replaces_single_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            trace = DiagnosticTrace(directory)
            trace.MAX_BYTES = 1
            trace.record('publish', published=[.5])
            trace.close()
            self.assertFalse(trace.enabled)
            trace.record_rejection(stage='raw_model', reason='first', raw=[[0],[.2]])
            trace.record_rejection(stage='post_ensemble_safety', reason='latest', raw=[[0]])
            files = list(Path(directory).glob('*-last_rejection.json'))
            self.assertEqual(len(files), 1)
            row = json.loads(files[0].read_text())
            self.assertEqual(row['reason'], 'latest')
            self.assertEqual(row['stage'], 'post_ensemble_safety')
