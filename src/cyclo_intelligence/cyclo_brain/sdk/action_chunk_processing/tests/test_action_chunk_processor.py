#!/usr/bin/env python3

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np


SDK_ROOT = Path(__file__).resolve().parents[1]
if str(SDK_ROOT) not in sys.path:
    sys.path.insert(0, str(SDK_ROOT))

from action_chunk_processing import ActionChunkProcessor  # noqa: E402


class ActionChunkProcessorTests(unittest.TestCase):
    def test_dynamic_resampling_preserves_legacy_16_step_timing(self) -> None:
        processor = ActionChunkProcessor(
            inference_hz=15.0,
            control_hz=100.0,
            alignment_mode="none",
        )
        chunk = np.arange(16, dtype=np.float64).reshape(16, 1)

        produced = processor.push_actions(chunk)

        self.assertEqual(produced, 100)
        first = processor.pop_action()
        last = None
        for _ in range(produced - 1):
            last = processor.pop_action()

        np.testing.assert_allclose(first, np.asarray([0.0]))
        np.testing.assert_allclose(last, np.asarray([14.85]))
        self.assertIsNone(processor.pop_action())

    def test_dynamic_resampling_scales_with_chunk_length(self) -> None:
        for source_count, expected_count in ((32, 207), (100, 660)):
            with self.subTest(source_count=source_count):
                processor = ActionChunkProcessor(
                    inference_hz=15.0,
                    control_hz=100.0,
                    alignment_mode="none",
                )
                chunk = np.zeros((source_count, 2), dtype=np.float64)

                produced = processor.push_actions(chunk)

                self.assertEqual(produced, expected_count)

    def test_dynamic_resampling_scales_with_inference_hz(self) -> None:
        chunk = np.zeros((16, 2), dtype=np.float64)

        slow_source = ActionChunkProcessor(
            inference_hz=10.0,
            control_hz=100.0,
            alignment_mode="none",
        )
        fast_source = ActionChunkProcessor(
            inference_hz=20.0,
            control_hz=100.0,
            alignment_mode="none",
        )

        self.assertEqual(slow_source.push_actions(chunk), 150)
        self.assertEqual(fast_source.push_actions(chunk), 75)

    def test_fixed_target_chunk_size_remains_available(self) -> None:
        processor = ActionChunkProcessor(
            inference_hz=15.0,
            control_hz=100.0,
            target_chunk_size=100,
            alignment_mode="none",
        )
        chunk = np.zeros((32, 2), dtype=np.float64)

        produced = processor.push_actions(chunk)

        self.assertEqual(produced, 100)

    def test_source_limit_applies_after_initial_latency_alignment(self) -> None:
        processor = ActionChunkProcessor(
            inference_hz=10.0,
            control_hz=10.0,
            source_chunk_limit=3,
        )
        chunk = np.arange(100, dtype=np.float64).reshape(100, 1)

        produced = processor.push_actions(
            chunk,
            scheduled_start_delay_s=0.5,
        )
        outputs = np.asarray(
            [processor.pop_action() for _ in range(produced)]
        ).reshape(-1)

        self.assertEqual(produced, 2)
        np.testing.assert_allclose(outputs, [5.0, 6.0])

    def test_rejects_non_positive_source_chunk_limit(self) -> None:
        with self.assertRaisesRegex(ValueError, "source_chunk_limit"):
            ActionChunkProcessor(source_chunk_limit=0)

    def test_source_limit_and_latency_alignment_survive_without_resampling(
        self,
    ) -> None:
        processor = ActionChunkProcessor(
            inference_hz=10.0,
            control_hz=100.0,
            postprocess=False,
            source_chunk_limit=3,
        )

        produced = processor.push_actions(
            np.arange(100, dtype=np.float64).reshape(100, 1),
            scheduled_start_delay_s=0.5,
        )
        outputs = np.asarray(
            [processor.pop_action() for _ in range(produced)]
        ).reshape(-1)

        self.assertEqual(produced, 3)
        np.testing.assert_allclose(outputs, [5.0, 6.0, 7.0])

    def test_empty_buffer_does_not_repeat_last_action(self) -> None:
        processor = ActionChunkProcessor(
            inference_hz=1.0,
            control_hz=1.0,
            postprocess=False,
        )
        processor.push_actions(np.asarray([[1.0, 2.0]], dtype=np.float64))

        first = processor.pop_action()
        second = processor.pop_action()

        np.testing.assert_allclose(first, np.asarray([1.0, 2.0]))
        self.assertIsNone(second)

    def test_deferred_action_returns_to_front_and_repairs_anchor(self) -> None:
        processor = ActionChunkProcessor(
            inference_hz=1.0,
            control_hz=1.0,
            postprocess=False,
        )
        processor.push_actions(
            np.asarray([[1.0, 2.0], [3.0, 4.0]], dtype=np.float64)
        )

        desired = processor.pop_action()
        processor.defer_action(
            desired,
            published_action=np.asarray([0.1, 0.2], dtype=np.float64),
        )

        np.testing.assert_allclose(
            processor._last_output_action,
            np.asarray([0.1, 0.2]),
        )
        np.testing.assert_allclose(
            processor.pop_action(),
            np.asarray([1.0, 2.0]),
        )
        np.testing.assert_allclose(
            processor.pop_action(),
            np.asarray([3.0, 4.0]),
        )

    def test_async_chunk_alignment_uses_scheduled_start_delay(self) -> None:
        processor = ActionChunkProcessor(
            inference_hz=10.0,
            control_hz=10.0,
            chunk_align_window_s=0.3,
        )
        processor.push_actions(np.arange(11, dtype=np.float64).reshape(11, 1))

        for _ in range(7):
            processor.pop_action()

        produced = processor.push_actions(
            np.arange(5, 16, dtype=np.float64).reshape(11, 1),
            scheduled_start_delay_s=0.4,
        )

        self.assertGreater(produced, 0)
        for _ in range(3):
            last_old_action = processor.pop_action()
        first_new_action = processor.pop_action()

        np.testing.assert_allclose(last_old_action, np.asarray([9.0]))
        self.assertGreater(first_new_action[0], last_old_action[0])

    def test_sync_chunk_can_skip_alignment(self) -> None:
        processor = ActionChunkProcessor(
            inference_hz=10.0,
            control_hz=10.0,
            chunk_align_window_s=0.3,
        )
        processor._last_output_action = np.asarray([20.0])

        produced = processor.push_actions(
            np.arange(20, 31, dtype=np.float64).reshape(11, 1),
            align=False,
        )
        first_new_action = processor.pop_action()

        self.assertEqual(produced, 10)
        np.testing.assert_allclose(first_new_action, np.asarray([20.0]))

    def test_step_sync_compatibility_smooths_one_duplicated_action(self) -> None:
        processor = ActionChunkProcessor(
            inference_hz=30.0,
            control_hz=100.0,
            sequential=False,
        )
        processor._last_output_action = np.asarray([0.0])

        produced = processor.push_actions(
            np.asarray([[0.04], [0.04]], dtype=np.float64),
            align=False,
        )
        outputs = np.asarray(
            [processor.pop_action() for _ in range(produced)]
        ).reshape(-1)

        # This intentionally reproduces the earlier smooth physical profile:
        # one 30 Hz source interval, spread over three 100 Hz ticks, with the
        # generic transition blend damping the new target to 25/50/75%.
        self.assertEqual(produced, 3)
        np.testing.assert_allclose(outputs, [0.01, 0.02, 0.03])

    def test_sequential_chunk_executes_each_source_action_for_one_period(self) -> None:
        processor = ActionChunkProcessor(
            inference_hz=30.0,
            control_hz=100.0,
            sequential=True,
        )
        processor._last_output_action = np.asarray([-3.0])

        produced = processor.push_actions(
            np.asarray([[0.0], [1.0], [2.0], [3.0]], dtype=np.float64),
            align=False,
        )
        outputs = np.asarray(
            [processor.pop_action() for _ in range(produced)]
        ).reshape(-1)

        self.assertEqual(produced, 13)
        # Only one 30 Hz source period joins the boundary.  The third 100 Hz
        # row is already the unmodified interpolated trajectory, and the full
        # four-action horizon remains reachable.
        self.assertGreater(outputs[0], -3.0)
        self.assertLess(outputs[0], 0.0)
        np.testing.assert_allclose(outputs[2], 0.6)
        self.assertGreater(outputs[-1], 2.99)
        self.assertIsNone(processor.pop_action())

    def test_sequential_boundary_join_reduces_replan_jump(self) -> None:
        processor = ActionChunkProcessor(
            inference_hz=30.0,
            control_hz=100.0,
            sequential=True,
        )
        processor._last_output_action = np.asarray([0.0])

        produced = processor.push_actions(
            np.asarray([[0.048], [0.048], [0.048], [0.048]]),
            align=False,
        )
        outputs = np.asarray(
            [processor.pop_action() for _ in range(produced)]
        ).reshape(-1)
        steps = np.diff(np.concatenate(([0.0], outputs)))

        self.assertEqual(produced, 13)
        self.assertLessEqual(np.max(np.abs(steps)), 0.0160001)
        np.testing.assert_allclose(outputs[2:], 0.048)

    def test_long_sequential_chunk_uses_full_transition_blend(self) -> None:
        processor = ActionChunkProcessor(
            inference_hz=30.0,
            control_hz=100.0,
            sequential=True,
        )
        processor._last_output_action = np.asarray([0.0])

        produced = processor.push_actions(
            np.ones((16, 1), dtype=np.float64),
            align=False,
        )
        outputs = np.asarray(
            [processor.pop_action() for _ in range(produced)]
        ).reshape(-1)

        self.assertEqual(produced, 53)
        np.testing.assert_allclose(outputs[0], 1.0 / 21.0)
        np.testing.assert_allclose(outputs[19], 20.0 / 21.0)
        np.testing.assert_allclose(outputs[20:], 1.0)

    def test_single_sequential_action_holds_one_source_period(self) -> None:
        processor = ActionChunkProcessor(
            inference_hz=30.0,
            control_hz=100.0,
            sequential=True,
        )

        produced = processor.push_actions(np.asarray([[7.0]]), align=False)
        outputs = [processor.pop_action() for _ in range(produced)]

        self.assertEqual(produced, 3)
        for output in outputs:
            np.testing.assert_allclose(output, np.asarray([7.0]))

    def test_late_async_chunk_falls_back_instead_of_dropping_all(self) -> None:
        processor = ActionChunkProcessor(
            inference_hz=15.0,
            control_hz=100.0,
            chunk_align_window_s=0.3,
        )
        processor.push_actions(np.arange(16, dtype=np.float64).reshape(16, 1))
        while processor.pop_action() is not None:
            pass

        produced = processor.push_actions(
            np.arange(16, 32, dtype=np.float64).reshape(16, 1),
            scheduled_start_delay_s=1.5,
        )

        self.assertGreater(produced, 0)

    def test_late_async_chunk_never_drops_all_when_window_covers_chunk(self) -> None:
        processor = ActionChunkProcessor(
            inference_hz=10.0,
            control_hz=10.0,
            chunk_align_window_s=10.0,
        )
        processor._last_output_action = np.asarray([99.0])

        produced = processor.push_actions(
            np.arange(5, dtype=np.float64).reshape(5, 1),
            scheduled_start_delay_s=99.0,
        )

        self.assertGreater(produced, 0)


if __name__ == "__main__":
    unittest.main()
