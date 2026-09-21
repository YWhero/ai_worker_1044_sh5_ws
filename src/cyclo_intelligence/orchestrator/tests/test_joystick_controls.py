#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


HELPER_PATH = (
    Path(__file__).resolve().parents[1]
    / 'orchestrator'
    / 'internal'
    / 'communication'
    / 'joystick_controls.py'
)

spec = importlib.util.spec_from_file_location('joystick_controls', HELPER_PATH)
joystick_controls = importlib.util.module_from_spec(spec)
spec.loader.exec_module(joystick_controls)


class JoystickRecordingControlsTests(unittest.TestCase):
    def test_right_long_press_toggles_recording(self) -> None:
        self.assertEqual(
            joystick_controls.get_joystick_recording_action('right_long_time'),
            joystick_controls.RECORD_TOGGLE_ACTION,
        )

    def test_left_long_press_cancels_recording(self) -> None:
        self.assertEqual(
            joystick_controls.get_joystick_recording_action('left_long_time'),
            joystick_controls.RECORD_CANCEL_ACTION,
        )

    def test_short_clicks_do_not_control_recording(self) -> None:
        for trigger in joystick_controls.ARM_TOGGLE_TRIGGERS:
            with self.subTest(trigger=trigger):
                self.assertIsNone(
                    joystick_controls.get_joystick_recording_action(trigger)
                )

    def test_unknown_or_empty_trigger_is_ignored(self) -> None:
        for trigger in ('', None, 'both'):
            with self.subTest(trigger=trigger):
                self.assertIsNone(
                    joystick_controls.get_joystick_recording_action(trigger)
                )


if __name__ == '__main__':
    unittest.main()
