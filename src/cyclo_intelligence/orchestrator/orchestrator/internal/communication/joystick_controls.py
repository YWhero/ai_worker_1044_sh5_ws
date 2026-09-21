"""Leader joystick gestures that are allowed to control recording."""

RECORD_TOGGLE_ACTION = 'record_toggle'
RECORD_CANCEL_ACTION = 'record_cancel'

_RECORDING_ACTION_BY_TRIGGER = {
    'right_long_time': RECORD_TOGGLE_ACTION,
    'left_long_time': RECORD_CANCEL_ACTION,
}

ARM_TOGGLE_TRIGGERS = frozenset({'right', 'left'})


def get_joystick_recording_action(trigger: str):
    """Return the recording action for a trigger, or ``None`` if it is not one."""
    return _RECORDING_ACTION_BY_TRIGGER.get(str(trigger or '').strip())
