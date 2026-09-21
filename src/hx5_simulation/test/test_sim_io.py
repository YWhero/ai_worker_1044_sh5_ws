import time
from types import SimpleNamespace

from builtin_interfaces.msg import Time

from hx5_simulation.sim_io import SimIO


def context():
    messages = {'l': [], 'r': []}
    node = SimpleNamespace(
        scale=8.0,
        samples={(side, finger): (time.monotonic(), [0.0] * 9, Time(sec=7))
                 for side in ('l', 'r') for finger in range(1, 6)},
        pressures={side: SimpleNamespace(publish=messages[side].append) for side in messages},
        forces={side: SimpleNamespace(publish=lambda message: None) for side in messages},
    )
    return node, messages


def test_pressure_names_match_robotis_hardware_contract():
    node, messages = context()
    SimIO.publish_tactile(node)
    for side, hand in (('l', 'left'), ('r', 'right')):
        message = messages[side][0]
        assert message.hand_name == hand
        assert message.header.stamp.sec == 7
        assert len(message.sensors) == 5
        for finger, sensor in enumerate(message.sensors, 1):
            assert sensor.sensor_name == f'finger_{side}_sensor{finger}'
            assert list(sensor.pressure_names) == [f'Present Pressure {index}' for index in range(1, 10)]
            assert list(sensor.pressure_values) == [0] * 9


def test_stale_finger_does_not_publish_a_fresh_zero_hand():
    node, messages = context()
    node.samples['l', 3] = (time.monotonic() - 1.0, [0.0] * 9, Time(sec=6))
    SimIO.publish_tactile(node)
    assert not messages['l']
    assert len(messages['r']) == 1
