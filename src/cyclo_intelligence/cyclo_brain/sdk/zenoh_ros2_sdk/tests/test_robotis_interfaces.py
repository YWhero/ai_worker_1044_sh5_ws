"""Official nested tactile schema registration and wire compatibility."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

from zenoh_ros2_sdk import _cache
from zenoh_ros2_sdk import message_registry as registry_module
from zenoh_ros2_sdk._repositories import MESSAGE_REPOSITORIES
from zenoh_ros2_sdk.message_registry import MessageRegistry
from zenoh_ros2_sdk.session import ZenohSession
from zenoh_ros2_sdk.utils import compute_type_hash_from_msg, load_dependencies_recursive


# ROBOTIS-GIT/robotis_interfaces at 9231cb1005dc03c14bdbf42f1f9b7114af7d3cfb.
DEFINITIONS = {
    'robotis_interfaces/msg/HandPressures':
        'std_msgs/Header header\nstring hand_name\nTactileSensor[] sensors\n',
    'robotis_interfaces/msg/TactileSensor':
        'string sensor_name\nstring[9] pressure_names\nuint8[9] pressure_values\n',
    'std_msgs/msg/Header': 'builtin_interfaces/Time stamp\nstring frame_id\n',
    'builtin_interfaces/msg/Time': 'int32 sec\nuint32 nanosec\n',
}
# Verified from ros2 topic info --verbose on the actual Jazzy hx5_sim_io publisher.
ROS_HAND_HASH = 'RIHS01_c62e924c58951db592fdc625782729b6d8184aa1f09c096511651f41b18ab623'


def local_registry(tmp_path, monkeypatch):
    for kind, definition in DEFINITIONS.items():
        package, folder, name = kind.split('/')
        path = tmp_path / package / folder / f'{name}.msg'
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(definition)
    def unexpected_download(*args, **kwargs):
        raise AssertionError('local tactile definitions must load without a network download')
    monkeypatch.setattr(registry_module, 'get_message_file_path', unexpected_download)
    return MessageRegistry(str(tmp_path))


def test_official_robotis_repository_uses_flat_message_layout():
    repo_name = _cache.get_repository_for_package('robotis_interfaces')
    repo = MESSAGE_REPOSITORIES[repo_name]
    assert repo.url == 'https://github.com/ROBOTIS-GIT/robotis_interfaces.git'
    assert repo.commit == '9231cb1005dc03c14bdbf42f1f9b7114af7d3cfb'
    path = _cache.construct_message_path('/cache/robotis_interfaces', repo,
                                        'robotis_interfaces', 'msg', 'HandPressures')
    assert Path(path) == Path('/cache/robotis_interfaces/msg/HandPressures.msg')


def test_local_hand_pressure_dependencies_match_jazzy_publisher_hash(tmp_path, monkeypatch):
    registry = local_registry(tmp_path, monkeypatch)
    kind = 'robotis_interfaces/msg/HandPressures'
    definition = registry.get_msg_file_path(kind).read_text()
    dependencies = load_dependencies_recursive(kind, definition, registry)
    assert set(dependencies) == {'std_msgs/msg/Header', 'builtin_interfaces/msg/Time',
                                 'robotis_interfaces/msg/TactileSensor'}
    assert compute_type_hash_from_msg(kind, definition, dependencies) == ROS_HAND_HASH


def test_nested_hand_pressure_registration_and_fixed_array_cdr_roundtrip(tmp_path, monkeypatch):
    registry = local_registry(tmp_path, monkeypatch)
    fake_session = SimpleNamespace(store=get_typestore(Stores.EMPTY), _registered_types={})
    def register(definition, kind):
        fake_session.store.register(get_types_from_msg(definition, kind))
        fake_session._registered_types[kind] = kind
    fake_session.register_message_type = register
    monkeypatch.setattr(ZenohSession, 'get_instance', classmethod(lambda cls: fake_session))
    kind = 'robotis_interfaces/msg/HandPressures'
    Hand = registry.get_message_class(kind)
    Sensor = registry.get_message_class('robotis_interfaces/msg/TactileSensor')
    Header = registry.get_message_class('std_msgs/msg/Header')
    Time = registry.get_message_class('builtin_interfaces/msg/Time')
    sensors = [Sensor(f'finger_l_sensor{i}', [f'pressure{j}' for j in range(9)],
                      np.arange(9, dtype=np.uint8) + i) for i in range(1, 6)]
    original = Hand(Header(Time(2, 3), 'left'), 'left', sensors)
    encoded = fake_session.store.serialize_cdr(original, kind)
    decoded = fake_session.store.deserialize_cdr(encoded, kind)
    assert decoded.header.stamp.sec == 2 and decoded.header.stamp.nanosec == 3
    assert decoded.hand_name == 'left' and len(decoded.sensors) == 5
    assert decoded.sensors[0].pressure_names == sensors[0].pressure_names
    for expected, actual in zip(sensors, decoded.sensors):
        assert actual.sensor_name == expected.sensor_name
        np.testing.assert_array_equal(actual.pressure_values, expected.pressure_values)
