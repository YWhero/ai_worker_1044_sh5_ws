import numpy as np
from pathlib import Path
import sys
import types


class _StubDependency:
    def __init__(self, *args, **kwargs):
        pass


bag_reader_module = types.ModuleType("cyclo_data.reader.bag_reader")
bag_reader_module.BagReader = _StubDependency
sys.modules.setdefault("cyclo_data.reader.bag_reader", bag_reader_module)

metadata_manager_module = types.ModuleType("cyclo_data.reader.metadata_manager")
metadata_manager_module.MetadataManager = _StubDependency
sys.modules.setdefault("cyclo_data.reader.metadata_manager", metadata_manager_module)

video_metadata_module = types.ModuleType("cyclo_data.reader.video_metadata_extractor")
video_metadata_module.VideoMetadataExtractor = _StubDependency
sys.modules.setdefault(
    "cyclo_data.reader.video_metadata_extractor",
    video_metadata_module,
)

try:
    import pyarrow  # noqa: F401
    import pyarrow.parquet  # noqa: F401
except ModuleNotFoundError:
    pyarrow_module = types.ModuleType("pyarrow")
    pyarrow_module.Table = type("Table", (), {})
    pyarrow_parquet_module = types.ModuleType("pyarrow.parquet")
    sys.modules.setdefault("pyarrow", pyarrow_module)
    sys.modules.setdefault("pyarrow.parquet", pyarrow_parquet_module)

from cyclo_data.converter.base_converter import (
    ConversionConfig,
    EpisodeData,
    RosbagToLerobotConverterBase,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
SH5_CONFIG = (
    REPO_ROOT
    / "shared"
    / "shared"
    / "robot_configs"
    / "ffw_sh5_rev1_config.yaml"
)


class _TactileSensor:
    def __init__(self, sensor_name, pressure_values):
        self.sensor_name = sensor_name
        self.pressure_values = pressure_values


class _HandPressures:
    def __init__(self, sensors):
        self.sensors = sensors


def test_hand_pressures_are_mean_pooled_per_finger(tmp_path):
    converter = RosbagToLerobotConverterBase(
        ConversionConfig(repo_id="test", output_dir=tmp_path)
    )
    msg = _HandPressures([
        _TactileSensor("finger_1", bytes(range(9))),
        _TactileSensor("finger_2", [10] * 9),
        _TactileSensor("finger_3", []),
    ])

    pooled = converter._extract_tactile_mean_pooled(msg)
    names = converter._extract_tactile_names("/left_hand/finger_pressures", msg)

    np.testing.assert_allclose(pooled, np.array([4.0, 10.0, 0.0]))
    assert names == [
        "left_hand_finger_pressures_finger_1_mean",
        "left_hand_finger_pressures_finger_2_mean",
        "left_hand_finger_pressures_finger_3_mean",
    ]


def test_tactile_state_dims_append_after_joint_state(tmp_path):
    converter = RosbagToLerobotConverterBase(
        ConversionConfig(repo_id="test", output_dir=tmp_path)
    )
    converter._joint_order_by_group = {
        "leader_upper_body": ["joint_0", "joint_1"],
    }
    converter._state_topic_key_map = {
        "/joint_states": "follower_upper_body",
        "/left_hand/finger_pressures": "tactile_left_hand_pressure",
        "/right_hand/finger_pressures": "tactile_right_hand_pressure",
    }

    merged = converter._merge_state_messages(
        {
            "/joint_states": [(0.0, np.array([1.0, 2.0], dtype=np.float32))],
            "/left_hand/finger_pressures": [
                (0.0, np.arange(10.0, 15.0, dtype=np.float32))
            ],
            "/right_hand/finger_pressures": [
                (0.0, np.arange(20.0, 25.0, dtype=np.float32))
            ],
        },
        {
            "/joint_states": ["joint_0", "joint_1"],
            "/left_hand/finger_pressures": [
                f"tactile_left_hand_pressure_finger_{i}_mean"
                for i in range(1, 6)
            ],
            "/right_hand/finger_pressures": [
                f"tactile_right_hand_pressure_finger_{i}_mean"
                for i in range(1, 6)
            ],
        },
    )

    assert len(merged) == 1
    np.testing.assert_allclose(
        merged[0][1],
        np.array(
            [1.0, 2.0, 10.0, 11.0, 12.0, 13.0, 14.0,
             20.0, 21.0, 22.0, 23.0, 24.0],
            dtype=np.float32,
        ),
    )
    assert converter._state_joint_names == [
        "joint_0",
        "joint_1",
        "tactile_left_hand_pressure_finger_1_mean",
        "tactile_left_hand_pressure_finger_2_mean",
        "tactile_left_hand_pressure_finger_3_mean",
        "tactile_left_hand_pressure_finger_4_mean",
        "tactile_left_hand_pressure_finger_5_mean",
        "tactile_right_hand_pressure_finger_1_mean",
        "tactile_right_hand_pressure_finger_2_mean",
        "tactile_right_hand_pressure_finger_3_mean",
        "tactile_right_hand_pressure_finger_4_mean",
        "tactile_right_hand_pressure_finger_5_mean",
    ]


def test_sh5_legacy_joint_states_fill_bilateral_state_and_full_action(tmp_path):
    converter = RosbagToLerobotConverterBase(
        ConversionConfig(
            repo_id="test",
            output_dir=tmp_path,
            robot_type="ffw_sh5_rev1",
            robot_config_path=str(SH5_CONFIG),
        )
    )

    left_arm = [f"arm_l_joint{i}" for i in range(1, 8)]
    right_arm = [f"arm_r_joint{i}" for i in range(1, 8)]
    left_hand = [f"finger_l_joint{i}" for i in range(1, 21)]
    right_hand = [f"finger_r_joint{i}" for i in range(1, 21)]
    arm_hand_state_names = left_arm + right_arm + left_hand + right_hand
    full_joint_names = (
        left_arm
        + right_arm
        + left_hand
        + right_hand
        + ["head_joint1", "head_joint2", "lift_joint"]
    )
    full_positions = np.arange(len(full_joint_names), dtype=np.float32)
    arm_hand_indices = [
        full_joint_names.index(name)
        for name in arm_hand_state_names
    ]

    converter._apply_legacy_joint_state_fallbacks({
        "/joint_states": "sensor_msgs/msg/JointState",
        "/left_hand/finger_pressures": "robotis_interfaces/msg/HandPressures",
        "/right_hand/finger_pressures": "robotis_interfaces/msg/HandPressures",
    })

    state_messages = converter._merge_state_messages(
        {
            "/joint_states": [(0.0, full_positions)],
            "/left_hand/finger_pressures": [
                (0.0, np.array([90.0, 91.0, 92.0, 93.0, 94.0], dtype=np.float32))
            ],
            "/right_hand/finger_pressures": [
                (0.0, np.array([100.0, 101.0, 102.0, 103.0, 104.0], dtype=np.float32))
            ],
        },
        {
            "/joint_states": full_joint_names,
            "/left_hand/finger_pressures": [
                f"tactile_left_hand_pressure_finger_{i}_mean"
                for i in range(1, 6)
            ],
            "/right_hand/finger_pressures": [
                f"tactile_right_hand_pressure_finger_{i}_mean"
                for i in range(1, 6)
            ],
        },
    )

    assert "/joint_states" in converter.config.state_topics
    assert len(state_messages) == 1
    assert state_messages[0][1].shape == (64,)
    np.testing.assert_allclose(
        state_messages[0][1],
        np.concatenate([
            full_positions[arm_hand_indices],
            np.array([90.0, 91.0, 92.0, 93.0, 94.0], dtype=np.float32),
            np.array([100.0, 101.0, 102.0, 103.0, 104.0], dtype=np.float32),
        ]),
    )
    assert converter._state_joint_names[:54] == arm_hand_state_names
    assert converter._state_joint_names[54:] == (
        [
            f"tactile_left_hand_pressure_finger_{i}_mean"
            for i in range(1, 6)
        ]
        + [
            f"tactile_right_hand_pressure_finger_{i}_mean"
            for i in range(1, 6)
        ]
    )

    left_arm_action = np.arange(0.0, 7.0, dtype=np.float32)
    right_arm_action = np.arange(10.0, 17.0, dtype=np.float32)
    left_hand_action = np.arange(40.0, 60.0, dtype=np.float32)
    right_hand_action = np.arange(20.0, 40.0, dtype=np.float32)
    action_messages = converter._merge_action_messages(
        {
            "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory": [
                (0.0, left_arm_action)
            ],
            "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory": [
                (0.0, right_arm_action)
            ],
            "/leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory": [
                (0.0, left_hand_action)
            ],
            "/leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory": [
                (0.0, right_hand_action)
            ],
        },
        {
            "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory":
                left_arm,
            "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory":
                right_arm,
            "/leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory":
                left_hand,
            "/leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory":
                right_hand,
        },
    )

    assert len(action_messages) == 1
    action = action_messages[0][1]
    assert action.shape == (60,)
    np.testing.assert_allclose(action[0:7], left_arm_action)
    np.testing.assert_allclose(action[7:14], right_arm_action)
    np.testing.assert_allclose(action[14:34], left_hand_action)
    np.testing.assert_allclose(action[34:54], right_hand_action)
    np.testing.assert_allclose(
        action[54:],
        np.array([0.682621, -0.349748, -0.15, 0.0, 0.0, 0.0], dtype=np.float32),
    )


def test_feature_names_allow_state_action_dimension_mismatch(tmp_path):
    converter = RosbagToLerobotConverterBase(
        ConversionConfig(repo_id="test", output_dir=tmp_path)
    )
    arm_names = [f"joint_{i}" for i in range(57)]
    mobile_names = ["linear_x", "linear_y", "angular_z"]

    converter._joint_order_by_group = {
        "leader_upper_body": arm_names,
        "leader_mobile": mobile_names,
    }
    converter._action_joint_names = []
    converter._state_joint_names = []

    episode = EpisodeData(
        episode_index=0,
        observation_state=[np.zeros(60, dtype=np.float32)],
        action=[np.zeros(57, dtype=np.float32)],
        observation_state_names=arm_names + mobile_names,
        action_names=arm_names,
    )

    converter._build_features([episode])

    assert converter._features["observation.state"]["shape"] == (60,)
    assert converter._features["observation.state"]["names"] == arm_names + mobile_names
    assert converter._features["action"]["shape"] == (57,)
    assert converter._features["action"]["names"] == arm_names


def test_feature_names_ignore_mismatched_config_fallback(tmp_path):
    converter = RosbagToLerobotConverterBase(
        ConversionConfig(repo_id="test", output_dir=tmp_path)
    )
    parsed_action_names = [f"arm_joint_{i}" for i in range(57)]
    converter._joint_order_by_group = {
        "leader_upper_body": parsed_action_names,
        "leader_mobile": ["linear_x", "linear_y", "angular_z"],
    }

    episode = EpisodeData(
        episode_index=0,
        observation_state=[np.zeros(60, dtype=np.float32)],
        action=[np.zeros(57, dtype=np.float32)],
        action_names=parsed_action_names,
    )

    converter._build_features([episode])

    assert converter._features["action"]["names"] == parsed_action_names
    assert converter._features["action"]["names"] != [f"joint_{i}" for i in range(57)]


def test_feature_names_can_use_later_episode_names(tmp_path):
    converter = RosbagToLerobotConverterBase(
        ConversionConfig(repo_id="test", output_dir=tmp_path)
    )
    action_names = [f"arm_joint_{i}" for i in range(57)]

    episodes = [
        EpisodeData(
            episode_index=0,
            observation_state=[np.zeros(60, dtype=np.float32)],
            action=[np.zeros(57, dtype=np.float32)],
        ),
        EpisodeData(
            episode_index=1,
            observation_state=[np.zeros(60, dtype=np.float32)],
            action=[np.zeros(57, dtype=np.float32)],
            action_names=action_names,
        ),
    ]

    converter._build_features(episodes)

    assert converter._features["action"]["names"] == action_names


def test_feature_names_ignore_none_name_sources(tmp_path):
    converter = RosbagToLerobotConverterBase(
        ConversionConfig(repo_id="test", output_dir=tmp_path)
    )
    action_names = [f"arm_joint_{i}" for i in range(57)]
    converter._state_joint_names = None
    converter._action_joint_names = None

    first = EpisodeData(
        episode_index=0,
        observation_state=[np.zeros(60, dtype=np.float32)],
        action=[np.zeros(57, dtype=np.float32)],
    )
    first.observation_state_names = None
    first.action_names = None
    second = EpisodeData(
        episode_index=1,
        observation_state=[np.zeros(60, dtype=np.float32)],
        action=[np.zeros(57, dtype=np.float32)],
        action_names=action_names,
    )

    converter._build_features([first, second])

    assert converter._features["action"]["names"] == action_names
