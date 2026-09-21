"""The real Cyclo converter filters the LG2 gripper out of its 54-D action."""
import json
from pathlib import Path

import numpy as np
import yaml

from cyclo_data.converter.base_converter import RosbagToLerobotConverterBase


def test_real_converter_merges_original_lg2_arm_messages_and_mapped_hx5_hands():
    directory = Path(__file__).resolve().parent
    profile = json.loads((directory / 'official_1044_hand_mapping.json').read_text())
    path = Path('/orchestrator_config/ffw_sh5_rev1_config.yaml')
    if not path.exists():
        path = directory.parents[1] / 'runtime/hx5_sim/robot_configs/ffw_sh5_rev1_config.yaml'
    robot = yaml.safe_load(path.read_text())['orchestrator']['ros__parameters']['ffw_sh5_rev1']
    actions = robot['action']
    converter = RosbagToLerobotConverterBase.__new__(RosbagToLerobotConverterBase)
    converter._fixed_action_by_group = {}
    converter._action_topic_key_map = {value['topic']: 'leader_' + group for group,value in actions.items()}
    converter._joint_order_by_group = {'leader_' + group: value['joint_names'] for group,value in actions.items()}
    converter._log_warning = lambda message: (_ for _ in ()).throw(AssertionError(message))
    messages = {}; names = {}; expected = []
    for group, definition in actions.items():
        topic = definition['topic']; desired = definition['joint_names']
        if group.startswith('arm_'):
            side = 'l' if group == 'arm_left' else 'r'
            names[topic] = [f'gripper_{side}_joint1'] + desired[::-1]
            values = [.2] + [.01] * 7
            expected.extend([.01]*7)
        else:
            hand = 'left' if group == 'hand_left' else 'right'
            names[topic] = desired
            endpoints = profile[hand]
            values = [release + .25*(grasp-release)
                      for release,grasp in zip(endpoints['release'], endpoints['grasp'])]
            expected.extend(values)
        messages[topic] = [(1.0, np.asarray(values)), (1.05, np.asarray(values))]
    merged = converter._merge_action_messages(messages, names)
    assert len(merged) == 2 and merged[0][1].shape == (54,)
    np.testing.assert_allclose(merged[0][1], expected, atol=1e-7)
    assert converter._action_joint_names == robot['observation']['state']['arm_hand']['joint_names']
    assert not any(name.startswith('gripper') or name.startswith('head') or name == 'lift_joint'
                   for name in converter._action_joint_names)
