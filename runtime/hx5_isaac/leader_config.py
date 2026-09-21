"""Create a private LG2 controller YAML using the actual Isaac SH5 limits."""
import argparse
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def merge(existing, incoming):
    for key, value in incoming.items():
        if key in existing and isinstance(existing[key], dict) and isinstance(value, dict):
            merge(existing[key], value)
        else:
            existing[key] = value
    return existing


class ControllerLoader(yaml.SafeLoader):
    """ROS controller files may repeat '/**'; preserve every controller."""


def construct_mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=True)
        value = loader.construct_object(value_node, deep=True)
        merge(result, {key: value})
    return result


ControllerLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, construct_mapping)


def make_config(source, urdf, mobile=False):
    config = deepcopy(yaml.load(Path(source).read_text(), Loader=ControllerLoader))
    controllers = config['/**']
    controllers['controller_manager']['ros__parameters']['use_sim_time'] = False
    robot = ET.parse(urdf).getroot()
    bounds = {joint.get('name'): (float(joint.find('limit').get('lower')),
                                  float(joint.find('limit').get('upper')))
              for joint in robot.findall('joint')
              if joint.find('limit') is not None and joint.get('type') != 'continuous'}
    broadcaster = controllers['joint_trajectory_command_broadcaster']['ros__parameters']
    broadcaster['follower_joint_states_topic'] = '/joint_states'
    for hand, side in (('left', 'l'), ('right', 'r')):
        expected = [f'arm_{side}_joint{number}' for number in range(1, 8)] + [f'gripper_{side}_joint1']
        if broadcaster[hand + '_joints'] != expected:
            raise ValueError('Expected official LG2 seven-arm-joint plus gripper group')
        # Official 1044 LG2 bringup already reverses the shoulder motors with
        # Drive Mode=1. Its broadcaster uses identical arm joint coordinates.
        # Do not inherit this workspace's older SG2-only shoulder negation.
        # Omit the parameter as upstream does: YAML [] is untyped in ROS and
        # would override the plugin's typed string-array default with NOT_SET.
        broadcaster.pop(hand + '_reverse_joints', None)
        broadcaster[hand + '_min_positions'] = [bounds[name][0] for name in expected[:-1]] + [0.0]
        broadcaster[hand + '_max_positions'] = [bounds[name][1] for name in expected[:-1]] + [1.05]
    joystick = controllers['joystick_controller']['ros__parameters']
    joystick['enable_joystick_axes'] = True
    joystick['enable_swerve_mode'] = bool(mobile)
    joystick['joint_states_topic'] = '/joint_states'
    for sensor in ('sensorxel_l_joy', 'sensorxel_r_joy'):
        names = joystick[sensor + '_controlled_joints']
        joystick[sensor + '_min_positions'] = [bounds[name][0] for name in names]
        joystick[sensor + '_max_positions'] = [bounds[name][1] for name in names]
    return config


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mobile', action='store_true', help='Enable official swerve mode (initial mode: swerve)')
    parser.add_argument('--output', type=Path, default=root / 'simulation/isaac/leader/hardware.yaml')
    args = parser.parse_args()
    config = make_config(root / 'src/ai_worker/ffw_bringup/config/ffw_lg2_leader/ffw_lg2_leader_ai_hardware_controller.yaml',
                         root / 'simulation/isaac/assets/sh5/robot.urdf', args.mobile)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix('.tmp')
    temporary.write_text(yaml.safe_dump(config, sort_keys=False))
    temporary.replace(args.output)
    print(f'Isaac private LG2 configuration: {args.output}; initial mode: {"swerve" if args.mobile else "arm_control"}')


if __name__ == '__main__':
    main()
