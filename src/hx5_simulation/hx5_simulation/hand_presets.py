from copy import deepcopy
from functools import partial
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParametersAtomically
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from hx5_simulation.hand_mapping import interpolate, normalize, validate_profile


class HandPresets(Node):
    def __init__(self):
        super().__init__('hx5_sim_hand_presets')
        self.path = Path(self.declare_parameter('preset_file', '/workspace/hand_presets.json').value)
        self.leader_hand_duration = self.validate_leader_hand_duration(
            self.declare_parameter('leader_hand_duration', 0.1).value)
        self.declare_parameter('robot_description', '')
        robot = ET.fromstring(self.get_parameter('robot_description').value)
        self.bounds = {joint.get('name'): (float(joint.find('limit').get('lower')),
                                         float(joint.find('limit').get('upper')))
                       for joint in robot.findall('joint') if joint.find('limit') is not None
                       and joint.get('type') != 'continuous'}
        self.templates = {}
        self.open_positions = {'l': [0.0] * 20, 'r': [0.0] * 20}
        self.gripper_range = (0.0, 1.05)
        self.thumb_threshold = 0.0
        self.mapping_metadata = {'name': 'Simulation generic', 'source': ''}
        for side in ('l', 'r'):
            positions = []
            for number in range(1, 21):
                lower, upper = self.bounds[f'finger_{side}_joint{number}']
                if number == 1:
                    target = min(upper, max(lower, -0.4 if side == 'l' else 0.4))
                elif number % 4 == 1:
                    target = min(upper, max(lower, 0.0))
                else:
                    target = 0.65 * (upper if upper > abs(lower) else lower)
                positions.append(target)
            self.templates[side] = positions
        self.defaults = {
            0: {'name': 'Simulation Open', 'description': 'Open all fingers', 'curls': [0.0] * 5},
            1: {'name': 'Simulation Power Grasp', 'description': 'Simulation-only generic curl', 'curls': [1.0] * 5},
            2: {'name': 'Simulation Pinch', 'description': 'Thumb/index curl', 'curls': [1.0, 1.0, 0.0, 0.0, 0.0]},
        }
        mapping_file = self.declare_parameter('mapping_profile', '').value
        if mapping_file:
            profile = json.loads(Path(mapping_file).read_text())
            validate_profile(profile, self.bounds)
            for hand, side in (('left', 'l'), ('right', 'r')):
                self.open_positions[side] = profile[hand]['release']
                self.templates[side] = profile[hand]['grasp']
            self.gripper_range = tuple(profile['gripper_range'])
            self.thumb_threshold = profile.get('thumb_threshold', 0.0)
            self.mapping_metadata = {'name': profile['name'], 'source': profile.get('source', ''),
                                     'gripper_range': list(self.gripper_range)}
            self.defaults[0].update(name='ROBOTIS 1044 Release', description='Official 1044 release endpoint')
            self.defaults[1].update(name='ROBOTIS 1044 Grasp', description='Official 1044 grasp endpoint')
            self.defaults[2].update(name='1044 Thumb/Index', description='Thumb/index mask of official 1044 endpoints')
        self.presets = deepcopy(self.defaults)
        if self.path.exists():
            loaded = json.loads(self.path.read_text())
            for key, value in loaded.items():
                self.validate_curls(value['curls'])
                self.presets[int(key)] = value
        self.active = {'left': 1, 'right': 1}
        self.publishers_by_side = {side: self.create_publisher(JointTrajectory,
            f'/leader/joint_trajectory_command_broadcaster_{hand}_hand/joint_trajectory', 10)
            for side, hand in (('l', 'left'), ('r', 'right'))}
        self.arm_publishers = {side: self.create_publisher(JointTrajectory,
            f'/simulation/arm_{side}_controller/joint_trajectory', 10) for side in ('l', 'r')}
        for side, hand in (('l', 'left'), ('r', 'right')):
            self.create_subscription(JointTrajectory,
                f'/leader/joint_trajectory_command_broadcaster_{hand}/joint_trajectory',
                partial(self.leader, side), 10)
        self.status = self.create_publisher(String, '/leader/hand_preset/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for action in ('set', 'custom/save', 'manage/update', 'manage/delete', 'manage/restore'):
            self.create_service(SetParametersAtomically, '/leader/hand_preset/' + action,
                                partial(self.service, action))
        self.create_timer(1.0, self.publish_status)

    @staticmethod
    def validate_leader_hand_duration(duration):
        duration = float(duration)
        if not math.isfinite(duration) or not 0.0 <= duration <= 1.0:
            raise ValueError('leader_hand_duration must be finite seconds in [0, 1]')
        return duration

    @staticmethod
    def validate_curls(curls):
        if len(curls) != 5 or any(not math.isfinite(value) or not 0 <= value <= 1 for value in curls):
            raise ValueError('Five finite curls in [0, 1] are required')

    def positions(self, side, curls):
        self.validate_curls(curls)
        return interpolate(self.open_positions[side], self.templates[side], curls,
                           self.bounds, side, self.thumb_threshold)

    def publish_hand(self, side, curls, duration=0.4):
        output = JointTrajectory(joint_names=[f'finger_{side}_joint{number}' for number in range(1, 21)])
        point = JointTrajectoryPoint(positions=self.positions(side, curls))
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        output.points = [point]
        self.publishers_by_side[side].publish(output)

    def leader(self, side, message):
        arm_names = {f'arm_{side}_joint{number}' for number in range(1, 8)}
        wanted = [index for index, name in enumerate(message.joint_names) if name in arm_names]
        if not wanted or len(set(message.joint_names)) != len(message.joint_names):
            return
        output = deepcopy(message)
        output.joint_names = [message.joint_names[index] for index in wanted]
        for point in output.points:
            for field in ('positions', 'velocities', 'accelerations', 'effort'):
                values = getattr(point, field)
                if values and (len(values) != len(message.joint_names) or any(not math.isfinite(value) for value in values)):
                    return
                setattr(point, field, [values[index] for index in wanted] if values else [])
            if point.positions:
                point.positions = [min(self.bounds[name][1], max(self.bounds[name][0], value))
                                   for name, value in zip(output.joint_names, point.positions)]
        self.arm_publishers[side].publish(output)
        gripper = f'gripper_{side}_joint1'
        if gripper in message.joint_names and message.points and message.points[-1].positions:
            value = message.points[-1].positions[message.joint_names.index(gripper)]
            curl = normalize(value, *self.gripper_range)
            hand = 'left' if side == 'l' else 'right'
            selected = self.presets[self.active[hand]]['curls']
            self.publish_hand(side, [curl * value for value in selected], self.leader_hand_duration)

    def service(self, action, request, response):
        values = {}
        for parameter in request.parameters:
            value = parameter.value
            values[parameter.name] = {2: value.integer_value, 4: value.string_value,
                                       8: list(value.double_array_value)}.get(value.type)
        previous = deepcopy(self.presets)
        try:
            if action == 'set':
                preset = self.presets[int(values['preset_id'])]
                side = values['side']
                if side not in ('left', 'right', 'both'):
                    raise ValueError('Invalid hand side')
                for name, suffix in (('left', 'l'), ('right', 'r')):
                    if side in (name, 'both'):
                        self.publish_hand(suffix, preset['curls'])
                        self.active[name] = int(values['preset_id'])
            else:
                if action == 'custom/save':
                    self.validate_curls(values['curls'])
                    preset_id = next(number for number in range(3, 256) if number not in self.presets)
                    self.presets[preset_id] = {'name': values['name'], 'description': values.get('description', ''),
                                              'curls': values['curls'], 'custom': True}
                else:
                    preset_id = int(values['preset_id'])
                    if action == 'manage/delete':
                        if preset_id in self.defaults:
                            raise ValueError('Built-in simulation presets cannot be deleted')
                        if preset_id in self.active.values():
                            raise ValueError('Select another preset before deleting the active preset')
                        del self.presets[preset_id]
                    elif action == 'manage/restore':
                        self.presets[preset_id] = deepcopy(self.defaults[preset_id])
                    else:
                        self.presets[preset_id].update({key: values.get(key, '') for key in ('name', 'description', 'note')})
                for preset in self.presets.values():
                    if not 1 <= len(preset['name'].strip()) <= 48 or len(preset.get('description', '')) > 160:
                        raise ValueError('Invalid preset name or description length')
                self.path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self.path.with_suffix('.tmp')
                temporary.write_text(json.dumps(self.presets, indent=2))
                temporary.replace(self.path)
            response.result = SetParametersResult(successful=True, reason='Simulation preset applied')
            self.publish_status()
        except (KeyError, ValueError, TypeError, StopIteration, OSError) as error:
            self.presets = previous
            response.result = SetParametersResult(successful=False, reason=str(error))
        return response

    def publish_status(self):
        payload = {'preset_ids': sorted(self.presets), 'enabled_sides': ['left', 'right'],
                   'mapping_profile': self.mapping_metadata,
                   'leader_hand_duration': self.leader_hand_duration,
                   'left_preset_id': self.active['left'], 'right_preset_id': self.active['right'],
                   'preview_joint_names': [f'finger_r_joint{number}' for number in range(1, 21)],
                   'custom_editor': {'control_labels': ['Thumb', 'Index', 'Middle', 'Ring', 'Little'],
                                     'template_preset_id': 1, 'preview_open_positions': self.positions('r', [0.0] * 5),
                                     'preview_closed_positions': self.positions('r', [1.0] * 5)}, 'presets': []}
        for preset_id, preset in sorted(self.presets.items()):
            payload['presets'].append({**preset, 'id': preset_id, 'editable': True,
                'deletable': preset_id not in self.defaults, 'editor_curls': preset['curls'],
                'editor_curls_exact': True, 'preview_open_positions': self.positions('r', [0.0] * 5),
                'preview_positions': self.positions('r', preset['curls'])})
        self.status.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = HandPresets()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
