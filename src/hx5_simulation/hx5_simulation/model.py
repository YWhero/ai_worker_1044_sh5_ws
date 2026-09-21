import copy
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

MODEL = 'ffw_sh5_rev1_follower'
SPAWN_X = 0.196735511165
SPAWN_Y = 1.196589404083
SPAWN_YAW = 1.561340467224
CAMERAS = {
    'head_left': ('zed/left/image_raw', 'zedm_left_camera_frame', '/zed/zed_node/left/image_rect_color'),
    'head_right': ('zed/right/image_raw', 'zedm_right_camera_frame', '/zed/zed_node/right/image_rect_color'),
    'wrist_left': ('camera_left/color/image_raw', 'camera_l_link', '/camera_left/camera_left/color/image_rect_raw'),
    'wrist_right': ('camera_right/color/image_raw', 'camera_r_link', '/camera_right/camera_right/color/image_rect_raw'),
}
JOINT_NAMES = (
    [f'arm_{side}_joint{joint}' for side in ('l', 'r') for joint in range(1, 8)]
    + [f'finger_{side}_joint{joint}' for side in ('l', 'r') for joint in range(1, 21)]
)


def text_element(parent, tag, value):
    element = ET.SubElement(parent, tag)
    element.text = str(value)
    return element


def simulation_controller_config(source):
    """Keep official controller topology with deadbands suitable for Gazebo.

    Official files repeat the ``/**`` namespace for each controller. Compose
    the YAML nodes rather than loading a dict, which would discard controllers
    with those repeated keys.
    """
    root = yaml.compose(source)
    mobile_sections = 0

    def visit(node):
        nonlocal mobile_sections
        if isinstance(node, yaml.MappingNode):
            for key, value in node.value:
                if key.value == 'use_sim_time':
                    value.value = 'true'
                    value.tag = 'tag:yaml.org,2002:bool'
                if key.value == 'swerve_drive_controller' and isinstance(value, yaml.MappingNode):
                    params = next((child for name, child in value.value
                                   if name.value == 'ros__parameters'), None)
                    if isinstance(params, yaml.MappingNode):
                        # Rotate's final 0.05 rad/s and Nav2's smoothed startup
                        # 0.075 rad/s both fall below the hardware default 0.1.
                        # In Gazebo they are valid commands, not joystick noise.
                        deadbands = {'linear_vel_deadband', 'angular_vel_deadband', 'enable_odom_tf'}
                        params.value = [(name, child) for name, child in params.value
                                        if name.value not in deadbands]
                        for name in ('angular_vel_deadband', 'linear_vel_deadband'):
                            params.value.append((
                                yaml.ScalarNode('tag:yaml.org,2002:str', name),
                                yaml.ScalarNode('tag:yaml.org,2002:float', '0.001'),
                            ))
                        params.value.append((
                            yaml.ScalarNode('tag:yaml.org,2002:str', 'enable_odom_tf'),
                            yaml.ScalarNode('tag:yaml.org,2002:bool', 'false'),
                        ))
                        mobile_sections += 1
                visit(value)
        elif isinstance(node, yaml.SequenceNode):
            for child in node.value:
                visit(child)

    visit(root)
    if mobile_sections != 1:
        raise ValueError('Expected exactly one swerve controller parameter section')
    return yaml.serialize(root)


def simulation_initial_positions(source, profile='inference'):
    """Use the captured ViTacFormer sync pose or an official SH5 pose."""
    if profile not in ('inference', 'navigation', 'task'):
        raise ValueError('initial_pose must be inference, navigation or task')
    positions = {}
    for _, namespace in yaml.compose(source).value:
        for name, section in yaml.safe_load(yaml.serialize(namespace)).items():
            params = section['ros__parameters']
            steps = params['step_names']
            step = steps[0] if profile != 'task' and name.startswith('arm_') else steps[-1]
            names, values = params['joint_names'], params[step]
            if len(names) != len(values):
                raise ValueError(f'Joint count mismatch in {name}')
            positions.update(zip(names, values))
    if profile == 'inference':
        snapshot = yaml.safe_load((Path(__file__).parent / 'initial_poses' /
                                   'vitacformer_task519_sync.yaml').read_text())
        captured = snapshot['initial_positions']
        expected = set(JOINT_NAMES + ['head_joint1', 'head_joint2', 'lift_joint'])
        if set(captured) != expected:
            raise ValueError('Inference initial pose must contain all 57 SH5 joints')
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(value) for value in captured.values()):
            raise ValueError('Inference initial pose must contain finite joint positions')
        positions.update(captured)
    return positions


def build_description(directory, initial_pose='inference'):
    description = Path(get_package_share_directory('ffw_description'))
    bringup = Path(get_package_share_directory('ffw_bringup'))
    root = ET.fromstring(xacro.process_file(
        str(description / 'urdf' / MODEL / 'ffw_sh5_follower.urdf.xacro'),
        mappings={'use_sim': 'true'},
    ).toxml())
    reference = ET.fromstring(xacro.process_file(
        str(description / 'urdf/ffw_sg2_rev1_follower/ffw_sg2_follower.urdf.xacro'),
        mappings={'use_sim': 'true'},  # camera and laser sensor definitions only
    ).toxml())
    links = {link.get('name') for link in root.findall('link')}
    for gazebo in reference.findall('gazebo'):
        if gazebo.get('reference') in links:
            root.append(copy.deepcopy(gazebo))
    left_camera = next(element for element in root.findall('gazebo')
                       if element.find("sensor[@type='camera']") is not None
                       and 'zed' in element.get('reference', ''))
    right_camera = copy.deepcopy(left_camera)
    right_camera.set('reference', 'zedm_right_camera_frame')
    for element in right_camera.iter():
        for key, value in list(element.attrib.items()):
            element.set(key, value.replace('left', 'right'))
        if element.text:
            element.text = element.text.replace('left', 'right')
    right_camera.find('sensor').set('name', 'head_right_rgb_camera')
    right_camera.find('sensor/camera').set('name', 'head_right_rgb_camera')
    root.append(right_camera)
    gazebo = ET.SubElement(root, 'gazebo')
    odometry = ET.SubElement(gazebo, 'plugin',
                             filename='gz-sim-odometry-publisher-system',
                             name='gz::sim::systems::OdometryPublisher')
    for key, value in {'odom_frame': 'l_sorting_workcell', 'robot_base_frame': 'base_link',
                       'odom_publish_frequency': 50, 'dimensions': 2,
                       'odom_topic': f'/model/{MODEL}/odometry',
                       'tf_topic': '/simulation/gazebo_odometry_tf'}.items():
        text_element(odometry, key, value)
    for side in ('l', 'r'):
        for finger in range(1, 6):
            link_name = f'finger_{side}_link{finger * 4}'
            link = root.find(f"link[@name='{link_name}']")
            collision = link.find('collision')
            collision_name = f'{link_name}_collision'
            collision.set('name', link_name)
            gazebo = ET.SubElement(root, 'gazebo', reference=link_name)
            text_element(gazebo, 'mu1', 1.0)
            text_element(gazebo, 'mu2', 1.0)
            sensor = ET.SubElement(gazebo, 'sensor', name=f'tactile_{side}_{finger}', type='contact')
            text_element(sensor, 'always_on', 'true')
            text_element(sensor, 'update_rate', 50)
            text_element(sensor, 'topic', f'/hx5/contact/{side}/finger{finger}')
            text_element(ET.SubElement(sensor, 'contact'), 'collision', collision_name)
    gazebo = ET.SubElement(root, 'gazebo')
    publisher = ET.SubElement(gazebo, 'plugin', filename='gz-sim-pose-publisher-system',
                              name='gz::sim::systems::PosePublisher')
    for key, value in {'publish_link_pose': 'true', 'publish_model_pose': 'true',
                       'publish_nested_model_pose': 'false', 'use_pose_vector_msg': 'true',
                       'update_frequency': '50'}.items():
        text_element(publisher, key, value)
    pose_file = bringup / 'config' / MODEL / 'ffw_sh5_follower_initial_positions.yaml'
    initial = simulation_initial_positions(pose_file.read_text(), initial_pose)
    for name, value in initial.items():
        joint = root.find(f"joint[@name='{name}']")
        if joint is None or joint.find('limit') is None:
            raise ValueError(f'Initial pose joint has no physical limit: {name}')
        limit = joint.find('limit')
        if not float(limit.get('lower')) <= value <= float(limit.get('upper')):
            raise ValueError(f'Initial pose exceeds physical limit: {name}={value}')
    for control in root.findall('ros2_control'):
        if not (control.findtext('hardware/plugin') or '').strip():
            root.remove(control)
            continue
        control.attrib.pop('is_async', None)
        for gpio in control.findall('gpio'):
            control.remove(gpio)
        for joint in control.findall('joint'):
            state = joint.find("state_interface[@name='position']")
            if state is None:
                continue
            if joint.get('name') in initial:
                for old in state.findall('param'):
                    state.remove(old)
                text_element(state, 'param', initial[joint.get('name')]).set('name', 'initial_value')
    config = bringup / 'config' / MODEL / 'ffw_sh5_follower_ai_hardware_controller.yaml'
    controller_file = directory / 'controllers.yaml'
    controller_file.write_text(simulation_controller_config(config.read_text()))
    for parameters in root.findall("gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']/parameters"):
        parameters.text = str(controller_file)
    robot_file = directory / 'robot.urdf'
    robot_file.write_text(ET.tostring(root, encoding='unicode'))
    return robot_file, root


def simulation_model(robot_file, directory):
    converted = subprocess.run(['gz', 'sdf', '-p', str(robot_file)],
                               check=True, capture_output=True, text=True)
    root = ET.fromstring(converted.stdout)
    for joint in root.findall('model/joint'):
        name = joint.get('name', '')
        if name.startswith(('finger_l_joint', 'finger_r_joint')) or name == 'lift_joint':
            limit = joint.find('axis/limit')
            limit.find('lower').text = str(float(limit.findtext('lower')) - 0.001)
            limit.find('upper').text = str(float(limit.findtext('upper')) + 0.001)
    target = directory / 'robot.sdf'
    target.write_text(ET.tostring(root, encoding='unicode'))
    return target


def build_world(directory, name='l_sorting_workcell'):
    bringup = Path(get_package_share_directory('ffw_bringup'))
    root = ET.parse(bringup / 'worlds' / f'{name}.sdf').getroot()
    world = root.find('world')
    world.find('physics/max_step_size').text = '0.002'
    world.find('physics/real_time_update_rate').text = '500'
    for plugin in world.findall('plugin'):
        if plugin.get('name') == 'gz::sim::systems::Contact':
            world.remove(plugin)
    ET.SubElement(world, 'plugin', filename='libhx5_contact_system.so',
                  name='hx5_simulation::ContactSystem')
    target = directory / 'world.sdf'
    target.write_text(ET.tostring(root, encoding='unicode'))
    return target, world.get('name')
