from types import SimpleNamespace
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from hx5_simulation.model import MODEL, build_description, simulation_controller_config, simulation_model
from hx5_simulation.model import simulation_initial_positions


def test_sim_deadbands_allow_rotate_tail_and_nav2_startup_without_losing_controllers():
    official_file = (Path(__file__).resolve().parents[2] / 'ai_worker' / 'ffw_bringup' /
                     'config/ffw_sh5_rev1_follower/ffw_sh5_follower_ai_hardware_controller.yaml')
    source = official_file.read_text()
    original_nodes = yaml.compose(source)
    output = simulation_controller_config(source)
    simulated_nodes = yaml.compose(output)
    assert len(original_nodes.value) == len(simulated_nodes.value)

    def sections(root):
        result = {}
        for namespace, controllers in root.value:
            assert namespace.value == '/**'
            result.update(yaml.safe_load(yaml.serialize(controllers)))
        return result

    original = sections(original_nodes)
    simulated = sections(simulated_nodes)
    mobile = simulated['swerve_drive_controller']['ros__parameters']
    assert mobile['angular_vel_deadband'] < 0.05  # official Rotate minimum
    assert mobile['angular_vel_deadband'] < 1.5 / 20.0  # Nav2 smoother first tick
    assert mobile['linear_vel_deadband'] < 0.5 / 20.0
    assert mobile['angular_vel_deadband'] > 0.0
    assert mobile['enable_odom_tf'] is False
    simulated['controller_manager']['ros__parameters']['use_sim_time'] = False
    mobile.pop('linear_vel_deadband')
    mobile.pop('angular_vel_deadband')
    mobile['enable_odom_tf'] = original['swerve_drive_controller']['ros__parameters']['enable_odom_tf']
    assert simulated == original  # all joint controllers and limits are preserved
    assert official_file.read_text() == source


def test_sim_controller_config_replaces_existing_deadbands():
    source = '''/**:
  swerve_drive_controller:
    ros__parameters:
      angular_vel_deadband: 0.1
      linear_vel_deadband: 0.1
'''
    output = yaml.safe_load(simulation_controller_config(source))
    assert output['/**']['swerve_drive_controller']['ros__parameters'] == {
        'angular_vel_deadband': 0.001, 'linear_vel_deadband': 0.001,
        'enable_odom_tf': False,
    }


def test_generated_model_preserves_official_physics_odometry_plugin(tmp_path):
    robot_file, description = build_description(tmp_path, 'navigation')
    model = ET.parse(simulation_model(robot_file, tmp_path)).getroot().find('model')
    plugin = model.find("plugin[@name='gz::sim::systems::OdometryPublisher']")
    assert plugin is not None
    assert plugin.get('filename') == 'gz-sim-odometry-publisher-system'
    assert plugin.findtext('odom_topic') == f'/model/{MODEL}/odometry'
    assert plugin.findtext('odom_frame') == 'l_sorting_workcell'
    assert plugin.findtext('robot_base_frame') == 'base_link'
    assert plugin.findtext('dimensions') == '2'
    assert plugin.findtext('tf_topic') != f'/model/{MODEL}/pose'
    initial = {joint.get('name'): float(joint.findtext(
        "state_interface[@name='position']/param[@name='initial_value']", '0'))
        for joint in description.findall('ros2_control/joint')}
    for side in ('l', 'r'):
        assert [initial[f'arm_{side}_joint{index}'] for index in range(1, 8)] == [
            1.57, 0.0, 0.0, -2.8, 0.0, 0.0, 0.0]
        assert all(initial[f'finger_{side}_joint{index}'] == 0 for index in range(1, 21))


def test_default_inference_pose_initializes_all_joints_with_head_down_and_lift_up(tmp_path):
    _, description = build_description(tmp_path)
    initial = {joint.get('name'): float(joint.findtext(
        "state_interface[@name='position']/param[@name='initial_value']", '0'))
        for joint in description.findall('ros2_control/joint')}
    selected = [name for name in initial if name.startswith(('arm_', 'finger_', 'head_'))
                or name == 'lift_joint']
    assert len(selected) == 57
    for name in selected:
        limit = description.find(f"joint[@name='{name}']/limit")
        assert math.isfinite(initial[name])
        assert float(limit.get('lower')) <= initial[name] <= float(limit.get('upper'))
    pitch = description.find("joint[@name='head_joint1']")
    yaw = description.find("joint[@name='head_joint2']")
    lift = description.find("joint[@name='lift_joint']")
    assert pitch.find('axis').get('xyz') == '0 1 0'
    assert yaw.find('axis').get('xyz') == '0 0 1'
    assert initial['head_joint1'] == float(pitch.find('limit').get('upper'))
    assert initial['head_joint2'] == 0  # centered yaw
    assert -math.sin(initial['head_joint1']) < 0  # camera's forward axis looks downward
    assert initial['lift_joint'] == float(lift.find('limit').get('upper'))
    assert initial['finger_l_joint2'] == 1.5
    assert initial['finger_r_joint2'] == -1.5
    # Learned finger targets are a captured prediction, not an all-open pose.
    assert any(abs(initial[f'finger_l_joint{i}']) > .01 for i in range(3, 21))


def test_official_sh5_task_profile_keeps_both_hands_head_and_lift():
    path = (Path(__file__).resolve().parents[2] / 'ai_worker' / 'ffw_bringup' / 'config' /
            MODEL / 'ffw_sh5_follower_initial_positions.yaml')
    task = simulation_initial_positions(path.read_text(), 'task')
    for side in ('l', 'r'):
        assert [task[f'arm_{side}_joint{index}'] for index in range(1, 8)] == [
            0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0]
        assert all(task[f'finger_{side}_joint{index}'] == 0 for index in range(1, 21))
    assert task['head_joint1'] == task['head_joint2'] == task['lift_joint'] == 0
    with pytest.raises(ValueError, match='initial_pose'):
        simulation_initial_positions(path.read_text(), 'legacy_sg2')


def test_only_simulated_finger_and_lift_stops_have_numerical_clearance(tmp_path, monkeypatch):
    robot_file = tmp_path / 'robot.urdf'
    robot_file.write_text('<robot name="original"/>')
    source = '''<sdf version="1.10"><model name="robot">
      <joint name="finger_r_joint4"><axis><limit><lower>0</lower><upper>1.57</upper>
        <effort>1000</effort><velocity>4.8</velocity></limit></axis></joint>
      <joint name="finger_l_joint4"><axis><limit><lower>-1.57</lower><upper>0</upper>
        <effort>1000</effort><velocity>4.8</velocity></limit></axis></joint>
      <joint name="arm_l_joint1"><axis><limit><lower>-3.14</lower><upper>3.14</upper>
        </limit></axis></joint>
      <joint name="lift_joint" type="prismatic"><axis><limit><lower>-0.5</lower><upper>0</upper>
        <effort>1000</effort><velocity>4.8</velocity></limit></axis></joint>
      <joint name="head_joint1"><axis><limit><lower>-0.2267</lower><upper>0.6901</upper>
        </limit></axis></joint>
    </model></sdf>'''
    monkeypatch.setattr('hx5_simulation.model.subprocess.run',
                        lambda *args, **kwargs: SimpleNamespace(stdout=source))
    root = ET.parse(simulation_model(robot_file, tmp_path)).getroot()
    right = root.find("model/joint[@name='finger_r_joint4']/axis/limit")
    left = root.find("model/joint[@name='finger_l_joint4']/axis/limit")
    arm = root.find("model/joint[@name='arm_l_joint1']/axis/limit")
    lift = root.find("model/joint[@name='lift_joint']/axis/limit")
    head = root.find("model/joint[@name='head_joint1']/axis/limit")
    assert float(right.findtext('lower')) == pytest.approx(-0.001)
    assert float(right.findtext('upper')) == pytest.approx(1.571)
    assert float(left.findtext('lower')) == pytest.approx(-1.571)
    assert float(left.findtext('upper')) == pytest.approx(0.001)
    assert right.findtext('effort') == '1000'
    assert right.findtext('velocity') == '4.8'
    assert arm.findtext('lower') == '-3.14'
    assert arm.findtext('upper') == '3.14'
    assert float(lift.findtext('lower')) == pytest.approx(-0.501)
    assert float(lift.findtext('upper')) == pytest.approx(0.001)
    assert lift.findtext('effort') == '1000'
    assert lift.findtext('velocity') == '4.8'
    assert head.findtext('lower') == '-0.2267'
    assert head.findtext('upper') == '0.6901'
    assert robot_file.read_text() == '<robot name="original"/>'
