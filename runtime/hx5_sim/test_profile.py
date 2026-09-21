from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[2]


def test_simulation_does_not_expose_robot_devices_or_old_workspace():
    path = ROOT / 'runtime/hx5_sim/compose.yaml'
    config = yaml.safe_load(path.read_text())
    assert 'hero_gazebo_ws' not in path.read_text()
    for service in config['services'].values():
        assert not service.get('privileged', False)
        assert not service.get('devices')
        assert service['ipc'] == 'private'
        assert service['environment']['ROS_DOMAIN_ID'] == '105'
        assert '7455' in service['environment']['ZENOH_CONFIG_OVERRIDE']
        assert not any(str(volume).startswith('/dev:') for volume in service['volumes'])


def test_simulation_configuration_preserves_original_hand_contract():
    original = yaml.safe_load((ROOT / 'src/cyclo_intelligence/shared/shared/robot_configs/ffw_sh5_rev1_config.yaml').read_text())
    simulation = yaml.safe_load((ROOT / 'runtime/hx5_sim/robot_configs/ffw_sh5_rev1_config.yaml').read_text())
    original = original['orchestrator']['ros__parameters']['ffw_sh5_rev1']
    simulation = simulation['orchestrator']['ros__parameters']['ffw_sh5_rev1']
    for key in ('observation', 'action'):
        assert simulation[key] == original[key]
    assert len(simulation['observation']['state']['arm_hand']['joint_names']) == 54
    for section in ('state', 'action'):
        for name, config in original['behavior_tree'][section].items():
            assert simulation['behavior_tree'][section][name] == config


def test_simulation_rotate_has_mobile_command_and_odometry_feedback():
    config = yaml.safe_load((ROOT / 'runtime/hx5_sim/robot_configs/ffw_sh5_rev1_config.yaml').read_text())
    channels = config['orchestrator']['ros__parameters']['ffw_sh5_rev1']['behavior_tree']
    assert channels['state']['mobile'] == {
        'topic': '/odom', 'msg_type': 'nav_msgs/msg/Odometry',
        'joint_names': ['linear_x', 'linear_y', 'angular_z'],
    }
    assert channels['action']['mobile'] == {
        'topic': '/cmd_vel', 'msg_type': 'geometry_msgs/msg/Twist',
        'joint_names': ['linear_x', 'linear_y', 'angular_z'],
    }


def test_simulation_mounts_bounded_service_cleanup():
    config = yaml.safe_load((ROOT / 'runtime/hx5_sim/compose.yaml').read_text())
    assert './ros_service_finish.sh:/usr/local/lib/s6-services/ros2_service_finish.sh:ro' in config['services']['cyclo']['volumes']


def test_leader_has_only_two_leader_devices_and_no_auto_launch():
    config = yaml.safe_load((ROOT / 'runtime/hx5_sim/leader-compose.yaml').read_text())
    leader = config['services']['leader']
    assert len(leader['devices']) == 2
    assert all('leader' in device and 'follower' not in device for device in leader['devices'])
    assert leader['command'] == ['-c', 'exec sleep infinity']


def test_policy_backends_replace_hardware_mounts_and_share_dedicated_simulation_transport():
    class PolicyLoader(yaml.SafeLoader):
        pass

    PolicyLoader.add_constructor('!override', lambda loader, node: loader.construct_sequence(node))
    text = (ROOT / 'runtime/hx5_sim/policies.yaml').read_text()
    config = yaml.load(text, Loader=PolicyLoader)
    # Without !override Compose would merge the official /dev mount back in.
    document = yaml.compose(text)
    services_node = next(value for key, value in document.value if key.value == 'services')
    for _, service_node in services_node.value:
        volumes_node = next(value for key, value in service_node.value if key.value == 'volumes')
        assert volumes_node.tag == '!override'
    for backend, service in config['services'].items():
        assert service['extends']['service'] == backend
        assert service['container_name'] == f'{backend}_server_1044_hx5_sim'
        assert service['ipc'] == 'private'
        assert service['environment']['ROS_DOMAIN_ID'] == '105'
        assert '7455' in service['environment']['ZENOH_CONFIG_OVERRIDE']
        assert not service.get('devices')
        assert not any(volume.startswith('/dev:') or volume.startswith('/dev/shm:') for volume in service['volumes'])
        assert '${HX5_SIM_ROOT:?}/simulation/cyclo:/workspace' in service['volumes']
        assert any(volume.endswith('/orchestrator_config/ffw_sh5_rev1_config.yaml:ro') for volume in service['volumes'])
