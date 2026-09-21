import os
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.actions import SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hx5_simulation.model import CAMERAS, MODEL, SPAWN_X, SPAWN_Y, SPAWN_YAW
from hx5_simulation.model import build_description, build_world, simulation_model


def launch_setup(context):
    directory = Path(tempfile.mkdtemp(prefix='hx5_sim_', dir='/tmp'))
    robot_file, _ = build_description(directory, LaunchConfiguration('initial_pose').perform(context))
    model_file = simulation_model(robot_file, directory)
    world_file, world_name = build_world(directory)
    bringup = Path(get_package_share_directory('ffw_bringup'))
    resources = [str(bringup / 'worlds/aws_robomaker_small_warehouse/models'),
                 '/root/ros2_ws/install/ffw_description/share',
                 '/root/ros2_ws/src/robotis_hand', '/opt/ros/jazzy/share',
                 '/root/ros2_ws/install/realsense2_description/share']
    bridge_config = []

    def bridge(ros_topic, gz_topic, ros_type, gz_type):
        bridge_config.append({'ros_topic_name': ros_topic, 'gz_topic_name': gz_topic,
                              'ros_type_name': ros_type, 'gz_type_name': gz_type,
                              'direction': 'GZ_TO_ROS'})

    bridge('/clock', f'/world/{world_name}/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock')
    bridge('/simulation/world_odom', f'/model/{MODEL}/odometry',
           'nav_msgs/msg/Odometry', 'gz.msgs.Odometry')
    for side in ('left', 'right'):
        bridge(f'/scan_{side}', f'/scan_{side}', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan')
    for key, (topic, _, target) in CAMERAS.items():
        bridge(f'/simulation/camera/{key}', '/' + topic, 'sensor_msgs/msg/Image', 'gz.msgs.Image')
        info = target.rsplit('/', 1)[0] + '/camera_info'
        bridge(info, '/' + topic.replace('image_raw', 'camera_info'), 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo')
    for side in ('l', 'r'):
        for finger in range(1, 6):
            topic = f'/hx5/contact/{side}/finger{finger}'
            bridge(topic, topic, 'ros_gz_interfaces/msg/Contacts', 'gz.msgs.Contacts')
    bridge('/simulation/gazebo_tf', f'/model/{MODEL}/pose', 'tf2_msgs/msg/TFMessage', 'gz.msgs.Pose_V')
    bridge_file = directory / 'bridge.yaml'
    bridge_file.write_text(yaml.safe_dump(bridge_config))
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    spawn = Node(package='ros_gz_sim', executable='create', arguments=[
        '-file', str(model_file), '-name', MODEL, '-world', world_name,
        '-x', str(SPAWN_X), '-y', str(SPAWN_Y), '-z', '0.04', '-Y', str(SPAWN_YAW),
    ], output='screen')
    controllers = ['joint_state_broadcaster', 'arm_l_controller', 'arm_r_controller',
                   'hand_l_controller', 'hand_r_controller', 'head_controller', 'lift_controller',
                   'swerve_drive_controller']
    remaps = {'arm_l': 'joint_trajectory_command_broadcaster_left',
              'arm_r': 'joint_trajectory_command_broadcaster_right',
              'hand_l': 'joint_trajectory_command_broadcaster_left_hand',
              'hand_r': 'joint_trajectory_command_broadcaster_right_hand',
              'head': 'joystick_controller_left', 'lift': 'joystick_controller_right'}
    controller_args = controllers + ['--controller-manager-timeout', '90', '--activate-as-group']
    # Preserve wheel feedback for inspection, but only physics pose publishes
    # navigation odometry and odom -> base_link in this Gazebo profile.
    controller_args += ['--controller-ros-args', '-r /odom:=/simulation/wheel_odom']
    for name, target in remaps.items():
        destination = (f'/simulation/{name}_controller/joint_trajectory' if name.startswith('arm_')
                       else f'/leader/{target}/joint_trajectory')
        controller_args += ['--controller-ros-args',
                            f'-r /{name}_controller/joint_trajectory:={destination}']
    controller_node = Node(package='controller_manager', executable='spawner',
                           arguments=controller_args, output='screen')
    unpause = ExecuteProcess(cmd=[
        'gz', 'service', '-s', f'/world/{world_name}/control',
        '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000', '--req', 'pause: false',
    ], output='screen')
    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(resources)),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', '/root/ros2_ws/install/hx5_contact_system/lib'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': f'{world_file} -v 2 ' + ('' if gui else '-s --headless-rendering'),
                              'on_exit_shutdown': 'true'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_file.read_text(), 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(bridge_file), 'use_sim_time': True}], output='screen'),
        Node(package='dual_laser_merger', executable='dual_laser_merger_node',
             parameters=[{'use_sim_time': True, 'laser_1_topic': '/scan_left',
                          'laser_2_topic': '/scan_right', 'merged_scan_topic': '/scan',
                          'merged_cloud_topic': '/scan_cloud', 'target_frame': 'base_link',
                          'angle_min': -3.141592654, 'angle_max': 3.141592654,
                          'angle_increment': 0.006544985, 'scan_time': 0.1,
                          'range_min': 0.05, 'range_max': 20.0, 'use_inf': True}], output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[controller_node])),
        RegisterEventHandler(OnProcessStart(target_action=controller_node,
                                            on_start=[TimerAction(period=2.0, actions=[unpause])])),
        spawn,
        Node(package='hx5_simulation', executable='sim_io', parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='hx5_simulation', executable='hand_presets',
             parameters=[{'use_sim_time': True, 'robot_description': robot_file.read_text()}], output='screen'),
        Node(package='rviz2', executable='rviz2', parameters=[{'use_sim_time': True}],
             arguments=['-d', str(Path(get_package_share_directory('ffw_description')) / 'rviz/ffw_sh5.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('initial_pose', default_value='inference',
                             choices=['inference', 'navigation', 'task'],
                             description='ViTacFormer sync snapshot with head down and lift up, '
                                         'or official SH5 navigation/task pose'),
        OpaqueFunction(function=launch_setup),
    ])
