#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Sungho Woo, Woojin Wie, Wonho Yun

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('model', default_value='ffw_sg2_rev1_follower',
                              description='Robot model name.'),
        DeclareLaunchArgument(
            'world',
            default_value='aws_small_warehouse',
            description='Gazebo Sim world file name from ffw_bringup/worlds.',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='small_warehouse',
            description='SDF world name used by Gazebo transport services.',
        ),
    ]

    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    world_name = LaunchConfiguration('world_name')
    initial_robot_x = '0.196735511165'
    initial_robot_y = '1.196589404083'
    initial_robot_z = '0.034224'
    initial_robot_yaw = '1.561340'
    initial_robot_orientation_z = '0.703755573384'
    initial_robot_orientation_w = '0.710442181272'

    ffw_description_path = os.path.join(
        get_package_share_directory('ffw_description'))

    ffw_bringup_path = os.path.join(
        get_package_share_directory('ffw_bringup'))

    warehouse_assets_path = os.path.join(
        ffw_bringup_path,
        'worlds',
        'aws_robomaker_small_warehouse',
    )

    # The AWS assets use both model://<name> and file://models/<path> URIs,
    # so expose the model directory and its parent to Gazebo Sim.
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(ffw_bringup_path, 'worlds'), ':',
            warehouse_assets_path, ':',
            os.path.join(warehouse_assets_path, 'models'), ':',
            str(Path(ffw_description_path).parent.resolve()),
        ],
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments=[
                    ('gz_args', [
                        world,
                        '.sdf',
                        ' -v 1',
                        # DART preserves the swerve wheel contact behavior used
                        # by the original default world. The AWS static
                        # Collada collisions are also supported by this setup.
                        ' --physics-engine gz-physics-dartsim-plugin',
                    ]),
                    ('on_exit_shutdown', 'true'),
                ]
             )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('ffw_description'),
                              'urdf',
                              model,
                              'ffw_sg2_follower.urdf.xacro']),
        ' ',
        'model:=', model,
        ' ',
        'use_sim:=true',
        ' ',
        'data_collection_initial_pose:=true',
        ' ',
        # Keep user-facing commands at their original limits while giving the
        # physics solver enough room to hold boundary poses without repeatedly
        # triggering controller-manager saturation.
        'lift_limit_margin:=0.05',
        ' ',
        'arm_joint2_limit_margin:=0.001',
        ' ',
        'gripper_lower_limit_margin:=0.05',
    ])

    robot_description = {'robot_description': robot_description_content}

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {
            'use_sim_time': True
        }],
        output='screen'
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-x', initial_robot_x,
                   '-y', initial_robot_y,
                   '-z', '0.2',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', initial_robot_yaw,
                   '-name', model,
                   '-allow_renaming', 'true',
                   '-use_sim', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['--inactive', 'joint_state_broadcaster'],
        output='screen'
    )

    # Multiple sources (leader teleoperation, BT actions, and policy nodes)
    # share the arm command topics. Validate them at the final simulation
    # boundary so no source can send an invalid target to controller_manager.
    sim_joint_trajectory_guard = Node(
        package='ffw_bringup',
        executable='sim_joint_trajectory_guard',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            '--inactive',
            '--controller-ros-args',
            '-r /arm_l_controller/joint_trajectory:='
            '/simulation/arm_l_controller/joint_trajectory',
            '--controller-ros-args',
            '-r /arm_r_controller/joint_trajectory:='
            '/simulation/arm_r_controller/joint_trajectory',
            '--controller-ros-args',
            '-r /head_controller/joint_trajectory:='
            '/leader/joystick_controller_left/joint_trajectory',
            '--controller-ros-args',
            '-r /lift_controller/joint_trajectory:='
            '/leader/joystick_controller_right/joint_trajectory',
            'arm_l_controller',
            'arm_r_controller',
            'head_controller',
            'lift_controller',
            'swerve_drive_controller',
        ],
        parameters=[robot_description],
    )

    # Keep physics paused while all controllers are loaded and configured.
    # Queue one group activation request before unpausing so the lift effort
    # controller claims its joint on the first controller-manager update.
    activate_controllers = ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'),
            'control',
            'switch_controllers',
            '--spin-time',
            '0.5',
            '--activate',
            'joint_state_broadcaster',
            'arm_l_controller',
            'arm_r_controller',
            'head_controller',
            'lift_controller',
            'swerve_drive_controller',
            '--strict',
            '--activate-asap',
        ],
        output='screen',
    )

    unpause_gazebo = ExecuteProcess(
        cmd=[
            FindExecutable(name='gz'),
            'service',
            '-s',
            PythonExpression([
                "'/world/' + '", world_name, "' + '/control'",
            ]),
            '--reqtype',
            'gz.msgs.WorldControl',
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            '5000',
            '--req',
            'pause: false',
        ],
        output='screen',
    )

    initial_camera_pose = ExecuteProcess(
        cmd=[
            FindExecutable(name='bash'),
            '-c',
            (
                'for attempt in $(seq 1 30); do '
                'if gz service -l | grep -qx /gui/move_to/pose; then '
                'gz topic -t /gui/track -m gz.msgs.CameraTrack '
                '-p "track_mode: NONE" >/dev/null 2>&1 || true; '
                'gz service -s /gui/move_to/pose '
                '--reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean '
                '--timeout 5000 --req '
                "'name: \"user_camera\", pose: {position: {x: 0.2414657325, "
                'y: 1.4090569019, z: 1.0779137611}, orientation: '
                '{x: -0.3267022669, y: 0.3273662925, z: 0.6262985468, '
                "w: 0.6275723577}}'; "
                'exit $?; '
                'fi; '
                'sleep 1; '
                'done; '
                'echo "Gazebo GUI camera service did not become ready." >&2; '
                'exit 1'
            ),
        ],
        output='screen',
    )

    initial_task_object_pose = ExecuteProcess(
        cmd=[
            FindExecutable(name='gz'),
            'service',
            '-s',
            PythonExpression([
                "'/world/' + '", world_name, "' + '/set_pose/blocking'",
            ]),
            '--reqtype',
            'gz.msgs.Pose',
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            '5000',
            '--req',
            (
                'name: "shelf_e_001_bottom_box_1", '
                'position: {x: 0.482531, y: 1.843682, z: 0.760000}, '
                'orientation: {x: 0.0, y: 0.7071067812, z: 0.0, '
                'w: 0.7071067812}'
            ),
        ],
        output='screen',
    )

    initial_robot_pose = ExecuteProcess(
        cmd=[
            FindExecutable(name='gz'),
            'service',
            '-s',
            PythonExpression([
                "'/world/' + '", world_name, "' + '/set_pose/blocking'",
            ]),
            '--reqtype',
            'gz.msgs.Pose',
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            '5000',
            '--req',
            [
                'name: "', model, '", position: {x: ', initial_robot_x,
                ', y: ', initial_robot_y, ', z: ', initial_robot_z,
                '}, orientation: {x: 0.0, y: 0.0, z: ',
                initial_robot_orientation_z, ', w: ',
                initial_robot_orientation_w, '}',
            ],
        ],
        output='screen',
    )

    initial_head_pose = Node(
        package='ffw_bringup',
        executable='joint_trajectory_executor',
        name='initial_head_pose_executor',
        output='screen',
        parameters=[{
            'joint_names': ['head_joint1', 'head_joint2'],
            'step_names': ['initial'],
            'initial': [0.6901, 0.0],
            'duration': 3.0,
            'position_tolerance': 0.01,
            'velocity_tolerance': 0.01,
            'action_topic': '/head_controller/follow_joint_trajectory',
            'joint_states_topic': '/joint_states',
        }],
    )

    initial_lift_pose = Node(
        package='ffw_bringup',
        executable='joint_trajectory_executor',
        name='initial_lift_pose_executor',
        output='screen',
        parameters=[{
            'joint_names': ['lift_joint'],
            'step_names': ['initial'],
            'initial': [-0.5],
            'duration': 5.0,
            'position_tolerance': 0.01,
            'velocity_tolerance': 0.01,
            'action_topic': '/lift_controller/follow_joint_trajectory',
            'joint_states_topic': '/joint_states',
        }],
    )

    initial_arm_l_pose = Node(
        package='ffw_bringup',
        executable='joint_trajectory_executor',
        name='initial_arm_l_pose_executor',
        output='screen',
        parameters=[{
            'joint_names': [
                'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
                'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6',
                'arm_l_joint7', 'gripper_l_joint1',
            ],
            'step_names': ['initial'],
            'initial': [
                0.04755340, 0.11044662, -0.18407769, -1.63215556,
                -0.01380583, -0.36201947, 0.15953400, 0.0,
            ],
            'duration': 5.0,
            'position_tolerance': 0.01,
            'velocity_tolerance': 0.01,
            'action_topic': '/arm_l_controller/follow_joint_trajectory',
            'joint_states_topic': '/joint_states',
        }],
    )

    initial_arm_r_pose = Node(
        package='ffw_bringup',
        executable='joint_trajectory_executor',
        name='initial_arm_r_pose_executor',
        output='screen',
        parameters=[{
            'joint_names': [
                'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
                'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6',
                'arm_r_joint7', 'gripper_r_joint1',
            ],
            'step_names': ['initial'],
            'initial': [
                0.04755340, -0.11044662, 0.18407769, -1.63215556,
                0.01380583, -0.36201947, -0.15953400, 0.0,
            ],
            'duration': 5.0,
            'position_tolerance': 0.01,
            'velocity_tolerance': 0.01,
            'action_topic': '/arm_r_controller/follow_joint_trajectory',
            'joint_states_topic': '/joint_states',
        }],
    )

    gz_bridge_params_path = os.path.join(
        ffw_bringup_path,
        'config',
        'common',
        'gz_bridge.yaml'
    )

    # Gazebo publishes the active world's clock under
    # /world/<world_name>/clock. Bridge it explicitly to ROS /clock so the
    # mapping remains valid when the world name is changed at launch time.
    gz_world_clock_topic = PythonExpression([
        "'/world/' + '", world_name, "' + '/clock'",
    ])

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='simulation_clock_bridge',
        arguments=[[
            gz_world_clock_topic,
            '@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ]],
        remappings=[(gz_world_clock_topic, '/clock')],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={gz_bridge_params_path}'],
        # The common config also contains an unscoped Gazebo `clock` bridge.
        # Keep it away from canonical /clock; this world publishes only the
        # world-scoped clock handled by clock_bridge above.
        remappings=[('/clock', '/gz/unscoped_clock')],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    # Publish simulation images under the same image_transport topic names used
    # by the physical ZED and RealSense drivers. This also provides /compressed
    # transports consumed by web_video_server and cyclo_data.
    camera_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='simulation_camera_image_bridge',
        arguments=[
            '/zed/left/image_raw',
            '/camera_left/color/image_raw',
            '/camera_right/color/image_raw',
        ],
        remappings=[
            ('/zed/left/image_raw', '/zed/zed_node/left/image_rect_color'),
            ('/zed/left/image_raw/compressed',
             '/zed/zed_node/left/image_rect_color/compressed'),
            ('/camera_left/color/image_raw',
             '/camera_left/camera_left/color/image_rect_raw'),
            ('/camera_left/color/image_raw/compressed',
             '/camera_left/camera_left/color/image_rect_raw/compressed'),
            ('/camera_right/color/image_raw',
             '/camera_right/camera_right/color/image_rect_raw'),
            ('/camera_right/color/image_raw/compressed',
             '/camera_right/camera_right/color/image_rect_raw/compressed'),
        ],
        parameters=[{
            'use_sim_time': True,
            'qos': 'sensor_data',
        }],
        output='screen'
    )

    # SG2 has one simulated head RGB stream. Mirror it to the right ZED preview
    # topic so existing stereo-camera consumers receive both expected topics.
    zed_right_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='simulation_zed_right_image_bridge',
        arguments=['/zed/left/image_raw'],
        remappings=[
            ('/zed/left/image_raw', '/zed/zed_node/right/image_rect_color'),
            ('/zed/left/image_raw/compressed',
             '/zed/zed_node/right/image_rect_color/compressed'),
        ],
        parameters=[{
            'use_sim_time': True,
            'qos': 'sensor_data',
        }],
        output='screen'
    )

    dual_laser_merger_node = Node(
        package='dual_laser_merger',
        executable='dual_laser_merger_node',
        output='screen',
        parameters=[{
            'laser_1_topic': '/scan_left',
            'laser_2_topic': '/scan_right',
            'merged_scan_topic': '/scan',
            'merged_cloud_topic': '/scan_cloud',
            'target_frame': 'base_link',
            'angle_min': -3.141592654,
            'angle_max': 3.141592654,
            'angle_increment': 0.006544985,
            'scan_time': 0.1,
            'range_min': 0.05,
            'range_max': 20.0,
            'use_inf': True,
            'tolerance': 0.05,
            'queue_size': 10,
            'enable_shadow_filter': True,
            'enable_average_filter': True,
        }, {
            'use_sim_time': True,
        }],
    )

    rviz_config_file = os.path.join(ffw_description_path, 'rviz', 'ffw_sg2.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        *declared_arguments,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=joint_state_broadcaster_spawner,
               on_exit=[robot_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_controller_spawner,
                on_exit=[activate_controllers],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=activate_controllers,
                on_start=[TimerAction(period=1.0, actions=[unpause_gazebo])],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=activate_controllers,
                on_exit=[
                    TimerAction(
                        period=1.0,
                        actions=[
                            initial_head_pose,
                            initial_lift_pose,
                            initial_arm_l_pose,
                            initial_arm_r_pose,
                        ],
                    ),
                    TimerAction(
                        period=8.0,
                        actions=[initial_task_object_pose],
                    ),
                ],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=initial_lift_pose,
                on_exit=[
                    TimerAction(period=1.0, actions=[initial_robot_pose]),
                ],
            )
        ),
        clock_bridge,
        bridge,
        camera_image_bridge,
        zed_right_image_bridge,
        dual_laser_merger_node,
        sim_joint_trajectory_guard,
        gazebo_resource_path,
        gazebo,
        TimerAction(period=1.0, actions=[initial_camera_pose]),
        robot_state_pub_node,
        gz_spawn_entity,
        rviz,
    ])
