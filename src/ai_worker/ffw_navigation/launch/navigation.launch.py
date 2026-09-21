# Copyright 2026 ROBOTIS CO., LTD.
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    IfElseSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    pkg_navigation = get_package_share_directory('ffw_navigation')
    os.environ.setdefault('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + pkg_navigation

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='navigation.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Flag to enable use_sim_time'
    )

    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Use SLAM instead of prebuilt map + AMCL'
    )

    localization_only_arg = DeclareLaunchArgument(
        'localization_only',
        default_value='false',
        description='Start map server and AMCL without the navigation stack'
    )

    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            pkg_navigation,
            'config',
            'navigation.yaml'
        ),
        description='Full path to the Nav2 params file (navigation + localization)'
    )

    base_params_path = LaunchConfiguration('params_file')
    simulation_params_path = RewrittenYaml(
        source_file=base_params_path,
        param_rewrites={
            'amcl.ros__parameters.update_min_d': '0.03',
            'amcl.ros__parameters.update_min_a': '0.08',
            'amcl.ros__parameters.resample_interval': '1',
            'amcl.ros__parameters.max_beams': '60',
            'amcl.ros__parameters.robot_model_type':
                'nav2_amcl::OmniMotionModel',
            'controller_server.ros__parameters.failure_tolerance': '1.0',
            'controller_server.ros__parameters.progress_checker.plugin':
                'nav2_controller::PoseProgressChecker',
            'controller_server.ros__parameters.progress_checker.'
            'required_movement_angle': '0.1',
            # stateful overrides removed 2026-08-12: with a tight (0.10 m)
            # xy_goal_tolerance, a non-stateful goal checker re-evaluates
            # position during the final rotation, and localization jitter
            # flipping in/out of tolerance caused in-place hunting. The yaml's
            # stateful: True must apply in sim as well.
            'controller_server.ros__parameters.FollowPath.desired_linear_vel':
                '0.3',
            'controller_server.ros__parameters.FollowPath.'
            'rotate_to_heading_min_angle': '0.8',
            'controller_server.ros__parameters.FollowPath.'
            'rotate_to_heading_angular_vel': '0.6',
            'bt_navigator.ros__parameters.navigate_to_pose.plugin':
                'ffw_navigation::SimTimeNavigateToPoseNavigator',
            'bt_navigator.ros__parameters.navigate_through_poses.plugin':
                'ffw_navigation::SimTimeNavigateThroughPosesNavigator',
        },
        convert_types=True,
    )
    effective_params_path = IfElseSubstitution(
        LaunchConfiguration('use_sim_time'),
        if_value=simulation_params_path,
        else_value=base_params_path,
    )

    localization_params_path = effective_params_path
    navigation_params_path = effective_params_path

    slam_params_path = os.path.join(
        pkg_navigation,
        'config',
        'mapper_params_online_sync.yaml'
    )

    map_file_path = os.path.join(
        pkg_navigation,
        'maps',
        'map.yaml'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Full path to the map yaml file'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            pkg_navigation, 'rviz', LaunchConfiguration('rviz_config')
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    real_localization_condition = IfCondition(AndSubstitution(
        NotSubstitution(LaunchConfiguration('use_sim_time')),
        NotSubstitution(LaunchConfiguration('use_slam')),
    ))
    simulation_localization_condition = IfCondition(AndSubstitution(
        LaunchConfiguration('use_sim_time'),
        NotSubstitution(LaunchConfiguration('use_slam')),
    ))

    real_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': localization_params_path,
                'map': LaunchConfiguration('map'),
        }.items(),
        condition=real_localization_condition,
    )

    simulation_localization_launch = GroupAction(
        actions=[
            SetRemap(src='/initialpose', dst='/initialpose_nav2'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_localization_launch_path),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': localization_params_path,
                    'map': LaunchConfiguration('map'),
                }.items(),
            ),
        ],
        condition=simulation_localization_condition,
    )

    initial_pose_stamp_adapter = Node(
        package='ffw_navigation',
        executable='initial_pose_stamp_adapter',
        name='initial_pose_stamp_adapter',
        parameters=[{'use_sim_time': True}],
        condition=simulation_localization_condition,
        output='screen',
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': navigation_params_path,
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('localization_only')),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, 'launch', 'online_sync_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': slam_params_path,
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_slam'))
    )

    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(use_slam_arg)
    launchDescriptionObject.add_action(localization_only_arg)
    launchDescriptionObject.add_action(params_file_arg)
    launchDescriptionObject.add_action(map_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(slam_launch)
    launchDescriptionObject.add_action(real_localization_launch)
    launchDescriptionObject.add_action(simulation_localization_launch)
    launchDescriptionObject.add_action(initial_pose_stamp_adapter)
    launchDescriptionObject.add_action(navigation_launch)

    return launchDescriptionObject
