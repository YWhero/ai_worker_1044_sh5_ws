"""Launch configurable previews independently of recording/image publishers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {'port': '7086', 'max_width': '424', 'jpeg_quality': '45',
                'preview_fps': '12.0'}
    arguments = [DeclareLaunchArgument(name, default_value=value)
                 for name, value in defaults.items()]
    parameters = {name: ParameterValue(LaunchConfiguration(name),
                                      value_type=float if name == 'preview_fps' else int)
                  for name in defaults}
    return LaunchDescription(arguments + [Node(
        package='cyclo_data', executable='camera_preview_node',
        name='camera_preview', output='screen', parameters=[parameters])])
