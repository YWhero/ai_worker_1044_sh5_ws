"""Isaac-only launch wrapper: all Cyclo transport nodes use the measured clock."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter


def generate_launch_description():
    os.environ.setdefault('CYCLO_CAMERA_PREVIEW_MAX_WIDTH', '1280')
    os.environ.setdefault('CYCLO_CAMERA_PREVIEW_JPEG_QUALITY', '95')
    os.environ.setdefault('CYCLO_CAMERA_PREVIEW_FPS', '12.0')
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        SetParameter(name='record_with_ros_clock', value=True),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('orchestrator'), 'launch', 'orchestrator_bringup.launch.py'))),
    ])
