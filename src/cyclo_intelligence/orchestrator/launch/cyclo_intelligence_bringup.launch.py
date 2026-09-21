#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Single-command bringup for cyclo_intelligence.

Launches orchestrator (+ rosbridge / rosbag_recorder / web_video_server)
and cyclo_data_node together so cyclo_manager / s6-agent can treat the
pair as one unit. ``cyclo_data`` and ``orchestrator`` aliases still work
when only one half needs to come up (debugging).
"""

import os
import fcntl

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable


_bringup_lock = None


def _guard_single_bringup(context):
    global _bringup_lock
    handle = open('/tmp/cyclo_intelligence_bringup.lock', 'a')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise RuntimeError('Cyclo Intelligence is already running. Reuse the existing '
                           'instance or stop it before launching again.')
    _bringup_lock = handle  # Retain the lock until this launch process exits.
    return []


def generate_launch_description():
    pkg_dir = get_package_share_directory('orchestrator')

    orchestrator_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'orchestrator_bringup.launch.py')
        )
    )

    cyclo_data_node = Node(
        package='cyclo_data',
        executable='cyclo_data_node',
        name='cyclo_data',
        output='screen',
    )

    return LaunchDescription([
        OpaqueFunction(function=_guard_single_bringup),
        SetParameter(name='use_sim_time', value=True,
                     condition=IfCondition(EnvironmentVariable('CYCLO_USE_SIM_TIME', default_value='false'))),
        orchestrator_bringup,
        cyclo_data_node,
    ])
