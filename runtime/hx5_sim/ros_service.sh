#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
if [ "${CYCLO_ROS_TRANSPORT_OVERLAY:-false}" = true ]; then
  source /workspace/ros_transport/install/local_setup.bash
fi
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
if [ -d /opt/venv/lib/python3.12/site-packages ]; then
  export PYTHONPATH="/opt/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
fi
printf '%s\n' "$(ps -o pgid= -p $$ | tr -d ' ')" > "/run/${SERVICE_NAME}.pgid"
exec bash --noprofile --norc -c "${ROS2_COMMAND}"
