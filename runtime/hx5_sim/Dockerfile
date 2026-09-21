FROM robotis/ai-worker@sha256:8fcd3e5305711c7d6a836f1ca17ce3c9ff82e0e31c9b175cdffb3fc67d5de3c2
SHELL ["/bin/bash", "-c"]
COPY src/ai_worker /root/ros2_ws/src/ai_worker
COPY src/robotis_hand/robotis_hand_description /root/ros2_ws/src/robotis_hand/robotis_hand_description
COPY src/robotis_interfaces /root/ros2_ws/src/robotis_interfaces
COPY src/hx5_simulation /root/ros2_ws/src/hx5_simulation
COPY src/hx5_contact_system /root/ros2_ws/src/hx5_contact_system
WORKDIR /root/ros2_ws
RUN source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
    colcon build --symlink-install --packages-select robotis_interfaces robotis_hand_description \
    ffw_description ffw_bringup ffw_navigation ffw_joystick_controller \
    ffw_joint_trajectory_command_broadcaster ffw_swerve_drive_controller hx5_simulation hx5_contact_system \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN printf '%s\n' 'source /opt/ros/jazzy/setup.bash' 'source /root/ros2_ws/install/setup.bash' > /root/.bashrc
COPY runtime/hx5_sim/ros_service.sh /usr/local/lib/s6-services/ros2_service_run.sh
COPY src/ai_worker/docker/s6-services/ai_worker_navigation/run /etc/s6-overlay/s6-rc.d/ai_worker_navigation/run
RUN chmod +x /usr/local/lib/s6-services/ros2_service_run.sh /etc/s6-overlay/s6-rc.d/ai_worker_navigation/run
COPY runtime/hx5_sim/router_run /etc/s6-overlay/s6-rc.d/hx5_zenoh_router/run
RUN chmod +x /etc/s6-overlay/s6-rc.d/hx5_zenoh_router/run && \
    echo longrun > /etc/s6-overlay/s6-rc.d/hx5_zenoh_router/type && \
    touch /etc/s6-overlay/s6-rc.d/user/contents.d/hx5_zenoh_router
