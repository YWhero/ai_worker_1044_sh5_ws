"""Supervise the measured bridge, one RSP and the shared SH5 hand preset API."""
import os
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

def write_parameters(urdf='/isaac_assets/robot.urdf', output='/tmp/hx5_isaac_rsp.yaml',
                     preset_file='/workspace/hand_presets.json',
                     mapping_profile='/hx5_isaac_runtime/official_1044_hand_mapping.json'):
    """Keep presets in the Isaac AI volume and reuse official joint limits."""
    root = ET.parse(urdf).getroot()
    # Optical coordinates: X right, Y down, Z forward. The mesh link uses
    # X forward, Y left, Z up. Author auxiliary frames in memory only.
    for side in ('l', 'r'):
        name = f'camera_{side}_color_optical_frame'
        if root.find(f"link[@name='{name}']") is not None:
            continue
        ET.SubElement(root, 'link', name=name)
        joint = ET.SubElement(root, 'joint', name=name + '_fixed', type='fixed')
        ET.SubElement(joint, 'parent', link=f'camera_{side}_link')
        ET.SubElement(joint, 'child', link=name)
        ET.SubElement(joint, 'origin', xyz='0.0038 0 0', rpy='-1.5707963267948966 0 -1.5707963267948966')
    description = ET.tostring(root, encoding='unicode')
    import yaml
    params = Path(output)
    params.write_text(yaml.safe_dump({
        'robot_state_publisher': {'ros__parameters': {
            'robot_description': description, 'use_sim_time': True, 'publish_frequency': 60.}},
        'hx5_sim_hand_presets': {'ros__parameters': {
            'robot_description': description, 'use_sim_time': True, 'preset_file': str(preset_file),
            'mapping_profile': str(mapping_profile),
            # Official 1044 preset_hand_controller publishes zero-duration
            # hand goals. Native Isaac drives retain mechanical velocity caps.
            'leader_hand_duration': 0.0}},
    }))
    return params


def process_commands(params):
    return [
        ['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
         '--ros-args', '--params-file', str(params)],
        ['python3', '/hx5_isaac/ros_bridge.py', '--ros-args', '-p', 'use_sim_time:=true'],
        # Reuse Gazebo's service/status protocol and preset implementation.
        # This node only moves hands on explicit preset/legacy gripper input.
        ['ros2', 'run', 'hx5_simulation', 'hand_presets',
         '--ros-args', '--params-file', str(params)],
    ]


def supervise(commands):
    children = []
    stopping = [False]
    def stop(*_):
        stopping[0] = True
    previous_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGTERM, signal.SIGINT)}
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        # Record each child before the next spawn, so failed preset startup
        # cannot leave an extra robot_state_publisher or bridge on retry.
        for command in commands:
            children.append(subprocess.Popen(command, start_new_session=True))
        while not stopping[0] and all(child.poll() is None for child in children):
            time.sleep(.1)
    finally:
        try:
            stop_process_groups(children)
        finally:
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)
    return 0 if stopping[0] else 1


def stop_process_groups(children, grace_sec=5.):
    """Stop ros2 CLI children and their ROS executables in our own sessions."""
    groups = {child.pid for child in children}
    def send(group, value):
        if group <= 0 or group == os.getpgrp():
            raise RuntimeError('refusing to signal the supervisor/parent process group')
        try:
            os.killpg(group, value)
            return True
        except ProcessLookupError:
            return False
    # Send even when the immediate child has already exited: its executable
    # can still be alive in the same session after ros2 run loses its parent.
    for group in groups:
        send(group, signal.SIGTERM)
    deadline = time.monotonic() + grace_sec
    while groups and time.monotonic() < deadline:
        for child in children:
            child.poll()  # Reap direct children while descendants shut down.
        groups = {group for group in groups if send(group, 0)}
        if groups:
            time.sleep(.05)
    for group in groups:
        send(group, signal.SIGKILL)
    for child in children:
        child.wait(timeout=1.)


def main():
    return supervise(process_commands(write_parameters()))


if __name__ == '__main__':
    raise SystemExit(main())
