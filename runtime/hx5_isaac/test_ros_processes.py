"""Check preset launch and failure cleanup without executing ROS processes."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import pytest
import yaml

SCRIPT = Path(__file__).with_name('ros_processes.py')
spec = importlib.util.spec_from_file_location('hx5_isaac_ros_processes', SCRIPT)
runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)
WORKSPACE = SCRIPT.parents[2]


def test_shared_presets_receive_official_description_and_persistent_isaac_path(tmp_path):
    urdf = WORKSPACE/'simulation/isaac/assets/sh5/robot.urdf'
    original = urdf.read_bytes()
    params = yaml.safe_load(runtime.write_parameters(urdf, tmp_path/'params.yaml').read_text())
    rsp = params['robot_state_publisher']['ros__parameters']
    presets = params['hx5_sim_hand_presets']['ros__parameters']
    assert presets['use_sim_time'] is True and rsp['use_sim_time'] is True
    assert presets['preset_file'] == '/workspace/hand_presets.json'
    assert presets['mapping_profile'] == '/hx5_isaac_runtime/official_1044_hand_mapping.json'
    assert presets['leader_hand_duration'] == 0.0
    assert isinstance(presets['leader_hand_duration'], float)
    assert presets['robot_description'] == rsp['robot_description']
    robot = ET.fromstring(presets['robot_description'])
    for side in ('l', 'r'):
        assert robot.find(f"link[@name='camera_{side}_color_optical_frame']") is not None
        for number in range(1, 21):
            joint = robot.find(f"joint[@name='finger_{side}_joint{number}']")
            assert joint is not None and joint.find('limit') is not None
    assert float(robot.find("joint[@name='finger_l_joint2']/limit").get('upper')) == 1.57
    assert float(robot.find("joint[@name='finger_r_joint2']/limit").get('lower')) == -1.57
    assert urdf.read_bytes() == original


def test_exactly_one_rsp_bridge_and_shared_preset_without_feedback_remaps():
    commands = runtime.process_commands('/tmp/config.yaml')
    assert len(commands) == 3
    assert sum('robot_state_publisher' in command for command in commands) == 1
    assert sum('/hx5_isaac/ros_bridge.py' in command for command in commands) == 1
    preset = next(command for command in commands if 'hand_presets' in command)
    assert preset[:4] == ['ros2', 'run', 'hx5_simulation', 'hand_presets']
    assert preset[-2:] == ['--params-file', '/tmp/config.yaml']
    assert '-r' not in preset


def test_failed_preset_spawn_cleans_started_rsp_and_bridge(monkeypatch):
    started = []
    class Child:
        def __init__(self, pid):
            self.pid = pid
            self.terminated = self.waited = False
        def poll(self):
            return None if not self.terminated else 0
        def wait(self, timeout=None):
            self.waited = True
    def popen(command, *, start_new_session):
        assert start_new_session is True
        if 'hand_presets' in command:
            raise OSError('missing preset executable')
        child = Child(900000 + len(started)); started.append(child)
        return child
    def killpg(pid, value):
        child = next(c for c in started if c.pid == pid)
        if value == signal.SIGTERM:
            child.terminated = True
        elif child.terminated:
            raise ProcessLookupError
    monkeypatch.setattr(runtime.signal, 'signal', lambda *args: None)
    monkeypatch.setattr(runtime.subprocess, 'Popen', popen)
    monkeypatch.setattr(runtime.os, 'killpg', killpg)
    with pytest.raises(OSError, match='preset executable'):
        runtime.supervise(runtime.process_commands('/tmp/config.yaml'))
    assert len(started) == 2
    assert all(child.terminated and child.waited for child in started)


def test_supervisor_sigterm_stops_real_child_and_grandchild(tmp_path):
    """A ros2-like wrapper does not forward TERM; group cleanup must reach its child."""
    tree = tmp_path / 'tree.py'
    ready = tmp_path / 'ready.json'
    tree.write_text('''import json,os,signal,subprocess,sys,time
child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'])
def stop(*_):
    child.wait(timeout=2)
    raise SystemExit(0)
signal.signal(signal.SIGTERM,stop)
with open(sys.argv[1],'w') as out:json.dump({'parent':os.getpid(),'grandchild':child.pid,'group':os.getpgrp()},out)
while True:time.sleep(.05)
''')
    code = '''import importlib.util,sys
spec=importlib.util.spec_from_file_location('runtime',sys.argv[1])
runtime=importlib.util.module_from_spec(spec);spec.loader.exec_module(runtime)
raise SystemExit(runtime.supervise([[sys.executable,sys.argv[2],sys.argv[3]]]))
'''
    supervisor = subprocess.Popen([sys.executable, '-c', code, str(SCRIPT), str(tree), str(ready)],
                                  start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    pids = None
    try:
        deadline = time.monotonic() + 5.
        while not ready.exists() and time.monotonic() < deadline:
            assert supervisor.poll() is None, supervisor.communicate()
            time.sleep(.02)
        assert ready.exists(), 'process-tree fixture did not start'
        pids = json.loads(ready.read_text())
        assert pids['group'] == pids['parent'] and pids['group'] != os.getpgrp()
        supervisor.send_signal(signal.SIGTERM)
        stdout, stderr = supervisor.communicate(timeout=7.)
        assert supervisor.returncode == 0, (stdout, stderr)
        for pid in (pids['parent'], pids['grandchild']):
            with pytest.raises(ProcessLookupError): os.kill(pid, 0)
    finally:
        if pids:
            try: os.killpg(pids['group'], signal.SIGKILL)
            except ProcessLookupError: pass
        if supervisor.poll() is None:
            supervisor.kill(); supervisor.wait(timeout=2.)


def test_group_cleanup_escalates_and_never_signals_parent(monkeypatch):
    calls = []
    class Child:
        pid = 900010
        def poll(self): return None
        def wait(self, timeout): assert timeout == 1.
    monkeypatch.setattr(runtime.os, 'killpg', lambda group, value: calls.append((group, value)))
    runtime.stop_process_groups([Child()], grace_sec=0.)
    assert calls == [(900010, signal.SIGTERM), (900010, signal.SIGKILL)]
    Child.pid = os.getpgrp()
    with pytest.raises(RuntimeError, match='parent process group'):
        runtime.stop_process_groups([Child()], grace_sec=0.)
    assert len(calls) == 2
