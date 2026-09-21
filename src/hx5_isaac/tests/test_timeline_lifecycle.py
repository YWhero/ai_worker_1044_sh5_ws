"""Exercise GUI timeline transitions without loading Isaac or commanding ROS."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulator import PhysicsRuntime


class World:
    def __init__(self):
        self.mode, self.current_time = 'playing', 12.
        self.steps = self.resets = self.renders = 0
        self.physics_sim_view = object()
        self.on_render = None

    def is_playing(self): return self.mode == 'playing'
    def is_stopped(self): return self.mode == 'stopped'

    def reset(self):
        self.resets += 1
        self.runtime.timeline_event(SimpleNamespace(type=self.runtime._timeline_stop))
        self.mode, self.current_time = 'playing', 0.
        self.physics_sim_view = object()
        self.runtime.robot.handles_initialized = True

    def step(self, render):
        assert self.is_playing() and render is False
        self.steps += 1
        self.current_time += 1/120

    def render(self):
        self.renders += 1
        if self.on_render: self.on_render()


@pytest.fixture
def runtime():
    value = PhysicsRuntime.__new__(PhysicsRuntime)
    value.world = World(); value.world.runtime = value
    value.np, value.dt, value.timeline = np, 1/120, object()
    value._timeline_stop, value._timeline_pause = 1, 2
    value._timeline_last_state = 'playing'
    value._needs_hard_reset = value._hold_after_pause = value._resetting = value._discard_pending_commands = False
    value.robot = Mock(); value.robot.handles_initialized = True
    def measured():
        assert value.robot.handles_initialized and not value.world.is_stopped(), 'read invalid STOP handles'
        return np.full(63, .15)
    value.robot.get_joint_positions.side_effect = measured
    value.robot.get_joint_velocities.return_value = np.zeros(63)
    value.robot.get_world_pose.return_value = (np.array([.36, -.728, 0.]), np.array([1., 0., 0., 0.]))
    value.robot.get_linear_velocity.return_value = value.robot.get_angular_velocity.return_value = np.zeros(3)
    value.initial_positions, value.target_positions = np.zeros(63), np.full(63, .9)
    value.initial_root_position, value.initial_root_orientation = np.array([.36, -.728, 0.]), np.array([1., 0., 0., 0.])
    value.trajectories, value.base_velocity, value.last_base_command = {'left': object()}, (.2, 0., 0.), 12.
    value.epoch, value.sequence, value.hold_count, value.last_hold_reason = 0, 9, 0, None
    value.stats = {'frames': 7, 'resets': 0, 'commands': 0}
    value.physics_scene = Mock()
    value.tip_views = {'tip': (Mock(), {})}
    value.dynamic_objects = Mock()
    value.object_positions, value.object_orientations = np.zeros((2, 3)), np.array([[1., 0., 0., 0.]]*2)
    spec = {'resolution': [672, 376], 'fx': 367., 'fy': 367., 'cx': 336., 'cy': 188.}
    value.cameras = {'head': (Mock(), spec)}
    value.camera_local_poses = {'head': (np.array([.01, .02, .03]), np.array([1., 0., 0., 0.]))}
    value.last_images = value.last_scan = 0.
    value.last_image_samples, value.last_scan_sample = {'old': object()}, {'old': object()}
    value.names = [f'joint{i}' for i in range(63)]
    value.drive, value.contacts, value.scan, value.images = Mock(), Mock(return_value={}), Mock(return_value={}), Mock(return_value={})
    value.shutdown_requested = False
    return value


def stopped(runtime):
    runtime.world.mode = 'stopped'
    runtime.robot.handles_initialized = False
    runtime.timeline_event(SimpleNamespace(type=runtime._timeline_stop))


def test_stop_suspends_queries_physics_and_feedback_without_automatic_play(runtime):
    stopped(runtime)
    assert runtime.sync_timeline() is False and runtime.step(render=True) is None
    assert runtime.world.mode == 'stopped' and runtime.world.steps == runtime.world.resets == 0
    assert runtime.stats['frames'] == 7 and runtime.sequence == 9
    assert not runtime.trajectories and runtime.base_velocity == (0., 0., 0.)
    runtime.drive.assert_not_called(); runtime.robot.get_joint_positions.assert_not_called()
    runtime.hold('disconnected while stopped')
    runtime.robot.get_joint_positions.assert_not_called()


def test_user_play_after_stop_rebuilds_views_and_restores_exact_initial_scene_once(runtime):
    spec = dict(runtime.cameras['head'][1])
    stopped(runtime); runtime.world.mode = 'playing'
    assert runtime.sync_timeline() is True and runtime.world.resets == 1
    assert runtime.epoch == 1 and runtime.sequence == 0 and runtime._needs_hard_reset is False
    assert np.array_equal(runtime.target_positions, runtime.initial_positions)
    runtime.robot.set_joint_positions.assert_called_once_with(runtime.initial_positions)
    runtime.robot.set_world_pose.assert_called_once_with(position=runtime.initial_root_position, orientation=runtime.initial_root_orientation)
    runtime.physics_scene.set_enabled_stabilization.assert_called_once_with(True)
    runtime.tip_views['tip'][0].initialize.assert_called_once_with(runtime.world.physics_sim_view)
    runtime.dynamic_objects.set_world_poses.assert_called_once_with(runtime.object_positions, runtime.object_orientations)
    assert np.array_equal(runtime.dynamic_objects.set_velocities.call_args.args[0], np.zeros((2, 6)))
    runtime.cameras['head'][0].post_reset.assert_called_once()
    assert runtime.cameras['head'][1] == spec
    assert np.array_equal(runtime.cameras['head'][0].set_local_pose.call_args.kwargs['translation'], runtime.camera_local_poses['head'][0])
    assert runtime.sync_timeline() is True and runtime.world.resets == 1


def test_pause_freezes_time_then_play_holds_measured_pose_without_default_reset(runtime):
    runtime.world.mode = 'paused'
    runtime.timeline_event(SimpleNamespace(type=runtime._timeline_pause))
    assert runtime.step(render=False) is None and runtime.world.current_time == 12.
    runtime.world.mode = 'playing'
    assert runtime.sync_timeline() is True and runtime.world.resets == 0
    assert np.array_equal(runtime.target_positions, np.full(63, .15))
    assert runtime.epoch == 0 and not runtime.trajectories and runtime.base_velocity == (0., 0., 0.)


def test_stop_inside_render_never_unpacks_none_or_publishes_a_false_fresh_frame(runtime):
    runtime.world.on_render = lambda: stopped(runtime)
    assert runtime.step(render=True) is None
    assert runtime.world.steps == 1 and runtime.world.renders == 1
    runtime.robot.get_world_pose.assert_not_called(); runtime.robot.get_joint_positions.assert_not_called()
    assert runtime.stats['frames'] == 7 and runtime.sequence == 9


def test_stop_and_play_between_polls_are_detected_by_event_even_if_current_state_is_playing(runtime):
    stopped(runtime); runtime.world.mode = 'playing'
    assert runtime.sync_timeline() is True and runtime.epoch == 1


@pytest.mark.parametrize('kind', ['trajectory', 'base_velocity', 'basevelocity', 'reset'])
def test_paused_motion_and_reset_requests_are_rejected_without_resuming(runtime, kind):
    stopped(runtime); server = Mock()
    assert runtime.command({'kind': kind, 'request_id': 'rejected'}, server) is False
    assert server.reply.call_args.args[0]['ok'] is False and 'Play' in server.reply.call_args.args[0]['message']
    assert runtime.world.mode == 'stopped' and runtime.world.resets == 0
    runtime.robot.get_joint_positions.assert_not_called()


def test_inactive_diagnostics_and_shutdown_ack_work_without_native_measurements(runtime):
    stopped(runtime); server = Mock()
    runtime.command({'kind': 'inspect', 'request_id': 'read'}, server)
    info = json.loads(server.reply.call_args.args[0]['message'])
    assert info['timeline_state'] == 'stopped' and info['physics_handles_valid'] is False
    assert 'measured_positions' not in info
    assert runtime.command({'kind': 'shutdown', 'request_id': 'close'}, server) is True
    assert server.reply.call_args.args[0]['ok'] and runtime.shutdown_requested
    runtime.robot.get_joint_positions.assert_not_called()


def test_recovery_discards_old_socket_motion_and_partial_buffers_but_keeps_diagnostics(runtime):
    stopped(runtime); runtime.world.mode = 'playing'
    server = Mock(incoming=bytearray(b'partial old trajectory'), latest_state={'time': 12.})
    def poll():
        assert server.incoming == bytearray()
        server.incoming.extend(b'incomplete old command')
        return [{'kind': 'trajectory', 'request_id': 'old'}, {'kind': 'inspect', 'request_id': 'read'}]
    server.poll.side_effect = poll
    playing, commands = runtime.poll_commands(server)
    assert playing and commands == [{'kind': 'inspect', 'request_id': 'read'}]
    assert server.latest_state is None and server.incoming == bytearray()
    assert server.reply.call_args.args[0]['ok'] is False and not runtime.trajectories


def test_unexpected_invalid_handles_while_playing_recover_through_world_reset(runtime):
    runtime.robot.handles_initialized = False
    assert runtime.sync_timeline() is True and runtime.world.resets == 1


def test_every_paused_poll_and_resume_clear_incomplete_or_queued_motion(runtime):
    runtime.world.mode = 'paused'
    runtime.timeline_event(SimpleNamespace(type=runtime._timeline_pause))
    server = Mock(incoming=bytearray(b'old partial'), latest_state={'time': 12.})
    def poll():
        assert server.incoming == bytearray()
        server.incoming.extend(b'new incomplete trajectory')
        return [{'kind': 'trajectory', 'request_id': 'queued'}, {'kind': 'inspect'}]
    server.poll.side_effect = poll
    for _ in range(2):
        playing, commands = runtime.poll_commands(server)
        assert not playing and commands == [{'kind': 'inspect'}]
        assert not server.incoming and server.latest_state is None
        server.incoming.extend(b'partial between inactive polls')
    runtime.world.mode = 'playing'
    playing, commands = runtime.poll_commands(server)
    assert playing and commands == [{'kind': 'inspect'}]
    assert not server.incoming and runtime.world.resets == 0
    assert server.reply.call_count == 3


def trajectory_runtime(runtime):
    runtime.names = ['arm_l_joint1', 'arm_l_joint2'] + [f'joint{i}' for i in range(61)]
    runtime.limits = {name: (-2., 2.) for name in runtime.names[:2]}
    runtime.trajectories.clear()
    return runtime


def trajectory_command(names=('arm_l_joint1',), positions=(1.,)):
    return {'kind': 'trajectory', 'group': 'left', 'joint_names': list(names),
            'points': [{'positions': list(positions), 'time_from_start': .2}], 'request_id': 'trajectory'}


def test_active_stream_replacement_starts_commanded_only_for_previously_controlled_joints(runtime):
    value, server = trajectory_runtime(runtime), Mock()
    value.command(trajectory_command(), server)
    assert value.trajectories['left'].points[0][1] == (.15,)
    value.target_positions[0] = .3  # Last physically applied drive setpoint.
    value.target_positions[1] = .8  # Unrelated old target must not seed a new joint.
    value.command(trajectory_command(('arm_l_joint1', 'arm_l_joint2'), (1., 1.)), server)
    assert server.reply.call_args.args[0]['ok'] is True
    assert value.trajectories['left'].points[0][1] == (.3, .15)
    assert np.array_equal(value.robot.get_joint_positions(), np.full(63, .15))
    value.robot.set_joint_positions.assert_not_called()


@pytest.mark.parametrize('boundary', ['hold', 'reset', 'stopped', 'paused'])
def test_control_boundaries_clear_stream_continuity_and_new_command_starts_measured(runtime, boundary):
    value, server = trajectory_runtime(runtime), Mock()
    value.command(trajectory_command(), server)
    value.target_positions[0] = .3
    if boundary == 'hold':
        value.hold('new control session')
    elif boundary == 'reset':
        value.reset()
    else:
        value.suspend_timeline(boundary)
    assert not value.trajectories
    value.target_positions[0] = .8  # Stale target after boundary is deliberately different.
    value.command(trajectory_command(), server)
    assert value.trajectories['left'].points[0][1] == (.15,)


def test_invalid_stream_replacement_leaves_prior_trajectory_and_targets_unchanged(runtime):
    value, server = trajectory_runtime(runtime), Mock()
    value.command(trajectory_command(), server)
    value.target_positions[0] = .3
    previous, targets, count = value.trajectories['left'], value.target_positions.copy(), value.stats['commands']
    value.command(trajectory_command(positions=(3.,)), server)
    assert server.reply.call_args.args[0]['ok'] is False
    assert value.trajectories['left'] is previous
    assert np.array_equal(value.target_positions, targets) and value.stats['commands'] == count
