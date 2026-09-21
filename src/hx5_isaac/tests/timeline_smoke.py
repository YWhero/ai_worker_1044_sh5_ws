#!/usr/bin/env python3
"""Opt-in native timeline/camera/streaming probe; run with Isaac python.sh.

One bounded SimulationApp, no ROS/TCP server and no source-stage writes.
GUI Pause holds the current scene; STOP waits for explicit Play, which restores
the configured initial scene and rebuilds PhysX/contact views. The final probe
streams small bounded joint targets to this temporary simulation only.
"""
import json
import logging
from pathlib import Path
import sys
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulator import PhysicsRuntime, arguments, json_safe


def check_state(runtime, state):
    np = runtime.np
    assert state is not None, "Expected fresh playing physics feedback"
    assert len(state['positions']) == len(state['velocities']) == 63
    for field in ('positions', 'velocities', 'base_position', 'base_orientation',
                  'base_linear_velocity', 'base_angular_velocity'):
        assert np.isfinite(state[field]).all(), field
    assert len(runtime.tip_views) == 10
    for view, _ in runtime.tip_views.values():
        assert view.is_physics_handle_valid(), "Fingertip view did not recover"
    samples = [sample for sensors in state['contacts'].values() for sample in sensors.values()]
    assert len(samples) == 10 and all(sample['valid'] for sample in samples)
    return state


def camera_snapshot(runtime):
    np = runtime.np
    assert len(runtime.cameras) == 4
    result = {}
    for name, (camera, spec) in runtime.cameras.items():
        cx, cy, fx, fy, _ = camera.get_opencv_pinhole_properties()
        calibration = np.array([cx, cy, fx, fy], dtype=float)
        assert np.isfinite(calibration).all()
        assert np.allclose(calibration, [spec[k] for k in ('cx', 'cy', 'fx', 'fy')], atol=1e-6)
        pixels = camera.get_rgb()
        assert pixels is not None, f'{name}: no native rendered RGB'
        pixels = np.asarray(pixels)
        width, height = spec['resolution']
        assert pixels.shape == (height, width, 3), (name, pixels.shape)
        assert np.isfinite(pixels).all()
        local_position, local_orientation = camera.get_local_pose(camera_axes='usd')
        result[name] = dict(k=calibration.tolist(), resolution=[width, height],
                            rgb_shape=list(pixels.shape), local_position=local_position.tolist(),
                            local_orientation=local_orientation.tolist())
    return result


def compare_cameras(runtime, before, after):
    np = runtime.np
    assert set(before) == set(after)
    for name in before:
        a, b = before[name], after[name]
        assert a['resolution'] == b['resolution'] and a['rgb_shape'] == b['rgb_shape']
        assert np.allclose(a['k'], b['k'], atol=1e-6), name
        assert np.allclose(a['local_position'], b['local_position'], atol=1e-6), name
        qa, qb = np.array(a['local_orientation']), np.array(b['local_orientation'])
        assert min(np.linalg.norm(qa-qb), np.linalg.norm(qa+qb)) < 1e-6, name


def change_timeline(runtime, operation):
    getattr(runtime.timeline, operation)()
    runtime.timeline.commit()
    # Official World.render updates GUI/events while disabling physics stepping.
    runtime.world.render()


def warm(runtime, frames=120):
    state = None
    for frame in range(frames):
        state = check_state(runtime, runtime.step(render=frame % 4 == 0))
    return state


def run_lifecycle_probe(runtime):
    np = runtime.np
    warm(runtime, 240)
    original_cameras = camera_snapshot(runtime)
    original_epoch = runtime.epoch
    change_timeline(runtime, 'pause')
    assert not runtime.sync_timeline() and not runtime.world.is_playing()
    frozen_time, frames, sequence = runtime.sim_time, runtime.stats['frames'], runtime.sequence
    paused_pose = runtime.robot.get_joint_positions().copy()
    for _ in range(5):
        assert runtime.step(render=True) is None
        runtime.world.render()
        assert runtime.sim_time == frozen_time and not runtime.world.is_playing()
    assert runtime.stats['frames'] == frames and runtime.sequence == sequence
    assert runtime.epoch == original_epoch
    change_timeline(runtime, 'play')
    assert runtime.sync_timeline() and runtime.epoch == original_epoch
    assert np.allclose(runtime.target_positions, paused_pose, atol=1e-6)
    check_state(runtime, runtime.step(render=True))

    change_timeline(runtime, 'stop')
    assert not runtime.sync_timeline() and runtime.world.is_stopped()
    assert not runtime.robot.handles_initialized
    frozen_time, frames, sequence = runtime.sim_time, runtime.stats['frames'], runtime.sequence
    for _ in range(5):
        runtime.hold('native smoke disconnected while stopped')
        assert runtime.step(render=True) is None
        runtime.world.render()
        assert runtime.world.is_stopped() and runtime.sim_time == frozen_time
    assert runtime.stats['frames'] == frames and runtime.sequence == sequence
    assert 'measured_positions' not in runtime.inspect()
    change_timeline(runtime, 'play')
    assert runtime.sync_timeline() and runtime.epoch == original_epoch+1
    assert runtime.robot.handles_initialized
    joint_error = float(np.max(np.abs(runtime.robot.get_joint_positions()-runtime.initial_positions)))
    root_position, root_orientation = runtime.robot.get_world_pose()
    root_error = float(np.max(np.abs(root_position-runtime.initial_root_position)))
    assert joint_error < 1e-5 and root_error < 1e-5
    assert abs(float(np.dot(root_orientation, runtime.initial_root_orientation))) > 1-1e-6
    object_error = 0.0
    if runtime.dynamic_objects is not None:
        positions, orientations = runtime.dynamic_objects.get_world_poses()
        object_error = float(np.max(np.abs(positions-runtime.object_positions)))
        assert object_error < 1e-5
        assert np.all(np.abs(np.sum(orientations*runtime.object_orientations, axis=1)) > 1-1e-6)
    assert runtime.stage.GetPrimAtPath('/World/PhysicsScene').GetAttribute(
        'physxScene:enableStabilization').Get() is True
    warm(runtime)
    recovered_cameras = camera_snapshot(runtime)
    compare_cameras(runtime, original_cameras, recovered_cameras)

    # STOP+PLAY can both arrive before the main loop next observes timeline state.
    runtime.timeline.stop()
    runtime.timeline.play()
    runtime.timeline.commit()
    runtime.world.render()
    assert runtime.sync_timeline() and runtime.epoch == original_epoch+2
    warm(runtime)
    compare_cameras(runtime, original_cameras, camera_snapshot(runtime))
    return dict(passed=True, pause_frames_published=0, stop_frames_published=0,
                explicit_play_recoveries=2, reset_joint_error_rad=joint_error,
                reset_root_error_m=root_error, reset_object_error_m=object_error,
                cameras_before=original_cameras, cameras_after=recovered_cameras,
                native_dofs=63, valid_contact_views=10, stabilization_enabled=True)


def main():
    args = arguments()  # parse_known_args preserves run_isaac.sh Kit arguments.
    summary = args.summary or Path('/tmp/hx5-isaac-timeline-smoke.json')
    if summary.exists():
        raise FileExistsError(f'Use a fresh --summary path: {summary}')
    logging.basicConfig(level=logging.INFO)
    from isaacsim import SimulationApp
    app = SimulationApp(dict(headless=args.headless, renderer='RaytracedLighting',
                             anti_aliasing=0, fast_shutdown=True, shutdown_watchdog_timeout=30.0))
    report, runtime, code = {}, None, 0
    try:
        metadata = json.loads(args.metadata.read_text())
        runtime = PhysicsRuntime(app, args.stage, metadata, cameras_enabled=True)
        report['lifecycle'] = run_lifecycle_probe(runtime)
        from leader_tracking_smoke import run_tracking_probe
        report['streamed_joint_tracking'] = run_tracking_probe(runtime, frames=300)
        assert report['streamed_joint_tracking']['passed'], 'Native streamed joint tracking failed'
        report.update(passed=True, stats=runtime.stats, scope='temporary native simulation; no ROS or source USD writes')
    except Exception as error:
        code = 1
        report.update(passed=False, error=str(error), traceback=traceback.format_exc())
        logging.exception('Native timeline smoke failed')
    finally:
        summary.parent.mkdir(parents=True, exist_ok=True)
        with summary.open('x') as stream:
            json.dump(json_safe(report), stream, indent=2, allow_nan=False)
            stream.write('\n')
        print(json.dumps(json_safe(report), indent=2, allow_nan=False), flush=True)
        if runtime is not None:
            runtime.hold('native timeline smoke finished')
        app.close(wait_for_replicator=False, exit_code=code)


if __name__ == '__main__':
    main()
