#!/usr/bin/env python3
"""Measure uncommanded pile startup/reset motion in native PhysX (opt-in)."""
import argparse
import json
import logging
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scene_builder import validate_kit_args
from simulator import PhysicsRuntime


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', required=True, type=Path)
    parser.add_argument('--metadata', required=True, type=Path)
    parser.add_argument('--summary', required=True, type=Path)
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--repeats', type=int, default=2)
    args, kit_args = parser.parse_known_args()
    validate_kit_args(kit_args)
    if args.seconds < 4 or args.repeats < 1:
        parser.error('Use at least 4 seconds and one repeat')
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'fast_shutdown': True})
    result, code = {'stage': str(args.stage), 'runs': [], 'passed': False}, 1
    runtime = None
    try:
        import numpy as np
        from pxr import Gf, UsdGeom
        metadata = json.loads(args.metadata.read_text())
        runtime = PhysicsRuntime(app, args.stage, metadata, cameras_enabled=False)
        result['scene_stabilization_enabled'] = runtime.stage.GetPrimAtPath(
            '/World/PhysicsScene').GetAttribute('physxScene:enableStabilization').Get()
        result['robot_stabilization_threshold'] = runtime.stage.GetPrimAtPath(
            metadata['articulation_root_path']).GetAttribute(
                'physxArticulation:stabilizationThreshold').Get()
        paths = list(runtime.dynamic_objects.prim_paths)
        kinds = {obj['prim_path']: obj['kind'] for obj in metadata['dynamic_objects']}
        cache = UsdGeom.XformCache()
        inverses = [cache.GetLocalToWorldTransform(runtime.stage.GetPrimAtPath(
            str(Path(path).parent.parent))).GetInverse() for path in paths]
        frames = round(args.seconds / runtime.dt)
        for repeat in range(args.repeats):
            if repeat:
                runtime.reset()
            initial, _ = runtime.dynamic_objects.get_world_poses()
            initial = initial.copy()
            previous_tail = None
            groups = {kind: {'count': sum(kinds[path] == kind for path in paths),
                             'peak_speed_m_s': 0., 'peak_upward_speed_m_s': 0.,
                             'max_rise_m': 0., 'escaped_paths': [],
                             'max_tail_drift_m': 0., 'final_peak_speed_m_s': 0.}
                      for kind in ('object1', 'object2')}
            for frame in range(frames):
                runtime.step(render=frame % 4 == 0)
                positions, _ = runtime.dynamic_objects.get_world_poses()
                velocities = runtime.dynamic_objects.get_velocities()[:, :3]
                if not np.isfinite(positions).all() or not np.isfinite(velocities).all():
                    raise AssertionError('Nonfinite native object pose/velocity')
                if frame == frames - round(2 / runtime.dt):
                    previous_tail = positions.copy()
                for index, path in enumerate(paths):
                    group = groups[kinds[path]]
                    speed = float(np.linalg.norm(velocities[index]))
                    group['peak_speed_m_s'] = max(group['peak_speed_m_s'], speed)
                    group['peak_upward_speed_m_s'] = max(group['peak_upward_speed_m_s'],
                                                         float(velocities[index, 2]))
                    group['max_rise_m'] = max(group['max_rise_m'],
                                             float(positions[index, 2] - initial[index, 2]))
                    local = inverses[index].Transform(Gf.Vec3d(*map(float, positions[index])))
                    if (abs(local[0]) > .19 or abs(local[1]) > .285
                            or local[2] < -.1 or local[2] > .8):
                        if path not in group['escaped_paths']:
                            group['escaped_paths'].append(path)
                    if frame == frames - 1:
                        group['final_peak_speed_m_s'] = max(group['final_peak_speed_m_s'], speed)
                        group['max_tail_drift_m'] = max(group['max_tail_drift_m'],
                            float(np.linalg.norm(positions[index] - previous_tail[index])))
            passed = all(group['max_rise_m'] < .25
                         and group['peak_upward_speed_m_s'] < 2
                         and not group['escaped_paths']
                         and group['max_tail_drift_m'] < .02
                         and group['final_peak_speed_m_s'] < .05 for group in groups.values())
            result['runs'].append({'repeat': repeat, 'seconds': args.seconds,
                                   'scene_stabilization_enabled': runtime.stage.GetPrimAtPath(
                                       '/World/PhysicsScene').GetAttribute(
                                           'physxScene:enableStabilization').Get(),
                                   'groups': groups, 'passed': passed})
            print(json.dumps(result['runs'][-1]), flush=True)
        result['passed'] = all(run['passed'] for run in result['runs'])
        code = 0 if result['passed'] else 1
    except Exception as error:
        result['error'] = str(error)
        logging.exception('Pile stability probe failed')
    finally:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        args.summary.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        if runtime:
            runtime.hold()
        app.close(exit_code=code)


if __name__ == '__main__':
    main()
