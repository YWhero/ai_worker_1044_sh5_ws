#!/usr/bin/env python3
"""Opt-in native runtime profiling; source stage and ROS are untouched."""
import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulator import PhysicsRuntime, JsonLineServer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', type=Path, required=True)
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--summary', type=Path, required=True)
    parser.add_argument('--threads', type=int, default=32)
    parser.add_argument('--frames', type=int, default=240)
    parser.add_argument('--gui', action='store_true')
    args, _ = parser.parse_known_args()
    assert 1 <= args.threads <= 32 and 120 <= args.frames <= 600
    assert not args.summary.exists(), 'Use a fresh summary path'
    from isaacsim import SimulationApp
    app = SimulationApp(dict(headless=not args.gui, renderer='RaytracedLighting',
                             anti_aliasing=0, limit_cpu_threads=args.threads,
                             fast_shutdown=True, shutdown_watchdog_timeout=30.0))
    runtime, code, report = None, 0, {}
    try:
        import numpy as np
        metadata = json.loads(args.metadata.read_text())
        runtime = PhysicsRuntime(app, args.stage, metadata, cameras_enabled=True)
        for frame in range(120):
            assert runtime.step(render=frame % 4 == 0) is not None
        durations = {}

        def instrument(owner, name, label):
            original = getattr(owner, name)
            def wrapped(*positional, **keywords):
                start = time.perf_counter()
                try:
                    return original(*positional, **keywords)
                finally:
                    durations.setdefault(label, []).append(time.perf_counter()-start)
            setattr(owner, name, wrapped)

        for name in ['drive', 'contacts', 'scan', 'images']:
            instrument(runtime, name, name)
        instrument(runtime.world, 'step', 'physics_step')
        instrument(runtime.world, 'render', 'render')
        camera_counts = {name: 0 for name in runtime.cameras}
        started = time.perf_counter(); start_sim = runtime.sim_time
        for frame in range(args.frames):
            before = time.perf_counter()
            state = runtime.step(render=frame % 4 == 0)
            assert state is not None and np.isfinite(state['positions']).all()
            durations.setdefault('whole_step', []).append(time.perf_counter()-before)
            before = time.perf_counter()
            packet = JsonLineServer.encode(state)
            assert packet.endswith(b'\n')
            durations.setdefault('json_encode', []).append(time.perf_counter()-before)
            for name in state.get('images', {}):
                camera_counts[name] += 1
        wall = time.perf_counter()-started
        phases = {label: dict(calls=len(values), total_seconds=sum(values),
                              mean_ms=float(np.mean(values)*1000),
                              p95_ms=float(np.percentile(values, 95)*1000))
                  for label, values in durations.items()}
        report.update(passed=True, threads=args.threads, headless=not args.gui,
                      physics_frames=args.frames, elapsed_wall_seconds=wall,
                      elapsed_sim_seconds=runtime.sim_time-start_sim,
                      realtime_factor=(runtime.sim_time-start_sim)/wall,
                      phases=phases, camera_counts=camera_counts,
                      stats=runtime.stats, diagnostic_only=True)
    except Exception as error:
        code = 1; report.update(passed=False, error=str(error))
        raise
    finally:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        args.summary.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps(report, indent=2, allow_nan=False), flush=True)
        if runtime is not None:
            runtime.hold('profiling completed')
        app.close(wait_for_replicator=False, exit_code=code)


if __name__ == '__main__':
    main()
