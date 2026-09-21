#!/usr/bin/env python3
"""Opt-in Isaac physical regression for continuously replaced LG2 trajectories.

Run with runtime/hx5_isaac/run_isaac.sh, never pytest or a physical leader.
There is no ROS/TCP connection, base command, teleport or saved-stage edit.
The reusable run_tracking_probe() can run after the timeline lifecycle probe
in the same SimulationApp. Every reported position comes from PhysX feedback.
"""
import argparse
import base64
import hashlib
import io
import json
import logging
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from control_math import GROUPS


class Replies:
    def __init__(self):
        self.messages = []

    def reply(self, message):
        self.messages.append(message)


def run_tracking_probe(runtime, frames=300):
    """Stream all 54 arm/hand targets before each step; validate real motion.

    Existing measured positions provide the hold targets. Only joint7 of each
    arm and finger6 of each hand moves by 0.08 rad within its official limits.
    Positive horizons reproduce the old freeze even if every frame replaces
    the preceding trajectory. Cameras stay enabled throughout the probe.
    """
    if not 240 <= frames <= 360:
        raise ValueError("Tracking smoke requires 240 to 360 physics frames")
    if not runtime.cameras:
        raise ValueError("Tracking smoke requires the four physical cameras")
    if any(abs(value) > 1e-12 for value in runtime.base_velocity):
        raise ValueError("Tracking smoke must start with zero base command")
    expected_cameras = {"head_left", "head_right", "wrist_left", "wrist_right"}
    if set(runtime.cameras) != expected_cameras:
        raise ValueError("Tracking smoke requires all four official camera names")
    from PIL import Image

    camera_samples = {}

    def step(frame):
        state = runtime.step(render=frame % 4 == 0)
        if state is None:
            raise RuntimeError("Tracking smoke requires a playing, initialized timeline")
        for name, image in state.get("images", {}).items():
            jpeg = base64.b64decode(image["jpeg"], validate=True)
            with Image.open(io.BytesIO(jpeg)) as decoded:
                decoded.load()
                size = list(decoded.size)
                if decoded.format != "JPEG" or size != [image["width"], image["height"]]:
                    raise AssertionError(f"Invalid physical camera JPEG: {name}")
            spec = runtime.cameras[name][1]
            if size != list(spec["resolution"]) or image["frame_id"] != spec["optical_frame"]:
                raise AssertionError(f"Camera does not match official metadata: {name}")
            previous = camera_samples.get(name)
            count = previous["samples"] + 1 if previous else 1
            camera_samples[name] = {
                "samples": count, "width": size[0], "height": size[1],
                "frame_id": image["frame_id"], "last_sim_time": image["time"],
                "first_sim_time": previous["first_sim_time"] if previous else image["time"],
                "jpeg_bytes": len(jpeg), "jpeg_sha256": hashlib.sha256(jpeg).hexdigest(),
            }
        return state

    # No native targets are replaced during settling. Let the existing drives
    # support gravity before choosing the measured starts for this test.
    for frame in range(60):
        state = step(frame)
    initial = dict(zip(state["joint_names"], state["positions"]))
    initial_base = list(state["base_position"])
    start_time = state["time"]
    plans = {}
    for group, selected in (("left", "arm_l_joint7"), ("right", "arm_r_joint7"),
                            ("left_hand", "finger_l_joint6"), ("right_hand", "finger_r_joint6")):
        names = GROUPS[group]
        targets = []
        adjusted_holds = {}
        for name in names:
            lower, upper = runtime.limits[name]
            target = min(upper, max(lower, initial[name]))
            targets.append(target)
            if target != initial[name]:
                adjusted_holds[name] = {"measured": initial[name], "bounded_target": target}
        lower, upper = runtime.limits[selected]
        position = initial[selected]
        if position + .08 <= upper and position + .08 >= lower:
            delta = .08
        elif lower <= position - .08 <= upper:
            delta = -.08
        else:
            raise AssertionError(f"No bounded 0.08-radian motion available for {selected}")
        targets[names.index(selected)] = position + delta
        plans[group] = {
            "names": names, "targets": targets, "selected": selected,
            "delta": delta, "horizon": .1 if group.endswith("_hand") else .2,
            "adjusted_hold_targets": adjusted_holds,
        }

    replies = Replies()
    history = []
    for frame in range(frames):
        # Deliberately replace every trajectory before every physical step;
        # querying a commanded target instead of actual feedback would hide
        # the original start-frame bug and is not used in these measurements.
        for group, plan in plans.items():
            runtime.command({
                "kind": "trajectory", "group": group,
                "joint_names": list(plan["names"]),
                "points": [{"positions": plan["targets"], "time_from_start": plan["horizon"]}],
            }, replies)
        if any(message.get("ok") is False for message in replies.messages):
            raise AssertionError(f"Trajectory rejected: {replies.messages}")
        state = step(frame + 60)
        measured = dict(zip(state["joint_names"], state["positions"]))
        if frame % 30 == 0 or frame == frames - 1:
            history.append({"frame": frame + 1, "time": state["time"],
                            "measured": {plan["selected"]: measured[plan["selected"]]
                                         for plan in plans.values()}})

    groups = {}
    for group, plan in plans.items():
        errors = {name: abs(measured[name] - target)
                  for name, target in zip(plan["names"], plan["targets"])}
        selected = plan["selected"]
        actual_delta = measured[selected] - initial[selected]
        signed_delta = math.copysign(1., plan["delta"]) * actual_delta
        maximum_error = max(errors.values())
        groups[group] = {
            "joint_count": len(plan["names"]), "selected_joint": selected,
            "initial_measured_rad": initial[selected], "target_rad": initial[selected] + plan["delta"],
            "final_measured_rad": measured[selected], "measured_delta_rad": actual_delta,
            "progress_in_target_direction_rad": signed_delta,
            "trajectory_horizon_seconds": plan["horizon"],
            "maximum_group_target_error_rad": maximum_error, "joint_errors_rad": errors,
            "adjusted_hold_targets": plan["adjusted_hold_targets"],
            "passed": signed_delta > .04 and maximum_error <= .03,
        }
    camera_passed = (set(camera_samples) == expected_cameras and
                     all(sample["samples"] >= 2 and sample["last_sim_time"] > sample["first_sim_time"]
                         for sample in camera_samples.values()))
    zero_base_command = all(abs(value) <= 1e-12 for value in runtime.base_velocity)
    return {
        "passed": all(group["passed"] for group in groups.values()) and camera_passed and zero_base_command,
        "streamed_physics_frames": frames, "settling_physics_frames": 60,
        "command_count": frames * len(plans), "physics_timestep_seconds": runtime.dt,
        "start_sim_time": start_time, "finish_sim_time": state["time"],
        "groups": groups, "cameras": camera_samples, "camera_payloads_passed": camera_passed,
        "zero_base_command": zero_base_command,
        "passive_base_displacement_m": math.dist(initial_base, state["base_position"]),
        "history": history,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    assets = Path(__file__).resolve().parents[3] / "simulation/isaac/assets/sh5"
    parser.add_argument("--stage", type=Path, default=assets / "logistics_sh5.usda")
    parser.add_argument("--metadata", type=Path, default=assets / "scene_metadata.json")
    parser.add_argument("--summary", type=Path, default=Path("/tmp/hx5-isaac-leader-tracking-smoke.json"))
    parser.add_argument("--frames", type=int, default=300)
    args, _kit_arguments = parser.parse_known_args()
    logging.basicConfig(level=logging.INFO)
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "fast_shutdown": True, "shutdown_watchdog_timeout": 30.})
    runtime = None
    result = {}
    code = 1
    try:
        from simulator import PhysicsRuntime
        metadata = json.loads(args.metadata.read_text())
        runtime = PhysicsRuntime(app, args.stage, metadata, cameras_enabled=True)
        result = run_tracking_probe(runtime, args.frames)
        code = 0 if result["passed"] else 1
    except Exception as error:
        logging.exception("Leader tracking smoke failed")
        result.update(passed=False, error=str(error))
    finally:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        args.summary.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        print(json.dumps(result, indent=2, allow_nan=False), flush=True)
        if runtime:
            runtime.hold("leader tracking smoke complete")
        app.close(wait_for_replicator=False, exit_code=code)


if __name__ == "__main__":
    main()
