#!/usr/bin/env python3
"""Bounded native PhysX PD/TGS sweep; no ROS and no source USD writes."""
import argparse
import json
import logging
import math
import hashlib
import shutil
import tempfile
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from simulator import PhysicsRuntime,json_safe
from control_math import GROUPS


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    assets=Path(__file__).resolve().parents[3]/"simulation/isaac/assets/sh5"
    parser.add_argument("--stage",type=Path,default=assets/"logistics_sh5.usda")
    parser.add_argument("--metadata",type=Path,default=assets/"scene_metadata.json")
    parser.add_argument("--summary",type=Path,default=Path("/tmp/hx5-isaac-pd-sweep.json"))
    parser.add_argument("--frames",type=int,default=120)
    args=parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    from isaacsim import SimulationApp
    app=SimulationApp({"headless":True,"fast_shutdown":True,"shutdown_watchdog_timeout":30.})
    result={"cases":[]}
    code=0
    try:
        import numpy as np
        from pxr import PhysxSchema,Usd
        source_hash=hashlib.sha256(args.stage.read_bytes()).hexdigest()
        scratch=Path(tempfile.mkdtemp(prefix="hx5-isaac-pd-"))/"stage.usda"
        shutil.copyfile(args.stage,scratch)
        runtime=PhysicsRuntime(app,scratch,json.loads(args.metadata.read_text()),cameras_enabled=False)
        runtime.stage.SetEditTarget(Usd.EditTarget(runtime.stage.GetSessionLayer()))
        scene_api=PhysxSchema.PhysxSceneAPI(runtime.stage.GetPrimAtPath("/World/PhysicsScene"))
        result.update(source_stage_sha256=source_hash,scratch_stage=str(scratch))
        controller=runtime.robot.get_articulation_controller()
        baseline_kp,baseline_kd=controller.get_gains()
        baseline_kp,baseline_kd=np.array(baseline_kp).copy(),np.array(baseline_kd).copy()
        hand_indices=np.array([runtime.indices[name] for group in ("left_hand","right_hand") for name in GROUPS[group]])
        selected=[runtime.indices[name] for group in GROUPS.values() for name in group]
        result["initial_native_diagnostics"]=runtime.inspect()
        cases=[(320.,damping,velocity_iters,False) for velocity_iters in (8,4,1) for damping in (32.,8.,2.)]
        cases += [(80.,8.,1,False),(160.,16.,1,False),(320.,32.,0,False),
                  (320.,32.,1,True),(80.,8.,1,True),(320.,32.,0,True)]
        for stiffness,damping,velocity_iters,external_every_iter in cases:
            scene_api.CreateEnableExternalForcesEveryIterationAttr(external_every_iter)
            runtime.reset()
            runtime.robot.set_solver_position_iteration_count(64)
            runtime.robot.set_solver_velocity_iteration_count(velocity_iters)
            kp,kd=baseline_kp.copy(),baseline_kd.copy()
            # Raw USD angular coefficients convert to native radian units.
            kp[hand_indices]=stiffness*180/math.pi
            kd[hand_indices]=damping*180/math.pi
            controller.set_gains(kps=kp,kds=kd,save_to_usd=False)
            errors,velocities=[],[]
            tactile_peak=0.
            for frame in range(args.frames):
                state=runtime.step(render=frame%4==0)
                if frame>=args.frames//2:
                    errors.append(np.array(state["positions"])-runtime.initial_positions)
                    velocities.append(np.array(state["velocities"]))
                tactile_peak=max(tactile_peak,max(sum(tip.get("forces",[])) for sensors in state["contacts"].values() for tip in sensors.values()))
            errors,velocities=np.array(errors),np.array(velocities)
            joints={name:{"mean_error":float(errors[:,runtime.indices[name]].mean()),
                          "max_abs_error":float(np.abs(errors[:,runtime.indices[name]]).max()),
                          "error_min":float(errors[:,runtime.indices[name]].min()),
                          "error_max":float(errors[:,runtime.indices[name]].max()),
                          "rms_velocity":float(np.sqrt((velocities[:,runtime.indices[name]]**2).mean())),
                          "max_abs_velocity":float(np.abs(velocities[:,runtime.indices[name]]).max())}
                    for names in GROUPS.values() for name in names}
            worst=max(joints,key=lambda name:joints[name]["max_abs_error"])
            native_kp,native_kd=controller.get_gains()
            case={"raw_finger_stiffness":stiffness,"raw_finger_damping":damping,
                  "position_iterations":int(runtime.robot.get_solver_position_iteration_count()),
                  "velocity_iterations":int(runtime.robot.get_solver_velocity_iteration_count()),
                  "native_thumb_stiffness":float(native_kp[runtime.indices["finger_l_joint1"]]),
                  "native_thumb_damping":float(native_kd[runtime.indices["finger_l_joint1"]]),
                  "external_forces_every_iteration":external_every_iter,
                  "worst_joint":worst,"worst_error":joints[worst]["max_abs_error"],
                  "all57":joints,"tactile_peak_newtons":tactile_peak,
                  "base_position":state["base_position"],"simulation_time":state["time"]}
            result["cases"].append(case)
            args.summary.write_text(json.dumps(json_safe(result),indent=2,allow_nan=False)+"\n")
            print(json.dumps({key:case[key] for key in case if key!="all57"})+" thumb="+json.dumps(joints["finger_l_joint1"]),flush=True)
        result["completed"]=True
        result["source_stage_unchanged"]=hashlib.sha256(args.stage.read_bytes()).hexdigest()==source_hash
    except Exception as error:
        result.update(completed=False,error=str(error));code=1
        logging.exception("Native PD sweep failed")
    finally:
        args.summary.write_text(json.dumps(json_safe(result),indent=2,allow_nan=False)+"\n")
        app.close(wait_for_replicator=False,exit_code=code)


if __name__=="__main__":main()
