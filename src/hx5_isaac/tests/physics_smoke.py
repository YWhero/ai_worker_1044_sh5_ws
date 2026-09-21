#!/usr/bin/env python3
"""Opt-in physical integration test; run with Isaac python.sh, never pytest.

This creates one temporary process and never saves a stage. It proves that
wheel contact moves the measured floating base, joint drives follow a timed
trajectory, an existing physical object produces nonzero fingertip contact,
and reset restores every dynamic object's initial position. The object probe
uses a one-time relocation only in this diagnostic, never for robot motion.
"""
import argparse
import json
import logging
import math
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from simulator import PhysicsRuntime


class Replies:
    def __init__(self):
        self.messages=[]
    def reply(self,message):
        self.messages.append(message)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    assets=Path(__file__).resolve().parents[3]/"simulation/isaac/assets/sh5"
    parser.add_argument("--stage",type=Path,default=assets/"logistics_sh5.usda")
    parser.add_argument("--metadata",type=Path,default=assets/"scene_metadata.json")
    parser.add_argument("--summary",type=Path,default=Path("/tmp/hx5-isaac-physics-smoke.json"))
    args=parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    from isaacsim import SimulationApp
    app=SimulationApp({"headless":True,"fast_shutdown":True,"shutdown_watchdog_timeout":30.})
    runtime=None
    code=0
    result={}
    try:
        import numpy as np
        from pxr import Usd,UsdGeom
        metadata=json.loads(args.metadata.read_text())
        runtime=PhysicsRuntime(app,args.stage,metadata,cameras_enabled=False)
        replies=Replies()
        for frame in range(60):
            state=runtime.step(render=frame%4==0)
        start=np.array(state["base_position"])
        initial=dict(zip(state["joint_names"],state["positions"]))
        lower,upper=runtime.limits["head_joint1"]
        target=initial["head_joint1"]+(.08 if initial["head_joint1"]+.08<upper else -.08)
        runtime.command({"kind":"trajectory","group":"head","joint_names":["head_joint1"],
                         "points":[{"positions":[target],"time_from_start":.5}]},replies)
        if replies.messages and replies.messages[-1].get("ok") is False:
            raise AssertionError(replies.messages[-1])
        for frame in range(120):
            if frame%30==0:
                runtime.command({"kind":"basevelocity","vx":.1,"vy":0,"w":0},replies)
            state=runtime.step(render=frame%4==0)
        runtime.command({"kind":"stop"},replies)
        for frame in range(30):
            state=runtime.step(render=frame%4==0)
        actual=dict(zip(state["joint_names"],state["positions"]))
        finish=np.array(state["base_position"])
        yaw=math.atan2(2*(state["base_orientation"][3]*state["base_orientation"][2]),
                       1-2*state["base_orientation"][2]**2)
        delta=finish-start
        forward=math.cos(yaw)*delta[0]+math.sin(yaw)*delta[1]
        lateral=-math.sin(yaw)*delta[0]+math.cos(yaw)*delta[1]
        result.update(measured_forward_m=float(forward),measured_lateral_m=float(lateral),
                      measured_head_error_rad=abs(actual["head_joint1"]-target))
        if not .02 < forward < .2 or abs(lateral)>.03:
            raise AssertionError(f"Physical wheel motion failed: forward={forward}, lateral={lateral}")
        if abs(actual["head_joint1"]-target)>.02:
            raise AssertionError("Measured head did not reach timed trajectory target")

        if runtime.dynamic_objects is None:
            raise AssertionError("Scene has no physical probe object")
        # Update USD transforms for bounding boxes; forces still come from the
        # PhysX contact tensor, never from overlap/bounding-box calculations.
        runtime.world.render()
        # Collision meshes are deliberately hidden from rendering. Their
        # physics geometry remains present and must be included explicitly.
        cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),["default","render","proxy","guide"],
                               useExtentsHint=False,ignoreVisibility=True)
        sensor="finger_l_sensor2"
        tip=metadata["contact_tips"][sensor]
        tip_box=cache.ComputeWorldBound(runtime.stage.GetPrimAtPath(tip["collider_paths"][0])).ComputeAlignedRange()
        object_spec=metadata["dynamic_objects"][0]
        object_box=cache.ComputeWorldBound(runtime.stage.GetPrimAtPath(object_spec["collider_paths"][0])).ComputeAlignedRange()
        if tip_box.IsEmpty() or object_box.IsEmpty():
            raise AssertionError("Diagnostic collider bounds are empty")
        tip_center=np.array(tip_box.GetMidpoint())
        object_center=np.array(object_box.GetMidpoint())
        positions,_=runtime.dynamic_objects.get_world_poses()
        relocated=positions[0]+tip_center-object_center
        tip_view,_=runtime.tip_views[sensor]
        body_position,body_orientation=tip_view.get_world_poses()
        result.update(probe_geometry={"tip_bbox_center":tip_center.tolist(),
                                     "tip_measured_body_position":body_position[0].tolist(),
                                     "object_bbox_center":object_center.tolist(),
                                     "object_measured_root_position":positions[0].tolist(),
                                     "probe_target_root_position":relocated.tolist(),
                                     "object_view_paths":list(runtime.dynamic_objects.prim_paths)[:3]})
        runtime.dynamic_objects.set_world_poses(positions=relocated.reshape(1,3),indices=np.array([0]))
        runtime.dynamic_objects.set_velocities(np.zeros((1,6)),indices=np.array([0]))
        maximum=0.
        net_maximum=0.
        filtered_maximum=0.
        touched=set()
        for frame in range(30):
            state=runtime.step(render=frame%4==0)
            net_maximum=max(net_maximum,float(np.linalg.norm(tip_view.get_net_contact_forces(dt=runtime.dt))))
            filtered_maximum=max(filtered_maximum,float(np.linalg.norm(tip_view.get_contact_force_matrix(dt=runtime.dt))))
            for hand,sensors in state["contacts"].items():
                for name,sample in sensors.items():
                    if sample["valid"]:
                        force=sum(sample["forces"])
                        maximum=max(maximum,force)
                        if force>0:
                            touched.add(name)
        result.update(measured_external_contact_peak_newtons=maximum,contact_sensors=sorted(touched),
                      probe_tip_net_force_newtons=net_maximum,probe_tip_filtered_force_newtons=filtered_maximum,
                      diagnostic_probe_object=object_spec["prim_path"])
        if maximum<=1e-6:
            raise AssertionError("Real external object produced no measured fingertip contact")
        runtime.command({"kind":"reset","request_id":"physical-smoke-reset"},replies)
        restored,_=runtime.dynamic_objects.get_world_poses()
        object_error=float(np.max(np.abs(restored-runtime.object_positions)))
        reset_position,_=runtime.robot.get_world_pose()
        root_error=float(np.linalg.norm(reset_position-runtime.initial_root_position))
        result.update(reset_max_object_error_m=object_error,reset_base_error_m=root_error,
                      reset_ack=replies.messages[-1],stats=runtime.stats)
        if object_error>1e-5 or root_error>1e-5:
            raise AssertionError("Reset did not restore initial physics state")
        result["passed"]=True
    except Exception as error:
        code=1
        result.update(passed=False,error=str(error))
        logging.exception("Physical smoke failed")
    finally:
        args.summary.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
        print(json.dumps(result,indent=2),flush=True)
        if runtime:
            runtime.hold()
        app.close(wait_for_replicator=False,exit_code=code)


if __name__=="__main__":
    main()
