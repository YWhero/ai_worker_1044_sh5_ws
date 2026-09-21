#!/usr/bin/env python3
"""Opt-in diagnostic of real scan retention by HX5 pads, never run by pytest.

One temporary Isaac app; no ROS connection, saved USD edits, grasp attachment,
kinematic payload, gravity removal or post-placement payload pose corrections.
Relocates one existing scan per trial onto a temporary support plate, closes
the hand through real drives, removes support, and attempts a real arm lift.
Failed trials remain evidence, not an automatic claim of a physics defect.
"""
import argparse
import json
import logging
import math
from pathlib import Path
import sys
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import scene_builder as builder
from simulator import PhysicsRuntime, json_safe
from control_math import GROUPS

OFFICIAL_SOURCE = 'https://raw.githubusercontent.com/ROBOTIS-GIT/cyclo_lab/main/source/cyclo_lab/cyclo_lab/assets/robots/FFW_SH5.py'


class Replies:
    def __init__(self): self.messages = []
    def reply(self, message):
        self.messages.append(message)
        if message.get('ok') is False: raise ValueError(message)


def rotation(np, quaternion):
    w, x, y, z = np.asarray(quaternion, dtype=float)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def transform(description, link, positions, root=None):
    chain, current = [], link
    parents = {joint['child']: (name, joint) for name, joint in description['joints'].items()}
    while current in parents and (root is None or current != root):
        name, joint = parents[current]; chain.append((name, joint)); current = joint['parent']
    result = ((0., 0., 0.), (1., 0., 0., 0.))
    for name, joint in reversed(chain):
        value, axis = positions.get(name, 0.), joint['axis']
        motion = ((tuple(value*x for x in axis), (1., 0., 0., 0.)) if joint['type'] == 'prismatic' else
                  ((0., 0., 0.), (math.cos(value/2), *(math.sin(value/2)*x for x in axis))))
        result = builder.compose(result, builder.compose(joint['origin'], motion))
    return result


def hand_values(profile, side, curl):
    letter = 'l' if side == 'left' else 'r'
    return {f'finger_{letter}_joint{i+1}': a+curl*(b-a)
            for i, (a, b) in enumerate(zip(profile[side]['release'], profile[side]['grasp']))}


def pinch_geometry(np, description, profile, side, thickness):
    letter, samples = ('l' if side == 'left' else 'r'), []
    for curl in np.linspace(.35, 1., 131):
        values = hand_values(profile, side, float(curl))
        centers = [np.array(transform(description, f'finger_end_{letter}_link{i}', values,
                                      f'hx5_d20_{side}_base')[0]) for i in (1, 2)]
        gap = float(np.linalg.norm(centers[1]-centers[0]))
        samples.append((abs(gap-(thickness+.008)), float(curl), gap, centers))
    _, curl, gap, (thumb, index) = min(samples, key=lambda sample: sample[0])
    normal = (index-thumb)/gap
    long_axis = np.array([1., 0., 0.])-normal*normal[0]
    long_axis /= np.linalg.norm(long_axis)
    frame = np.column_stack((long_axis, np.cross(normal, long_axis), normal))
    return (thumb+index)/2, frame, dict(placement_reference_curl=curl,
        taxel_center_gap_m=gap, thumb_center_palm_m=thumb.tolist(), index_center_palm_m=index.tolist())


def scan_geometry(runtime, obj):
    from pxr import UsdGeom
    np = runtime.np
    cache = UsdGeom.XformCache()
    root = runtime.stage.GetPrimAtPath(obj['prim_path'])
    inverse = cache.GetLocalToWorldTransform(root).GetInverse()
    points = []
    for path in obj['collider_paths']:
        mesh = runtime.stage.GetPrimAtPath(path)
        vertices = np.asarray(UsdGeom.Mesh(mesh).GetPointsAttr().Get(), dtype=float)
        local = np.asarray(cache.GetLocalToWorldTransform(mesh)*inverse, dtype=float)
        points.append((np.column_stack((vertices, np.ones(len(vertices)))) @ local)[:, :3])
    points = np.concatenate(points)
    mean = points.mean(axis=0)
    _, axes = np.linalg.eigh(np.cov((points-mean).T))
    axes = axes[:, ::-1]
    if np.linalg.det(axes) < 0: axes[:, 1] *= -1
    projected = (points-mean) @ axes
    lo, hi = projected.min(axis=0), projected.max(axis=0)
    center = mean+axes @ ((lo+hi)/2)
    return points, axes, center, hi-lo


def set_profile(runtime, profile_name, defaults):
    from pxr import PhysxSchema, UsdPhysics, UsdShade
    np = runtime.np
    controller = runtime.robot.get_articulation_controller()
    kp, kd = [np.asarray(value).reshape(-1).copy() for value in defaults[:2]]
    material = UsdShade.Material.Define(runtime.stage, '/World/GraspProbeFingerMaterial')
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    official = profile_name.startswith('official')
    api.CreateStaticFrictionAttr(2. if official else 1.)
    api.CreateDynamicFrictionAttr(1.8 if official else 1.)
    api.CreateRestitutionAttr(0.)
    physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx.CreateFrictionCombineModeAttr('max' if official else 'average')
    physx.CreateRestitutionCombineModeAttr('min')
    for path in runtime.metadata['robot_collider_prim_paths']:
        prim = runtime.stage.GetPrimAtPath(path)
        if any(part.startswith(('finger_l_link', 'finger_r_link')) for part in path.split('/')):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')
    finger_names = [name for name in runtime.names if name.startswith('finger_')]
    for name in finger_names:
        index = runtime.indices[name]
        joint = runtime.stage.GetPrimAtPath(runtime.metadata['joints'][name]['prim_path'])
        drive = UsdPhysics.DriveAPI(joint, 'angular')
        drive.CreateMaxForceAttr(3.09 if profile_name == 'official_full' else defaults[2][name])
        if profile_name == 'official_full': kp[index], kd[index] = 500., 30.
    controller.set_gains(kps=kp, kds=kd, save_to_usd=False)
    # While PLAYing, the public controller API writes PhysX's tensor directly;
    # changing a USD attribute alone is not proof of the live drive force cap.
    expected_caps = np.array([3.09 if profile_name == 'official_full' else defaults[2][name]
                              for name in finger_names])
    finger_indices = np.array([runtime.indices[name] for name in finger_names])
    controller.set_max_efforts(expected_caps, joint_indices=finger_indices)
    actual_caps = np.asarray(controller.get_max_efforts()).reshape(-1)
    assert np.allclose(actual_caps[finger_indices], expected_caps, atol=1e-5), 'Native PhysX effort caps did not apply'


def trial(runtime, description, profile, side, kind, profile_name, plate, palm_view, defaults):
    from pxr import Gf, UsdPhysics
    np, replies = runtime.np, Replies()
    UsdPhysics.CollisionAPI(plate.GetPrim()).GetCollisionEnabledAttr().Set(False)
    runtime.reset(); palm_view.initialize(runtime.world.physics_sim_view)
    set_profile(runtime, profile_name, defaults)
    names = GROUPS[side+'_hand']
    def trajectory(group, joint_names, positions, seconds):
        runtime.command(dict(kind='trajectory', group=group, joint_names=list(joint_names),
                             points=[dict(positions=list(positions), time_from_start=seconds)]), replies)
    def step(frames):
        state = None
        for frame in range(frames):
            state = runtime.step(render=frame % 120 == 0)
            if state is None: raise RuntimeError('Timeline stopped during grasp trial')
        return state
    trajectory(side+'_hand', names, profile[side]['release'], .75)
    state = step(120)
    obj = next(obj for obj in runtime.metadata['dynamic_objects'] if obj['kind'] == kind)
    object_index = list(runtime.dynamic_objects.prim_paths).index(obj['prim_path'])
    vertices, source_axes, center_local, dimensions = scan_geometry(runtime, obj)
    midpoint, frame, placement = pinch_geometry(np, description, profile, side, float(dimensions[2]))
    # Grip the broad scan face near its edge; avoid burying it in the palm hull.
    object_center_palm = midpoint+frame[:, 0]*max(0., dimensions[0]/2-.025)
    mount = builder.fixed_mount(description, f'hx5_d20_{side}_base')[1]
    def palm_pose():
        positions, orientations = palm_view.get_world_poses()
        return builder.compose((tuple(positions[0]), tuple(orientations[0])), mount)
    palm_position, palm_orientation = palm_pose(); palm_rotation = rotation(np, palm_orientation)
    object_rotation = palm_rotation @ frame @ source_axes.T
    # Gf decomposition retains the real scan's scale on descendant Xforms.
    matrix = Gf.Matrix4d(1.)
    for i in range(3):
        for j in range(3): matrix[i, j] = float(object_rotation[j, i])
    q = matrix.ExtractRotationQuat()
    object_orientation = np.array([q.GetReal(), *q.GetImaginary()])
    world_center = np.array(palm_position)+palm_rotation @ object_center_palm
    object_position = world_center-object_rotation @ center_local
    transformed = (vertices-center_local) @ object_rotation.T+world_center
    bottom = float(transformed[:, 2].min())
    plate.GetPrim().GetAttribute('xformOp:translate').Set(Gf.Vec3d(float(world_center[0]), float(world_center[1]), bottom-.012))
    UsdPhysics.CollisionAPI(plate.GetPrim()).GetCollisionEnabledAttr().Set(True)
    runtime.dynamic_objects.set_world_poses(positions=object_position.reshape(1,3),
        orientations=object_orientation.reshape(1,4), indices=np.array([object_index]))
    runtime.dynamic_objects.set_velocities(np.zeros((1,6)), indices=np.array([object_index]))
    runtime.world.render()
    curl = .9583333333333334 if profile_name == 'baseline_0958' else 1.
    closed = hand_values(profile, side, curl)
    trajectory(side+'_hand', names, [closed[name] for name in names], 1.5)
    state = step(180)
    def observation(state):
        positions, _ = runtime.dynamic_objects.get_world_poses(indices=np.array([object_index]))
        palm_position, palm_orientation = palm_pose()
        local = rotation(np, palm_orientation).T @ (positions[0]-np.array(palm_position))
        sensors = state['contacts'][side]
        forces = {sensor:sum(sample.get('forces', [])) for sensor, sample in sensors.items() if sample['valid']}
        efforts = runtime.robot.get_measured_joint_efforts()
        return dict(time=state['time'], object_position_m=positions[0].tolist(), object_palm_position_m=local.tolist(),
                    pad_forces_newtons=forces, measured_joint_efforts_nm={name:float(efforts[runtime.indices[name]]) for name in names},
                    measured_hand_positions_rad={name:float(state['positions'][runtime.indices[name]]) for name in names})
    supported = observation(state)
    UsdPhysics.CollisionAPI(plate.GetPrim()).GetCollisionEnabledAttr().Set(False)
    states = []
    for _ in range(8): states.append(observation(step(30)))
    released = states[-1]
    origin = np.array(supported['object_palm_position_m'])
    drift = max(float(np.linalg.norm(np.array(sample['object_palm_position_m'])-origin)) for sample in states)
    letter = 'l' if side == 'left' else 'r'
    opposing = sum(sample['pad_forces_newtons'].get(f'finger_{letter}_sensor1', 0)>1e-4 and
        any(force>1e-4 for sensor,force in sample['pad_forces_newtons'].items() if not sensor.endswith('sensor1')) for sample in states)
    # Select a small arm movement with positive predicted palm Z displacement;
    # this is a real joint drive, never a hand/object teleport.
    measured = dict(zip(runtime.names, state['positions']))
    root_position, root_orientation = runtime.robot.get_world_pose()
    base = (tuple(root_position), tuple(root_orientation))
    palm_link = f'hx5_d20_{side}_base'
    start = np.array(builder.compose(base, transform(description, palm_link, measured))[0])
    candidates = []
    for name in GROUPS[side]:
        for delta in (-.12, .12):
            position = measured[name]+delta
            lower, upper = runtime.limits[name]
            if not lower <= position <= upper: continue
            values = dict(measured); values[name] = position
            finish = np.array(builder.compose(base, transform(description, palm_link, values))[0])
            candidates.append((float(finish[2]-start[2]), name, position))
    rise, joint, target = max(candidates)
    lift_before = palm_pose()[0]
    trajectory(side, GROUPS[side], [target if name == joint else measured[name] for name in GROUPS[side]], 1.)
    lifted = observation(step(180))
    lift_after = palm_pose()[0]
    lift_drift = float(np.linalg.norm(np.array(lifted['object_palm_position_m'])-origin))
    lift_m = float(lift_after[2]-lift_before[2])
    applied = runtime.robot.get_articulation_controller().get_applied_action()
    actual_kp, actual_kd = runtime.robot.get_articulation_controller().get_gains()
    actual_kp, actual_kd = np.asarray(actual_kp).reshape(-1), np.asarray(actual_kd).reshape(-1)
    return dict(side=side, kind=kind, mass_kg=obj['mass_kg'], profile=profile_name, curl=curl,
                source_scan=obj['prim_path'], scan_pca_dimensions_m=dimensions.tolist(), placement=placement,
                supported=supported, released=released, lifted=lifted, release_samples=states,
                maximum_release_palm_drift_m=drift, opposing_contact_samples=opposing,
                release_retained=drift<=.02 and opposing>=4, lift_joint=joint,
                predicted_lift_m=rise, measured_lift_m=lift_m, lift_palm_drift_m=lift_drift,
                lift_retained=lift_m>=.01 and lift_drift<=.025,
                applied_joint_efforts=getattr(applied,'joint_efforts',None),
                native_applied_joint_efforts_nm=runtime.robot.get_applied_joint_efforts(),
                native_hand_gains={name:dict(stiffness_nm_per_rad=float(actual_kp[runtime.indices[name]]),
                    damping_nm_s_per_rad=float(actual_kd[runtime.indices[name]])) for name in names},
                effort_interpretation='implicit position drives; applied effort command may be absent/zero, measured efforts are native projected joint reactions',
                native_drive_force_caps_nm={name:float(runtime.robot.get_articulation_controller().get_max_efforts()[runtime.indices[name]]) for name in names},
                authored_drive_force_caps_nm={name:runtime.stage.GetPrimAtPath(runtime.metadata['joints'][name]['prim_path']).GetAttribute('drive:angular:physics:maxForce').Get() for name in names})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    assets = Path(__file__).resolve().parents[3]/'simulation/isaac/assets/sh5'
    parser.add_argument('--stage', type=Path, default=assets/'logistics_sh5.usda')
    parser.add_argument('--metadata', type=Path, default=assets/'scene_metadata.json')
    parser.add_argument('--summary', type=Path, required=True)
    parser.add_argument('--sides', nargs='+', choices=['left','right'], default=['left','right'])
    parser.add_argument('--kinds', nargs='+', choices=['object1','object2'], default=['object1','object2'])
    parser.add_argument('--profiles', nargs='+', choices=['baseline_0958','baseline_full','official_friction','official_full'],
                        default=['baseline_0958','baseline_full','official_friction','official_full'])
    args, _kit = parser.parse_known_args()
    if args.summary.exists(): raise FileExistsError('Use a fresh --summary path')
    from isaacsim import SimulationApp
    app = SimulationApp(dict(headless=True, fast_shutdown=True, shutdown_watchdog_timeout=30.))
    result, runtime, code = dict(trials=[], official_source=OFFICIAL_SOURCE), None, 0
    try:
        from pxr import Gf, UsdGeom, UsdPhysics
        from isaacsim.core.prims import RigidPrim
        metadata = json.loads(args.metadata.read_text())
        description = builder.read_robot_description(metadata['source_urdf'])
        profile = json.loads((Path(__file__).resolve().parents[3]/'runtime/hx5_isaac/official_1044_hand_mapping.json').read_text())
        runtime = PhysicsRuntime(app, args.stage, metadata, cameras_enabled=False)
        defaults = (*runtime.robot.get_articulation_controller().get_gains(),
                    {name:metadata['joints'][name]['drive']['max_force'] for name in runtime.names if name.startswith('finger_')})
        plate = UsdGeom.Cube.Define(runtime.stage, '/World/GraspDiagnosticSupport')
        plate.CreateSizeAttr(1.); plate.AddTranslateOp().Set(Gf.Vec3d(0.)); plate.AddScaleOp().Set(Gf.Vec3d(.5,.5,.02))
        plate.CreateVisibilityAttr('invisible'); UsdPhysics.CollisionAPI.Apply(plate.GetPrim()).CreateCollisionEnabledAttr(False)
        filtered = UsdPhysics.FilteredPairsAPI.Apply(plate.GetPrim()).CreateFilteredPairsRel()
        for path in metadata['robot_collider_prim_paths']: filtered.AddTarget(path)
        for side in args.sides:
            letter = 'l' if side == 'left' else 'r'
            palm_view = RigidPrim(prim_paths_expr=metadata['joints'][f'arm_{letter}_joint7']['body_path'], name='grasp_palm_'+side, reset_xform_properties=False)
            for kind in args.kinds:
                for chosen in args.profiles:
                    result['trials'].append(trial(runtime, description, profile, side, kind, chosen, plate, palm_view, defaults))
        result.update(completed=True, stats=runtime.stats,
            scope='diagnostic scan placement and support release; no human teleoperation or tactile calibration proof',
            all_trials_retained=all(t['release_retained'] and t['lift_retained'] for t in result['trials']))
    except Exception as error:
        code=1; result.update(completed=False,error=str(error),traceback=traceback.format_exc())
        logging.exception('Grasp diagnostic failed')
    finally:
        args.summary.parent.mkdir(parents=True,exist_ok=True)
        with args.summary.open('x') as stream: json.dump(json_safe(result),stream,indent=2,allow_nan=False)
        concise = {key:result[key] for key in ('completed','error','all_trials_retained') if key in result}
        concise['summary'] = str(args.summary)
        concise['trials'] = [{key:trial[key] for key in ('side','kind','profile','release_retained',
            'maximum_release_palm_drift_m','opposing_contact_samples','lift_retained','measured_lift_m')}
            for trial in result['trials']]
        print(json.dumps(json_safe(concise),indent=2,allow_nan=False),flush=True)
        if runtime: runtime.hold('grasp diagnostic finished')
        app.close(wait_for_replicator=False,exit_code=code)


if __name__ == '__main__': main()
