#!/usr/bin/env python3
"""Prepare a separate floating-base SH5/HX5 PhysX stage from a read-only cell.

Run with Isaac Sim's python.sh. The source cell/URDF/assets are never saved.
All generated layers and the sensor/control receipt are written beside --output.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_ENVIRONMENT = Path('/home/robotis-ai/workspaces/isaac_logistics_cell_ws/logistics_cell.environment.usda')
DEFAULT_URDF = WORKSPACE / 'simulation/isaac/assets/sh5/robot.urdf'
DEFAULT_OUTPUT = DEFAULT_URDF.parent / 'logistics_sh5.usda'
DEFAULT_POSE = WORKSPACE / 'src/hx5_simulation/hx5_simulation/initial_poses/vitacformer_task519_sync.yaml'
ROBOT_PATH = '/World/FFWSH5'
POSITION_JOINT_NAMES = ([f'arm_{s}_joint{i}' for s in ('l', 'r') for i in range(1, 8)]
                        + [f'finger_{s}_joint{i}' for s in ('l', 'r') for i in range(1, 21)]
                        + ['head_joint1', 'head_joint2', 'lift_joint'])
STEER_JOINT_NAMES = [f'{s}_wheel_steer' for s in ('left', 'right', 'rear')]
WHEEL_JOINT_NAMES = [f'{s}_wheel_drive' for s in ('left', 'right', 'rear')]


def qmul(a, b):
    w, x, y, z = a; v, p, q, r = b
    return (w*v-x*p-y*q-z*r, w*p+x*v+y*r-z*q,
            w*q-x*r+y*v+z*p, w*r+x*q-y*p+z*v)


def qrotate(q, xyz):
    return qmul(qmul(q, (0.0, *xyz)), (q[0], -q[1], -q[2], -q[3]))[1:]


def compose(a, b):
    rotated = qrotate(a[1], b[0])
    return (tuple(a[0][i] + rotated[i] for i in range(3)), qmul(a[1], b[1]))


IDENTITY = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))


def origin(element):
    if element is None:
        return IDENTITY
    xyz = tuple(map(float, element.get('xyz', '0 0 0').split()))
    roll, pitch, yaw = map(float, element.get('rpy', '0 0 0').split())
    q = qmul(qmul((math.cos(yaw/2), 0, 0, math.sin(yaw/2)),
                  (math.cos(pitch/2), 0, math.sin(pitch/2), 0)),
             (math.cos(roll/2), math.sin(roll/2), 0, 0))
    return xyz, q


def read_robot_description(path):
    root = ET.parse(path).getroot()
    links = {e.get('name'): e for e in root.findall('link')}
    joints = {}
    by_child = {}
    for element in root.findall('joint'):
        name, kind = element.get('name'), element.get('type')
        parent, child = element.find('parent').get('link'), element.find('child').get('link')
        limit = element.find('limit')
        data = {'name': name, 'type': kind, 'parent': parent, 'child': child,
                'origin': origin(element.find('origin')),
                'axis': tuple(map(float, element.find('axis').get('xyz', '1 0 0').split()))
                        if element.find('axis') is not None else (1.0, 0.0, 0.0),
                'limit': {k: float(v) for k, v in limit.attrib.items()} if limit is not None else {}}
        joints[name] = data; by_child[child] = data
    roots = set(links) - set(by_child)
    if len(roots) != 1:
        raise ValueError(f'Expected one URDF root link, found {sorted(roots)}')
    expected = set(POSITION_JOINT_NAMES + STEER_JOINT_NAMES + WHEEL_JOINT_NAMES)
    actual = {name for name, j in joints.items() if j['type'] != 'fixed'}
    if actual != expected:
        raise ValueError(f'Expected exactly 63 SH5 driven joints; missing={expected-actual}, extra={actual-expected}')
    return {'name': root.get('name'), 'links': links, 'joints': joints, 'by_child': by_child,
            'root_link': roots.pop()}


def fixed_mount(description, link):
    """Return the surviving rigid link and transform after fixed-joint merging."""
    transform = IDENTITY
    while link in description['by_child']:
        joint = description['by_child'][link]
        if joint['type'] != 'fixed':
            break
        transform = compose(joint['origin'], transform)
        link = joint['parent']
    return link, transform


def validate_initial_positions(description, positions):
    if set(positions) != set(POSITION_JOINT_NAMES):
        raise ValueError('Initial pose must explicitly provide all 57 SH5 arm/hand/head/lift joints')
    checked = {}
    for name, value in positions.items():
        value = float(value); limits = description['joints'][name]['limit']
        if not math.isfinite(value) or value < limits.get('lower', -math.inf)-1e-7 or value > limits.get('upper', math.inf)+1e-7:
            raise ValueError(f'Initial position outside official URDF limit: {name}={value}, {limits}')
        checked[name] = value
    return checked


def drive_settings(name, description):
    """Simulation tuning assumptions, not gains measured on the real robot."""
    if name in WHEEL_JOINT_NAMES:
        stiffness, damping, mode = 0.0, 80.0, 'velocity'
    elif name == 'lift_joint':
        # A ~27 kg upper assembly sagged 13.9 mm at 20 kN/m in the
        # measured 120 Hz smoke run. 100 kN/m limits spring deflection to
        # about 3 mm, allowing the 0.01 m position gate to settle.
        stiffness, damping, mode = 100000.0, 4000.0, 'position'
    elif name.startswith('finger_'):
        # Native PhysX sweeps measured 13.54 mrad thumb error at raw USD
        # 320/32 and 3.366 mrad at 320/8. Angular USD gains are Nm/degree
        # and Nm*s/degree; native tensor gains use radians.
        stiffness, damping, mode = 320.0, 8.0, 'position'
    elif name.startswith('head_'):
        stiffness, damping, mode = 300.0, 30.0, 'position'
    elif name in STEER_JOINT_NAMES:
        stiffness, damping, mode = 2000.0, 200.0, 'position'
    else:
        stiffness, damping, mode = 1000.0, 100.0, 'position'
    # Honor URDF effort limits. The imported description's large 1000 values
    # are retained as source values; no hardware effort calibration is implied.
    return {'stiffness': stiffness, 'damping': damping, 'mode': mode, 'type': 'force',
            'max_force': description['joints'][name]['limit'].get('effort', 1000.0)}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_kit_args(args):
    """Allow Kit settings and the isolated portable-root argument only."""
    index = 0
    while index < len(args):
        arg = args[index]
        if arg == '--portable-root':
            index += 1
            if index >= len(args) or not args[index] or args[index].startswith('--'):
                raise ValueError('--portable-root requires a directory path')
        elif arg == '--portable':
            pass
        elif arg.startswith('--portable-root=') and arg.split('=', 1)[1]:
            pass
        elif arg.startswith('--/') and '=' in arg:
            pass
        else:
            raise ValueError(f'Unknown non-Kit option: {arg}')
        index += 1


def _set_transform(prim, transform):
    from pxr import Gf, UsdGeom
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    # Distinct names avoid existing importer ops whose precision may differ.
    xyz, q = transform
    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble, 'hx5Initial').Set(Gf.Vec3d(*xyz))
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble, 'hx5Initial').Set(Gf.Quatd(q[0], Gf.Vec3d(*q[1:])))


def _find_named_prim(stage, name, root=ROBOT_PATH, schema=None):
    from pxr import Usd, UsdPhysics
    matches = [p for p in Usd.PrimRange(stage.GetPrimAtPath(root))
               if p.GetName() == name and (schema is None or p.IsA(schema))]
    # The importer often names the visual/collision mesh after its enclosing
    # rigid link. Link lookup must select the actual rigid-body Xform.
    if schema is None:
        bodies = [p for p in matches if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        if bodies:
            matches = bodies
    if len(matches) != 1:
        raise RuntimeError(f'Expected one imported {name} prim, found {[str(p.GetPath()) for p in matches]}')
    return matches[0]


def import_robot(urdf, output_dir):
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    config = URDFImporterConfig(
        urdf_path=str(urdf), usd_path=str(output_dir), fix_base=False,
        # Camera/end-marker fixed links are merged rather than becoming
        # independently constrained massless rigid bodies. Optical frames
        # below are rebuilt from the original URDF's complete fixed chains.
        merge_fixed_joints=True, merge_mesh=False, collision_from_visuals=False,
        allow_self_collision=False, robot_type='Mobile Manipulators',
        joint_drive_type='force', joint_target_type={'.*': 'position', '.*_wheel_drive$': 'velocity'},
        override_joint_stiffness={'.*': 1000.0, '.*_wheel_drive$': 0.0},
        override_joint_damping=100.0,
        run_asset_transformer=True, run_multi_physics_conversion=True)
    result = Path(URDFImporter(config).import_urdf()).resolve()
    if not result.is_file():
        raise RuntimeError(f'URDF importer did not write its output: {result}')
    return result


def compose_scene(environment, robot_usd, urdf, pose, output, layout,
                  floor_friction=1.0, physics_hz=120, spawn=None, camera_profile='official'):
    """Author a stronger layer; return JSON-compatible discovered metadata."""
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    description = read_robot_description(urdf)
    positions = validate_initial_positions(description, pose['initial_positions'])
    output = Path(output).resolve(); output.parent.mkdir(parents=True, exist_ok=True)
    if output == Path(environment).resolve() or output == Path(robot_usd).resolve():
        raise ValueError('Output must be separate from all source stages')
    # Keep an existing usable stage intact when import/validation fails.
    # All references below are absolute, so an in-memory authoring layer can
    # be exported safely after every robot/contact/object check succeeds.
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [str(Path(environment).resolve())]
    stage.OverridePrim('/World/Robot').SetActive(False)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0); UsdGeom.SetStageUpAxis(stage, 'Z')
    source_stage = Usd.Stage.Open(str(Path(environment).resolve()))
    stage.SetTimeCodesPerSecond(source_stage.GetTimeCodesPerSecond())
    stage.SetFramesPerSecond(source_stage.GetFramesPerSecond())
    stage.SetStartTimeCode(source_stage.GetStartTimeCode())
    stage.SetEndTimeCode(source_stage.GetEndTimeCode())
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    scene = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1)); scene.CreateGravityMagnitudeAttr(9.81)
    scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    scene_api.CreateTimeStepsPerSecondAttr(physics_hz)
    scene_api.CreateEnableCCDAttr(True)
    scene_api.CreateSolverTypeAttr('TGS')
    scene_api.CreateEnableStabilizationAttr(True)
    # This stage is specifically the PhysX runtime variant; Isaac6's Newton
    # import metadata is converted by the official multi-physics pipeline.
    robot = stage.DefinePrim(ROBOT_PATH, 'Xform')
    robot.GetReferences().AddReference(str(Path(robot_usd).resolve()))
    physics = robot.GetVariantSet('Physics')
    if physics.IsValid():
        physics.SetVariantSelection('physx')
    # Collision meshes are instance proxies in the transformed asset. Make
    # their enclosing references non-instanceable in this stronger layer so
    # material/contact offsets can be authored locally without touching the
    # reusable imported asset. Visual-only/material instances remain shared.
    for prim in list(Usd.PrimRange(robot)):
        if prim.IsInstance() and any(p.HasAPI(UsdPhysics.CollisionAPI)
                                     for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())):
            prim.SetInstanceable(False)
    explicit_spawn = spawn is not None
    spawn = spawn or [*layout['robot']['position_m'], layout['robot']['yaw_degrees']]
    orientation = (math.cos(math.radians(spawn[3])/2), 0, 0, math.sin(math.radians(spawn[3])/2))
    if not explicit_spawn and 'orientation_wxyz' in layout['robot']:
        orientation = tuple(map(float,layout['robot']['orientation_wxyz']))
        if len(orientation) != 4 or not all(math.isfinite(v) for v in orientation) or abs(sum(v*v for v in orientation)-1) > 1e-5:
            raise ValueError('Robot startup orientation must be a normalized finite WXYZ quaternion')
    _set_transform(robot, (tuple(spawn[:3]), orientation))
    root = _find_named_prim(stage, description['root_link'])
    if not root.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError('Imported floating root lacks RigidBodyAPI')
    # Root is authored on the actual floating rigid body, with no fixed joint
    # between the robot and world. The lift remains a live prismatic joint.
    for prim in Usd.PrimRange(robot):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI) and prim != root:
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    articulation = PhysxSchema.PhysxArticulationAPI.Apply(root)
    articulation.CreateEnabledSelfCollisionsAttr(False)
    articulation.CreateSolverPositionIterationCountAttr(64)
    articulation.CreateSolverVelocityIterationCountAttr(4)
    articulation.CreateStabilizationThresholdAttr(0.0)
    PhysxSchema.PhysxRigidBodyAPI.Apply(root).CreateLinearDampingAttr(2.0)
    PhysxSchema.PhysxRigidBodyAPI.Apply(root).CreateAngularDampingAttr(5.0)
    joints = {}
    for name in POSITION_JOINT_NAMES + STEER_JOINT_NAMES + WHEEL_JOINT_NAMES:
        joint = description['joints'][name]
        prim = _find_named_prim(stage, name, schema=UsdPhysics.Joint)
        body = _find_named_prim(stage, joint['child'])
        parent, fixed = fixed_mount(description, joint['parent'])
        target = positions.get(name, 0.0)
        length = math.sqrt(sum(x*x for x in joint['axis']))
        if length == 0:
            raise ValueError(f'Invalid zero joint axis: {name}')
        axis = tuple(x/length for x in joint['axis'])
        motion = ((tuple(target*x for x in axis), (1, 0, 0, 0)) if joint['type'] == 'prismatic'
                  else ((0, 0, 0), (math.cos(target/2), *(math.sin(target/2)*x for x in axis))))
        _set_transform(body, compose(compose(fixed, joint['origin']), motion))
        settings = drive_settings(name, description)
        drive = UsdPhysics.DriveAPI.Apply(prim, 'linear' if joint['type'] == 'prismatic' else 'angular')
        drive.CreateTypeAttr('force'); drive.CreateStiffnessAttr(settings['stiffness'])
        drive.CreateDampingAttr(settings['damping']); drive.CreateMaxForceAttr(settings['max_force'])
        drive.CreateTargetPositionAttr(target if joint['type'] == 'prismatic' else math.degrees(target))
        drive.CreateTargetVelocityAttr(0.0)
        velocity_limit = joint['limit'].get('velocity')
        if velocity_limit is not None:
            # PhysX USD angular attributes use degrees/second; Isaac's tensor
            # feedback and the official URDF use radians/second.
            PhysxSchema.PhysxJointAPI.Apply(prim).CreateMaxJointVelocityAttr(
                velocity_limit if joint['type'] == 'prismatic' else math.degrees(velocity_limit))
        joints[name] = {'prim_path': str(prim.GetPath()), 'body_path': str(body.GetPath()),
                        'type': joint['type'], 'axis': list(axis), 'limits': joint['limit'],
                        'initial_position': target, 'drive': settings}
    # One shared explicit physics material. Dynamic scanned goods retain
    # geometry, placement and appearance from the source cell.
    material = UsdShade.Material.Define(stage, '/World/HX5PhysicsMaterial')
    mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    mat.CreateStaticFrictionAttr(floor_friction); mat.CreateDynamicFrictionAttr(floor_friction)
    mat.CreateRestitutionAttr(0.0)
    from payload_physics import (author_basket_colliders, configure_payload_body,
                                 configure_payload_mesh)
    basket_collider_paths = author_basket_colliders(stage, layout)
    dynamic_objects = []
    for prim in list(stage.Traverse()):
        kind = prim.GetCustomDataByKey('assetKind')
        if kind not in ('object1', 'object2'):
            continue
        mass = float(layout['packing'][kind]['mass_kg_for_settling'])
        if not math.isfinite(mass) or mass <= 0:
            raise ValueError(f'Dynamic payload mass must be nonzero: {kind}')
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateRigidBodyEnabledAttr(True)
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(False)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        configure_payload_body(prim)
        colliders = []
        for mesh in Usd.PrimRange(prim):
            if mesh.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(mesh).CreateCollisionEnabledAttr(True)
                configure_payload_mesh(mesh)
                colliders.append(str(mesh.GetPath()))
        if not colliders:
            raise RuntimeError(f'Scanned payload has no collider geometry: {prim.GetPath()}')
        dynamic_objects.append({'prim_path': str(prim.GetPath()), 'kind': kind,
                                'mass_kg': mass, 'collider_paths': colliders})
    expected_count = sum(layout['packing'][k]['count'] for k in ('object1', 'object2'))
    if len(dynamic_objects) != expected_count:
        raise RuntimeError(f'Expected {expected_count} pickable objects, found {len(dynamic_objects)}')
    collider_paths = []
    for prim in list(stage.Traverse()):
        if (prim.HasAPI(UsdPhysics.CollisionAPI)
                and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')
            collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collision.CreateContactOffsetAttr(0.001); collision.CreateRestOffsetAttr(0.0)
            collider_paths.append(str(prim.GetPath()))
    # Match the contact skin used when producing the saved gravity-drop pile.
    # Robot and the remaining environment retain their existing offsets.
    for path in basket_collider_paths + [path for obj in dynamic_objects
                                         for path in obj['collider_paths']]:
        collision = PhysxSchema.PhysxCollisionAPI.Apply(stage.GetPrimAtPath(path))
        collision.CreateContactOffsetAttr(0.003)
        collision.CreateRestOffsetAttr(0.0015)
    cameras = _add_cameras(stage, description, camera_profile)
    from camera_geometry import clear_head_optical_apertures
    visual_apertures = clear_head_optical_apertures(stage, cameras)
    contacts = {}
    for side in ('l', 'r'):
        for finger, distal in enumerate((4, 8, 12, 16, 20), 1):
            body = _find_named_prim(stage, f'finger_{side}_link{distal}')
            sensor = f'finger_{side}_sensor{finger}'
            PhysxSchema.PhysxContactReportAPI.Apply(body).CreateThresholdAttr(0.0)
            marker = f'finger_end_{side}_link{finger}'
            _, mount = fixed_mount(description, marker)
            marker_prim = UsdGeom.Xform.Define(stage, str(body.GetPath()) + '/HX5ContactFrame').GetPrim()
            _set_transform(marker_prim, mount)
            contacts[sensor] = {'body_path': str(body.GetPath()), 'rigid_body_path': str(body.GetPath()),
                                'frame_path': str(marker_prim.GetPath()),
                                'source_tip_link': marker,
                                'taxel_translation': list(mount[0]),
                                'taxel_orientation_xyzw': [*mount[1][1:], mount[1][0]],
                                'collider_paths': [str(p.GetPath()) for p in Usd.PrimRange(body)
                                                   if p.HasAPI(UsdPhysics.CollisionAPI)],
                                'taxel_layout': '3x3', 'calibration': 'simulation contact impulse; no hardware pressure calibration'}
            if not contacts[sensor]['collider_paths']:
                raise RuntimeError(f'Distal contact body has no collision mesh: {sensor}')
    receipt = {'schema_version': 1, 'stage': str(output), 'source_environment': str(Path(environment).resolve()),
               'source_environment_sha256': sha256(environment), 'source_urdf': str(Path(urdf).resolve()),
               'source_urdf_sha256': sha256(urdf), 'robot_usd': str(Path(robot_usd).resolve()),
               'robot_prim_path': ROBOT_PATH, 'articulation_root_path': str(root.GetPath()),
               'base_body_path': str(root.GetPath()), 'floating_base': True, 'lift_active': True,
               'position_joint_names': POSITION_JOINT_NAMES, 'steer_joint_names': STEER_JOINT_NAMES,
               'wheel_joint_names': WHEEL_JOINT_NAMES, 'driven_joint_names': list(joints),
               'joint_count': len(joints), 'joints': joints, 'cameras': cameras, 'contact_tips': contacts,
               'head_camera_visual_apertures': visual_apertures,
               'dynamic_objects': dynamic_objects, 'dynamic_object_count': len(dynamic_objects),
               'payload_collision_model': 'convexHull256',
               'basket_collision_model': 'open_five_box_settling_proxy',
               'basket_collider_prim_paths': basket_collider_paths,
               'physics_hz': physics_hz, 'floor_friction': floor_friction, 'spawn': spawn,
               'robot_spawn': {'position': list(spawn[:3]),
                               'orientation_wxyz': list(orientation)},
               'dynamic_object_prim_paths': [obj['prim_path'] for obj in dynamic_objects],
               'robot_collider_prim_paths': [path for path in collider_paths
                                             if path.startswith(ROBOT_PATH + '/')],
               'contact_filter_prim_paths': [path for path in collider_paths
                                             if not path.startswith(ROBOT_PATH + '/')],
               'initial_positions': positions, 'initial_pose_metadata': pose.get('metadata', {}),
               'assumptions': ['Payload masses are user cell settling assumptions (0.08/0.05 kg).',
                               'Drive gains and friction are simulation tuning values.',
                               'Self collision is disabled; environment and payload collisions remain active.',
                               'A 3x3 synthetic taxel projection requires explicit simulation calibration.'],
               'source_unchanged': True}
    stage.GetRootLayer().customLayerData = {'description': 'SH5/HX5 floating-base PhysX overlay; original logistics cell read-only'}
    import os
    temporary_output = output.with_name(f'.{output.stem}.build-{os.getpid()}.usda')
    try:
        if not stage.GetRootLayer().Export(str(temporary_output)):
            raise RuntimeError(f'Failed to export composed stage: {temporary_output}')
        temporary_output.replace(output)
    finally:
        temporary_output.unlink(missing_ok=True)
    return receipt


def _add_cameras(stage, description, profile='official'):
    from pxr import Gf, Sdf, UsdGeom
    from camera_profiles import camera_profile
    specs = [('head_left', 'zedm_left_camera_optical_frame', '/zed/zed_node/left/image_rect_color'),
             ('head_right', 'zedm_right_camera_optical_frame', '/zed/zed_node/right/image_rect_color'),
             ('wrist_left', 'camera_l_link', '/camera_left/camera_left/color/image_rect_raw'),
             ('wrist_right', 'camera_r_link', '/camera_right/camera_right/color/image_rect_raw')]
    results = {}
    for name, link, topic in specs:
        settings = camera_profile(name, profile)
        surviving, mount = fixed_mount(description, link)
        optical_frame = link
        if name.startswith('wrist'):
            optical_frame = link.replace('_link', '_color_optical_frame')
            optical = ET.Element('origin', {'xyz': '0.0038 0 0', 'rpy': '-1.5707963267948966 0 -1.5707963267948966'})
            mount = compose(mount, origin(optical))
        body = _find_named_prim(stage, surviving)
        frame = UsdGeom.Xform.Define(stage, str(body.GetPath()) + f'/HX5Sensors/{optical_frame}')
        _set_transform(frame.GetPrim(), mount)
        camera = UsdGeom.Camera.Define(stage, str(frame.GetPath()) + '/Camera')
        # ROS optical axes +Z forward, +Y down -> USD camera -Z forward,+Y up.
        _set_transform(camera.GetPrim(), ((0, 0, 0), (0, 1, 0, 0)))
        camera.CreateProjectionAttr('perspective')
        # USD camera units are tenths of a stage unit; this stage uses meters.
        camera.CreateHorizontalApertureAttr(settings['horizontal_aperture_mm'] * .01)
        camera.CreateVerticalApertureAttr(settings['vertical_aperture_mm'] * .01)
        camera.CreateFocalLengthAttr(settings['focal_length_mm'] * .01)
        camera.CreateFStopAttr(0.0)
        prim = camera.GetPrim()
        prim.AddAppliedSchema('OmniLensDistortionOpenCvPinholeAPI')
        prim.CreateAttribute('omni:lensdistortion:model', Sdf.ValueTypeNames.Token).Set('opencvPinhole')
        prefix = 'omni:lensdistortion:opencvPinhole:'
        for attribute in ('fx', 'fy', 'cx', 'cy'):
            prim.CreateAttribute(prefix + attribute, Sdf.ValueTypeNames.Float).Set(settings[attribute])
        for attribute in ('k1', 'k2', 'p1', 'p2', 'k3', 'k4', 'k5', 'k6', 's1', 's2', 's3', 's4'):
            prim.CreateAttribute(prefix + attribute, Sdf.ValueTypeNames.Float).Set(0.0)
        prim.CreateAttribute(prefix + 'imageSize', Sdf.ValueTypeNames.Int2).Set(Gf.Vec2i(*settings['resolution']))
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 30.0))
        results[name] = {**settings, 'prim_path': str(camera.GetPath()), 'mount_body_path': str(body.GetPath()),
                          'source_link': link, 'optical_frame': optical_frame,
                          'image_topic': topic, 'mount_xyz': list(mount[0]), 'mount_quat_wxyz': list(mount[1]),
                          'optical_frame_in_source_urdf': link == optical_frame}
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--environment', type=Path, default=DEFAULT_ENVIRONMENT)
    parser.add_argument('--urdf', type=Path, default=DEFAULT_URDF)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--initial-pose', type=Path, default=DEFAULT_POSE)
    parser.add_argument('--layout', type=Path, default=DEFAULT_ENVIRONMENT.parent/'config/layout.json')
    parser.add_argument('--robot-usd', type=Path, help='Reuse an already imported private robot asset')
    parser.add_argument('--metadata', type=Path, help='Explicit receipt path; default beside the composed stage')
    parser.add_argument('--floor-friction', type=float, default=1.0)
    parser.add_argument('--physics-hz', type=int, default=120)
    parser.add_argument('--camera-profile', choices=('official', 'hd720'), default='official')
    parser.add_argument('--spawn', type=float, nargs=4, metavar=('X', 'Y', 'Z', 'YAW_DEG'))
    args, kit_args = parser.parse_known_args()
    # run_isaac.sh forwards Kit settings in sys.argv. SimulationApp consumes
    # them itself; reject unrelated typos rather than silently swallowing them.
    try:
        validate_kit_args(kit_args)
    except ValueError as error:
        parser.error(str(error))
    if not math.isfinite(args.floor_friction) or args.floor_friction <= 0 or args.physics_hz < 60:
        parser.error('Friction must be positive/finite and physics_hz >= 60')
    # Check all inputs and initial limits before creating any simulator app.
    import yaml
    pose = yaml.safe_load(args.initial_pose.read_text())
    layout = json.loads(args.layout.read_text())
    validate_initial_positions(read_robot_description(args.urdf), pose['initial_positions'])
    hashes = {str(p.resolve()): sha256(p) for p in (args.environment, args.urdf, args.initial_pose, args.layout)}
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'enable_cameras': True,
                         'fast_shutdown': True, 'extra_args': ['--/app/extensions/registryEnabled=false']})
    # Isaac6.1's fast shutdown exits the process from close(). Explicitly
    # preserve failure until the stage, source checks and receipt all finish.
    exit_code = 1
    try:
        import isaacsim.core.experimental.utils.app as app_utils
        app_utils.enable_extension('isaacsim.asset.importer.urdf')
        app.update()
        usd = args.robot_usd or import_robot(args.urdf.resolve(), args.output.parent/'usd')
        receipt = compose_scene(args.environment, usd, args.urdf, pose, args.output, layout,
                                args.floor_friction, args.physics_hz, args.spawn, args.camera_profile)
        changed = [p for p, h in hashes.items() if sha256(p) != h]
        if changed:
            raise RuntimeError(f'Source inputs changed while building: {changed}')
        metadata_path = args.metadata or args.output.with_name('scene_metadata.json')
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        metadata_path.write_text(json.dumps(receipt, indent=2) + '\n')
        exit_code = 0
        print(f'HX5_ISAAC_STAGE={args.output.resolve()}')
        print(f'HX5_ISAAC_METADATA={metadata_path.resolve()}')
    except Exception:
        import traceback, sys
        traceback.print_exc(); sys.stderr.flush()
        raise
    finally:
        app.close(exit_code=exit_code)


if __name__ == '__main__':
    main()
