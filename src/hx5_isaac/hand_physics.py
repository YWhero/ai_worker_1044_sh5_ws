"""Opt-in, hand-only PhysX profile derived from ROBOTIS Cyclo Lab.

Nothing is applied on import. The caller owns the edit target and persistence.
Native angular gains use radians; USD DriveAPI uses degrees. These simulation
settings are not a calibration of the physical HX5 fingertip force or motors.
"""
from __future__ import annotations

import math
import re

SOURCE_COMMIT = '47b7d22f1843c8778a39b50eabecc2eb992ea03e'
SOURCE_URL = ('https://github.com/ROBOTIS-GIT/cyclo_lab/blob/' + SOURCE_COMMIT
              + '/source/cyclo_lab/cyclo_lab/assets/robots/FFW_SH5.py')
SOURCE_SHA256 = 'f9174b1f99607969d49caa8a5f6c4c919966232bbcbbaed968facada0be9c730'
HAND_JOINT_NAMES = tuple(f'finger_{side}_joint{index}'
                         for side in ('l', 'r') for index in range(1, 21))
PROFILE_OWNER = 'robotis_cyclo_lab_hand_only_v1'
MATERIAL_NAME = 'HX5FingerPhysicsMaterial'
OFFICIAL_STIFFNESS = 500.0
OFFICIAL_DAMPING = 30.0
OFFICIAL_EFFORT_LIMIT = 3.09
OFFICIAL_VELOCITY_LIMIT = 15.0
FINGER_MATERIAL = {'static_friction': 2.0, 'dynamic_friction': 1.8,
                   'restitution': 0.0, 'friction_combine_mode': 'max',
                   'restitution_combine_mode': 'min'}


def _finite(value, label, *, positive=False):
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'{label} must be finite and nonnegative') from exc
    if not math.isfinite(value) or value < 0 or (positive and value == 0):
        raise ValueError(f'{label} must be finite and ' +
                         ('positive' if positive else 'nonnegative'))
    return value


def native_gain_to_usd(value):
    """Convert N*m/rad (or N*m*s/rad) to its USD degree-based value."""
    return _finite(value, 'Native angular gain') * math.pi / 180.0


def usd_gain_to_native(value):
    """Convert a USD angular gain to the native radian-based value."""
    return _finite(value, 'USD angular gain') * 180.0 / math.pi


def hand_drive_settings(limits, *, stiffness=OFFICIAL_STIFFNESS,
                        damping=OFFICIAL_DAMPING,
                        effort_limit=OFFICIAL_EFFORT_LIMIT,
                        velocity_limit=OFFICIAL_VELOCITY_LIMIT):
    """Retain mechanical URDF bounds and cap torque/rate by the official profile.

    In this workspace URDF velocity is 4.8 rad/s, so it remains 4.8 rather than
    the official Isaac Lab simulation's 15 rad/s. Torque is unchanged by the
    radians/degrees conversion. Effort=1000 in the URDF is not a motor rating.
    """
    stiffness = _finite(stiffness, 'Stiffness', positive=True)
    damping = _finite(damping, 'Damping')
    try:
        urdf_effort = _finite(limits['effort'], 'URDF effort', positive=True)
        urdf_velocity = _finite(limits['velocity'], 'URDF velocity', positive=True)
    except (KeyError, TypeError) as exc:
        raise ValueError('Hand URDF requires explicit effort and velocity limits') from exc
    effort = min(_finite(effort_limit, 'Effort limit', positive=True),
                 OFFICIAL_EFFORT_LIMIT, urdf_effort)
    velocity = min(_finite(velocity_limit, 'Velocity limit', positive=True),
                   OFFICIAL_VELOCITY_LIMIT, urdf_velocity)
    return {'native_stiffness': stiffness, 'native_damping': damping,
            'stiffness': native_gain_to_usd(stiffness),
            'damping': native_gain_to_usd(damping), 'max_force': effort,
            'velocity_rad_s': velocity, 'velocity_deg_s': math.degrees(velocity),
            'mode': 'position', 'type': 'force'}


def _prim_path(value, label):
    if (not isinstance(value, str) or not value.startswith('/')
            or value.endswith('/') or not all(re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', part)
                                               for part in value.split('/')[1:])):
        raise ValueError(f'{label} must be an absolute USD prim path')
    return value


def plan_hand_physics(metadata, **drive_options):
    """Select all 40 finger bodies, their colliders and drives without USD.

    Actual importer paths do not contain the official asset's ``/collisions/``
    segment. Metadata identifies CollisionAPI paths; choose their closest
    finger rigid-body ancestor so nested finger chains cannot be mislabeled.
    Arms, hand bases, goods, wheel/floor colliders and visuals are excluded.
    """
    robot = _prim_path(metadata.get('robot_prim_path'), 'Robot path')
    joints = metadata.get('joints', {})
    planned = {}
    body_to_joint = {}
    joint_paths = set()
    for name in HAND_JOINT_NAMES:
        joint = joints.get(name, {})
        if joint.get('type') != 'revolute':
            raise ValueError(f'Missing revolute hand joint: {name}')
        path = _prim_path(joint.get('prim_path'), f'{name} joint path')
        body = _prim_path(joint.get('body_path'), f'{name} body path')
        side, index = re.fullmatch(r'finger_([lr])_joint([0-9]+)', name).groups()
        if (not path.startswith(robot + '/') or not body.startswith(robot + '/')
                or path in joint_paths or body in body_to_joint
                or body.rsplit('/', 1)[-1] != f'finger_{side}_link{index}'):
            raise ValueError(f'Invalid or duplicated hand joint/body path: {name}')
        joint_paths.add(path); body_to_joint[body] = name
        planned[name] = {'prim_path': path, 'body_path': body,
                         'drive': hand_drive_settings(joint.get('limits', {}), **drive_options),
                         'collider_paths': []}
    paths = metadata.get('robot_collider_prim_paths')
    if not isinstance(paths, (list, tuple)) or len(paths) != len(set(paths)):
        raise ValueError('Expected unique robot collider paths')
    selected = []
    for value in paths:
        path = _prim_path(value, 'Robot collider path')
        candidates = [body for body in body_to_joint
                      if path == body or path.startswith(body + '/')]
        if candidates:
            body = max(candidates, key=len)
            planned[body_to_joint[body]]['collider_paths'].append(path)
            selected.append(path)
    missing = [name for name, joint in planned.items() if not joint['collider_paths']]
    if missing:
        raise ValueError(f'Hand bodies have no metadata colliders: {missing}')
    return {'profile': PROFILE_OWNER, 'source': SOURCE_URL, 'source_sha256': SOURCE_SHA256,
            'robot_prim_path': robot, 'joints': planned,
            'material_path': robot + '/' + MATERIAL_NAME,
            'material': dict(FINGER_MATERIAL), 'collider_paths': sorted(selected),
            'hardware_force_calibrated': False,
            'gravity_and_self_collision_unchanged': True}


def author_hand_physics(stage, metadata, **drive_options):
    """Opt in on a caller-owned USD layer; never save or alter simulation state.

    Validate the full plan before authoring. Bind only the physics purpose,
    strongly, leaving render bindings and inherited CAD assets untouched.
    Targets, masses, inertia, gravity, contact offsets and collision filters
    are preserved. The scene builder must already de-instance colliders.
    """
    from pxr import PhysxSchema, UsdGeom, UsdPhysics, UsdShade

    plan = plan_hand_physics(metadata, **drive_options)
    if (not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1.0)
            or not math.isclose(UsdPhysics.GetStageKilogramsPerUnit(stage), 1.0)):
        raise ValueError('Hand physics requires meter/kilogram stage units')
    prims = {}
    for name, joint in plan['joints'].items():
        prim = stage.GetPrimAtPath(joint['prim_path'])
        body = stage.GetPrimAtPath(joint['body_path'])
        if (not prim or not prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsInstanceProxy()
                or not body or not body.HasAPI(UsdPhysics.RigidBodyAPI)):
            raise ValueError(f'Hand metadata does not match a writable joint/body: {name}')
        prims[name] = prim
    colliders = []
    for path in plan['collider_paths']:
        prim = stage.GetPrimAtPath(path)
        if (not prim or prim.IsInstanceProxy() or not prim.HasAPI(UsdPhysics.CollisionAPI)
                or not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()):
            raise ValueError(f'Expected writable active finger collider: {path}')
        colliders.append(prim)
    existing = stage.GetPrimAtPath(plan['material_path'])
    if existing and (not existing.IsA(UsdShade.Material)
                     or existing.GetCustomDataByKey('hx5HandPhysicsOwner') != PROFILE_OWNER):
        raise ValueError('Refusing to replace an unrelated hand physics material')

    material = UsdShade.Material.Define(stage, plan['material_path'])
    material.GetPrim().SetCustomDataByKey('hx5HandPhysicsOwner', PROFILE_OWNER)
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(FINGER_MATERIAL['static_friction'])
    physics.CreateDynamicFrictionAttr(FINGER_MATERIAL['dynamic_friction'])
    physics.CreateRestitutionAttr(FINGER_MATERIAL['restitution'])
    physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx.CreateFrictionCombineModeAttr(FINGER_MATERIAL['friction_combine_mode'])
    physx.CreateRestitutionCombineModeAttr(FINGER_MATERIAL['restitution_combine_mode'])
    for prim in colliders:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose='physics')
    for name, joint in plan['joints'].items():
        drive = UsdPhysics.DriveAPI.Apply(prims[name], 'angular')
        settings = joint['drive']
        drive.CreateTypeAttr(settings['type'])
        drive.CreateStiffnessAttr(settings['stiffness'])
        drive.CreateDampingAttr(settings['damping'])
        drive.CreateMaxForceAttr(settings['max_force'])
        PhysxSchema.PhysxJointAPI.Apply(prims[name]).CreateMaxJointVelocityAttr(
            settings['velocity_deg_s'])
    return plan
