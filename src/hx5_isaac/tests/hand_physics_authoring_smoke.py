#!/usr/bin/env python3
"""CPU-only hand-profile check on the real composed cell, never saving USD.

Run with matched USD/Physx libraries, not SimulationApp. Overrides live only in
an anonymous layer; --report writes JSON evidence to a separate chosen path.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hand_physics import author_hand_physics, plan_hand_physics, usd_gain_to_native


def _snapshot(stage, joint_paths, collider_paths):
    """Record everything except this profile's explicitly authored fields."""
    from pxr import Usd
    allowed_joint = {'drive:angular:physics:type', 'drive:angular:physics:stiffness',
                     'drive:angular:physics:damping', 'drive:angular:physics:maxForce',
                     'physxJoint:maxJointVelocity'}
    snapshot = {}
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        info = {key: str(value) for key, value in prim.GetAllMetadata().items()
                if key not in ('apiSchemas',)}
        properties = {}
        # Applying PhysxJointAPI exposes additional unauthored schema defaults;
        # compare concrete authored properties rather than virtual defaults.
        for prop in prim.GetAuthoredProperties():
            name = prop.GetName()
            if ((path in joint_paths and name in allowed_joint)
                    or (path in collider_paths and name == 'material:binding:physics')):
                continue
            value = prop.GetTargets() if isinstance(prop, Usd.Relationship) else prop.Get()
            properties[name] = {'value': str(value), 'metadata': str(prop.GetAllMetadata())}
        # Allow only the joint/collider API schemas to change, never body or
        # environment schema metadata. All other prim metadata stays equal.
        if path not in joint_paths and path not in collider_paths:
            info['apiSchemas'] = str(prim.GetMetadata('apiSchemas'))
        snapshot[path] = {'metadata': info, 'properties': properties}
    return snapshot


def verify(stage_path, metadata_path):
    from pxr import Plug, Usd, UsdGeom, UsdShade, UsdPhysics, PhysxSchema

    provider = Path(PhysxSchema.__file__).resolve().parents[2]
    Plug.Registry().RegisterPlugins(str(provider / 'plugins/PhysxSchema/resources/plugInfo.json'))

    metadata = json.loads(Path(metadata_path).read_text())
    plan = plan_hand_physics(metadata)
    source_path = Path(stage_path).resolve()
    source_hash = hashlib.sha256(source_path.read_bytes()).hexdigest()
    # A fresh root layer references the real stage; no imported/current layer
    # receives edits and no timeline or physics solver is instantiated.
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [str(source_path)]
    # Stage-level metric metadata is read from the root layer, rather than
    # inherited through sublayers; explicitly mirror the source metrics.
    source_stage = Usd.Stage.Open(str(source_path))
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(source_stage))
    UsdPhysics.SetStageKilogramsPerUnit(stage, UsdPhysics.GetStageKilogramsPerUnit(source_stage))
    joint_paths = {joint['prim_path'] for joint in plan['joints'].values()}
    collider_paths = set(plan['collider_paths'])
    before = _snapshot(stage, joint_paths, collider_paths)
    receipt = author_hand_physics(stage, metadata)
    after = _snapshot(stage, joint_paths, collider_paths)
    # Only the owned material is new; every inherited prim's remaining
    # properties, including finger mass/inertia/targets, must stay equal.
    changed = [path for path in before if before[path] != after.get(path)]
    if changed:
        raise ValueError(f'Unexpected changes outside hand profile fields: {changed[:10]}')
    new_paths = set(after) - set(before)
    expected_new_paths = {receipt['material_path']} - set(before)
    if new_paths != expected_new_paths:
        raise ValueError(f'Unexpected added prims: {sorted(new_paths)}')
    drives = {}
    for name, joint in receipt['joints'].items():
        prim = stage.GetPrimAtPath(joint['prim_path'])
        drive = UsdPhysics.DriveAPI(prim, 'angular')
        settings = joint['drive']
        actual = {'native_stiffness': usd_gain_to_native(drive.GetStiffnessAttr().Get()),
                  'native_damping': usd_gain_to_native(drive.GetDampingAttr().Get()),
                  'max_force_Nm': drive.GetMaxForceAttr().Get(),
                  'velocity_rad_s': math.radians(
                      PhysxSchema.PhysxJointAPI(prim).GetMaxJointVelocityAttr().Get())}
        for key, expected in [('native_stiffness', settings['native_stiffness']),
                              ('native_damping', settings['native_damping']),
                              ('max_force_Nm', settings['max_force']),
                              ('velocity_rad_s', settings['velocity_rad_s'])]:
            if not math.isclose(actual[key], expected, rel_tol=1e-6, abs_tol=1e-7):
                raise ValueError(f'Wrong authored hand drive {name}: {actual}')
        if drive.GetTypeAttr().Get() != 'force':
            raise ValueError(f'Wrong hand drive type: {name}')
        drives[name] = actual
    for path in collider_paths:
        binding = UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(path))
        if (str(binding.ComputeBoundMaterial('physics')[0].GetPath()) != receipt['material_path']
                or binding.GetMaterialBindingStrength(binding.GetDirectBindingRel('physics'))
                != UsdShade.Tokens.strongerThanDescendants):
            raise ValueError(f'Wrong composed hand physics binding: {path}')
    if hashlib.sha256(source_path.read_bytes()).hexdigest() != source_hash:
        raise ValueError('Source stage unexpectedly changed')
    return {'passed': True, 'source_stage': str(source_path), 'source_stage_sha256': source_hash,
            'metadata': str(Path(metadata_path).resolve()),
            'source': receipt['source'], 'source_sha256': receipt['source_sha256'],
            'joint_count': len(drives), 'finger_collider_count': len(collider_paths),
            'all_existing_non_profile_properties_unchanged': True,
            'hardware_force_calibrated': False, 'usd_saved': False,
            'joint_drives': drives, 'material': receipt['material']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', type=Path, required=True)
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    try:
        report = verify(args.stage, args.metadata)
    except Exception as exc:
        report = {'passed': False, 'error': str(exc), 'usd_saved': False}
    if args.report:
        if args.report.resolve() in {args.stage.resolve(), args.metadata.resolve()}:
            parser.error('Report must be separate from source stage and metadata')
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
