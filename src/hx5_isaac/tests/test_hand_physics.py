"""Check units, torque/rate bounds and selective USD hand authoring on CPU."""
import importlib.util
import json
import math
from pathlib import Path

import pytest

MODULE = Path(__file__).resolve().parents[1] / 'hand_physics.py'
spec = importlib.util.spec_from_file_location('hx5_hand_physics', MODULE)
hand = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hand)
WORKSPACE = MODULE.parents[2]


def metadata():
    joints = {}
    colliders = ['/World/Robot/arm_l_link7/Collider', '/World/Robot/hx5_l_base/Collider']
    for side in ('l', 'r'):
        for index in range(1, 21):
            name = f'finger_{side}_joint{index}'
            first = ((index - 1) // 4) * 4 + 1
            chain = '/'.join(f'finger_{side}_link{i}' for i in range(first, index + 1))
            body = '/World/Robot/' + chain
            joints[name] = {'prim_path': '/World/Robot/Physics/' + name,
                            'body_path': body, 'type': 'revolute',
                            'limits': {'effort': 1000.0, 'velocity': 4.8}}
            colliders.append(body + '/Collider')
    return {'robot_prim_path': '/World/Robot', 'joints': joints,
            'robot_collider_prim_paths': colliders}


def test_official_native_gains_convert_once_to_usd_degrees():
    settings = hand.hand_drive_settings({'effort': 1000, 'velocity': 4.8})
    assert settings['stiffness'] == pytest.approx(8.726646259971648)
    assert settings['damping'] == pytest.approx(.5235987755982988)
    assert hand.usd_gain_to_native(settings['stiffness']) == pytest.approx(500)
    assert hand.usd_gain_to_native(settings['damping']) == pytest.approx(30)
    assert settings['max_force'] == 3.09
    assert settings['velocity_rad_s'] == 4.8
    assert settings['velocity_deg_s'] == pytest.approx(275.0197416627951)
    error_rad = .001
    assert settings['stiffness'] * math.degrees(error_rad) == pytest.approx(500 * error_rad)


def test_old_authored_drive_is_not_the_official_native_gain():
    assert hand.usd_gain_to_native(320) == pytest.approx(18334.649444186343)
    assert hand.usd_gain_to_native(8) == pytest.approx(458.3662361046586)


@pytest.mark.parametrize('effort,velocity,request_effort,request_velocity,expected', [
    (1000, 4.8, 9999, 9999, (3.09, 4.8)),
    (.9, 2, 3.09, 15, (.9, 2)),
    (1000, 4.8, .5, 1, (.5, 1)),
])
def test_limits_honor_urdf_official_ceiling_and_lower_trial_values(
        effort, velocity, request_effort, request_velocity, expected):
    result = hand.hand_drive_settings({'effort': effort, 'velocity': velocity},
                                      effort_limit=request_effort, velocity_limit=request_velocity)
    assert (result['max_force'], result['velocity_rad_s']) == expected


@pytest.mark.parametrize('field', ['stiffness', 'damping', 'effort_limit', 'velocity_limit'])
@pytest.mark.parametrize('value', [-1, float('nan'), float('inf'), None])
def test_nonfinite_or_negative_options_fail(field, value):
    with pytest.raises(ValueError, match='finite'):
        hand.hand_drive_settings({'effort': 1000, 'velocity': 4.8}, **{field: value})


@pytest.mark.parametrize('limits', [{}, {'effort': 1000}, {'velocity': 4.8},
                                    {'effort': 0, 'velocity': 4.8},
                                    {'effort': float('inf'), 'velocity': 4.8}])
def test_limits_must_be_explicit_finite_and_positive(limits):
    with pytest.raises(ValueError):
        hand.hand_drive_settings(limits)


def test_plan_selects_every_link_and_assigns_nested_colliders_to_closest_body():
    result = hand.plan_hand_physics(metadata())
    assert len(result['joints']) == len(result['collider_paths']) == 40
    for joint in result['joints'].values():
        assert joint['collider_paths'] == [joint['body_path'] + '/Collider']
    assert not any('arm_l_link7' in path or 'hx5_l_base' in path for path in result['collider_paths'])
    assert result['hardware_force_calibrated'] is False
    assert result['gravity_and_self_collision_unchanged'] is True


def test_real_import_metadata_selects_all_40_finger_collision_bodies():
    path = WORKSPACE / 'simulation/isaac/assets/sh5/scene_metadata.json'
    result = hand.plan_hand_physics(json.loads(path.read_text()))
    assert len(result['joints']) == 40
    assert all(joint['collider_paths'] for joint in result['joints'].values())
    assert all(joint['drive']['velocity_rad_s'] == 4.8 for joint in result['joints'].values())


@pytest.mark.parametrize('kind', ['missing_joint', 'duplicate_body', 'duplicate_joint',
                                  'missing_collider', 'duplicate_collider', 'wrong_body',
                                  'bad_path'])
def test_malformed_metadata_is_rejected(kind):
    source = metadata()
    left = source['joints']['finger_l_joint1']
    if kind == 'missing_joint':
        del source['joints']['finger_l_joint1']
    elif kind == 'duplicate_body':
        source['joints']['finger_r_joint1']['body_path'] = left['body_path']
    elif kind == 'duplicate_joint':
        source['joints']['finger_r_joint1']['prim_path'] = left['prim_path']
    elif kind == 'missing_collider':
        source['robot_collider_prim_paths'].remove(left['body_path'] + '/Collider')
    elif kind == 'duplicate_collider':
        source['robot_collider_prim_paths'].append(source['robot_collider_prim_paths'][-1])
    elif kind == 'wrong_body':
        left['body_path'] = '/World/Robot/arm_l_link7'
    else:
        left['body_path'] = '/World/Robot/../finger_l_link1'
    with pytest.raises(ValueError):
        hand.plan_hand_physics(source)


@pytest.fixture(scope='module')
def usd():
    pytest.importorskip('pxr.Usd')
    PhysxSchema = pytest.importorskip('pxr.PhysxSchema')
    from pxr import Plug, Usd, UsdGeom, UsdPhysics, UsdShade
    provider = Path(PhysxSchema.__file__).resolve().parents[2]
    Plug.Registry().RegisterPlugins(str(provider / 'plugins/PhysxSchema/resources/plugInfo.json'))
    return Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema


@pytest.fixture
def hand_stage(usd):
    Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema = usd
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    source = metadata()
    render = UsdShade.Material.Define(stage, '/World/RenderMaterial')
    default = UsdShade.Material.Define(stage, '/World/DefaultPhysicsMaterial')
    UsdPhysics.MaterialAPI.Apply(default.GetPrim()).CreateStaticFrictionAttr(1.0)
    for path in source['robot_collider_prim_paths']:
        prim = UsdGeom.Cube.Define(stage, path).GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
        PhysxSchema.PhysxCollisionAPI.Apply(prim).CreateContactOffsetAttr(.001)
        binding = UsdShade.MaterialBindingAPI.Apply(prim)
        binding.Bind(render)
        binding.Bind(default, materialPurpose='physics')
    for joint in source['joints'].values():
        body = UsdGeom.Xform.Define(stage, joint['body_path']).GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(body)
        UsdPhysics.MassAPI.Apply(body).CreateMassAttr(.04)
        PhysxSchema.PhysxRigidBodyAPI.Apply(body).CreateDisableGravityAttr(False)
        prim = UsdPhysics.RevoluteJoint.Define(stage, joint['prim_path']).GetPrim()
        drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
        drive.CreateStiffnessAttr(320)
        drive.CreateDampingAttr(8)
        drive.CreateMaxForceAttr(1000)
        drive.CreateTargetPositionAttr(30)
    return stage, source


def test_authoring_preserves_render_material_targets_mass_gravity_and_nonhand_shapes(hand_stage, usd):
    _, _, UsdPhysics, UsdShade, PhysxSchema = usd
    stage, source = hand_stage
    unrelated = [source['robot_collider_prim_paths'][0], source['robot_collider_prim_paths'][1]]
    before = {path: str(stage.GetPrimAtPath(path).GetPrimStack()[0]) for path in unrelated}
    result = hand.author_hand_physics(stage, source)
    assert len(result['collider_paths']) == 40
    for path in result['collider_paths']:
        prim = stage.GetPrimAtPath(path)
        bindings = UsdShade.MaterialBindingAPI(prim)
        assert str(bindings.ComputeBoundMaterial()[0].GetPath()) == '/World/RenderMaterial'
        assert str(bindings.ComputeBoundMaterial('physics')[0].GetPath()) == result['material_path']
        assert bindings.GetMaterialBindingStrength(bindings.GetDirectBindingRel('physics')) == 'strongerThanDescendants'
        assert PhysxSchema.PhysxCollisionAPI(prim).GetContactOffsetAttr().Get() == pytest.approx(.001)
    for joint in source['joints'].values():
        prim = stage.GetPrimAtPath(joint['prim_path'])
        drive = UsdPhysics.DriveAPI(prim, 'angular')
        assert drive.GetTargetPositionAttr().Get() == 30
        assert drive.GetMaxForceAttr().Get() == pytest.approx(3.09)
        assert hand.usd_gain_to_native(drive.GetStiffnessAttr().Get()) == pytest.approx(500, rel=1e-6)
        body = stage.GetPrimAtPath(joint['body_path'])
        assert UsdPhysics.MassAPI(body).GetMassAttr().Get() == pytest.approx(.04)
        assert PhysxSchema.PhysxRigidBodyAPI(body).GetDisableGravityAttr().Get() is False
    mat = stage.GetPrimAtPath(result['material_path'])
    assert PhysxSchema.PhysxMaterialAPI(mat).GetFrictionCombineModeAttr().Get() == 'max'
    assert PhysxSchema.PhysxMaterialAPI(mat).GetRestitutionCombineModeAttr().Get() == 'min'
    assert UsdPhysics.MaterialAPI(mat).GetStaticFrictionAttr().Get() == 2.0
    assert UsdPhysics.MaterialAPI(mat).GetDynamicFrictionAttr().Get() == pytest.approx(1.8)
    assert {path: str(stage.GetPrimAtPath(path).GetPrimStack()[0]) for path in unrelated} == before
    first = stage.GetRootLayer().ExportToString()
    hand.author_hand_physics(stage, source)
    assert stage.GetRootLayer().ExportToString() == first


@pytest.mark.parametrize('kind', ['disabled_collider', 'missing_collider', 'missing_body',
                                  'wrong_joint_type', 'foreign_material', 'wrong_units'])
def test_invalid_stage_fails_before_any_authoring(hand_stage, usd, kind):
    _, UsdGeom, UsdPhysics, UsdShade, _ = usd
    stage, source = hand_stage
    first = source['joints']['finger_l_joint1']
    path = first['body_path'] + '/Collider'
    if kind == 'disabled_collider':
        UsdPhysics.CollisionAPI(stage.GetPrimAtPath(path)).CreateCollisionEnabledAttr(False)
    elif kind == 'missing_collider':
        stage.RemovePrim(path)
    elif kind == 'missing_body':
        stage.GetPrimAtPath(first['body_path']).RemoveAPI(UsdPhysics.RigidBodyAPI)
    elif kind == 'wrong_joint_type':
        UsdPhysics.PrismaticJoint.Define(stage, first['prim_path'])
    elif kind == 'foreign_material':
        UsdShade.Material.Define(stage, source['robot_prim_path'] + '/' + hand.MATERIAL_NAME)
    else:
        UsdGeom.SetStageMetersPerUnit(stage, .01)
    before = stage.GetRootLayer().ExportToString()
    with pytest.raises(ValueError):
        hand.author_hand_physics(stage, source)
    assert stage.GetRootLayer().ExportToString() == before


def test_cpu_existing_stage_smoke_keeps_source_bytes_and_all_nonprofile_properties(hand_stage, tmp_path):
    stage, source = hand_stage
    smoke_spec = importlib.util.spec_from_file_location(
        'hx5_hand_authoring_smoke', Path(__file__).with_name('hand_physics_authoring_smoke.py'))
    smoke = importlib.util.module_from_spec(smoke_spec)
    smoke_spec.loader.exec_module(smoke)
    stage_path = tmp_path / 'source.usda'
    metadata_path = tmp_path / 'metadata.json'
    stage.GetRootLayer().Export(str(stage_path))
    metadata_path.write_text(json.dumps(source))
    before = stage_path.read_bytes()
    receipt = smoke.verify(stage_path, metadata_path)
    assert receipt['passed'] is True
    assert receipt['joint_count'] == receipt['finger_collider_count'] == 40
    assert receipt['all_existing_non_profile_properties_unchanged'] is True
    assert receipt['usd_saved'] is False
    assert stage_path.read_bytes() == before
