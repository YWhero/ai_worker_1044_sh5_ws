"""Validate description/pose contracts without starting Isaac or touching source assets."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import types

import pytest
import yaml

MODULE = Path(__file__).resolve().parents[1] / 'scene_builder.py'
sys.path.insert(0, str(MODULE.parent))
spec = importlib.util.spec_from_file_location('hx5_scene_builder', MODULE)
scene = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scene)


@pytest.fixture(scope='session', autouse=True)
def register_optional_physx_provider():
    # USD caches its schema registry on first stage use; register the matched
    # optional provider before inspecting any generated stage in CPU tests.
    try:
        from pxr import Plug, PhysxSchema
    except ImportError:
        return
    provider = Path(PhysxSchema.__file__).resolve().parents[2]
    Plug.Registry().RegisterPlugins(str(provider/'plugins/PhysxSchema/resources/plugInfo.json'))


@pytest.fixture
def description():
    return scene.read_robot_description(scene.DEFAULT_URDF)


def test_real_export_has_63_independent_driven_joints_and_live_lift(description):
    actuated = [j for j in description['joints'].values() if j['type'] != 'fixed']
    assert len(actuated) == 63
    assert description['joints']['lift_joint']['type'] == 'prismatic'
    assert description['joints']['finger_l_joint2']['limit']['upper'] == 1.57
    assert description['joints']['finger_r_joint2']['limit']['lower'] == -1.57
    assert len(scene.POSITION_JOINT_NAMES) == 57


def test_reference_57_pose_is_inside_official_hx5_and_head_limits(description):
    pose = yaml.safe_load(scene.DEFAULT_POSE.read_text())
    checked = scene.validate_initial_positions(description, pose['initial_positions'])
    assert checked['head_joint1'] == 0.6951
    assert checked['head_joint2'] == checked['lift_joint'] == 0
    assert checked['finger_l_joint2'] == 1.5
    assert checked['finger_r_joint2'] == -1.5
    assert pose['metadata']['input_dependent'] is True


@pytest.mark.parametrize('value', [1.58, float('nan'), float('inf')])
def test_reject_unsafe_hand_initial_pose(description, value):
    pose = yaml.safe_load(scene.DEFAULT_POSE.read_text())['initial_positions']
    pose['finger_l_joint2'] = value
    with pytest.raises(ValueError, match='URDF limit'):
        scene.validate_initial_positions(description, pose)


def test_partial_pose_fails_closed(description):
    with pytest.raises(ValueError, match='all 57'):
        scene.validate_initial_positions(description, {'lift_joint': 0})


def test_camera_chain_merges_to_head_body_without_losing_stereo_baseline(description):
    left, left_tf = scene.fixed_mount(description, 'zedm_left_camera_optical_frame')
    right, right_tf = scene.fixed_mount(description, 'zedm_right_camera_optical_frame')
    assert left == right == 'head_link2'
    assert left_tf[0][1] - right_tf[0][1] == pytest.approx(0.063)
    assert sum(v*v for v in left_tf[1]) == pytest.approx(1)
    assert scene.qrotate(left_tf[1], (0, 0, 1)) == pytest.approx((1, 0, 0), abs=1e-10)


def test_finger_tips_merge_to_distal_bodies(description):
    for side in ('l', 'r'):
        for i, distal in enumerate((4, 8, 12, 16, 20), 1):
            body, transform = scene.fixed_mount(description, f'finger_end_{side}_link{i}')
            assert body == f'finger_{side}_link{distal}'
            assert sum(v*v for v in transform[0]) > 0
            # Official thumb tip points along +/-Y; the other tips along +Z.
            assert transform[0][2] > 0 if i > 1 else abs(transform[0][1]) > 0


def test_wheels_velocity_drive_and_lift_remains_position_drive(description):
    wheel = scene.drive_settings('left_wheel_drive', description)
    lift = scene.drive_settings('lift_joint', description)
    assert wheel['mode'] == 'velocity' and wheel['stiffness'] == 0 and wheel['damping'] > 0
    assert lift['mode'] == 'position' and lift['stiffness'] > 0
    assert lift['stiffness'] == 100000 and lift['damping'] == 4000
    assert lift['max_force'] == description['joints']['lift_joint']['limit']['effort']


def test_simulation_finger_gains_keep_official_effort_and_position_mode(description):
    for side in ('l', 'r'):
        for index in range(1, 21):
            name = f'finger_{side}_joint{index}'
            settings = scene.drive_settings(name, description)
            assert settings['stiffness'] == 320.0 and settings['damping'] == 8.0
            assert settings['mode'] == 'position' and settings['type'] == 'force'
            assert settings['max_force'] == description['joints'][name]['limit']['effort']


def test_reported_production_thumb_error_exceeds_gate_without_changing_reference(description):
    reference = yaml.safe_load(scene.DEFAULT_POSE.read_text())['initial_positions']
    measured_left_thumb = -0.280809
    error = abs(measured_left_thumb - reference['finger_l_joint1'])
    assert error == pytest.approx(0.0135416536)
    assert error > 0.01
    # Preserve the requested model-derived reference; drive tuning must
    # not make this failed measurement pass by shifting the desired pose.
    assert reference['finger_l_joint1'] == pytest.approx(-0.2672673463821411)
    settings = scene.drive_settings('finger_l_joint1', description)
    assert settings['stiffness'] == 320 and settings['damping'] == 8


def test_measured_native_pd_solution_fits_existing_gate_tolerance(description):
    measured_thumb_error_rad = 0.003366
    measured_lift_error_m = 0.00279976
    assert measured_thumb_error_rad < 0.01 and measured_lift_error_m < 0.01
    hand = scene.drive_settings('finger_l_joint1', description)
    # Native Isaac gain conversion multiplies angular raw USD gains by
    # degrees/radian. These values are degree gains, not native radian gains.
    assert hand['stiffness'] * math.degrees(1) == pytest.approx(18334.64944)
    assert hand['damping'] * math.degrees(1) == pytest.approx(458.366236)


def test_transform_composition_rotates_translation_correctly():
    transform = scene.compose(((1, 2, 3), (math.sqrt(0.5), 0, 0, math.sqrt(0.5))),
                              ((1, 0, 0), (1, 0, 0, 0)))
    assert transform[0] == pytest.approx((1, 3, 3))


def test_isolated_wrapper_kit_args_are_accepted_without_swallowing_typos():
    scene.validate_kit_args(['--portable-root', '/tmp/cell portable',
                            '--/app/settings/persistent=false', '--/log/file=/tmp/isaac.log',
                            '--/app/window/title=Isaac Sim | Isolated Logistics Cell'])
    with pytest.raises(ValueError, match='non-Kit'):
        scene.validate_kit_args(['--floor-fricion=1'])
    with pytest.raises(ValueError, match='requires'):
        scene.validate_kit_args(['--portable-root', '--/log/file=/tmp/x'])


@pytest.mark.parametrize('fail', [False, True])
def test_fast_shutdown_preserves_build_success_and_failure_status(monkeypatch, tmp_path, fail):
    closes, configs = [], []
    class FakeApp:
        def __init__(self, config):
            configs.append(config)
        def update(self):
            pass
        def close(self, *, exit_code):
            closes.append(exit_code)
    modules = ['isaacsim', 'isaacsim.core', 'isaacsim.core.experimental',
               'isaacsim.core.experimental.utils', 'isaacsim.core.experimental.utils.app']
    for name in modules:
        module = types.ModuleType(name)
        module.__path__ = []
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules['isaacsim'].SimulationApp = FakeApp
    sys.modules[modules[-1]].enable_extension = lambda extension: None
    def compose(*args, **kwargs):
        if fail:
            raise RuntimeError('simulated composition validation failure')
        return {'ok': True}
    monkeypatch.setattr(scene, 'compose_scene', compose)
    receipt = tmp_path/'receipt.json'
    monkeypatch.setattr(sys, 'argv', ['scene_builder.py', '--robot-usd', '/tmp/robot.usda',
                                     '--output', str(tmp_path/'stage.usda'), '--metadata', str(receipt)])
    if fail:
        with pytest.raises(RuntimeError, match='composition validation failure'):
            scene.main()
        assert not receipt.exists()
    else:
        scene.main()
        assert json.loads(receipt.read_text()) == {'ok': True}
    assert configs[0]['fast_shutdown'] is True
    assert closes == [1 if fail else 0]


def test_generated_usd_keeps_cell_placements_and_has_actual_dynamic_shapes():
    """Integration receipt is optional until the official importer has run."""
    Usd = pytest.importorskip('pxr.Usd')
    from pxr import UsdGeom, UsdPhysics
    metadata_path = scene.DEFAULT_OUTPUT.with_name('scene_metadata.json')
    if not metadata_path.is_file():
        pytest.skip('Run scene_builder.py to create the reviewed Isaac stage')
    metadata = json.loads(metadata_path.read_text())
    stage = Usd.Stage.Open(metadata['stage'])
    environment = Usd.Stage.Open(metadata['source_environment'])
    assert stage.GetTimeCodesPerSecond() == environment.GetTimeCodesPerSecond()
    assert stage.GetFramesPerSecond() == environment.GetFramesPerSecond()
    assert stage.GetStartTimeCode() == environment.GetStartTimeCode()
    assert stage.GetEndTimeCode() == environment.GetEndTimeCode()
    assert not stage.GetPrimAtPath('/World/Robot').IsActive()
    base = stage.GetPrimAtPath(metadata['articulation_root_path'])
    assert base.HasAPI(UsdPhysics.ArticulationRootAPI)
    assert base.HasAPI(UsdPhysics.RigidBodyAPI)
    lift = stage.GetPrimAtPath(metadata['joints']['lift_joint']['prim_path'])
    assert lift.IsActive() and lift.IsA(UsdPhysics.PrismaticJoint)
    assert lift.GetAttribute('physxJoint:maxJointVelocity').Get() == pytest.approx(4.8)
    arm = stage.GetPrimAtPath(metadata['joints']['arm_l_joint1']['prim_path'])
    assert arm.GetAttribute('physxJoint:maxJointVelocity').Get() == pytest.approx(math.degrees(4.8))
    assert len(metadata['joints']) == 63
    assert len(metadata['robot_collider_prim_paths']) >= 63
    assert len(metadata['cameras']) == 4 and len(metadata['contact_tips']) == 10
    for tip in metadata['contact_tips'].values():
        assert tip['collider_paths']
        assert stage.GetPrimAtPath(tip['rigid_body_path']).HasAPI(UsdPhysics.RigidBodyAPI)
        assert stage.GetPrimAtPath(tip['frame_path']).IsA(UsdGeom.Xform)
        for path in tip['collider_paths']:
            assert stage.GetPrimAtPath(path).HasAPI(UsdPhysics.CollisionAPI)
    assert len(metadata['dynamic_objects']) == 72
    for obj in metadata['dynamic_objects']:
        prim = stage.GetPrimAtPath(obj['prim_path'])
        source = environment.GetPrimAtPath(obj['prim_path'])
        assert prim.HasAPI(UsdPhysics.RigidBodyAPI)
        assert UsdPhysics.MassAPI(prim).GetMassAttr().Get() == pytest.approx(obj['mass_kg'])
        assert UsdGeom.Xformable(prim).GetLocalTransformation() == UsdGeom.Xformable(source).GetLocalTransformation()
        for path in obj['collider_paths']:
            assert UsdPhysics.MeshCollisionAPI(stage.GetPrimAtPath(path)).GetApproximationAttr().Get() == 'convexHull'
    assert metadata['initial_positions'] == yaml.safe_load(scene.DEFAULT_POSE.read_text())['initial_positions']
    assert scene.sha256(metadata['source_environment']) == metadata['source_environment_sha256']


def test_failed_composition_preserves_previous_stage(tmp_path):
    """A malformed scene must not replace a previously usable default USD."""
    pytest.importorskip('pxr.Usd')
    PhysxSchema = pytest.importorskip('pxr.PhysxSchema')
    from pxr import Plug
    provider = Path(PhysxSchema.__file__).resolve().parents[2]
    Plug.Registry().RegisterPlugins(str(provider/'plugins/PhysxSchema/resources/plugInfo.json'))
    metadata_path = scene.DEFAULT_OUTPUT.with_name('scene_metadata.json')
    if not metadata_path.is_file():
        pytest.skip('The private robot import has not run yet')
    receipt = json.loads(metadata_path.read_text())
    pose = yaml.safe_load(scene.DEFAULT_POSE.read_text())
    layout = json.loads((scene.DEFAULT_ENVIRONMENT.parent/'config/layout.json').read_text())
    layout['packing']['object1']['count'] += 1
    output = tmp_path/'default.usda'
    output.write_bytes(b'previous usable stage')
    with pytest.raises(RuntimeError, match='Expected 73'):
        scene.compose_scene(receipt['source_environment'], receipt['robot_usd'], receipt['source_urdf'],
                            pose, output, layout)
    assert output.read_bytes() == b'previous usable stage'
