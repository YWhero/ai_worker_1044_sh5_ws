"""Check open basket contact geometry and native pickable-goods properties."""
import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    'hx5_payload_physics', Path(__file__).resolve().parents[1]/'payload_physics.py')
payload = importlib.util.module_from_spec(spec)
spec.loader.exec_module(payload)


@pytest.fixture(scope='module')
def usd():
    Usd = pytest.importorskip('pxr.Usd')
    from pxr import Gf, UsdGeom, UsdPhysics
    # Register the matched optional provider before the first stage touches
    # USD's cached schema registry, including when this file runs alone.
    try:
        from pxr import Plug, PhysxSchema
    except ImportError:
        pass
    else:
        provider = Path(PhysxSchema.__file__).resolve().parents[2]
        Plug.Registry().RegisterPlugins(str(provider/'plugins/PhysxSchema/resources/plugInfo.json'))
    return Usd, Gf, UsdGeom, UsdPhysics


@pytest.fixture
def basket(usd):
    Usd, Gf, UsdGeom, UsdPhysics = usd
    stage = Usd.Stage.CreateInMemory()
    placement = UsdGeom.Xform.Define(stage, '/World/Basket')
    placement.AddTranslateOp().Set(Gf.Vec3d(.54, -.095, .75))
    placement.AddRotateZOp().Set(90.0)
    placement.GetPrim().SetCustomDataByKey('assetKind', 'basket')
    mesh = UsdGeom.Mesh.Define(stage, '/World/Basket/Scan/Mesh')
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    return stage, placement, mesh


def layout():
    return {'packing': {'interior_half_width': .143, 'interior_half_depth': .225,
                        'floor_z_in_basket': .032}}


def test_open_basket_proxy_preserves_placement_and_visible_scan(basket, usd):
    _, _, UsdGeom, UsdPhysics = usd
    stage, placement, mesh = basket
    before = placement.GetLocalTransformation()
    contents = UsdGeom.Mesh.Define(stage, '/World/Basket/Contents/object2_001/Scan/Mesh')
    UsdPhysics.CollisionAPI.Apply(contents.GetPrim())
    paths = payload.author_basket_colliders(stage, layout())
    assert len(paths) == 5
    assert not UsdPhysics.CollisionAPI(mesh.GetPrim()).GetCollisionEnabledAttr().Get()
    assert mesh.GetPrim().IsActive()
    assert mesh.ComputeVisibility() == UsdGeom.Tokens.inherited
    assert UsdPhysics.CollisionAPI(contents.GetPrim()).GetCollisionEnabledAttr().Get()
    assert placement.GetLocalTransformation() == before
    for path in paths:
        cube = UsdGeom.Cube(stage.GetPrimAtPath(path))
        assert cube.ComputeVisibility() == UsdGeom.Tokens.invisible
        assert cube.GetPurposeAttr().Get() == UsdGeom.Tokens.guide
        assert UsdPhysics.CollisionAPI(cube.GetPrim()).GetCollisionEnabledAttr().Get()
        ops = cube.GetOrderedXformOps()
        center, size = tuple(ops[0].Get()), tuple(ops[1].Get())
        if cube.GetPrim().GetName() == 'Floor':
            assert center[2]+size[2]/2 == pytest.approx(.032)
        else:
            assert center[2]+size[2]/2 == pytest.approx(.27)
            # Side-wall boxes do not cap the interior opening.
            assert (abs(center[0])-size[0]/2 >= .143-1e-10
                    or abs(center[1])-size[1]/2 >= .225-1e-10)
    assert payload.author_basket_colliders(stage, layout()) == paths
    assert placement.GetLocalTransformation() == before


def test_unrelated_proxy_prim_is_rejected_without_disabling_scan(basket, usd):
    _, _, UsdGeom, UsdPhysics = usd
    stage, _, mesh = basket
    UsdGeom.Xform.Define(stage, '/World/Basket/'+payload.PROXY_NAME)
    with pytest.raises(ValueError, match='unrelated'):
        payload.author_basket_colliders(stage, layout())
    assert UsdPhysics.CollisionAPI(mesh.GetPrim()).GetCollisionEnabledAttr().Get()


def test_owned_proxy_with_unexpected_noncollider_child_is_rejected(basket, usd):
    _, _, UsdGeom, _ = usd
    stage, _, _ = basket
    paths = payload.author_basket_colliders(stage, layout())
    UsdGeom.Mesh.Define(stage, paths[0])
    before = stage.GetRootLayer().ExportToString()
    with pytest.raises(ValueError, match='non-collider'):
        payload.author_basket_colliders(stage, layout())
    assert stage.GetRootLayer().ExportToString() == before


@pytest.mark.parametrize('value', [0, -0.1, float('nan'), float('inf')])
def test_invalid_dimensions_fail_before_scene_mutation(basket, usd, value):
    _, _, _, UsdPhysics = usd
    stage, _, mesh = basket
    malformed = layout()
    malformed['packing']['interior_half_width'] = value
    with pytest.raises(ValueError, match='finite and positive'):
        payload.author_basket_colliders(stage, malformed)
    assert UsdPhysics.CollisionAPI(mesh.GetPrim()).GetCollisionEnabledAttr().Get()
    assert not stage.GetPrimAtPath('/World/Basket/'+payload.PROXY_NAME)


def test_payload_remains_dynamic_with_bounded_overlap_recovery_and_single_hull(usd):
    Usd, _, UsdGeom, UsdPhysics = usd
    PhysxSchema = pytest.importorskip('pxr.PhysxSchema')
    from pxr import Plug
    provider = Path(PhysxSchema.__file__).resolve().parents[2]
    Plug.Registry().RegisterPlugins(str(provider/'plugins/PhysxSchema/resources/plugInfo.json'))
    stage = Usd.Stage.CreateInMemory()
    prim = UsdGeom.Xform.Define(stage, '/World/Payload').GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(.05)
    mesh = UsdGeom.Mesh.Define(stage, '/World/Payload/Scan/Mesh')
    payload.configure_payload_body(prim)
    payload.configure_payload_mesh(mesh)
    assert UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get()
    assert not UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get()
    assert UsdPhysics.MassAPI(prim).GetMassAttr().Get() == pytest.approx(.05)
    props = PhysxSchema.PhysxRigidBodyAPI(prim)
    assert props.GetEnableCCDAttr().Get()
    assert props.GetSolverPositionIterationCountAttr().Get() == 32
    assert props.GetSolverVelocityIterationCountAttr().Get() == 4
    assert props.GetMaxDepenetrationVelocityAttr().Get() == pytest.approx(.2)
    assert UsdPhysics.MeshCollisionAPI(mesh.GetPrim()).GetApproximationAttr().Get() == 'convexHull'
    assert PhysxSchema.PhysxConvexHullCollisionAPI(mesh.GetPrim()).GetHullVertexLimitAttr().Get() == 256
