"""Match runtime goods/basket collision geometry to the saved gravity-drop pile.

These are simulation assumptions, not measured robot or packaging properties.
Only the caller's USD edit target is authored; visual assets and placements are
preserved. Collision material/contact offsets remain the scene builder's job.
"""
from __future__ import annotations

import math

PROXY_NAME = 'HX5BasketColliders'
PROXY_OWNER = 'matching_settled_piles_v1'
WALL_THICKNESS = 0.02
WALL_HEIGHT = 0.27


def _packing_dimensions(layout):
    packing = layout.get('packing', {})
    result = []
    for name in ('interior_half_width', 'interior_half_depth', 'floor_z_in_basket'):
        try:
            value = float(packing[name])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'Missing or invalid settled basket dimension: {name}') from exc
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'Settled basket dimension must be finite and positive: {name}')
        result.append(value)
    if result[2] >= WALL_HEIGHT:
        raise ValueError('Settled basket floor must be below the wall top')
    return tuple(result)


def author_basket_colliders(stage, layout):
    """Use the same open-top five-box basket as ``settle_piles.py``.

    Disable collisions on the inherited scanned Mesh, without hiding it.
    Return the new collider paths so contact filters can use the active
    physical shapes. Reject unrelated existing prims before authoring anything.
    """
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    halfx, halfy, floor = _packing_dimensions(layout)
    thickness = WALL_THICKNESS
    specifications = [
        ('Floor', (2*halfx+2*thickness, 2*halfy+2*thickness, thickness),
         (0.0, 0.0, floor-thickness/2)),
        ('WallX0', (thickness, 2*halfy+2*thickness, WALL_HEIGHT),
         (-halfx-thickness/2, 0.0, WALL_HEIGHT/2)),
        ('WallX2', (thickness, 2*halfy+2*thickness, WALL_HEIGHT),
         (halfx+thickness/2, 0.0, WALL_HEIGHT/2)),
        ('WallY0', (2*halfx, thickness, WALL_HEIGHT),
         (0.0, -halfy-thickness/2, WALL_HEIGHT/2)),
        ('WallY2', (2*halfx, thickness, WALL_HEIGHT),
         (0.0, halfy+thickness/2, WALL_HEIGHT/2)),
    ]
    baskets = [prim for prim in stage.Traverse()
               if prim.GetCustomDataByKey('assetKind') == 'basket']
    scanned_meshes = {}
    for basket in baskets:
        # Goods live under basket/Contents: only the basket's own Scan
        # reference should lose triangle collisions, never its contents.
        scan = basket.GetChild('Scan')
        meshes = [prim for prim in Usd.PrimRange(scan) if prim.IsA(UsdGeom.Mesh)] if scan else []
        if not meshes:
            raise ValueError(f'Basket has no scanned visual geometry: {basket.GetPath()}')
        scanned_meshes[str(basket.GetPath())] = meshes
        proxy_path = basket.GetPath().AppendChild(PROXY_NAME)
        existing = stage.GetPrimAtPath(proxy_path)
        if existing:
            if (not existing.IsA(UsdGeom.Xform)
                    or existing.GetCustomDataByKey('hx5BasketProxyOwner') != PROXY_OWNER):
                raise ValueError(f'Refusing to overwrite an unrelated basket prim: {proxy_path}')
            expected = {name for name, _, _ in specifications}
            if {child.GetName() for child in existing.GetChildren()} != expected:
                raise ValueError(f'Unexpected basket proxy children: {proxy_path}')
            for name in expected:
                child = stage.GetPrimAtPath(proxy_path.AppendChild(name))
                if not child.IsA(UsdGeom.Cube) or not child.HasAPI(UsdPhysics.CollisionAPI):
                    raise ValueError(f'Unexpected non-collider basket proxy prim: {child.GetPath()}')
        for mesh in meshes:
            if mesh.IsInstanceProxy():
                raise ValueError(f'Cannot override instanced basket collision: {mesh.GetPath()}')

    paths = []
    for basket in baskets:
        for mesh in scanned_meshes[str(basket.GetPath())]:
            if mesh.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(mesh).CreateCollisionEnabledAttr(False)
        proxy_path = basket.GetPath().AppendChild(PROXY_NAME)
        proxy = UsdGeom.Xform.Define(stage, proxy_path)
        proxy.GetPrim().SetCustomDataByKey('hx5BasketProxyOwner', PROXY_OWNER)
        for name, size, position in specifications:
            cube = UsdGeom.Cube.Define(stage, proxy_path.AppendChild(name))
            cube.CreateSizeAttr(1.0)
            cube.CreateExtentAttr([Gf.Vec3f(-0.5), Gf.Vec3f(0.5)])
            cube.CreatePurposeAttr(UsdGeom.Tokens.guide)
            cube.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
            cube.ClearXformOpOrder()
            # Reuse named ops for idempotent scene composition.
            translate = cube.GetPrim().GetAttribute('xformOp:translate')
            scale = cube.GetPrim().GetAttribute('xformOp:scale')
            translate_op = (UsdGeom.XformOp(translate) if translate else
                            cube.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble))
            scale_op = (UsdGeom.XformOp(scale) if scale else
                        cube.AddScaleOp(UsdGeom.XformOp.PrecisionDouble))
            translate_op.Set(Gf.Vec3d(*position))
            scale_op.Set(Gf.Vec3d(*size))
            cube.SetXformOpOrder([translate_op, scale_op])
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
            paths.append(str(cube.GetPath()))
    return paths


def configure_payload_body(prim):
    """Keep pickable goods dynamic and bound overlap-recovery impulses."""
    from pxr import PhysxSchema, UsdPhysics

    if not prim or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        raise ValueError('Payload tuning requires an existing rigid body')
    body = UsdPhysics.RigidBodyAPI(prim)
    body.CreateRigidBodyEnabledAttr(True)
    body.CreateKinematicEnabledAttr(False)
    physx = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    physx.CreateEnableCCDAttr(True)
    physx.CreateSolverPositionIterationCountAttr(32)
    physx.CreateSolverVelocityIterationCountAttr(4)
    physx.CreateLinearDampingAttr(0.3)
    physx.CreateAngularDampingAttr(0.5)
    physx.CreateMaxDepenetrationVelocityAttr(0.2)


def configure_payload_mesh(mesh):
    """Use the single scan hull used to generate the saved pile."""
    from pxr import PhysxSchema, UsdGeom, UsdPhysics

    prim = mesh.GetPrim() if isinstance(mesh, UsdGeom.Mesh) else mesh
    if not prim or not prim.IsA(UsdGeom.Mesh):
        raise ValueError('Payload hull requires a Mesh prim')
    UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
    PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim).CreateHullVertexLimitAttr(256)
