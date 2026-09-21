"""Clear optical apertures in the visual-only SH5 head CAD overlay.

The official camera frames are inside simplified opaque STL housings. Those
STLs describe the robot's appearance/collision envelope, not the transmissive
camera optics. This helper clips only the head/ZED *visual* shell polygons in
the two head cameras' optical frusta. It never edits source CAD, collision
geometry, camera extrinsics, intrinsics, clipping distances, or other links.

This is an optical-clearance approximation, not a calibrated head CAD model.
All USD imports are deferred so the clipping math is independently testable.
"""
from __future__ import annotations

EPSILON = 1e-10


def _dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def _rotate(q, p):
    w, x, y, z = q
    cross = (y*p[2]-z*p[1], z*p[0]-x*p[2], x*p[1]-y*p[0])
    second = (y*cross[2]-z*cross[1], z*cross[0]-x*cross[2], x*cross[1]-y*cross[0])
    return tuple(p[i]+2*(w*cross[i]+second[i]) for i in range(3))


def optical_frustum(spec):
    """Five inward planes, in the surviving rigid body's coordinate frame."""
    center, q = spec['mount_xyz'], spec['mount_quat_wxyz']
    forward = _rotate(q, (0, 0, 1))
    right, down = _rotate(q, (1, 0, 0)), _rotate(q, (0, 1, 0))
    horizontal = spec['horizontal_aperture_mm']/(2*spec['focal_length_mm'])
    vertical = spec['vertical_aperture_mm']/(2*spec['focal_length_mm'])
    normals = [forward]
    for axis, tangent in ((right, horizontal), (down, vertical)):
        normals += [tuple(tangent*forward[i]+sign*axis[i] for i in range(3))
                    for sign in (-1, 1)]
    return [(normal, -_dot(normal, center)) for normal in normals]


def _split_polygon(polygon, plane):
    normal, offset = plane
    distances = [_dot(normal, vertex)+offset for vertex in polygon]
    inside, outside = [], []
    for index, current in enumerate(polygon):
        previous = polygon[index-1]
        d0, d1 = distances[index-1], distances[index]
        before, after = d0 >= -EPSILON, d1 >= -EPSILON
        if before != after:
            fraction = d0/(d0-d1)
            intersection = tuple(previous[i]+fraction*(current[i]-previous[i]) for i in range(3))
            inside.append(intersection); outside.append(intersection)
        (inside if after else outside).append(current)
    return inside, outside


def subtract_frustum(polygon, planes):
    """Keep polygon fragments outside a convex optical-clearance volume."""
    retained = []
    pending = list(polygon)
    for plane in planes:
        if len(pending) < 3:
            break
        pending, outside = _split_polygon(pending, plane)
        if len(outside) >= 3:
            retained.append(outside)
    return retained


def clipped_triangles(triangle, frusta):
    polygons = [list(triangle)]
    for planes in frusta:
        polygons = [fragment for polygon in polygons for fragment in subtract_frustum(polygon, planes)]
    result = []
    for polygon in polygons:
        for index in range(1, len(polygon)-1):
            tri = (polygon[0], polygon[index], polygon[index+1])
            a = tuple(tri[1][i]-tri[0][i] for i in range(3))
            b = tuple(tri[2][i]-tri[0][i] for i in range(3))
            cross = (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
            if _dot(cross, cross) > 1e-22:
                result.append(tri)
    return result


def head_visual_shells(stage, cameras):
    """Return exact visual mounts; do not select their collision siblings."""
    from pxr import UsdGeom, UsdPhysics
    head_path = cameras['head_left']['mount_body_path']
    if cameras['head_right']['mount_body_path'] != head_path:
        raise ValueError('Head stereo cameras must share one surviving head body')
    head = stage.GetPrimAtPath(head_path)
    result = []
    for name in ('head_link2', 'zedm', 'zed'):
        prim = head.GetChild(name)
        if prim and prim.IsA(UsdGeom.Xform) and not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            result.append(prim)
    if not any(prim.GetName() == 'head_link2' for prim in result):
        raise ValueError('Could not identify the SH5 visual head shell')
    return result


def clear_head_optical_apertures(stage, cameras):
    """Author bounded visual mesh edits in the caller's current USD layer."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt
    frusta = [optical_frustum(cameras[name]) for name in ('head_left', 'head_right')]
    head = stage.GetPrimAtPath(cameras['head_left']['mount_body_path'])
    cache = UsdGeom.XformCache()
    world_to_head = cache.GetLocalToWorldTransform(head).GetInverse()
    receipts = []
    for shell in head_visual_shells(stage, cameras):
        if shell.IsInstance():
            shell.SetInstanceable(False)
        for prim in Usd.PrimRange(shell):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                raise ValueError('Optical aperture edits must never include a collider')
            mesh = UsdGeom.Mesh(prim)
            points = list(mesh.GetPointsAttr().Get() or [])
            counts = list(mesh.GetFaceVertexCountsAttr().Get() or [])
            indices = list(mesh.GetFaceVertexIndicesAttr().Get() or [])
            if not points or not counts:
                continue
            to_head = cache.GetLocalToWorldTransform(prim)*world_to_head
            from_head = to_head.GetInverse()
            head_points = [tuple(to_head.Transform(Gf.Vec3d(*point))) for point in points]
            output = []; original_triangles = 0; changed = 0; cursor = 0
            for count in counts:
                face = indices[cursor:cursor+count]; cursor += count
                for index in range(1, count-1):
                    triangle = tuple(head_points[i] for i in (face[0], face[index], face[index+1]))
                    clipped = clipped_triangles(triangle, frusta)
                    original_triangles += 1
                    if clipped != [triangle]:
                        changed += 1
                    output.extend(clipped)
            if not changed:
                continue
            vertices = [Gf.Vec3f(*from_head.Transform(Gf.Vec3d(*point))) for tri in output for point in tri]
            if not vertices:
                raise ValueError('Optical approximation would remove an entire visual shell')
            mesh.CreatePointsAttr(Vt.Vec3fArray(vertices))
            mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3]*len(output)))
            mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(range(len(vertices))))
            # Imported STL assets can also encode per-face normals as the
            # primvar `normal`, which takes precedence in RTX. Its inherited
            # topology-sized buffer must not survive a topology change.
            for primvar in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
                if primvar.GetInterpolation() in (UsdGeom.Tokens.uniform,
                                                 UsdGeom.Tokens.vertex,
                                                 UsdGeom.Tokens.faceVarying,
                                                 UsdGeom.Tokens.varying):
                    primvar.GetAttr().Block()
                    if primvar.IsIndexed():
                        primvar.GetIndicesAttr().Block()
            # The original STL normals are indexed by its original vertices.
            # Recompute flat normals for the new triangles instead of retaining
            # invalid arrays that can produce RTX shading artifacts.
            normals = []
            for index in range(0, len(vertices), 3):
                normal = Gf.Cross(vertices[index+1]-vertices[index], vertices[index+2]-vertices[index])
                normal.Normalize(); normals.append(normal)
            mesh.CreateNormalsAttr(Vt.Vec3fArray(normals))
            mesh.SetNormalsInterpolation(UsdGeom.Tokens.uniform)
            UsdGeom.PrimvarsAPI(prim).CreatePrimvar('normal',
                mesh.GetNormalsAttr().GetTypeName(), UsdGeom.Tokens.uniform).Set(Vt.Vec3fArray(normals))
            mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            mesh.CreateExtentAttr(Vt.Vec3fArray([
                Gf.Vec3f(*(min(point[i] for point in vertices) for i in range(3))),
                Gf.Vec3f(*(max(point[i] for point in vertices) for i in range(3)))]))
            receipts.append({'visual_mesh_path': str(prim.GetPath()),
                             'original_triangles': original_triangles,
                             'intersected_triangles': changed,
                             'result_triangles': len(output)})
    return receipts
