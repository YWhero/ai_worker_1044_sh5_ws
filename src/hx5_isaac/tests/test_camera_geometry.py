"""Optical-clearance clipping preserves geometry outside camera frusta."""
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from camera_geometry import clipped_triangles, optical_frustum


@pytest.fixture
def frustum():
    return optical_frustum({'mount_xyz': [0, 0, 0], 'mount_quat_wxyz': [1, 0, 0, 0],
                            'horizontal_aperture_mm': 2, 'vertical_aperture_mm': 2,
                            'focal_length_mm': 1})


def area(triangle):
    a = [triangle[1][i]-triangle[0][i] for i in range(3)]
    b = [triangle[2][i]-triangle[0][i] for i in range(3)]
    cross = (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
    return math.sqrt(sum(v*v for v in cross))/2


def test_front_opaque_cap_removed_but_rear_shell_preserved(frustum):
    front = ((-.5, -.5, 1), (.5, -.5, 1), (0, .5, 1))
    rear = tuple((x, y, -z) for x, y, z in front)
    assert clipped_triangles(front, [frustum]) == []
    assert clipped_triangles(rear, [frustum]) == [rear]


def test_crossing_surface_clipped_without_deleting_outside_area(frustum):
    triangle = ((-2, -2, 1), (2, -2, 1), (0, 2, 1))
    clipped = clipped_triangles(triangle, [frustum])
    assert sum(map(area, clipped)) == pytest.approx(4.5)
    assert sum(map(area, clipped)) < area(triangle)
    for result in clipped:
        center = tuple(sum(v[i] for v in result)/3 for i in range(3))
        assert any(sum(n[i]*center[i] for i in range(3))+d < 0 for n, d in frustum)


def test_official_optical_rotation_points_forward_and_stereo_keeps_baseline():
    camera = {'mount_xyz': [.0238122, .02498203, -.0109594],
              'mount_quat_wxyz': [.5, -.5, .5, -.5],
              'horizontal_aperture_mm': 20.955, 'vertical_aperture_mm': 20.955*188/336,
              'focal_length_mm': 12}
    planes = optical_frustum(camera)
    assert planes[0][0] == pytest.approx((1, 0, 0))
    # A solid cap 22.55 mm ahead of the official lens origin is transparent
    # in the optical-clearance approximation without moving that origin.
    triangle = ((.0463622, .02, -.015), (.0463622, .03, -.015), (.0463622, .025, -.005))
    assert clipped_triangles(triangle, [planes]) == []
    assert camera['mount_xyz'] == [.0238122, .02498203, -.0109594]


def test_two_cameras_subtract_union_of_clearances(frustum):
    shifted = [(normal, offset-normal[0]*4) for normal, offset in frustum]
    triangle = ((-2, -2, 1), (6, -2, 1), (2, 3, 1))
    once = clipped_triangles(triangle, [frustum])
    twice = clipped_triangles(triangle, [frustum, shifted])
    assert 0 < sum(map(area, twice)) < sum(map(area, once))
