import math
from types import SimpleNamespace as Values

import pytest

from hx5_simulation.tactile import contact_forces, pressure_values, taxel_index, world_to_local


def vector(values):
    return Values(x=values[0], y=values[1], z=values[2])


def transform(angle=0.0):
    return Values(translation=vector((0, 0, 0)),
                  rotation=Values(x=0, y=0, z=math.sin(angle / 2), w=math.cos(angle / 2)))


def contact(names=('ffw_sh5_rev1_follower::finger_l_link4::collision', 'object::collision')):
    return Values(collision1=Values(name=names[0]), collision2=Values(name=names[1]),
        positions=[vector((0, -0.02, 0))],
        wrenches=[Values(body_1_wrench=Values(force=vector((0, 0, 4.0))))])


def test_force_is_binned_and_saturates():
    forces = contact_forces(Values(contacts=[contact()]), transform(), True, 'ffw_sh5_rev1_follower')
    assert sum(forces) == 4.0
    assert forces[4] == 4.0
    assert pressure_values(forces, 8.0)[4] == 128
    assert pressure_values([16.0] * 9, 8.0) == [255] * 9


def test_no_contact_and_self_contact_are_zero():
    assert contact_forces(Values(contacts=[]), transform(), True, 'robot') == [0.0] * 9
    source = Values(contacts=[contact(('robot::tip', 'robot::palm'))])
    assert contact_forces(source, transform(), True, 'robot') == [0.0] * 9


def test_rotation_and_taxel_edges():
    assert world_to_local(vector((0, 1, 0)), transform(math.pi / 2)) == pytest.approx([1, 0, 0])
    assert taxel_index((-100, -100, 0), True) == 0
    assert taxel_index((100, 100, 0), True) == 8
    assert taxel_index((0, 0, 0.02), False) == 4


def test_thumb_contact_centers_are_mirrored_between_hands():
    assert taxel_index((0, -0.02, 0), True) == 4
    assert taxel_index((0, 0.02, 0), True, right=True) == 4


@pytest.mark.parametrize('scale', [0.0, -1.0, float('nan'), float('inf')])
def test_invalid_scale(scale):
    with pytest.raises(ValueError):
        pressure_values([0.0] * 9, scale)


def test_missing_force_is_not_fabricated():
    source = contact()
    source.wrenches = []
    with pytest.raises(ValueError):
        contact_forces(Values(contacts=[source]), transform(), True, 'robot')


def test_invalid_forces():
    with pytest.raises(ValueError):
        pressure_values([float('nan')] * 9, 8.0)
    with pytest.raises(ValueError):
        pressure_values([-1.0] * 9, 8.0)
