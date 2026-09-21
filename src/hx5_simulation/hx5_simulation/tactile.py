import math


def world_to_local(point, transform):
    quaternion = transform.rotation
    vector = [point.x - transform.translation.x, point.y - transform.translation.y,
              point.z - transform.translation.z]
    axis = [-quaternion.x, -quaternion.y, -quaternion.z]
    cross = [axis[1] * vector[2] - axis[2] * vector[1],
             axis[2] * vector[0] - axis[0] * vector[2],
             axis[0] * vector[1] - axis[1] * vector[0]]
    cross_twice = [axis[1] * cross[2] - axis[2] * cross[1],
                   axis[2] * cross[0] - axis[0] * cross[2],
                   axis[0] * cross[1] - axis[1] * cross[0]]
    return [value + 2 * (quaternion.w * first + second)
            for value, first, second in zip(vector, cross, cross_twice)]


def taxel_index(point, thumb, right=False):
    horizontal, vertical = (point[0], point[1] if right else -point[1]) if thumb else (point[1], point[2])
    column = max(0, min(2, int((horizontal + 0.012) / 0.008)))
    row = max(0, min(2, int((0.040 - vertical) / (0.040 / 3))))
    return row * 3 + column


def contact_forces(message, transform, thumb, model_name, right=False):
    cells = [0.0] * 9
    for contact in message.contacts:
        names = (contact.collision1.name, contact.collision2.name)
        if all(name.startswith(model_name + '::') for name in names):
            continue
        if len(contact.positions) != len(contact.wrenches):
            raise ValueError('Contact positions/wrenches length mismatch')
        for point, wrench in zip(contact.positions, contact.wrenches):
            force = wrench.body_1_wrench.force
            magnitude = math.sqrt(force.x ** 2 + force.y ** 2 + force.z ** 2)
            if not math.isfinite(magnitude):
                raise ValueError('Non-finite contact force')
            cells[taxel_index(world_to_local(point, transform), thumb, right)] += magnitude
    return cells


def pressure_values(forces, full_scale_newtons):
    if not math.isfinite(full_scale_newtons) or full_scale_newtons <= 0:
        raise ValueError('full_scale_newtons must be finite and positive')
    if len(forces) != 9 or any(not math.isfinite(value) or value < 0 for value in forces):
        raise ValueError('Nine finite non-negative forces required')
    return [min(255, round(255 * value / full_scale_newtons)) for value in forces]
