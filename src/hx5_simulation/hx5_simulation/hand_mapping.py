"""Validated release/grasp interpolation from an explicitly selected profile.

ROBOTIS's 1044 preset controller normalizes the gripper into [0, 1],
then interpolates its authored release/grasp arrays. Custom five-finger
curls use the same interpolation; this module adds URDF limit enforcement.
"""
import math


def normalize(value, lower, upper):
    if not all(math.isfinite(x) for x in (value, lower, upper)) or lower >= upper:
        raise ValueError('Finite gripper range with lower < upper is required')
    return (min(upper, max(lower, value)) - lower) / (upper - lower)


def validate_profile(profile, bounds):
    lower, upper = profile['gripper_range']
    normalize(lower, lower, upper)
    threshold = profile.get('thumb_threshold', 0.0)
    if not math.isfinite(threshold) or not 0 <= threshold < 1:
        raise ValueError('Thumb threshold must be finite and in [0, 1)')
    for hand, side in (('left', 'l'), ('right', 'r')):
        for endpoint in ('release', 'grasp'):
            values = profile[hand][endpoint]
            if len(values) != 20 or any(not math.isfinite(value) for value in values):
                raise ValueError('Each hand endpoint needs 20 finite joint positions')
            for index in range(20):
                joint_lower, joint_upper = bounds[f'finger_{side}_joint{index + 1}']
                if not all(math.isfinite(x) for x in (joint_lower, joint_upper)) or joint_lower > joint_upper:
                    raise ValueError('Invalid hand joint bounds')


def interpolate(open_positions, closed_positions, curls, bounds, side, thumb_threshold=0.0):
    if len(curls) != 5 or any(not math.isfinite(value) or not 0 <= value <= 1 for value in curls):
        raise ValueError('Five finite curls in [0, 1] are required')
    values = []
    for index, (release, grasp) in enumerate(zip(open_positions, closed_positions)):
        curl = curls[index // 4]
        if index < 4:
            curl = normalize(curl, thumb_threshold, 1.0)
        position = release + curl * (grasp - release)
        lower, upper = bounds[f'finger_{side}_joint{index + 1}']
        values.append(min(upper, max(lower, position)))
    return values
