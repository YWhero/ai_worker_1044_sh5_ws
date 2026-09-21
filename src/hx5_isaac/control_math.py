"""Isaac-independent motion and sensor math for the official SH5/HX5 model.

Swerve dimensions, limits and the shortest-steering rule are taken from
ffw_sh5_follower_ai_hardware_controller.yaml. Joint positions are radians,
except lift_joint (metres); this module never produces robot feedback.
"""
from dataclasses import dataclass
import math

MODULES = ((0.1371, 0.2554), (0.1371, -0.2554), (-0.2899, 0.0))
WHEEL_RADIUS = 0.0865
STEER_NAMES = tuple(f"{side}_wheel_steer" for side in ("left", "right", "rear"))
DRIVE_NAMES = tuple(f"{side}_wheel_drive" for side in ("left", "right", "rear"))
GROUPS = {
    "left": tuple(f"arm_l_joint{i}" for i in range(1, 8)),
    "right": tuple(f"arm_r_joint{i}" for i in range(1, 8)),
    "left_hand": tuple(f"finger_l_joint{i}" for i in range(1, 21)),
    "right_hand": tuple(f"finger_r_joint{i}" for i in range(1, 21)),
    "head": ("head_joint1", "head_joint2"),
    "lift": ("lift_joint",),
}


def finite_vector(values, size, label="vector"):
    if not isinstance(values, (list, tuple)) or len(values) != size:
        raise ValueError(f"{label} requires {size} values")
    result = tuple(float(v) for v in values)
    if any(not math.isfinite(v) for v in result):
        raise ValueError(f"{label} contains a non-finite value")
    return result


def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def swerve_targets(vx, vy, angular, current_angles, steering_limits=None):
    """Return steering targets and wheel velocities, without teleporting a base.

    Wheel motion waits while a module differs by >= the official 1 radian
    alignment threshold. Cosine scaling below that threshold avoids sideways
    scrubbing while the position drive is steering. The body twist is applied
    by wheel/ground contact; odometry must come from measured physics poses.
    Steering joints have position limits: equivalent headings must be chosen
    by actual joint travel, rather than a wrapped angle through a hard stop.
    """
    vx, vy, angular = finite_vector([vx, vy, angular], 3, "base velocity")
    current_angles = finite_vector(list(current_angles), 3, "steering angles")
    steering_limits = steering_limits or [(-6.28, 6.28)] * 3
    targets, velocities = [], []
    for (x, y), current, (lower, upper) in zip(MODULES, current_angles, steering_limits):
        mx, my = vx - angular * y, vy + angular * x
        speed = math.hypot(mx, my)
        if speed < 1e-9:
            targets.append(current)
            velocities.append(0.0)
            continue
        heading = math.atan2(my, mx)
        available = []
        for angle, direction in ((heading, 1.0), (heading + math.pi, -1.0)):
            first_turn = math.ceil((lower - angle) / (2 * math.pi))
            last_turn = math.floor((upper - angle) / (2 * math.pi))
            available.extend((angle + turn * 2 * math.pi, direction)
                             for turn in range(first_turn, last_turn + 1))
        if not available:
            raise ValueError("No equivalent steering target within joint limits")
        target, direction = min(available, key=lambda candidate: abs(candidate[0] - current))
        # A bounded position drive travels target-current. Wrapping this error
        # would enable wheels during an almost full turn at a steering limit.
        error = target - current
        velocity = direction * speed / WHEEL_RADIUS
        velocity *= max(0.0, math.cos(error)) if abs(error) < 1.0 else 0.0
        targets.append(target)
        velocities.append(velocity)
    scale = max(1.0, max(abs(value) for value in velocities) / 50.0)
    return targets, [value / scale for value in velocities]


@dataclass(frozen=True)
class Trajectory:
    names: tuple
    # Each point is (seconds from start, positions, optional velocities).
    points: tuple
    start_time: float

    @classmethod
    def from_command(cls, command, measured_positions, sim_time, limits=None):
        group = command.get("group")
        if group not in GROUPS:
            raise ValueError("Unknown trajectory group")
        names = tuple(command.get("joint_names", command.get("names", [])))
        if not names or len(set(names)) != len(names) or not set(names).issubset(GROUPS[group]):
            raise ValueError("Trajectory joint names do not belong to its group")
        requested = command.get("points")
        if not isinstance(requested, list) or not requested:
            raise ValueError("Trajectory requires at least one point")
        points = [(0.0, tuple(float(measured_positions[name]) for name in names), None)]
        previous_time = -1.0
        for point in requested:
            stamp = float(point.get("time_from_start", 0.0))
            if not math.isfinite(stamp) or stamp < 0 or stamp <= previous_time:
                raise ValueError("Trajectory times must be finite, non-negative and increasing")
            position = finite_vector(point.get("positions"), len(names), "trajectory positions")
            velocity = (finite_vector(point["velocities"], len(names), "trajectory velocities")
                        if point.get("velocities") else None)
            for name, value in zip(names, position):
                if limits and name in limits:
                    lower, upper = limits[name]
                    if not lower - 1e-6 <= value <= upper + 1e-6:
                        raise ValueError(f"Trajectory exceeds {name} limits [{lower}, {upper}]")
            if stamp == 0:
                points[0] = (stamp, position, velocity)
            else:
                points.append((stamp, position, velocity))
            previous_time = stamp
        return cls(names, tuple(points), float(sim_time))

    def sample(self, sim_time):
        elapsed = max(0.0, float(sim_time) - self.start_time)
        if elapsed >= self.points[-1][0]:
            return self.points[-1][1], True
        for first, last in zip(self.points, self.points[1:]):
            if elapsed <= last[0]:
                span = last[0] - first[0]
                fraction = (elapsed - first[0]) / span
                # Linear interpolation stays within the validated joint limits.
                # Positions remain drives, not reported/measured joint states.
                return tuple(a + fraction * (b - a) for a, b in zip(first[1], last[1])), False
        return self.points[0][1], True


def trajectory_start_positions(measured_positions, previous_trajectory=None, last_commanded_positions=None):
    """Keep desired-state continuity only while the same group is active.

    ROS2 Control Jazzy's ``interpolate_from_desired_state`` starts a streamed
    replacement at the last command. Replanning from actual feedback every
    frame would repeatedly erase drive error caused by gravity. Feedback is
    still measured; this returned map is only the trajectory's desired start.
    First commands, newly introduced joints and commands after hold/reset use
    measured starts. The caller supplies the previous trajectory of this group
    and validates the replacement before updating any live control state.
    """
    start = {name: float(value) for name, value in measured_positions.items()}
    if any(not math.isfinite(value) for value in start.values()):
        raise ValueError("Measured trajectory start contains a non-finite value")
    if previous_trajectory is not None and last_commanded_positions is not None:
        for name in previous_trajectory.names:
            if name in start and name in last_commanded_positions:
                value = float(last_commanded_positions[name])
                if not math.isfinite(value):
                    raise ValueError(f"Last desired trajectory start for {name} is non-finite")
                start[name] = value
    return start


def sample_trajectory_step(trajectory, sim_time, timestep):
    """Drive toward the end of the upcoming physics step.

    Incoming trajectories begin at a validated measured/desired pose at
    ``sim_time``.
    Sampling that same instant after every streamed replacement always returns
    the start pose. The next-step horizon allows a positive-duration trajectory
    to advance while preserving its measured start, waypoints and end clamp.
    """
    sim_time, timestep = float(sim_time), float(timestep)
    if not math.isfinite(sim_time) or not math.isfinite(timestep) or timestep <= 0:
        raise ValueError("Trajectory sampling requires finite simulation time and a positive finite timestep")
    return trajectory.sample(sim_time + timestep)


def coalesce_trajectory_commands(commands, limits=None):
    """Keep latest unapplied repeated targets without crossing order barriers.

    A physical leader can publish several replacements before one physics
    step. Only unacknowledged, valid trajectories with the same group and
    ordered joint names may be merged. Any other command, invalid trajectory
    or change in one group's joint list flushes the batch, preserving partial
    commands and stop/reset/diagnostic/acknowledgement ordering.
    Structure/limits use the actual trajectory validator. Its zero initial
    map is exclusively a validation input, never a drive or robot feedback;
    retained commands are validated again against the real measured/desired
    start when they are applied. Caller inputs are never modified.
    """
    result, pending, latest, group_names = [], [], {}, {}

    def flush():
        result.extend(command for index, (key, command) in enumerate(pending)
                      if latest[key] == index)
        pending.clear()
        latest.clear()
        group_names.clear()

    for command in commands:
        key = None
        if (isinstance(command, dict) and command.get("kind") == "trajectory" and
                "request_id" not in command):
            try:
                names = tuple(command.get("joint_names", command.get("names", [])))
                Trajectory.from_command(command, dict.fromkeys(names, 0.0), 0.0, limits)
                key = (command["group"], names)
            except (ValueError, TypeError, KeyError, AttributeError, OverflowError):
                pass  # Invalid input must still reach the normal rejection.
        if key is None:
            flush()
            result.append(command)
            continue
        group, names = key
        if group in group_names and group_names[group] != names:
            flush()
        group_names[group] = names
        latest[key] = len(pending)
        pending.append((key, command))
    flush()
    return result


def rotate_vector(vector, quaternion_xyzw):
    x, y, z, w = quaternion_xyzw
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-12:
        raise ValueError("Zero quaternion")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    vx, vy, vz = vector
    cx, cy, cz = y*vz-z*vy, z*vx-x*vz, x*vy-y*vx
    ccx, ccy, ccz = y*cz-z*cy, z*cx-x*cz, x*cy-y*cx
    return [vx + 2*(w*cx+ccx), vy + 2*(w*cy+ccy), vz + 2*(w*cz+ccz)]


def world_to_local(point, position, orientation_xyzw):
    x, y, z, w = orientation_xyzw
    return rotate_vector([p-v for p, v in zip(point, position)], [-x, -y, -z, w])


def taxel_index(point, thumb=False, right=False):
    """Same physical 3x3 fingertip grid convention as the Gazebo adapter."""
    horizontal, vertical = ((point[0], point[1] if right else -point[1])
                            if thumb else (point[1], point[2]))
    column = max(0, min(2, int((horizontal + 0.012) / 0.008)))
    row = max(0, min(2, int((0.040 - vertical) / (0.040 / 3))))
    return row * 3 + column


def contact_taxels(forces, points, position, orientation_xyzw, thumb=False, right=False):
    if len(forces) != len(points):
        raise ValueError("Contact forces and points differ in length")
    cells = [0.0] * 9
    for force, point in zip(forces, points):
        magnitude = abs(float(force))
        if not math.isfinite(magnitude):
            raise ValueError("Non-finite measured contact force")
        cell = taxel_index(world_to_local(point, position, orientation_xyzw), thumb, right)
        cells[cell] += magnitude
    return cells


def contact_depth_taxels(separations, points, position, orientation_xyzw, thumb=False, right=False):
    """Largest measured penetration per cell, in metres (separation < 0)."""
    if len(separations) != len(points):
        raise ValueError("Contact separations and points differ in length")
    cells = [0.0] * 9
    for separation, point in zip(separations, points):
        separation = float(separation)
        if not math.isfinite(separation):
            raise ValueError("Non-finite measured contact separation")
        cell = taxel_index(world_to_local(point,position,orientation_xyzw),thumb,right)
        cells[cell] = max(cells[cell],max(0.0,-separation))
    return cells
