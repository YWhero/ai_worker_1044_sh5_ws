import math
import sys
import unittest
from copy import deepcopy
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from control_math import (DRIVE_NAMES, GROUPS, MODULES, STEER_NAMES, Trajectory,
                          WHEEL_RADIUS, coalesce_trajectory_commands,
                          contact_depth_taxels, contact_taxels, rotate_vector,
                          sample_trajectory_step, swerve_targets, trajectory_start_positions)


class SwerveTests(unittest.TestCase):
    def test_forward_uses_official_wheel_radius(self):
        angles, speeds = swerve_targets(.3, 0, 0, [0, 0, 0])
        self.assertEqual(angles, [0, 0, 0])
        for speed in speeds:
            self.assertAlmostEqual(speed, .3 / .0865)

    def test_reverse_flips_wheel_direction_without_180_degree_steering(self):
        angles, speeds = swerve_targets(-.3, 0, 0, [0, 0, 0])
        for angle, speed in zip(angles, speeds):
            self.assertAlmostEqual(angle, 0)
            self.assertLess(speed, 0)

    def test_body_twist_reconstructs_from_aligned_modules(self):
        for vx, vy, angular in ((.3, -.1, .7), (0, 0, .6), (0, .2, 0)):
            initial = [math.atan2(vy + angular*x, vx - angular*y) for x, y in MODULES]
            angles, speeds = swerve_targets(vx, vy, angular, initial)
            for (x, y), angle, speed in zip(MODULES, angles, speeds):
                self.assertAlmostEqual(speed * WHEEL_RADIUS * math.cos(angle), vx-angular*y)
                self.assertAlmostEqual(speed * WHEEL_RADIUS * math.sin(angle), vy+angular*x)

    def test_alignment_blocks_unaligned_wheels_and_zero_holds_steering(self):
        angles, speeds = swerve_targets(0, .2, 0, [0, 0, 0])
        self.assertEqual(speeds, [0, 0, 0])
        angles, speeds = swerve_targets(0, 0, 0, [.1, -.2, .3])
        self.assertEqual(angles, [.1, -.2, .3])
        self.assertEqual(speeds, [0, 0, 0])

    def test_wrapped_steering_stays_within_official_controller_limits(self):
        angles, _ = swerve_targets(.1, -.1, 0, [6.2, -6.2, 0])
        self.assertTrue(all(-6.28 <= angle <= 6.28 for angle in angles))

    def test_steering_limit_selects_reversed_heading_by_actual_joint_travel(self):
        for current, heading in ((6.2, .1), (-6.2, -.1)):
            vx, vy = .2 * math.cos(heading), .2 * math.sin(heading)
            angles, speeds = swerve_targets(vx, vy, 0, [current] * 3)
            expected = heading + (-math.pi if current < 0 else math.pi)
            for angle in angles:
                self.assertAlmostEqual(angle, expected)
                self.assertLess(abs(angle-current), math.pi)
            self.assertEqual(speeds, [0, 0, 0])
            aligned_angles, aligned_speeds = swerve_targets(vx, vy, 0, angles)
            for angle, speed in zip(aligned_angles, aligned_speeds):
                self.assertLess(speed, 0)
                self.assertAlmostEqual(speed * WHEEL_RADIUS * math.cos(angle), vx)
                self.assertAlmostEqual(speed * WHEEL_RADIUS * math.sin(angle), vy)

    def test_narrower_mechanical_limits_flip_before_crossing_a_hard_stop(self):
        for current, heading in ((2.9, -3.), (-2.9, 3.)):
            angles, speeds = swerve_targets(.2 * math.cos(heading), .2 * math.sin(heading),
                                           0, [current] * 3, [(-math.pi, math.pi)] * 3)
            expected = heading + (math.pi if current > 0 else -math.pi)
            for angle in angles:
                self.assertAlmostEqual(angle, expected)
                self.assertGreater(abs(angle-current), 1.0)
                self.assertLess(abs(angle-current), math.pi)
            self.assertEqual(speeds, [0, 0, 0])

    def test_equivalent_heading_near_limit_remains_a_short_aligned_move(self):
        heading = -.1
        angles, speeds = swerve_targets(.2 * math.cos(heading), .2 * math.sin(heading),
                                       0, [6.2] * 3)
        for angle, speed in zip(angles, speeds):
            self.assertAlmostEqual(angle, heading + 2 * math.pi)
            self.assertLess(abs(angle-6.2), .02)
            self.assertGreater(speed, 0)


class TrajectoryTests(unittest.TestCase):
    def command(self):
        return {"group":"head", "joint_names":["head_joint1"],
                "points":[{"positions":[.6],"time_from_start":2.0}]}

    def test_interpolates_from_measured_start_and_holds_after_end(self):
        trajectory = Trajectory.from_command(self.command(), {"head_joint1":.2}, 10)
        self.assertAlmostEqual(trajectory.sample(11)[0][0], .4)
        self.assertFalse(trajectory.sample(11)[1])
        self.assertEqual(trajectory.sample(20), ((.6,), True))

    def test_rejects_wrong_group_names_and_out_of_range_targets(self):
        command = self.command()
        command["joint_names"] = ["finger_l_joint1"]
        with self.assertRaises(ValueError):
            Trajectory.from_command(command, {"finger_l_joint1":0}, 0)
        with self.assertRaises(ValueError):
            Trajectory.from_command(self.command(), {"head_joint1":0}, 0,
                                    {"head_joint1":(-.5,.5)})

    def test_rejects_duplicate_times_and_nonfinite_values(self):
        command = self.command()
        command["points"] *= 2
        with self.assertRaises(ValueError):
            Trajectory.from_command(command, {"head_joint1":0}, 0)
        command = self.command()
        command["points"][0]["positions"] = [float("nan")]
        with self.assertRaises(ValueError):
            Trajectory.from_command(command, {"head_joint1":0}, 0)

    def test_streamed_positive_delay_trajectory_progresses_when_replaced_every_physics_frame(self):
        measured = {"arm_l_joint1":0.0}
        timestep = 1/120
        for frame in range(120):
            now = frame*timestep
            command = {"group":"left", "joint_names":["arm_l_joint1"],
                       "points":[{"positions":[.5], "time_from_start":.2}]}
            trajectory = Trajectory.from_command(command, measured, now,
                                                 {"arm_l_joint1":(-.6,.6)})
            self.assertEqual(trajectory.sample(now)[0], (measured["arm_l_joint1"],))
            target, done = sample_trajectory_step(trajectory, now, timestep)
            self.assertGreater(target[0], measured["arm_l_joint1"])
            self.assertLessEqual(target[0], .5)
            self.assertFalse(done)
            # An ideal plant gives the next command a new measured start;
            # neither helper nor trajectory modifies the feedback dictionary.
            self.assertEqual(trajectory.points[0][1], (measured["arm_l_joint1"],))
            measured["arm_l_joint1"] = target[0]
        self.assertGreater(measured["arm_l_joint1"], .49)

    def test_streamed_hand_delay_can_follow_both_open_and_close_targets(self):
        measured = {"finger_l_joint3":0.0}
        timestep = 1/120
        for goal in (-.6, 0.0):
            for frame in range(120):
                command = {"group":"left_hand", "joint_names":["finger_l_joint3"],
                           "points":[{"positions":[goal], "time_from_start":.1}]}
                trajectory = Trajectory.from_command(command, measured, frame*timestep,
                                                     {"finger_l_joint3":(-1.57,0.0)})
                target, _ = sample_trajectory_step(trajectory, frame*timestep, timestep)
                self.assertGreaterEqual(target[0], -.6)
                self.assertLessEqual(target[0], 0.0)
                measured["finger_l_joint3"] = target[0]
            self.assertAlmostEqual(measured["finger_l_joint3"], goal, places=4)

    def test_next_step_sampling_preserves_multipoint_waypoints_and_clamps_after_end(self):
        command = {"group":"head", "joint_names":["head_joint1"],
                   "points":[{"positions":[.2], "time_from_start":.1},
                             {"positions":[.5], "time_from_start":.2}]}
        trajectory = Trajectory.from_command(command, {"head_joint1":0.0}, 10,
                                             {"head_joint1":(-.2317,.6951)})
        target, done = sample_trajectory_step(trajectory, 10.075, .025)
        self.assertAlmostEqual(target[0], .2)
        self.assertFalse(done)
        target, done = sample_trajectory_step(trajectory, 10.1, .025)
        self.assertAlmostEqual(target[0], .275)
        self.assertFalse(done)
        self.assertEqual(sample_trajectory_step(trajectory, 10.2, .025), ((.5,),True))

    def test_next_step_sampling_keeps_zero_duration_immediate(self):
        command = self.command(); command["points"][0]["time_from_start"] = 0.0
        measured = {"head_joint1":.2}
        trajectory = Trajectory.from_command(command, measured, 10)
        self.assertEqual(sample_trajectory_step(trajectory, 10, 1/120), ((.6,),True))
        self.assertEqual(measured, {"head_joint1":.2})

    def test_next_step_sampling_rejects_invalid_time_and_timestep(self):
        trajectory = Trajectory.from_command(self.command(), {"head_joint1":.2}, 10)
        for now, timestep in ((10,0),(10,-.1),(10,float("nan")),
                              (10,float("inf")),(float("nan"),1/120)):
            with self.assertRaises(ValueError):
                sample_trajectory_step(trajectory, now, timestep)

    def test_streamed_desired_continuity_preserves_drive_error_under_gravity(self):
        # A plant with a constant gravity offset reproduces the native PhysX
        # evidence: measured-start replanning multiplies static error by
        # horizon/dt. Desired continuity keeps it at the physical drive offset.
        for group, name, horizon, bias in (("left", "arm_l_joint7", .2, .0013),
                                          ("left_hand", "finger_l_joint1", .1, .0033)):
            final_errors = []
            for use_continuity in (False, True):
                measured = {name: -bias}
                desired = {name: 0.0}
                previous = None
                for frame in range(300):
                    command = {"group": group, "joint_names": [name],
                               "points": [{"positions": [.5], "time_from_start": horizon}]}
                    start = trajectory_start_positions(measured, previous if use_continuity else None, desired)
                    previous = Trajectory.from_command(command, start, frame/120, {name: (-1., 1.)})
                    target, _ = sample_trajectory_step(previous, frame/120, 1/120)
                    desired[name] = target[0]
                    measured[name] = desired[name] - bias
                    self.assertAlmostEqual(desired[name] - measured[name], bias)
                final_errors.append(abs(.5 - measured[name]))
            self.assertGreater(final_errors[0], .03)
            self.assertLess(final_errors[1], .004)
            self.assertAlmostEqual(final_errors[1], bias, places=5)

    def test_continuity_overrides_only_previous_active_joint_intersection(self):
        previous = Trajectory.from_command(self.command(), {"head_joint1": .2}, 10)
        measured = {"head_joint1": .25, "head_joint2": -.1, "arm_l_joint7": .3}
        commanded = {"head_joint1": .28, "head_joint2": .5, "arm_l_joint7": .6}
        start = trajectory_start_positions(measured, previous, commanded)
        self.assertEqual(start, {"head_joint1": .28, "head_joint2": -.1, "arm_l_joint7": .3})
        command = {"group": "head", "joint_names": ["head_joint1", "head_joint2"],
                   "points": [{"positions": [.6, .2], "time_from_start": .2}]}
        replacement = Trajectory.from_command(command, start, 10.1)
        self.assertEqual(replacement.points[0][1], (.28, -.1))
        self.assertEqual(measured["head_joint1"], .25)
        self.assertEqual(commanded["head_joint2"], .5)

    def test_first_and_cleared_trajectory_starts_from_actual_measurement(self):
        measured = {"head_joint1": .25}
        self.assertEqual(trajectory_start_positions(measured, None, {"head_joint1": .6}), measured)
        start = trajectory_start_positions(measured)
        start["head_joint1"] = .8
        self.assertEqual(measured["head_joint1"], .25)

    def test_missing_previous_setpoint_uses_measured_start_and_ignores_other_group(self):
        previous = Trajectory.from_command(self.command(), {"head_joint1": .2}, 10)
        measured = {"head_joint1": .25}
        self.assertEqual(trajectory_start_positions(measured, previous, {}), measured)
        self.assertEqual(trajectory_start_positions(measured, previous, {"arm_l_joint7": float("nan")}), measured)

    def test_nonfinite_actual_or_active_desired_start_is_rejected_without_mutation(self):
        previous = Trajectory.from_command(self.command(), {"head_joint1": .2}, 10)
        measured = {"head_joint1": .25}
        with self.assertRaises(ValueError):
            trajectory_start_positions({"head_joint1": float("nan")})
        for value in (float("nan"), float("inf"), -float("inf")):
            with self.assertRaises(ValueError):
                trajectory_start_positions(measured, previous, {"head_joint1": value})
        self.assertEqual(measured, {"head_joint1": .25})


class CoalescingTests(unittest.TestCase):
    def command(self, group="left", value=.2, names=None):
        names = names or list(GROUPS[group])
        return {"kind": "trajectory", "group": group, "joint_names": names,
                "points": [{"positions": [value] * len(names), "time_from_start": .2}]}

    def test_latest_identical_groups_keep_order_of_surviving_commands(self):
        left1, right1 = self.command(), self.command("right")
        left2, hand1, right2 = self.command(value=.3), self.command("left_hand"), self.command("right", .4)
        result = coalesce_trajectory_commands([left1, right1, left2, hand1, right2])
        self.assertEqual(result, [left2, hand1, right2])
        self.assertIs(result[0], left2)

    def test_all_nontrajectory_commands_are_order_barriers(self):
        for kind in ("stop", "reset", "shutdown", "inspect", "basevelocity", "base_velocity", "unknown"):
            before1, before2 = self.command(value=.1), self.command(value=.2)
            barrier = {"kind": kind}
            after1, after2 = self.command(value=.3), self.command(value=.4)
            self.assertEqual(coalesce_trajectory_commands([before1, before2, barrier, after1, after2]),
                             [before2, barrier, after2])

    def test_request_id_including_none_preserves_every_acknowledgement(self):
        for request_id in ("request-42", None):
            first, requested, last = self.command(value=.1), self.command(value=.2), self.command(value=.3)
            requested["request_id"] = request_id
            self.assertEqual(coalesce_trajectory_commands([first, requested, last]), [first, requested, last])

    def test_changed_partial_joint_set_flushes_entire_batch(self):
        first = self.command()
        right1 = self.command("right", .2)
        partial = self.command(value=.3, names=["arm_l_joint1"])
        right2 = self.command("right", .4)
        full = self.command(value=.5)
        commands = [first, right1, partial, right2, full]
        self.assertEqual(coalesce_trajectory_commands(commands), commands)

    def test_changed_joint_order_is_also_a_barrier(self):
        first = self.command(names=["arm_l_joint1", "arm_l_joint2"])
        reordered = self.command(names=["arm_l_joint2", "arm_l_joint1"])
        last = self.command(value=.3, names=["arm_l_joint1", "arm_l_joint2"])
        self.assertEqual(coalesce_trajectory_commands([first, reordered, last]), [first, reordered, last])

    def test_invalid_last_command_never_hides_a_previous_valid_command(self):
        invalid = []
        bad = self.command(); bad["points"] = []; invalid.append(bad)
        bad = self.command(); bad["points"] = [None]; invalid.append(bad)
        bad = self.command(); bad["points"][0]["positions"] = [.3]; invalid.append(bad)
        bad = self.command(); bad["points"][0]["positions"][0] = float("nan"); invalid.append(bad)
        bad = self.command(); bad["points"][0]["velocities"] = [float("inf")] * 7; invalid.append(bad)
        bad = self.command(); bad["points"][0]["velocities"] = [0.]; invalid.append(bad)
        bad = self.command(); bad["points"][0]["time_from_start"] = -.1; invalid.append(bad)
        bad = self.command(); bad["points"] *= 2; invalid.append(bad)
        bad = self.command(); bad["group"] = "unknown"; invalid.append(bad)
        bad = self.command(); bad["joint_names"] = ["arm_r_joint1"] * 7; invalid.append(bad)
        bad = self.command(); bad["joint_names"] = [["arm_l_joint1"]]; invalid.append(bad)
        invalid.extend([None, "trajectory"])
        for bad in invalid:
            first, last = self.command(value=.1), self.command(value=.4)
            result = coalesce_trajectory_commands([first, bad, last])
            self.assertEqual(len(result), 3)
            self.assertIs(result[0], first)
            self.assertIs(result[1], bad)
            self.assertIs(result[2], last)

    def test_target_outside_actual_joint_limits_is_a_barrier(self):
        names = ["head_joint1"]
        first = self.command("head", .2, names)
        invalid = self.command("head", .8, names)
        last = self.command("head", .3, names)
        limits = {"head_joint1": (-.2317, .6951)}
        self.assertEqual(coalesce_trajectory_commands([first, invalid, last], limits), [first, invalid, last])

    def test_zero_duration_multipoint_and_empty_velocities_use_same_validator(self):
        first, last = self.command(), self.command(value=.4)
        first["points"][0]["time_from_start"] = 0.
        last["points"][0]["velocities"] = []
        last["points"].append({"positions": [.5] * 7, "time_from_start": .4})
        self.assertEqual(coalesce_trajectory_commands([first, last]), [last])

    def test_supported_names_alias_is_canonicalized_without_modifying_inputs(self):
        first, last = self.command(), self.command(value=.4)
        last["names"] = last.pop("joint_names")
        commands = [first, last]
        original = deepcopy(commands)
        self.assertEqual(coalesce_trajectory_commands(commands), [last])
        self.assertEqual(commands, original)
        self.assertEqual(coalesce_trajectory_commands([]), [])


class TaxelTests(unittest.TestCase):
    def test_rotated_world_contact_projects_into_tip_grid(self):
        orientation = [0, 0, math.sin(math.pi/4), math.cos(math.pi/4)]
        local = [0, -.01, .035]
        offset = rotate_vector(local, orientation)
        point = [value + origin for value, origin in zip(offset, [1, 2, 3])]
        cells = contact_taxels([3.5], [point], [1,2,3], orientation)
        self.assertEqual(cells[0], 3.5)
        self.assertAlmostEqual(sum(cells), 3.5)

    def test_valid_empty_contacts_are_zero_and_nonfinite_contacts_invalid(self):
        self.assertEqual(contact_taxels([], [], [0,0,0], [0,0,0,1]), [0.] * 9)
        with self.assertRaises(ValueError):
            contact_taxels([float("nan")], [[0,0,0]], [0,0,0], [0,0,0,1])

    def test_depth_grid_uses_max_measured_penetration_and_excludes_positive_separation(self):
        points=[[0,-.01,.035]]*3
        cells=contact_depth_taxels([-.001,-.003,.01],points,[0,0,0],[0,0,0,1])
        self.assertEqual(cells[0],.003)
        self.assertEqual(sum(cells),.003)


if __name__ == "__main__":
    unittest.main()
