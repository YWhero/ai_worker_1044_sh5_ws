# ffw_navigation

Nav2-based navigation stack for the FFW swerve-drive platform: bringup launch
files, Nav2/AMCL/slam_toolbox parameters, maps, and custom Nav2 plugins
(`SimTimeNavigate*` navigators, `IsPathLengthUnder` BT condition node).

## Validated localization configuration — 2026-08-12

Field-validated in simulation as "good": localization no longer diverges
while driving, and arc/curve segments (steering while translating) track
correctly. **These are the EFFECTIVE values verified live on the running
`/amcl` node** — note the two-source caveat below.

| Parameter | Validated value | Where it lives today | Note |
|---|---|---|---|
| `update_min_d` / `update_min_a` | **0.03 m / 0.08 rad** | `launch/navigation.launch.py` sim override (yaml has 0.05/0.05) | Correction cadence — the fix for drive-time divergence. Practical floor is scan-bounded (10 Hz). |
| `max_beams` | **60** | **`config/navigation.yaml` (promoted 2026-08-12)** — sim override carries the same value | Scan-matching constraint per update. Keep under ~100 (over-peaked particle weights). |
| `robot_model_type` | **`nav2_amcl::OmniMotionModel`** | **`config/navigation.yaml` (promoted 2026-08-12)** — sim override carries the same value | The swerve base moves laterally on arcs/module re-alignment; the omni model explains that motion. Activates `alpha5` (0.05, as validated). |
| `alpha2` / `alpha4` | **0.12** | `config/navigation.yaml` (applies everywhere) | Motion-model CROSS terms (rotate-while-translating) — raised for arc drift; straight/in-place terms stay low. |
| `alpha1` / `alpha3` / `alpha5` | **0.05** | `config/navigation.yaml` | Straight lines and in-place turns were fine at stock. `alpha5` (lateral, omni-only) untuned yet — see future work. |
| `resample_interval` | **1** | both (identical) | Resample every update. |
| `laser_likelihood_max_dist` | **2.0** | `config/navigation.yaml` | Wider likelihood gradient pulls the estimate back when error grows. |
| `recovery_alpha_fast/slow` | **0.0 / 0.0** | `config/navigation.yaml` | Recovery injection off (stock). |
| `FollowPath.rotate_to_heading_angular_vel` | **0.6 rad/s** (sim) | sim override (yaml has 1.5) | In-place heading alignment speed at waypoints. The two values have NOT been reconciled. |

### ⚠️ Two-source caveat (read before touching anything)

`launch/navigation.launch.py` applies a `RewrittenYaml` override block **only
when `use_sim_time` is true** (i.e. under Gazebo). Therefore:

- **Simulation runs**: `config/navigation.yaml` *patched by* the override
  block — the validated column above.
- **Real robot runs**: `config/navigation.yaml` **as-is, no overrides** —
  today that means `DifferentialMotionModel`, 0.05/0.05 cadence, 40 beams,
  1.5 rad/s rotation: **a combination that was never validated as a whole.**

**To deploy the validated configuration to the real robot**, promote the
remaining override values into `config/navigation.yaml`:
- ~~`max_beams: 60`~~ — promoted 2026-08-12
- ~~`robot_model_type: nav2_amcl::OmniMotionModel`~~ — promoted 2026-08-12
- `update_min_d: 0.03` / `update_min_a: 0.08` (yaml still 0.05/0.05)
- decide `rotate_to_heading_angular_vel` (sim 0.6 vs yaml 1.5)

Then strip the duplicated entries from the launch override so simulation and
the real robot read one source of truth. Until that is done, sim results do
not transfer to the robot.

Verify what a running stack actually uses with
`ros2 param get /amcl <name>` — do not trust the yaml alone.

### Tuning history (condensed)

- **Root symptom**: pose diverged from reality while driving until
  localization was lost. Cause: stock correction cadence (0.35 m/0.3 rad)
  let odometry error accumulate faster than corrections recovered.
  Tightening the cadence fixed it (validated 2026-08-12).
- **Arc drift**: localization drifted specifically while steering
  mid-drive. Fixed by raising only the cross-term noises `alpha2`/`alpha4`
  0.05 → 0.12 (validated) — on arcs the swerve modules re-align
  continuously, exactly where swerve odometry is weakest. The omni motion
  model (active in sim) targets the same regime.
- **Rejected (2026-08-11)**: raising ALL alphas to 0.15 + recovery
  injection as one bundle made localization noticeably worse — the swerve
  odometry is better than assumed; tripled noise diffused the particle
  cloud. Lesson encoded since: change ONE knob at a time.
- **Accepted trade-off**: corrections at 5–10 Hz read as pose micro-jitter.
  Measured NOT a CPU problem (amcl ≤20% of one core under forced max rate;
  24-core host 77% idle). If it needs taming, try `resample_interval: 1 → 2`
  in isolation.
- Older beam experiments (20/40/80 via yaml) were partially masked by the
  sim override (60) and are inconclusive; 60 is the validated value.

### Future work

- Tune `alpha5` (lateral noise, active under the omni model): pure-strafe
  teleop test while watching `/particle_cloud` — raise if the pose lags then
  snaps during crab motion, lower if the cloud stays diffuse.
- `sigma_hit` 0.3 → 0.2 for a sharper likelihood field once stable.
- Decide the real-robot `rotate_to_heading_angular_vel` (sim validated 0.6;
  yaml carries 1.5 from earlier slow-rotation fixes).

### Odometry itself (prerequisite for everything above)

AMCL only corrects odometry. Validate with a 5 m straight-line, an in-place
360°, and an S-curve test against `/odom`; also check that pure
steering-module re-alignment (no base motion) does not move `/odom`.
Calibration parameters (`wheel_radius`, `module_x/y_offsets`,
`module_angle_offsets`) live in
`ffw_bringup/config/ffw_sg2_rev1_follower/ffw_sg2_follower_ai_hardware_controller.yaml`
(swerve_drive_controller block); integration logic in
`ffw_swerve_drive_controller/src/swerve_drive_controller.cpp`. These are
ros2_control parameters — changing them requires a robot bringup restart,
not just a navigation restart.

### Applying changes

The workspace is `--symlink-install`ed and configs are symlinked, so edits to
`config/navigation.yaml` (or the launch override) take effect on the **next
navigation (re)start** — no rebuild. To A/B on a running stack without a
restart: `ros2 param set /amcl <name> <value>`, watching `/particle_cloud`
in RViz. AMCL is motion-gated — a stationary robot gets no corrections
regardless of these values (the `/nomotion-update` supervisor endpoint
forces one).
