# ViTacFormer policy backend

ViTacFormer is deployed as an independent Cyclo policy backend:

- backend/service namespace: `vitacformer`
- container: `vitacformer_server`
- image: `robotis/vitacformer-zenoh:1.0.0-<arch>`
- engine module: `vitacformer_engine`
- default model root: `/workspace/model/vitacformer`

Unlike the LeRobot development container, the dedicated image copies the
engine, common runtime, and SDK sources into the image. Runtime Compose mounts
only workspace data, caches, devices, and robot configuration.

RTX 5090 / Blackwell requires a compatible PyTorch build. The original amd64
base uses CUDA12.6 and predates native `sm_120` support. PyTorch documents
[Blackwell support in 2.7 with CUDA12.8](https://pytorch.org/blog/pytorch-2-7/)
and the matched
[2.7.1 / torchvision0.22.1 / torchaudio2.7.1 CUDA12.8 installation](https://pytorch.org/get-started/previous-versions/).
Build the separate simulation image from `cyclo_brain/` after the ordinary
ViTacFormer image is available:

```bash
docker build -f policy/vitacformer/Dockerfile.hx5_sim.amd64 \
  -t cyclo-1044-sh5/model-vitacformer:hx5-sim-amd64 .
```

This retains the original image and upgrades only the separate simulation tag
using official wheels. `BASE_IMAGE` and matched version/index arguments are
configurable. Verify CUDA availability, `torch.cuda.get_arch_list()` including
`sm_120`, and non-actuating model parity on the actual GPU before using it in
the simulation backend.

The backend accepts the audited SH5 ViTacFormer artifact layout containing
`train_config.json`, `normalization_stats.pt`, and a supported checkpoint.
It is intentionally fail-closed for the `ffw_sh5_rev1` 54-joint contract.

Supported action horizons are the existing 100-step exports and the 200-step
`vitacformer_sh5_h200_v1` / `sh5_h200_original_tactile_r1` export. H200 uses
200 action queries and a 202-row latent position table, while future tactile
prediction remains 18 rows. Its learned tactile residuals apply to both hands;
the legacy loader's right-hand persistence fallback remains unchanged for
100-step exports. H200 also requires the original `inference_config.json` and
normalization hashes bound into its checkpoint. Do not edit artifact metadata
to bypass a contract mismatch.

The pour H100 export (`vitacformer_sh5_pour_h100_v2`, recipe
`task519_folder159_gt75_residual_unpenalized_r1`) is also supported. It stores
architecture metadata under `train_config.json`'s `model_config` and its
preprocessing contract in `inference_config.json`. The loader validates these
original files and the checkpoint's config/stats hashes, then adapts the
metadata in memory. Both hands retain their learned tactile residuals for this
export. Legacy H100 right-hand persistence and H200 behavior are preserved.
Selecting the run folder or its `checkpoints` folder loads `best_model.pt`;
an explicit `.pt` file or numeric checkpoint folder selects those weights.

The corrected Pour LR1e-4/B512 export is supported with recipe
`task519_folder159_lr1e4_b512_w5_gt75_scratch_r1`. The pinned reference is
[`Dongkkka/Task000519_PourWater_ViTacFormer_H100_LR1e4_B512_Hand_Intern`](https://huggingface.co/Dongkkka/Task000519_PourWater_ViTacFormer_H100_LR1e4_B512_Hand_Intern/tree/b4f21caa4e6a7a05814958b63d5ced46a9374321).
Its default is `checkpoints/best_validation.pt`, matching the reference loader;
missing validation-best weights fail rather than selecting `latest_model.pt`
or legacy `best_model.pt`. Select another `.pt` explicitly to compare it.
The older Pour recipe keeps its `best_model.pt` default. Both retain the same
54-D state/action, 180-D tactile, image preprocessing, bounds and normalization
contract. Checkpoint recipe must match the selected run, and the original
normalization/config hashes remain enforced without changing artifact files.
The package reports `offline_release_pass=false` and does not approve powered
SH5 deployment; loader compatibility and synthetic inference do not establish
task success.

The Upright H100 export (`vitacformer_sh5_upright_h100_v2`, recipe
`task608_upright159_gt75_residual_unpenalized_r1`) is also supported, using the
same H100 tensor/preprocessing contract and both hands' learned tactile
residuals. Its run folder or `checkpoints` folder selects
`checkpoints/best_validation.pt`, matching its reference loader. It never
silently falls back to `latest_model.pt` or legacy `best_model.pt`; select a
different `.pt` explicitly. Legacy, H200 and older Pour defaults remain unchanged.
Checkpoint architecture/recipe, normalization/config hashes, strict parameter
loading and the original joint bounds/warm-start behavior remain enforced.

Reference package and integration details:
[`VITACFORMER_UPRIGHT_H100_INTEGRATION.md`](../../../docs/VITACFORMER_UPRIGHT_H100_INTEGRATION.md).
The package describes an incomplete research checkpoint, not approved for
powered deployment. Format support does not validate task performance or
authorize real-robot execution.

For regression tests, run `pytest policy/vitacformer/tests` from `cyclo_brain`
in an environment with compatible PyTorch/torchvision. Set
`VITACFORMER_POUR_TEST_ROOT` to the complete original pour package to additionally
check strict checkpoint loading and output parity with its packaged reference
loader on CPU and CUDA (when available). These tests use synthetic observations
and do not connect to robot services.

For the Upright source-only comparison, set
`VITACFORMER_UPRIGHT_REFERENCE_ROOT` to the pinned package containing
`preprocess.py` and `source_snapshot/ViTacFormer_SH5`. Run
`pytest policy/vitacformer/tests/test_upright_h100.py` in a fresh process.
This uses identical random weights, not trained checkpoints, to compare the
reference and Cyclo action/tactile outputs on CPU without robot connections.

The prediction horizon is separate from the runtime execution limit. The image
retains `POLICY_SOURCE_CHUNK_LIMIT=20`, 30 Hz source actions, and the existing
alignment/refill settings.

The isolated HX5 simulation policy mounts the official workspace
`robotis_interfaces` package read-only at `/zenoh_sdk/messages/robotis_interfaces`.
The SDK also registers the official repository at commit
`9231cb1005dc03c14bdbf42f1f9b7114af7d3cfb` as a network fallback. Its nested
`HandPressures`/`TactileSensor` definitions preserve the Jazzy wire type hash;
no tactile message fields are fabricated.

RobotClient histories use monotonic callback receipt timestamps. ROS headers
can use simulation time, which cannot directly replace this clock. Hardware
defaults require the latest tactile frame and each causal resampling gap to
be at most one requested sample period (33.3 ms for ViTacFormer). The isolated
simulation Compose profile explicitly sets `CYCLO_SENSOR_HISTORY_MODE=simulation`
and `CYCLO_SIM_TACTILE_HISTORY_MAX_AGE_S=0.075` to tolerate measured simulator
delivery jitter. An override without simulation mode, invalid number, or value
above 0.1 s fails configuration. The allowance applies to both the latest
receipt and resampling gaps; the 30 Hz, 18-frame causal grid, complete-history
coverage, raw pressure values, 32-frame baseline calibration and policy
artifact hash checks are unchanged. It does not zero-fill missing input.
If the simulator is paused or drops data beyond this budget, LOAD or inference
still fails with the concrete history error. This is a simulator timing
adaptation and does not validate hardware timing or PourWater task success.

For `initial_pose_sync=true` in this isolated simulator, the measured Pour
checkpoint's first action closes each open thumb joint2 by 1.5 rad; its first
arm targets differ by about 0.00001 rad. The profile explicitly sets
`CYCLO_INITIAL_POSE_SYNC_MODE=simulation` and
`CYCLO_SIM_INITIAL_POSE_SYNC_HAND_MAX_DELTA_BY_KEY=hand_right=1.55`.
Only the initial sync may use this envelope, only for SH5 hands and with a
transition duration of at least 5 seconds. The hand cap is 1.55 rad, below the
official HX5 rev2 thumb joint2 stroke of 1.57 rad. The existing joint-limit,
fresh-state, first-step and source-step validation still runs before publishing
the trajectory. Arm limits, ordinary warm-start, RESUME preflight and tracking
limits keep their original hardware values. This setting does not permit a
generic change to `REAL_*` safeguards; larger or invalid targets still fail.
The orchestrator accepts both `syncing` and `initial pose sync started` runtime
responses and publishes `SYNCING` throughout its initial-sync timer. BT LOAD
waits for `INFERENCING` before its automatic pause, and BT RESUME waits before
advancing to a following Gate. RESUME's bounded 70-second phase timeout covers
the supported 60-second maximum sync duration. The timer reports the scheduled
transition interval; measured target achievement remains a separate joint Gate
and the runtime's fresh-state/tracking checks.

Temporal ensemble is enabled by `POLICY_TEMPORAL_ENSEMBLE_COEFF=0.01` in both
ViTacFormer images. The common Main runtime averages full, absolute, raw action
predictions before safety bridging, L2 alignment, execution slicing, and output
interpolation. Engine prediction and checkpoint decoding remain unchanged.
The coefficient follows the [reference inference.py](https://github.com/RoboVerseOrg/ViTacFormer/blob/main/inference.py):
newer overlapping predictions receive larger exponential weights. Actual
monotonic request-start times and the checkpoint's 30 Hz action grid replace
the reference's fixed inference interval; fractional offsets use interpolation.
Only unaveraged predictions enter history, and pause, stop, mode changes,
reconfiguration, model switch (which pauses), preflight, and safety rejection
clear history. Late results from an invalid generation cannot repopulate it.

`POLICY_TEMPORAL_ENSEMBLE_TAIL_FADE_S=0.2` fades older contributions before their
prediction horizon expires. This is a Cyclo adaptation: full-horizon asynchronous
output otherwise acquires discontinuities where the number of overlapping
plans changes. Setting it to `0` gives the reference's untapered weights.
Setting the coefficient to `none` disables ensemble; `0` means uniform averaging.
These are runtime settings, independent of the immutable export's historical
`runtime_proposal.temporal_ensembling_enabled=false` metadata. The export is not
rewritten because its hashes are verified against the checkpoint.

Raw model predictions now pass the existing raw-step and joint-limit checks
before entering real-run ensemble history. After averaging multiple plans,
the existing `REAL_SOURCE_STEP_MAX_DELTA_RAD` (0.03 rad) also limits the change
of each joint per source row, retaining the first row and 30 Hz time grid.
This bounds discontinuities caused by disagreement between expiring plans;
fixed tail fading alone cannot bound them for arbitrary predictions. The final
result still passes the usual safety checks, including publish-time tracking.
Unsafe raw model output remains a hard failure; it is not silently smoothed.
Ensemble logs report `limited_values`, and trace metadata records the unlimited
maximum step. On asynchronous chunk rejection, a separate last-rejection JSON
preserves the raw and processed chunks even if high-rate tracing hit 100 MB.

This enables reference-style ensemble, not exact replication of reference
deployment timing: 20-row execution, L2 alignment, 0.2-second boundary blend,
SH5 decoder arm ramp, and real-robot safety limits remain in use. The Main log
reports the effective ensemble settings and overlap counts; chunk traces include
`raw`, `ensembled`, `prepared`, `observed_at`, and `ensemble` metadata.

Updating host engine files requires rebuilding the
image and recreating the container; restarting an old container alone does not
install the updated loader. Loading and offline prediction do not verify robot
execution or timing.

Cycle Home waits for a request-specific completion from the SH5 bringup
returner before either Start path can activate inference. The returner checks
the measured arm and hand targets and reports failure on timeout; publishing
a home trigger alone is not completion. Stop after Home retains the pending
fresh-cycle requirement. The next successful Start recreates the ViTacFormer
policy object, sensor histories and tactile calibration, then uses a fresh
preflight instead of continuation. This cycle boundary reloads the checkpoint
and therefore takes longer than an ordinary Stop/Resume.

Build on Jetson:

```bash
docker compose -f docker/docker-compose.yml build vitacformer
```

Build and start through the repository helper:

```bash
docker/container.sh start-vitacformer --build
```
