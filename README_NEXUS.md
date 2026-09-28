# NEXUS

NEXUS is a ROS 2 Humble workspace for one selected robot assembly per launch. It keeps Astral's teleoperation, collection, data processing and inference code, and brings in Nero, dual XHand and Gemini cameras. The original `astral_ws` and `xnero_ws` directories are preserved. Source provenance is in [docs/MIGRATION_MANIFEST.json](docs/MIGRATION_MANIFEST.json).

## Built-in assemblies

| Profile | State/action | Input | Output |
| --- | ---: | --- | --- |
| `astral_dual_gripper` | 16 | Quest 3 | Astral dual arms and two grippers |
| `astral_gripper_wuji` | 35 | Quest 3 | Astral dual arms, left gripper, right Wuji hand |
| `astral_gripper_wuji_glove` | 35 | Quest 3 wrists, Wuji glove hand landmarks | Astral dual arms, left gripper, right Wuji hand |
| `nero_dual_xhand` | 38 | Quest 3 | Nero dual arms and two XHands |

The schema v2 profile JSON in `src/nexus_core/profiles` is the single assembly definition: it selects registered drivers, IK and retargeting plugins, input topics, camera roles, joint order/units/limits, and model/data semantics. The core launch, input bridge, driver manager, recorder, processing and policy service consume that same profile; there is no `robot` family switch in the core. The profile digest and frozen component order are embedded in each recorded episode. The model manifest must match this layout before inference starts. Existing adapters combine by changing profile fields; a new hardware family needs its own driver or IK plugin registered once.

## Build and launch on the robot

Use Ubuntu 22.04 with ROS 2 Humble and Python 3.10. Install the source packages and external SDKs described in [docs/DEPENDENCIES.md](docs/DEPENDENCIES.md), then build from this workspace root:

```bash
source /opt/ros/humble/setup.bash
python3 -m pip install -e src/pyAgxArm
colcon build --symlink-install
source install/setup.bash
ros2 run nexus_core nexus_profile $(ros2 pkg prefix nexus_core)/share/nexus_core/profiles/nero_dual_xhand.json
ros2 launch nexus_core system.launch.py profile:=nero_dual_xhand dry_run:=true with_cameras:=false
```

`dry_run:=true` is the default. A real launch requires configured device serials, CAN interfaces, calibrations, installed vendor libraries and the explicit site checks in [docs/NEXUS_ACCEPTANCE.md](docs/NEXUS_ACCEPTANCE.md). The Wuji profile has placeholder serials by design and rejects a real launch until they are replaced in a copied profile file. Put site copies in `NEXUS_PROFILES_DIR` for web selection; use a filename matching `profile_id`. A site copy with the same name overrides the built-in profile. Example:

```bash
ros2 launch nexus_core system.launch.py profile:=/absolute/path/to/site_profile.json dry_run:=false with_cameras:=true
```

The launch prints the active profile ID and SHA256. This release starts one NEXUS assembly per launch; every NEXUS input, camera, control, policy and driver lifecycle interface is already scoped under `/nexus/<instance>`.

## ROS control contract

See [docs/NEXUS_ADAPTER_CONTRACT.md](docs/NEXUS_ADAPTER_CONTRACT.md). The sole final command publisher is `nexus_command_mux`; all teleop, policy and playback commands are candidates. The mux validates names, dimensions, nonfinite values, state freshness and limits, and holds measured pose during transitions. On stale feedback it stops publishing to that component so the local adapter's watchdog can take over. `nexus_driver_manager` exposes one ready/enable/home/estop API for the selected assembly. NEXUS readiness is read-only; Astral's legacy `/astral_robot_driver/ready` performs motion and is not used by the manager.

## Recording, processing, training and inference

The NEXUS launch starts the profile-driven recorder and policy controller by default. The web monitor has a NEXUS tab for assembly selection, camera previews, recording, control modes, HITL and GPU jobs. The same backend can be used from CLI:

```bash
ros2 run nexus_core nexus_gpu_job sync_process --profile /absolute/path/to/site_profile.json --session demo
ros2 run nexus_core nexus_gpu_job train_act --profile /absolute/path/to/site_profile.json --session demo --steps 1000
```

Set `NEXUS_DATA_ROOT`, `NEXUS_GPU_HOST`, `NEXUS_GPU_ROOT`, `NEXUS_GPU_PYTHON` and `NEXUS_GPU_ACT_SCRIPT` on the robot. SSH keys and rsync must work without an interactive prompt. The GPU host needs this NEXUS source installed in its Python environment and access to LeRobot/ACT. `sync_process` copies immutable raw episodes and the profile, then aligns, checks, quarantines failed episodes, and exports pi0.5 (LeRobot v2.1) and ACT (v3). Training checks the dataset layout and stamps checkpoints with `nexus_layout.json`. Cancel is available from the web job list.

pi0.5 training depends on a separately installed OpenPI tree with a configuration for **each** state/action layout. Set `NEXUS_PI05_TRAIN_<PROFILE_ID>` to a command template whose arguments include `{dataset}`, `{output}`, `{steps}` and, if needed, `{profile}`. For example, a site wrapper may call OpenPI's norm-statistics and training scripts with its profile-specific config. NEXUS fails clearly when the command is unset or when the dataset layout does not match. The imported Astral-only `scripts/openpi_train.sh` is not a generic Nero trainer.

The remote model service keeps the existing websocket protocol. Pass `--nexus-manifest` to `astral_policy_inference/scripts/serve.py` so it checks the checkpoint layout and advertises the profile hash; launch the robot with `model_manifest:=/path/to/nexus_layout.json`. The web console can generate a local layout from the selected profile when its manifest field is blank, but the remote service must still advertise the same hash from its stamped checkpoint. Inference refuses a missing/mismatched manifest for non-stub backends.

Legacy Nero raw HDF5 and LeRobot import commands, mapping rules and preservation checks are in [docs/NEXUS_ADAPTER_CONTRACT.md](docs/NEXUS_ADAPTER_CONTRACT.md). Old Astral episodes retain their original schema reader. Imported data never overwrites its source.

## Verification status

Local Python profile/arbiter/import/align/quality tests and the web frontend build pass. This Windows workspace has no ROS 2 installation, physical robots, cameras or GPU training service, so ROS graph, ACT/pi0.5 short training, physical safety and full HITL acceptance remain open. Do not enable a real profile until the corresponding checklist in [docs/NEXUS_ACCEPTANCE.md](docs/NEXUS_ACCEPTANCE.md) passes on site.
