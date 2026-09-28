# NEXUS adapter contract

## Topics and ownership

All canonical component topics live under `/nexus/<instance>/`. For each `<component>` defined in profile order:

| Topic | Type | Publisher |
| --- | --- | --- |
| `components/<component>/joint_states` | `sensor_msgs/JointState` | hardware adapter, measured whenever available |
| `candidates/{teleop,policy,playback}/<component>/joint_commands` | `sensor_msgs/JointState` | source adapter |
| `components/<component>/joint_commands` | `sensor_msgs/JointState` | **only** `nexus_command_mux` |

Every JointState has exactly the profile's joint names in the declared order; arms/hands/grippers use radians and model vectors concatenate components in profile order. The Astral gripper's normalized closure input becomes a joint angle before arbitration. Native Wuji/XHand retargeting may publish only to `legacy/*candidate`, then their bridges convert it. Wuji/XHand bridge feedback comes from the hardware driver, never the retargeter's command echo. Astral gripper feedback is explicitly marked `command_echo` in the profile.

Input bridge contracts: `input/<side>/wrist_pose` (`PoseStamped`), `input/<side>/hand_landmarks` (`PoseArray`), `input/<side>/controller_joy` (`Joy`); cameras publish `camera/<role>/image/compressed` (`CompressedImage`). All stamped sources must have a stable frame ID. The Orbbec node owns each serial-numbered device once and publishes the same semantic role to recording and preview.

The mux owns `IDLE`, `TELEOP`, `POLICY`, `PLAYBACK`, `PAUSED`, `ESTOP`. Handoff first pauses at measured joints; arm teleop adapters reanchor to fresh measured feedback before HUMAN control. NEXUS enables Astral's strict measured-state reanchor guard; failed policy startup cannot claim HUMAN control until takeover succeeds. Expired candidate/state data causes pause or stops final command publication. The driver must reject wrong joint order, dimensions, NaN/Inf and out-of-range targets, and apply a local command watchdog. Software stop and physical motor disable are distinct. Nero and Astral arm adapters expose physical disable. NEXUS launches Wuji with motors initially disabled and its bridge forwards enable/stop to the native `set_enabled` service. The XHand bridge stop remains a software gate because the imported serial driver has no enable/disable service. Site safety must account for that hardware limitation.

## Bringing in another robot

1. Copy a built-in JSON profile; define a unique `instance`, component order, hardware IDs, limits, base frames, orthogonal VR mapping, semantic camera roles, dataset action semantics and model camera slots. `nexus_profile` must accept it before launch.
2. Implement one driver adapter for each new actuator. It must own the physical connection and publish actual `JointState` feedback. It subscribes **only** to its final canonical command and exposes ready, enable, home and stop behavior. Document whether its emergency stop disables physical power.
3. Implement an IK adapter that converts canonical wrist pose and measured arm joints into teleop candidates; keep the solver core private to the robot package. Hand retargeters likewise publish candidates only.
4. Add the adapter nodes to the assembly launch and ensure no second final-command publisher exists. Declare camera serial-to-role mapping and publish one compressed stream per role.
5. Run the tests in `NEXUS_ACCEPTANCE.md`, record baseline rates and latency, and produce a matching `nexus_layout.json` for every model. A new driver/IK implementation is required for a genuinely new hardware family; profile changes alone are sufficient only for existing adapters.

The current built-in profile validator intentionally recognizes Astral and Nero adapter joint orders. Extend its adapter registry and launch assembly when adding a new robot family.

## Legacy Nero data import

The raw HDF5 importer maps `left_arm/joints`, `right_arm/joints`, `left_hand/joints`, `right_hand/joints` to canonical component streams. The camera map must explicitly bind `base`, `left_wrist`, `right_wrist` to old IDs such as `cam_0`, `cam_1`, `cam_2`:

```bash
ros2 run nexus_core nexus_import_nero --profile /abs/nero_profile.json --source /abs/old_episode --output /abs/new_session/episode_000001 --camera-map '{"base":"cam_0","left_wrist":"cam_1","right_wrist":"cam_2"}'
```

The LeRobot importer maps 38D state/action order and explicit image feature names. It verifies old actions agree with next-state semantics; use `--allow-action-rewrite` only when deliberately regenerating actions from observations during NEXUS processing:

```bash
ros2 run nexus_core nexus_import_nero_lerobot --profile /abs/nero_profile.json --source /abs/old_lerobot --output-session /abs/new_session --camera-map '{"base":"observation.image","left_wrist":"observation.wrist_image_left","right_wrist":"observation.wrist_image_right"}'
```

Both importers create new raw episodes with frozen profile metadata and `conversion_report.json`. The original dataset is read only. Validate camera roles against real video before training; the old Nero exporter filled missing camera frames with black, which may be present in source episodes.
