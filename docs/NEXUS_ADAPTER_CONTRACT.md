# NEXUS profile and adapter contract

NEXUS does not select a robot family in its core. A schema v2 profile composes
actuator components, registered driver adapters, IK and retargeting adapters,
input sources, camera sources, and model/data semantics. The same profile is
used by launch, recording, processing, training, and inference. The component
array order is the state/action vector order and is frozen into every episode
and model manifest.

Existing hardware combinations are profile-only selections. A genuinely new
hardware interface needs its driver or IK implementation registered once; it
does not add another robot-specific branch to the command mux, recorder, or
policy service.

## Profile structure

Profiles live in `src/nexus_core/profiles`. A component declares its logical
name, side, type, fixed joint order, limits, units, driver adapter, optional IK
or retargeter, and input channel. Hardware IDs and settings live under
`adapter_config` beside the component layout:

```json
{
  "schema_version": 2,
  "profile_id": "example_dual_arm",
  "instance": "cell_a",
  "components": [
    {
      "name": "left_arm",
      "side": "left",
      "kind": "arm",
      "joints": ["left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5", "left_joint6", "left_joint7"],
      "lower": [-2.0, -1.0, -2.0, -1.0, -2.0, -1.0, -1.5],
      "upper": [2.0, 1.0, 2.0, 2.0, 2.0, 1.0, 1.5],
      "unit": "rad",
      "driver": "nero_can",
      "feedback": "measured",
      "ik": "nero_analytic",
      "input_channel": "left",
      "teleop": {
        "arm_base_frame": "left_arm_base",
        "vr_to_arm_rot": [1,0,0,0,1,0,0,0,1],
        "motion_scale": 0.65,
        "tcp_offset": [0,0,0,0,0,0]
      }
    }
  ],
  "adapter_config": {
    "nero_can": {"channels": {"left": "can_left"}}
  }
}
```

This abbreviated example is illustrative; each registered hardware adapter
defines its actual joint count, order, limits, settings, and hardware identity.
The profile checker rejects unknown adapters, mismatched component kinds,
incorrect native joint order, missing input references, duplicate devices,
bad dimensions, invalid limits, unstable/missing input frames, and a model
manifest whose frozen layout differs.

Input channels map semantic data to source adapter topics. For each channel,
`wrist`, `hand`, and `controller_joy` normalize to `PoseStamped`, `PoseArray`,
and `Joy`; `body_joints` and `body_joint_names` are optional Quest inputs. A
stamped input's frame is preserved and monitored for changes. Camera entries
bind a semantic role to one source, device ID, and (where applicable) source
topic. This lets recording and inference consume `base`, `left_wrist`, and
`right_wrist` without knowing the camera driver.

Schema v1 profiles remain readable so recorded episode metadata keeps its
original SHA256 identity. New site profiles should use schema v2 and store
hardware settings in `adapter_config`.

## ROS contract and lifecycle

All runtime APIs are scoped to `/nexus/<instance>`.

| Topic/service | Type | Owner/meaning |
| --- | --- | --- |
| `components/<component>/joint_states` | `sensor_msgs/JointState` | Driver adapter feedback in profile joint order |
| `candidates/{teleop,policy,playback}/<component>/joint_commands` | `sensor_msgs/JointState` | Source candidates; never connect these directly to hardware |
| `components/<component>/joint_commands` | `sensor_msgs/JointState` | Sole output of `nexus_command_mux` |
| `input/<channel>/wrist_pose` | `geometry_msgs/PoseStamped` | Selected wrist source, timestamp and frame preserved |
| `input/<channel>/hand_landmarks` | `geometry_msgs/PoseArray` | Selected hand source |
| `input/<channel>/controller_joy` | `sensor_msgs/Joy` | Selected controller input |
| `camera/<role>/image/compressed` | `sensor_msgs/CompressedImage` | Single semantic camera stream |
| `drivers/{ready,enable,home,estop}` | `std_srvs/Trigger` | Unified assembly lifecycle API |
| `control/{cmd,state}` | `std_msgs/String` | Mode requests and arbiter status |
| `control/{teleop_start,teleop_disarm,teleop_armed}` | `std_msgs/Bool` | Namespaced teleoperation gates |
| `policy/{cmd,state}` | `std_msgs/String` | Policy/HITL requests and state |

`nexus_driver_manager` exposes one lifecycle API and fans calls out to the
driver services declared by the adapter registry. `ready` only checks current
feedback and adapter readiness; it never homes a robot. `enable`, `home`, and
`estop` report partial failures. A failed partial enable triggers estop on all
selected adapters. Homing is allowed only while the arbiter is `IDLE` or
`PAUSED`. Astral's legacy `ready` service performs motion and is deliberately
not used as NEXUS readiness.

The arbiter owns `IDLE`, `TELEOP`, `POLICY`, `PLAYBACK`, `PAUSED`, and `ESTOP`.
It seeds every source transition from measured state, checks command and state
timeouts, and is the only final command publisher. Each hardware adapter checks
joint names/order, vector dimensions, limits, finite values, enable state, and
its local watchdog. Input loss or inference timeout pauses command ownership;
HITL must reanchor to fresh measured state before teleoperation resumes.

## Adding or changing an assembly

1. Copy the closest profile. Keep the component order explicit; fill in joint
   names, units, limits, input channels, camera roles, adapter settings, frame
   transforms, dataset action semantics, and policy camera slots.
2. Run `ros2 run nexus_core nexus_profile /path/to/profile.json`. Resolve all
   reported errors before launching. The profile SHA256 printed by launch is
   the identity to record with the site calibration and model.
3. Select existing registered adapters by changing profile fields. This is
   sufficient for supported Astral, Nero, gripper, Wuji, and XHand layouts.
4. For a new hardware family, implement its driver adapter against canonical
   `JointState` topics and ready/enable/home/estop services, register its
   capabilities and ordered joints, and implement an IK/retargeter adapter if
   its kinematics or hand mapping differ. Keep vendor ROS topics inside that
   adapter. Do not add driver-specific conversions to data or policy code.
5. Run the fake-driver, camera replay, data-layout, model-manifest, timeout,
   takeover, and site hardware acceptance checks in `NEXUS_ACCEPTANCE.md`.

Launch still runs one assembly at a time in this initial release. The instance
namespace is part of every NEXUS topic and service so multiple assemblies can
be enabled later without changing the core interfaces.

## Legacy Nero import

The raw HDF5 importer maps `left_arm/joints`, `right_arm/joints`,
`left_hand/joints`, and `right_hand/joints` to canonical component streams. The
camera map must explicitly bind `base`, `left_wrist`, and `right_wrist` to old
IDs such as `cam_0`, `cam_1`, and `cam_2`:

```bash
ros2 run nexus_core nexus_import_nero --profile /abs/nero_profile.json --source /abs/old_episode --output /abs/new_session/episode_000001 --camera-map '{"base":"cam_0","left_wrist":"cam_1","right_wrist":"cam_2"}'
```

The LeRobot importer checks the legacy 38D joint-block order and explicit image
feature mapping before writing a new session:

```bash
ros2 run nexus_core nexus_import_nero_lerobot --profile /abs/nero_profile.json --source /abs/old_lerobot --output-session /abs/new_session --camera-map '{"base":"observation.image","left_wrist":"observation.wrist_image_left","right_wrist":"observation.wrist_image_right"}'
```

Both importers preserve source data and write a conversion report. Validate
camera roles against source videos before training; the old Nero exporter may
have filled missing camera frames with black.
