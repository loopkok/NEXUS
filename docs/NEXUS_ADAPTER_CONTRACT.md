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
        "motion_scale": 0.65
      },
      "frame_transforms": {
        "link7_to_xhand_palm": [0.125,0,-0.0235,1.5708,0,1.5708]
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

Quest wrist mapping is selected by `input_settings.quest3_wrist_pose_mapping`.
The default `global` mode preserves the Quest node's generic conversion. A
profile may select `per_side` and provide left/right rotation matrices plus
stable frame IDs; the Quest adapter then applies that side mapping to the full
pose, bypassing the generic conversion for wrist/controller poses. Hand/body
conversion remains controlled by `convert_to_robot`. Nero Quest wrist poses
are already the link7 flange target: its profile puts the legacy XNero
left/right rotations at the Quest adapter and keeps NEXUS `vr_to_arm_rot` at
identity. NEXUS anchors and solves directly in link7, without applying the
physical flange-to-palm frame transform. Physical tool geometry belongs under
`frame_transforms`, not in the teleoperation mapping.

Schema v1 profiles remain readable so recorded episode metadata keeps its
original SHA256 identity. New site profiles should use schema v2 and store
hardware settings in `adapter_config`.

## ROS contract and lifecycle

### Nero 真机归位到位阈值

编辑实际启动所用 profile 的 `adapter_config.nero_can.home_tolerance_rad`。
内置真机配置是 `src/nexus_core/profiles/nero_dual_xhand.json`，默认值：

```json
"home_tolerance_rad": 0.05
```

单位为 **rad（弧度）**，`0.05 rad ≈ 2.86°`。一个数应用于两臂所有关节；
也可以填写按组件 `joints` 顺序排列的七个值，或用 `left` / `right` 分别配置。
以下仅是格式示例（J7 为 `0.10 rad ≈ 5.73°`），内置默认值仍为 `0.05`：

```json
"home_tolerance_rad": {
  "left":  [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.10],
  "right": [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.10]
}
```

每个关节的绝对误差必须**小于**其阈值，且保持稳定 0.3 秒、漂移不超过
0.002 rad 才通过。该配置只影响到位判定，不改变归位目标、速度、模式确认、
超时、失联保护或遥操限位。诊断与失败日志使用同一组阈值。
缺省字段保持旧行为；非法值、非正数、布尔值和不等于七项的数组会被拒绝，
使用分侧配置时必须提供所启动机械臂对应的键。

修改后需停止当前机器人会话，再重新启动才生效；不是运行中热更新。
Web 使用冻结的会话 profile，编辑源文件不会改变已经运行的会话。
从 Web 重新选择并校验更新后的配置后启动。主机路径通常为：
`/home/loopkok/NEXUS/src/nexus_core/profiles/nero_dual_xhand.json`。
此设置属于 Nero CAN 真机驱动，MuJoCo 仿真归位不使用这个字段。

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

Source candidates are commands and use `RELIABLE`, `KEEP_LAST` depth 1 QoS from
the source adapter through the mux subscription. Input poses, measured states,
camera frames, and final driver commands use sensor-data QoS; the driver-local
watchdog protects final command loss. New teleop, retargeter, policy, and replay
plugins must use the candidate QoS contract so one delayed message cannot build
an old command queue.

`nexus_driver_manager` exposes one lifecycle API and fans calls out to the
driver services declared by the adapter registry. `ready` only checks current
feedback and adapter readiness; it never homes a robot. `enable`, `home`, and
`estop` report partial failures. A failed partial enable triggers estop on all
selected adapters. Homing is allowed only while the arbiter is `IDLE` or
`PAUSED`; the manager first moves the arbiter to `HOMING`, which suppresses all
final command output while the driver performs its lifecycle action. Adapters
with an asynchronous home target register a target resolver so the manager can
wait for fresh measured feedback to reach it before returning to `IDLE`. Other
adapters must keep their home service active until homing is complete. Astral's
legacy `ready` service performs motion and is deliberately not used as NEXUS
readiness.

The arbiter owns `IDLE`, `TELEOP`, `POLICY`, `PLAYBACK`, `PAUSED`, `HOMING`, and
`ESTOP`.
It seeds every source transition from measured state, checks command and state
timeouts, and is the only final command publisher. Each hardware adapter checks
joint names/order, vector dimensions, limits, finite values, enable state, and
its local watchdog. Input loss or inference timeout pauses command ownership;
HITL must reanchor to fresh measured state before teleoperation resumes.

## Adding a hardware plugin package

The core discovers Python entry points from installed ROS packages. A new
robot package can register its driver, IK, retargeter, input, and camera
plugins without changing `nexus_core`. Install the package into the same ROS 2
environment as NEXUS, then build a profile that names those plugin IDs.

For example, the new package's `setup.py` can register an arm driver, IK
solver, and a custom end-effector mapping:

```python
entry_points={
    "nexus.driver_adapters": [
        "acme_arm = nexus_acme.nexus_plugin:arm_adapter",
        "acme_tool_driver = nexus_acme.nexus_plugin:tool_driver_adapter",
    ],
    "nexus.ik_adapters": [
        "acme_cartesian = nexus_acme.nexus_plugin:ik_adapter",
    ],
    "nexus.retargeter_adapters": [
        "acme_parallel_gripper = nexus_acme.nexus_plugin:gripper_adapter",
    ],
}
```

Each factory returns one of the NEXUS adapter spec dataclasses. A driver
adapter example:

```python
from nexus_core.adapter_registry import Adapter

def arm_adapter():
    return Adapter(
        name="acme_arm",
        kinds=frozenset({"arm"}),
        joint_names=lambda kind, side: [f"{side}_axis_{i}" for i in range(1, 8)],
        supports_home=True,
        launcher="nexus_acme.launchers:launch_driver",
        preflight="nexus_acme.launchers:check_devices",
        validate_config="nexus_acme.profile:validate_driver_config",
    )
```

The site profile selects those IDs and supplies only hardware-specific
settings. Adding or removing an arm is adding or removing a component row; a
tool can use a driver-defined kind such as `parallel_gripper`:

```json
{
  "components": [
    {"name":"left_arm", "kind":"arm", "driver":"acme_arm", "feedback":"measured", "unit":"rad", "ik":"acme_cartesian", "input_channel":"left", "teleop":{"arm_base_frame":"left_base", "vr_to_arm_rot":[1,0,0,0,1,0,0,0,1], "motion_scale":0.5}, "joints":["left_axis_1", "left_axis_2"], "lower":[-2.0, -2.0], "upper":[2.0, 2.0]},
    {"name":"left_tool", "kind":"parallel_gripper", "driver":"acme_tool_driver", "feedback":"measured", "unit":"m", "retargeter":"acme_parallel_gripper", "joints":["left_finger"], "lower":[0.0], "upper":[0.08]}
  ],
  "adapter_config": {
    "acme_arm":{"can":"can0"},
    "acme_cartesian":{"tcp_frame":"tool0"},
    "acme_tool_driver":{"serial":"/dev/ttyUSB2"},
    "acme_parallel_gripper":{"close_threshold":0.7}
  }
}
```

The full profile still includes `schema_version`, input channels, cameras,
dataset and policy sections. If the robot has a second arm, add another arm
component in the desired vector order and point it at its input channel.

`launcher` and optional validation hooks use `module:callable` strings. They
are loaded only when needed. This keeps profile inspection and data processing
independent of ROS launch. IDs must be unique across all adapter families.
`adapter_config.<plugin_id>` is passed to the plugin and checked by its
`validate_config(config)` hook when supplied. All launch hooks return ROS
launch actions and use this signature:

```python
launch(profile, selected_items, profile_path, dry_run) -> list[LaunchAction]
```

The robot-side launch and profile command require every referenced plugin to
be installed and validate its capabilities. GPU-side episode processing and
manifest checks validate the frozen profile layout without importing vendor
plugins, so the GPU host does not need the robot's ROS driver packages.

For drivers, `selected_items` is a component list. Component scoped adapters
receive one component; assembly scoped adapters receive all components using
that driver. IK and component retargeters receive one component. Assembly
retargeters receive all matching components. Input adapters receive channel
names, and camera adapters receive camera profile rows. Optional preflight
hooks use `(profile, selected_items)` and raise an error before launch when a
device or calibration is missing.

## ROS adapter contract

A driver plugin owns vendor SDKs, CAN, serial ports, and conversions. It
publishes measured state on
`/nexus/<instance>/components/<component>/joint_states`, subscribes only to
the final mux output at `.../joint_commands`, and exposes ready/enable/home/
estop services at `/nexus/<instance>/drivers/<component>/<operation>`. Its
callback checks exact profile joint order, dimensions, limits, finite values,
enable state, and command timeout. Drivers that manage a whole assembly may
declare `scope="assembly"` and `lifecycle_namespace` for their aggregate
services. The common driver manager validates feedback and fans lifecycle
calls out using that declaration.

An arm IK plugin subscribes to the configured input channel's
`PoseStamped` wrist target and measured joint state, then publishes only a
teleop candidate to
`/nexus/<instance>/candidates/teleop/<component>/joint_commands`. It must
provide `/<teleop_<component>>/reanchor` as a `Trigger` service; HITL calls it
before control returns to the human. An end-effector retargeter consumes the
standard hand/controller inputs and publishes the same candidate contract for
its component. It must not publish to the final driver command topic.

Input plugins publish their profile-configured source topic using the
contracted ROS type and stable frame IDs. Camera plugins can publish
`CompressedImage` to a configured `capture_topic`; the shared input bridge
relays it to the semantic camera topic. A camera plugin may instead publish
directly to `/nexus/<instance>/camera/<role>/image/compressed`.

Joint component `kind` values are extensible identifiers. For a new tool kind,
the driver declares that kind and the retargeter plugin declares support for
the same kind. The ordered component list remains the data/model vector order;
the collector, aligner, quality checker, exporters, ACT path, and policy
service consume it without robot-specific branches. A model is accepted only
when its frozen layout matches the profile.

## Adding or changing an assembly

1. Copy the closest profile. Keep component order explicit and set joint
   names, units, limits, input channels, camera roles, hardware settings,
   transforms, data action semantics, and policy camera slots.
2. Select already installed plugin IDs. A new single-arm or dual-arm assembly
   can use any number of components, including a custom end-effector `kind`.
3. For new hardware, put its vendor driver and NEXUS adapter hooks in a
   separate ROS package, register entry points there, and install it beside
   NEXUS. Add IK and retargeter plugins when the robot kinematics or tool
   mapping differ. NEXUS core does not need a new robot switch or core edit.
4. Run `ros2 run nexus_core nexus_profile /path/to/profile.json` and
   `ros2 launch nexus_core system.launch.py profile:=/path/to/profile.json
   dry_run:=true`. Resolve profile and fake-driver checks before enabling
   hardware.
5. Run the camera replay, data-layout, model-manifest, timeout, takeover, and
   site hardware acceptance checks in `NEXUS_ACCEPTANCE.md`.

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
