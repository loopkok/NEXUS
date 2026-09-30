# nero_mujoco_sim

NEXUS MuJoCo driver plugin for the existing Nero dual-arm and XHand dual-hand
URDF. The description remains in `xhand_nero_description`; the simulator
converts that installed URDF to MJCF at startup, including the meshes, inertias,
joint frames and contact geometry. The MuJoCo model runs position actuators with
rigid-body dynamics and publishes measured `JointState` feedback.
Arm servos use `kp=100`, `kv=10` at the default 2 ms step; this keeps mirrored
left/right wrist tracking stable under the URDF's small wrist inertias.

The driver subscribes to the selected profile's final
`/nexus/<instance>/components/<component>/joint_commands` topics and publishes
matching `joint_states`. It exposes NEXUS `ready`, `enable`, `home` and `estop`
services per component. The XHand candidate bridge remains in the normal
retargeting chain; only the final hardware driver is replaced.

Build and launch the integrated NEXUS teleop profile from the workspace root:

```bash
python3 -m pip install 'mujoco>=3.2'
colcon build --symlink-install
source install/setup.bash
ros2 launch nexus_core system.launch.py profile:=nero_dual_xhand_mujoco dry_run:=false with_cameras:=false with_recording:=false with_policy:=false
```

The default profile runs headless and adds a table and free pick cube. In the
Web NEXUS tab, enable the host-desktop MuJoCo window before starting the assembly.
Set `adapter_config.nero_mujoco.enable_viewer` to `false` in a site profile for
headless physics tests. The simulator does not publish camera images or tactile
data. Nero J2's MJCF limit is taken from the NEXUS profile because the URDF
mount frame contains a +90 degree zero offset while the driver contract uses
motor angles.

ROS callbacks run in a dedicated executor thread and share a lock with physics
and lifecycle operations. Final commands use best-effort depth-1 QoS so the
driver reads the latest target. Expired stamped commands do not refresh the
watchdog. Physics keeps the 2 ms timestep; viewer synchronization is limited by
the separate `viewer_rate` parameter (30 Hz by default). The log reports physics
rate, real-time factor, actual viewer sync rate, and maximum sync time.

To measure the driver independently of IK and real devices, run the safe probe
from a host desktop terminal. It rejects physical-driver profiles, publishes
100 Hz joint targets to all four simulated components, and estimates servo
phase lag using a small 0.8 Hz sine motion:

```bash
ROS_DOMAIN_ID=119 ROS_LOCALHOST_ONLY=1 python3 scripts/nexus_nero_sim_runtime_probe.py \
  --profile src/nexus_core/profiles/nero_dual_xhand_mujoco.json --viewer --duration 12
```

This phase lag includes the physical position actuator response and is not
Quest-to-screen latency. The legacy Astral viewer directly assigns joint
positions, whereas this simulator integrates position actuators and contact
dynamics. See the [viewer regression report](../../docs/test_logs/2026-09-30-nero-viewer-latency/README.md).
