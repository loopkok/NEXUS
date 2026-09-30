# nero_mujoco_sim

NEXUS MuJoCo driver plugin for the existing Nero dual-arm and XHand dual-hand
URDF. The description remains in `xhand_nero_description`; the simulator
converts that installed URDF to MJCF at startup, including the meshes, inertias,
joint frames and contact geometry. Interactive simulation defaults to
`simulation_mode=kinematic`: accepted final joint targets directly update MuJoCo
`qpos`, velocities are cleared, and `mj_forward` updates link geometry. Feedback
is the resulting simulated joint position. This matches Astral's direct joint
visualization and removes physical position-servo settling delay.

`simulation_mode=physics` remains available for rigid-body and contact tests.
It integrates position actuators (`kp=100`, `kv=10` for arms) using `mj_step`.
Kinematic mode does not simulate gravity, actuator response, or contact forces;
objects will not be dynamically manipulated by directly positioned hands.

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
headless tests. Select `adapter_config.nero_mujoco.simulation_mode` as
`kinematic` (default) or `physics` in a site profile. The standalone launch also
accepts `simulation_mode:=physics`. The simulator does not publish camera images or tactile
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

The phase estimate includes ROS command delivery and feedback sampling; it is
not Quest-to-screen latency. In default kinematic mode there is no physical
servo response. Nero simulation also defaults to zero wrist EMA smoothing;
`pos_smoothing` / `rot_smoothing` in a component's teleop config can override
this. Physical Nero retains the original 0.8 EMA defaults. Both Nero and
Astral arm teleop now default to 100 Hz (10 ms); at this rate the 0.8 EMA
low-frequency delay is about 40 ms. This is a configured control frequency,
not a guarantee of 100 distinct Quest samples or completed IK solves per second.

An unreachable IK target now publishes a hold of fresh measured joints while
fresh wrist input continues. Returning to a reachable pose resumes following
without rearming. Input/feedback loss still disarms and triggers the mux timeout.
The original IK core and Quest coordinate transformations remain unchanged.
Both physical and simulated Nero use a shared interactive adapter. It derives
warm-start arm angle from the seed elbow, rejects targets outside the actual
shoulder/wrist triangle, validates full-pose residuals and profile joint limits,
and retains global fallback on a separate latest-request worker. The former
base-origin 0.58 m sphere is disabled unless a site explicitly configures
`workspace_radius`. Slow fallback does not block input/feedback callbacks;
obsolete results and results from before reanchor cannot issue new commands.
Failure warnings include `reason`, wrist distance, target position, method,
and solve time. `limits_or_search` means no candidate was found within the
limits/search; it is not proof that a pose is mathematically unreachable.
See the [IK and physical-profile audit](../../docs/test_logs/2026-09-30-nero-ik-audit/README.md).
See the [direct joint regression report](../../docs/test_logs/2026-09-30-nero-kinematic-teleop/README.md)
and the earlier [physical-mode viewer report](../../docs/test_logs/2026-09-30-nero-viewer-latency/README.md).
