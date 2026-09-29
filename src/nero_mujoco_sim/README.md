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

The default profile opens a MuJoCo viewer and adds a table and free pick cube.
Set `adapter_config.nero_mujoco.enable_viewer` to `false` in a site profile for
headless physics tests. The simulator does not publish camera images or tactile
data. Nero J2's MJCF limit is taken from the NEXUS profile because the URDF
mount frame contains a +90 degree zero offset while the driver contract uses
motor angles.
