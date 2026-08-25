# astral_mujoco_sim

Astral 双臂 MuJoCo 仿真：订阅 teleop 的 `joint_commands`，驱动 MJCF（真机驱动替身）。

```text
quest3 (convert_to_robot:=true)
  Mixed → robot_world   IOBT → robot_body   # mocap 保证 frame_id 稳定
  → astral_arm_teleop_{left,right}
      analytic_dh: R_baseᵀ + flip_q → 发布旧约定 q
      urdf_numerical: 无 flip，直接旧约定 q
  → /{side}_arm/joint_commands
  → astral_mujoco_sim_node (viewer)     # 旧 MJCF，不再 flip
  → /{side}_arm/joint_states (seed)

quest3 hand_landmarks/left
  → astral_gripper_teleop (with_gripper:=true)
  → /left_gripper/command | /left_gripper/joint_commands
  → astral_mujoco_sim_node              # 仅 echo 到 /left_gripper/joint_states
                                         # MJCF 暂无夹爪关节，viewer 里不动
```

**不要**与 `astral_robot_control` 同时跑同一 topic。

## 夹爪（sim 现状）

MJCF `astral_dual.xml` **没有夹爪关节**，所以 sim 节点对夹爪只做「订阅 + echo」：
- 订 `/left_gripper/command`（Float64 0=开 1=合）和 `/left_gripper/joint_commands`（JointState 弧度）
- 映射 0–1 → `left_gripper_open_rad`…`left_gripper_closed_rad`（默认 0.8…0.0：真机 0.8=张开、0.0=合拢，与 `astral_robot_control` 一致）
- 发 `/left_gripper/joint_states`，日志每秒打一次 `[Gripper][left] cmd rad=...`
- **不**驱动 MJCF qpos/ctrl（viewer 里夹爪不会动）

目的：在给 MJCF 加夹爪关节之前，先把 pinch→夹爪这条链在 sim 里跑通并可观。关夹爪 echo：`with_gripper:=false` 或 yaml `enable_gripper_cmd: false`。

## 模型（运行时未改）

默认加载 **`assets/mjcf/astral_dual.xml`**（由 `astral_robot_description` 的 SW URDF 生成，14 铰链 + STL，**原轴符号**）。

| 文件 | 用途 |
|------|------|
| `astral_dual.xml` | **默认仿真**。`init_pose_*` 与 teleop yaml 同为旧约定 |
| `astral_dual_clean.xml` | 干净 MDH / 翻转限位的可选 MJCF。架构 B 下 **不接入** 默认 launch |

`mujoco_sim_node.py` / `astral_mujoco_sim.yaml` 的 `init_pose_left/right` 保持：

```text
left:  [0.32,  0.11, -0.53, -0.80, 0.28, 0, 0]
right: [0.32, -0.11,  0.53, -0.80, 0.28, 0, 0]
```

DH teleop 已在发布前 `flip_q`，仿真按旧约定解释即可。  
若误用 `astral_dual_clean.xml` 接当前 teleop，肘/肩符号会对反。

## 依赖

```bash
pip install mujoco
```

## 构建

```bash
cd /home/robot/loopkok/sdk/astral_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select astral_mujoco_sim astral_arm_teleop --symlink-install
source install/setup.bash
```

## 启动

```bash
# 仅仿真（另开终端发 joint_commands 或 keyboard_vr_sim + teleop）
ros2 launch astral_mujoco_sim astral_mujoco_sim.launch.py

# Quest 遥操 → MuJoCo（solver / protocol 在 yaml，默认 urdf_numerical + tcp_wired）
ros2 launch astral_mujoco_sim astral_sim_pipeline.launch.py

# 一次性覆盖求解器或协议（不改 yaml）
ros2 launch astral_mujoco_sim astral_sim_pipeline.launch.py solver_type:=analytic_dh
```

键盘假 VR：

```bash
ros2 run astral_arm_teleop keyboard_vr_sim
```

IK / DH / flip 细节见 [`astral_arm_teleop/README.md`](../astral_arm_teleop/README.md)。
