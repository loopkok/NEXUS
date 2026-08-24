# astral_gripper_teleop

Quest3 左手食指–拇指距离 → 夹爪开合。独立于 `quest3_hand_mocap` 和臂 IK。

```text
hand_landmarks/left
        → pinch_gripper_node
              ├── /left_gripper/command          Float64  0=开  1=合
              └── /left_gripper/joint_commands   JointState 弧度
                    → astral_robot_control → set_gripper_angle（左，0x31）
```

以后换硬件：新节点发同一个 `/left_gripper/command`（或直接发 `joint_commands` 弧度），关掉本节点即可。左臂仍走 `quest3/left_wrist_pose`。

```bash
ros2 launch astral_gripper_teleop gripper_teleop.launch.py
# 已包含在 astral_teleop/full_teleop.launch.py（with_gripper:=true）
```

调参：`config/gripper_teleop.yaml` 里 `open_dist_m` / `close_dist_m`（米）和 `open_rad` / `closed_rad`（夹爪电机）。
