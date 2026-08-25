# astral_gripper_teleop

Quest3 左手食指–拇指距离 → 夹爪开合。独立于 `quest3_hand_mocap` 和臂 IK。

```text
hand_landmarks/left
        → pinch_gripper_node
              ├── /left_gripper/command          Float64  0=开  1=合
              └── /left_gripper/joint_commands   JointState 弧度
                    → astral_robot_control → set_gripper_angle（左，CMD 0x97）
```

以后换硬件：新节点发同一个 `/left_gripper/command`（或直接发 `joint_commands` 弧度），关掉本节点即可。左臂仍走 `quest3/left_wrist_pose`。

```bash
ros2 launch astral_gripper_teleop gripper_teleop.launch.py
# 已包含在 astral_teleop/full_teleop.launch.py（with_gripper:=true）
```

调参：`config/gripper_teleop.yaml` 里 `open_dist_m` / `close_dist_m`（米）和 `open_rad` / `closed_rad`（夹爪电机）。真机方向 `open_rad=2.0`（全张开，CLI 递增 rad 实测）、`closed_rad=0.0`（合拢），故捏合→合、张开→开。

**自适应量程**（`auto_range:=true`，默认开）：跟踪你实际捏合距离的 min/max（带 `auto_range_forget_s` 遗忘，双向适应），把当前距离映射到这个真实范围，用满夹爪行程——避免 `open_dist_m/close_dist_m` 与你实际手部范围不符时夹爪只动一小段。`open_dist_m/close_dist_m` 退化为先验/回退；关掉则用固定 open/close 映射。日志会打印 `dist=Xmm range=[lo,hi]mm close=ratio`，可直接观察你的真实范围。
