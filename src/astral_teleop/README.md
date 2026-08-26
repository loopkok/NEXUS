# astral_teleop

整机遥操 **只拼现有包**，这里没有 IK / 夹爪 / retarget 源码。

```text
Quest3 mocap（仅一份）
  wrist ×2     → astral_arm_teleop → 双臂
  landmarks/L  → astral_gripper_teleop → 左夹爪
  landmarks/R  → wujihand_retargeting → 右 Wuji   （right_hand_source:=quest3）
手套            → 同一 hand_landmarks/right         （right_hand_source:=glove）
  landmarks/R  → astral_gripper_teleop → 右夹爪     （right_hand_source:=gripper）
```

`right_hand_source` 四选一：
- `quest3` — 右灵巧手，来自 Quest landmarks（默认）
- `glove` — 右灵巧手，来自 Wuji Glove（关掉 Quest 的 `hand_landmarks/right`，右腕 `wrist_pose` 仍给右臂 IK）
- `gripper` — 右捏合 → 右夹爪（不起 Wuji，右手用裸手做 wrist+pinch）
- `none` — 无右手设备

```bash
# 真机：Quest 双臂 + 左捏合夹爪 + Quest 右手 Wuji
ros2 launch astral_teleop full_teleop.launch.py \
  with_arm_driver:=true with_hand_driver:=true \
  right_hand_source:=quest3

# 右手改手套（关 Studio，勿与 Quest 右手 landmark 同时发）
ros2 launch astral_teleop full_teleop.launch.py \
  with_arm_driver:=true with_hand_driver:=true \
  right_hand_source:=glove

# 双夹爪：左/右捏合 → 左/右夹爪（无灵巧手）
ros2 launch astral_teleop full_teleop.launch.py \
  with_arm_driver:=true with_hand_driver:=false \
  right_hand_source:=gripper with_gripper:=true
```

不要再单独起 `astral_dual_arm_teleop` / `wujihand_real_pipeline`（会抢 mocap 或 `joint_commands`）。
