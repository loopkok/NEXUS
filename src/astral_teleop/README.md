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

## 手柄集成

`full_teleop.launch.py` 默认起 `controller_start_gate` 节点：

- **左手柄 grip 键（中指，mask bit 5）→ `/teleop/start`**：按下沿（rising edge）发一次性启动信号，等价于 web「开始遥操」或 `ros2 topic pub --once /teleop/start`。配合 `require_start_signal:=true`：手摆好初始位姿后按左 grip 即开始遥操；再按一次 = 重新记零点（re-center）。

`pinch_gripper_node` 同时订阅 `quest3/{side}_controller_joy`：

- **左右手柄 trigger 模拟量 → 左右夹爪开合**：`axes[0]`（0=开 … 1=合）驱动夹爪，与手部捏合并存（手柄 Joy 新鲜时优先 trigger，否则回退 pinch）。握手柄控臂时用 trigger 控夹爪，裸手时用捏合控夹爪。

手柄位姿本身经 `controller_as_wrist` 镜像到 `wrist_pose` 驱动臂 IK（既有）。
