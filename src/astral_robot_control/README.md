# astral_robot_control

ROS2 驱动包：把 [`astral_robot_sdk`](../../../astral_robot_sdk) 包成 Wuji/XHand 风格的
`joint_commands` / `joint_states` 话题，供后续 `astral_arm_teleop` 使用。

**本包不做 IK / Quest**；只负责连接控制板、收指令、发反馈。

## 话题契约

18 轴顺序与 SDK `ROBOT_JOINT_NAMES` 一致：

```text
[ left_arm×7 | right_arm×7 | waist×2 | head×2 ]
```

真机：`0x21/0x22`=腰，`0x31/0x32`=头。机械夹爪走 `0x97`/`0x98`（`set_gripper_angle`），不是 0x31/0x32。

| 话题 | 方向 | 内容 |
|------|------|------|
| `/left_arm/joint_commands` | sub | `JointState.position[7]` → `move_arm_js` |
| `/right_arm/joint_commands` | sub | `JointState.position[7]` → `move_arm_js` |
| `/astral/joint_commands` | sub | `JointState.position[18]` → `move_js`（有新数据时优先） |
| `/head/joint_commands` | sub | `[head_yaw, head_pitch]` → `move_head_js` |
| `/left_gripper/command` | sub | `Float64` 0=开 1=合 → `set_gripper_angle`（0x97）。**遥操唯一发布流**，rad 经本节点 `left/right_gripper_open_rad` 映射 |
| `/left_gripper/joint_commands` | sub | `JointState` 弧度 → `set_gripper_angle`（保留给直接发弧度的适配器；**勿与 /command 同时发**——两流覆盖同一目标会抽搐） |
| `/left_arm/joint_states` | pub | 左臂 7 |
| `/right_arm/joint_states` | pub | 右臂 7 |
| `/astral/joint_states` | pub | 全身 18（含腰+头） |
| `/head/joint_states` | pub | 头 2 |
| `/left_gripper/joint_states` | pub | 左夹爪（最近指令） |

- QoS：**BEST_EFFORT**（SensorData）
- 关节名可选；无名时按位置顺序；有名时按 `joint_layout.py` 对齐
- `command_timeout_s`（默认 0.5）：超时不再下发，避免僵持旧指令

左臂关节名：`left_shoulder_pitch` … `left_wrist_roll`  
右臂：`right_shoulder_pitch` … `right_wrist_roll`

## 服务

| 服务 | 说明 |
|------|------|
| `/astral_robot_driver/ready` | `one_click_ready`（WORK→POSITION→enable→zero） |
| `/astral_robot_driver/home` | `set_all_joints_zero`（全关节归零） |
| `/astral_robot_driver/estop` | **真急停**：`e_stop` / disable（断电，臂失去保持力） |
| `/astral_robot_driver/damping` | 阻尼释放：`motion_mode=0`（电机仍上电、关节可手动拖拽） |
| `/astral_robot_driver/position` | 位置保持：`motion_mode=1`（恢复位置保持） |

> 典型遥操收尾流程：遥操中 → 停止（臂保持末位姿）→ `damping`（手动拖回 home）→ `position` 或 `home`。
> 真急停 `estop` 会断电，恢复需重新 `ready`。

## 依赖

```bash
# SDK（含 native）
cd /home/robot/loopkok/sdk/astral_robot_sdk
pip install -e .

cd /home/robot/loopkok/sdk/astral_ws
export PATH=/usr/bin:$PATH
source /opt/ros/humble/setup.bash
colcon build --packages-select astral_robot_control --symlink-install
source install/setup.bash
```

## 启动

```bash
# 无硬件冒烟（只打日志）
ros2 launch astral_robot_control astral_drivers.launch.py dry_run:=true

# 真机
ros2 launch astral_robot_control astral_drivers.launch.py \
  control_board_ip:=192.168.10.2 local_port:=8081
```

手动发指令：

```bash
ros2 topic pub --once /left_arm/joint_commands sensor_msgs/msg/JointState \
  "{name: [left_shoulder_pitch,left_shoulder_roll,left_elbow_roll,left_elbow_pitch,left_forearm_roll,left_wrist_pitch,left_wrist_roll],
    position: [0.1,0,0,-0.1,0,0,0]}"
```

看反馈：

```bash
ros2 topic hz /astral/joint_states
ros2 topic echo /left_arm/joint_states --once
```

## 参数

见 `config/astral_robot.yaml`。常用：`control_board_ip`、`local_port`、`dry_run`、`auto_ready`、`control_rate` / `state_publish_rate`。

## 与遥操

全链路见 [`astral_arm_teleop`](../astral_arm_teleop/README.md)：

```bash
ros2 launch astral_arm_teleop astral_dual_arm_teleop.launch.py \
  with_driver:=true control_board_ip:=192.168.10.2
```

```text
quest3 / IK  →  /{left,right}_arm/joint_commands
pinch_gripper →  /left_gripper/command
                     ↓
              astral_robot_driver  →  astral_robot_sdk  →  板
                     ↓
              /{left,right,astral,left_gripper}/joint_states
```
