# astral_ws

Quest3 → **Astral 双臂** + **Wuji 双手** 的 ROS 2 工作空间。  
从 `xnero_ws-main` 迁出，**不含** Nero / XHand / pyAgxArm。

```text
Quest3 (quest3_hand_mocap, convert_to_robot:=true)
  ├─ mixed: 一侧手柄 + 一侧手（controller 默认同写 wrist_pose）
  ├─ IOBT: hips 世界系；head/wrist/controller/body_joints 在 hips 系
  ├─ quest3/{left,right}_wrist_pose
  │     → astral_quest_teleop → /{side}_arm/joint_commands
  │           → 仿真 astral_mujoco_sim    或  真机 astral_robot_control
  └─ hand_landmarks/{left,right}
        → wujihand_retargeting → /{side}_hand/joint_commands
              → 仿真 wujihand_mujoco_sim  或  真机 wujihand_control

Quest3 ← WebRTC（quest3_video_streamer，信令 :8765）
        PC 相机：D435i / USB 腕部 → 头显 3D 面板（第一视角）
```

臂与手两条链路独立；Quest 可同时喂两边。  
**仿真与真机不要抢同一条 `joint_commands`。**

## 包一览

### 共用

| 包 | 作用 |
|----|------|
| `quest3_hand_mocap` | Quest 腕/手柄/头/IOBT 身体（Wuji 用 `landmark_preprocess:=raw`） |
| `quest3_video_streamer` | PC 相机 WebRTC 推到 Quest（遥操第一视角）；与 mocap 独立 |
| `wuji_glove` | 手套 mocap（可选） |

### Astral 臂

| 包 | 作用 |
|----|------|
| `astral_quest_teleop` | IK（`analytic_dh` / `urdf_numerical`）+ 安全滤波 |
| `astral_robot_description` | 双臂 URDF（`astral_robot.pin.urdf`，SW 原约定） |
| `astral_robot_control` | 真机驱动（`astral_robot_sdk`） |
| `astral_mujoco_sim` | 双臂 MuJoCo（默认 `astral_dual.xml`） |
| `astral_arm_description` | 单臂 URDF（查看用） |
| `astral_arm_clean_description` | 可选干净 MDH URDF，**默认闭环不用** |

空目录 `astral_analytic_ik` / `astral_urdf_ik` 是旧 C++ IK 残留，无 `package.xml`，不参与编译。

### Wuji 手

| 包 | 作用 |
|----|------|
| `wujihand_retargeting` | landmark → 20 关节 |
| `wujihand_control` | 真机 launch 薄层 |
| `wujihandros2` | vendored 驱动：`wujihand_driver` / `wujihand_msgs` / `wujihand_bringup` |
| `wujihand_mujoco_sim` | 手仿真 + TuningViewer |

开源 retarget 库仍可放在本仓库旁：`../wuji-retargeting`。

## 构建

```bash
cd /home/robot/loopkok/sdk/astral_ws
export PATH=/usr/bin:$PATH
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  quest3_hand_mocap quest3_video_streamer \
  astral_robot_description astral_quest_teleop astral_robot_control astral_mujoco_sim \
  wuji_glove wujihand_retargeting wujihand_control \
  wujihand_driver wujihand_msgs wujihand_bringup \
  wujihand_mujoco_sim
source install/setup.bash
```

真机臂还需：`pip install -e ../astral_robot_sdk`  
真机手还需：`wujihandcpp` deb（见 `wujihand_control/README.md`）。  
URDF 数值 IK：`pip install pin`。

## 启动

```bash
# 臂仿真（求解器 / 协议在 yaml，默认 URDF IK + HTS 有线 TCP）
ros2 launch astral_mujoco_sim astral_sim_pipeline.launch.py

# 臂仿真改 DH：把 astral_teleop_{left,right}.yaml 的 solver_type 改成 analytic_dh
# 臂仿真改 UDP：把 quest3_mocap.yaml 的 protocol 改成 udp

# 臂真机
ros2 launch astral_quest_teleop astral_dual_arm_teleop.launch.py \
  with_driver:=true control_board_ip:=192.168.10.2

# 手仿真 / 调参
ros2 launch wujihand_mujoco_sim wujihand_sim_pipeline.launch.py \
  input_source:=quest3 retarget_backend:=wuji_retargeting
ros2 launch wujihand_mujoco_sim wujihand_tuning.launch.py \
  input_source:=quest3 retarget_backend:=wuji_retargeting

# 手真机
ros2 launch wujihand_control wujihand_real_pipeline.launch.py \
  input_source:=quest3 hand_side:=right retarget_backend:=wuji_retargeting
```

Quest 有线 HTS：`adb reverse tcp:8000 tcp:8000`。  
Quest 视频回传：`ros2 launch quest3_video_streamer multi_camera.launch.py`；有线可 `adb reverse tcp:8765 tcp:8765`。细节见 [`src/quest3_video_streamer/README.md`](src/quest3_video_streamer/README.md)。

## IK 约定（臂）

| `solver_type` | 位姿帧 | 发布的 q |
|---------------|--------|----------|
| `analytic_dh` | 干净 MDH 基座；yaml `vr_to_arm_rot=I` 再乘 \(R_\text{base}^\top\) | DH 约定求解后 `flip_q` 成 SW 约定 |
| `urdf_numerical`（yaml 默认） | SW `*_base_link`；`astral_robot.pin.urdf` | 已是硬件约定，不 flip |

仿真 MJCF **未改**（`astral_dual.xml`）。细节见 [`astral_quest_teleop/README.md`](src/astral_quest_teleop/README.md)。

## 文档

| 文件 | 内容 |
|------|------|
| [`src/astral_quest_teleop/README.md`](src/astral_quest_teleop/README.md) | DH / flip / `vr_to_arm_rot` |
| [`src/astral_mujoco_sim/README.md`](src/astral_mujoco_sim/README.md) | 臂仿真 |
| [`src/astral_robot_control/README.md`](src/astral_robot_control/README.md) | 臂真机驱动 |
| [`src/wujihand_control/README.md`](src/wujihand_control/README.md) | 手真机 |
| [`src/wujihand_retargeting/README.md`](src/wujihand_retargeting/README.md) | 重定向 |
| [`src/wujihand_mujoco_sim/README.md`](src/wujihand_mujoco_sim/README.md) | 手仿真 / tuning |
| [`src/quest3_video_streamer/README.md`](src/quest3_video_streamer/README.md) | Quest 相机 WebRTC 回传 |
| [`CHANGELOG.md`](CHANGELOG.md) | 版本记录 |
