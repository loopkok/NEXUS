# nero_dual_data_collect

Nero 双臂 + XHand 双手遥操作数据采集系统。支持在 Quest3 VR 遥操作过程中实时采集多模态数据（机械臂关节角 + 灵巧手关节角 + 三路相机），经时间对齐后转换为 LeRobot 标准格式，用于 VLA 模型（如 pi0）训练，以及将录制数据回放到真实机器人上。

## 系统架构

```
Quest3 VR 遥操 (已有)
  ├── quest3_udp_mocap ──▶ 手部 landmarks
  ├── xhand_retargeting ──▶ /left_hand/xhand_command, /right_hand/xhand_command
  └── nero_teleop_node ──▶ /nero_teleop_{side}/joint_states (新增反馈)

                    ↓ ROS2 topics

数据采集 (本包)
  ├── data_collect_node ── 订阅臂/手反馈, 键盘控制, 写入 HDF5
  ├── camera_manager    ── 独立进程, 3路 Gemini 相机采集
  ├── data_writer       ── 分块 HDF5 写入
  └── keyboard_controller ── 终端热键控制

                    ↓ 后处理

  align_data.py         ── 时间对齐 → (state, action) 对
  convert_to_lerobot.py ── LeRobotDataset 格式
  replay_episode.py     ── CAN 直驱机械臂回放
```

## 目录结构

```
src/nero_dual_data_collect/
├── package.xml
├── setup.py / setup.cfg
├── config/
│   └── data_collect.yaml              # 采集配置（路径、相机参数）
├── launch/
│   └── data_collect.launch.py         # ROS2 launch 文件
├── nero_dual_data_collect/
│   ├── data_collect_node.py           # 主 ROS2 节点（订阅、状态机、调度）
│   ├── data_writer.py                 # HDF5 写入器（robot_data.h5）
│   ├── camera_manager.py              # 多相机进程（Gemini 335/305）
│   ├── keyboard_controller.py         # 终端热键输入
│   ├── align_data.py                  # 后处理：时间对齐
│   ├── convert_to_lerobot.py          # 后处理：转 LeRobot 格式
│   └── replay_episode.py              # CAN 直驱回放
└── resource/
    └── nero_dual_data_collect
```

## 依赖

| 依赖 | 用途 |
|------|------|
| ROS2 (rclpy, sensor_msgs, geometry_msgs) | 订阅臂/手反馈话题 |
| xhand_control_interfaces | 订阅 XHandStateArray |
| pyAgxArm | replay 中 CAN 直驱机械臂 |
| pyorbbecsdk | Gemini 335/305 相机采集 |
| h5py | HDF5 数据存储 |
| numpy, opencv-python, scipy | 数据处理 |
| lerobot (可选) | convert_to_lerobot.py |

## 安装

```bash
cd ~/loopkok/xnero_ws
colcon build --packages-select nero_dual_data_collect --symlink-install
source install/setup.bash
```

## 使用方法

### 第一步：启动遥操作 + 数据采集

需要两个终端：

**终端 1** — 启动遥操作（已有系统）：
```bash
source install/setup.bash
ros2 launch nero_quest_teleop teleop_full_dual.launch.py
```

**启动前检查**（每次启动必须执行）：

```bash
# 1. 检查手部串口
ls /dev/ttyUSB*
sudo chmod 777 /dev/ttyUSB*

# 2. 检查并激活 CAN 接口
bash find_all_can_port.sh
bash can_muti_activate.sh
```

**终端 2** — 启动数据采集节点：
```bash
source install/setup.bash
ros2 launch nero_dual_data_collect data_collect.launch.py task_name:=pick_place
```

> **注意**：数据采集节点依赖 `nero_teleop_node` 新发布的 `~/joint_states` 话题。请确保已重新编译 `nero_quest_teleop` 包。

### 第二步：启动键盘控制器（T3）

T3 终端启动键盘控制器（不需要 source ROS2）：

```bash
python3 src/nero_dual_data_collect/scripts/collect_control.py
```

### 第三步：遥操作采集

T2 启动后显示热键帮助和 TCP 控制地址。T3 按键控制：

| 键 | 命令 | 作用 |
|---|------|------|
| `s` | start | 开始录制 |
| `q` | stop | 停止并保存（臂回 init_pose，手回 HOME，重置握拳） |
| `d` | discard | 丢弃当前 episode（臂回 init_pose，重置握拳） |
| `n` | next | 保存当前并开始下一个 episode |
| `p` | pause | 暂停/恢复 |
| `ESC` | — | 退出控制器 |

采集流程：
```
握拳 → ARMED → 按 s → 遥操作业 → 按 q（保存，臂归位）
     → 重新握拳 → ARMED → 按 s → 下一个 episode ...

--- Episode 0 ready ---
  Path: /home/xxx/xnero_data/Data/pick_place/episode0
  Press [s] to start recording
```

**操作流程**：
1. 戴上 Quest3，确认遥操作正常
2. 在数据采集终端按 `s` → 开始录制
3. 执行遥操作任务
4. 按 `p` → 暂停（可恢复），按 `q` → 保存当前 episode
5. 如果操作失误，按 `d` → 丢弃当前 episode，重新开始
6. 按 `n` → 保存当前并自动创建下一个 episode
7. 结束时按 `Ctrl+C`

### 第三步：数据对齐（后处理）

```bash
# 对齐所有 episode
python3 -m nero_dual_data_collect.align_data \
    --data_dir ~/xnero_data/Data/pick_place

# 从指定 episode 开始
python3 -m nero_dual_data_collect.align_data \
    --data_dir ~/xnero_data/Data/pick_place \
    --start_episode 5

# 自定义输出分辨率
python3 -m nero_dual_data_collect.align_data \
    --data_dir ~/xnero_data/Data/pick_place \
    --resize_width 448 --resize_height 256

# 降采样（隔帧保留，减少数据量）
python3 -m nero_dual_data_collect.align_data \
    --data_dir ~/xnero_data/Data/pick_place \
    --downsample 2
```

### 第四步：转换为 LeRobot 格式

```bash
python3 -m nero_dual_data_collect.convert_to_lerobot \
    --data_dir ~/xnero_data/Data/pick_place \
    --repo_id pick_place_lerobot \
    --task_description "dual arm pick and place"

# 指定输出目录
python3 -m nero_dual_data_collect.convert_to_lerobot \
    --data_dir ~/xnero_data/Data/pick_place \
    --output_dir ~/datasets \
    --repo_id pick_place_v1
```

### 第五步：回放验证（可选）

```bash
# 回放原始数据
python3 -m nero_dual_data_collect.replay_episode \
    --episode_dir ~/xnero_data/Data/pick_place/episode0

# 回放对齐后的数据
python3 -m nero_dual_data_collect.replay_episode \
    --episode_dir ~/xnero_data/Data/pick_place/episode0 --aligned

# 半速回放，循环 3 遍
python3 -m nero_dual_data_collect.replay_episode \
    --episode_dir ~/xnero_data/Data/pick_place/episode0 \
    --speed 0.5 --loop_count 3
```

回放流程：
1. 通过 CAN 总线连接左右臂，5% 速度慢速移动到第一帧位姿
2. 等待用户按 Enter 确认
3. 逐帧发送关节角到机械臂 + 灵巧手
4. 支持 Ctrl+C 急停

## 数据格式

### 原始采集数据（robot_data.h5）

```
/left_arm/joints       (N, 7)   float32   左臂 7 个关节角 (rad)
/right_arm/joints      (N, 7)   float32   右臂 7 个关节角 (rad)
/left_hand/joints      (N,12)   float32   左手 12 个手指关节 (rad)
/right_hand/joints     (N,12)   float32   右手 12 个手指关节 (rad)
/timestamps            (N,)     float64   采集时刻 (time.time())
```

N ≈ 50Hz × 录制时长。即使手部反馈只有 10Hz，也以 50Hz 写入（相邻帧复用上一次的值）。

### 原始采集数据（camera_data.h5）

```
/cam_0/images      (M_0,)  vlen uint8   固定 overhead Gemini 335, JPEG 字节
/cam_0/timestamps   (M_0,)  float64
/cam_1/images      (M_1,)  vlen uint8   左腕 Gemini 305, JPEG 字节
/cam_1/timestamps   (M_1,)  float64
/cam_2/images      (M_2,)  vlen uint8   右腕 Gemini 305, JPEG 字节
/cam_2/timestamps   (M_2,)  float64
```

相机独立于机械臂采集，速率不同（通常 30Hz vs 50Hz）。

### 对齐后数据（aligned_data.h5）

```
/observation/left_arm_joints      (F, 7)   float32   state(t)
/observation/right_arm_joints     (F, 7)   float32
/observation/left_hand_joints     (F,12)   float32
/observation/right_hand_joints    (F,12)   float32
/action/left_arm_joints           (F, 7)   float32   state(t+1)
/action/right_arm_joints          (F, 7)   float32
/action/left_hand_joints          (F,12)   float32
/action/right_hand_joints         (F,12)   float32
/timestamps                       (F,)     float64
+ frames/cam0/{000000..}.jpg              对齐后的图像帧
+ frames/cam1/{000000..}.jpg
+ frames/cam2/{000000..}.jpg
```

F = 相机帧数（对齐到 cam_0 时间戳），action = 下一个时间步的 state（Behavior Cloning 标准格式）。末尾追加 20 帧静止 hold。

### LeRobot 数据集

```
{repo_id}/
├── data/chunk-000/episode_*.parquet
├── images/observation.images.cam_0/...
├── images/observation.images.cam_1/...
├── images/observation.images.cam_2/...
└── meta/info.json
```

每个 frame：
```python
{
    "observation.state":              float32[38],  # 7+7+12+12
    "action":                         float32[38],
    "observation.images.cam_0":       uint8[H,W,3],
    "observation.images.cam_1":       uint8[H,W,3],
    "observation.images.cam_2":       uint8[H,W,3],
    "task":                           str,
}
```

## 时间对齐策略

```
采集阶段（实时）：
  ┌─────────┐    ┌─────────┐    ┌───────────────┐
  │ 臂 50Hz │    │ 手 10Hz │    │ 相机 30Hz × 3  │
  │ (CAN)   │    │ (USB)   │    │ (USB)         │
  └────┬────┘    └────┬────┘    └───────┬───────┘
       │              │                │
       └──────────────┴────────────────┘
                      │
               time.time() 时间戳

对齐阶段（后处理）：
  cam_0 时间戳 ──基准──▶ target_timestamps (30Hz)
                            │
       robot 时间戳 ──最近邻──▶ state(t)
       robot 时间戳 ──最近邻──▶ state(t+1) = action(t)
       cam_1,2 ──最近邻─────▶ 对应图像帧
```

## 配置

`config/data_collect.yaml`:

```yaml
data_collect_node:
  ros__parameters:
    data_base_path: "~/xnero_data"    # 数据根目录
    task_name: "default_task"         # 任务名称（子目录）
    camera_configs:
      - id: "cam_0"                   # 固定 overhead Gemini 335
        width: 1280
        height: 720
        fps: 30
      - id: "cam_1"                   # 左腕 Gemini 305
        width: 640
        height: 480
        fps: 30
      - id: "cam_2"                   # 右腕 Gemini 305
        width: 640
        height: 480
        fps: 30
```

可通过命令行覆盖：
```bash
ros2 launch nero_dual_data_collect data_collect.launch.py \
    task_name:=my_task \
    data_base_path:=/mnt/data/xnero_data
```

## 注意事项

1. **不依赖 agx_arm_ros**：机械臂通过 `pyAgxArm` + CAN 总线直接控制，不使用 `agx_arm_ctrl` 节点。臂反馈由 `nero_teleop_node` 新增的 publisher 提供。

2. **手部缺失不阻塞**：如果只启动单只手（如 launch 文件中右手被注释），对应数据源写入 zeros 并 warning 一次，不影响采集流程。

3. **相机热启动**：相机进程启动后有预热阶段（AE/AWB 稳定），按 `s` 才开始真正写入。

4. **暂停恢复**：暂停后 resume 会创建新的相机进程（需要重新预热约 1 秒），机械臂数据直接追加到同一 HDF5 文件。

5. **CAN 总线冲突**：回放时需要 CAN 总线空闲。如果遥操作节点正在运行（占用了 CAN），需先停止遥操作再回放。

6. **帧率不一致**：臂 50Hz、手 10Hz、相机 30Hz。采集时以 50Hz 统一写入 robot_data.h5；对齐时以 camera_0（30Hz）为基准。
