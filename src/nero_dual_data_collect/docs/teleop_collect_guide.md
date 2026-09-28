# 遥操 + 数据采集完整流程

## 一、前置检查（每次必做）

```bash
# 1. 手部串口
ls /dev/ttyUSB* && sudo chmod 777 /dev/ttyUSB*

# 2. CAN 接口
bash find_all_can_port.sh && bash can_muti_activate.sh

# 3. 相机权限
for dev in /dev/bus/usb/*/*; do sudo chmod 777 $dev 2>/dev/null; done
```

---

## 二、启动命令

### 2.1 双臂双手 + 三相机（完整模式）

```bash
# 终端1：遥操（握拳启动）
ros2 launch nero_quest_teleop teleop_full_dual.launch.py

# 终端2：数据采集
ros2 launch nero_dual_data_collect data_collect.launch.py task_name:=my_task
```

### 2.2 仅左臂左手 + overhead + 左腕相机

```bash
# 终端1：遥操（直接启动，无需握拳）
ros2 launch nero_quest_teleop teleop_full_left.launch.py require_clench_to_start:=false

# 终端2：数据采集
ros2 launch nero_dual_data_collect data_collect.launch.py \
    task_name:=left_only collect_right:=false use_cam_2:=false
```

### 2.3 仅右臂右手

```bash
ros2 launch nero_quest_teleop teleop_full_right.launch.py require_clench_to_start:=false
ros2 launch nero_dual_data_collect data_collect.launch.py \
    task_name:=right_only collect_left:=false use_cam_1:=false
```

### 2.4 无手的纯臂遥操（不握拳，不采集手数据）

```bash
ros2 launch nero_quest_teleop teleop_dual_nero.launch.py
ros2 launch nero_quest_teleop teleop_single_nero.launch.py arm_side:=left
```

---

## 三、数据采集操作

### 3.1 启动流程

```
1. 数据采集终端打印热键帮助
2. 相机预热（几秒）
3. --- Episode 0 ready ---
4. Press [s] to start recording
```

如果使用握拳模式，按下 `[s]` 前需要**双手同时握拳保持 0.5 秒**触发遥操。未握拳时按 `[s]` 会提示 `Not armed yet`。

### 3.2 键盘控制

| 按键 | 作用 | 说明 |
|------|------|------|
| `s` | 开始录制 | 握拳模式下需先 ARMED |
| `p` | 暂停 / 恢复 | 相机停止，臂手继续跟手 |
| `q` | 保存 episode | 自动 disarm → 臂归 init_pose → 手 HOME → 等待重新握拳 |
| `d` | 丢弃重来 | 同上 + 删除数据文件 |
| `n` | 保存并开始下一个 | 等同于 [q] → 再准备下一个 |
| `Ctrl+C` | 退出 | 臂归零 → 手 HOME → 所有进程退出 |

### 3.3 典型采集会话

```
系统启动 → 相机预热
    │
  双手握拳 → ARMED
    │
  [s] 开始录 episode0 → 遥操作业 → [q] 保存
    │                                    │
    │                         臂→init_pose 手→HOME
    │                                    │
    │                           重新握拳 → ARMED
    │                                    │
  [s] 开始录 episode1 → 做错了 → [d] 丢弃
    │                                    │
    │                         臂→init_pose 手→HOME
    │                                    │
    │                           重新握拳 → ARMED
    │                                    │
  [s] 开始录 episode1 → 做完 → [n] 保存 → 自动准备 episode2
    │
  ... 继续 ...
    │
  Ctrl+C 退出
    臂归零 (J2→90°, J4→0, J1→0, all→0)
    手 HOME
```

---

## 四、产出数据

```
~/xnero_data/Data/my_task/
├── episode0/
│   ├── robot_data.h5      ← 臂关节 (20Hz) + 手关节 (20Hz)
│   ├── camera_data.h5     ← 相机 JPEG (30fps)
│   └── metrics.h5         ← 延时/精度
├── episode1/
│   └── ...
└── ...
```

**robot_data.h5 结构**：
```
/left_arm/joints      (N, 7)  float32
/right_arm/joints     (N, 7)  float32
/left_hand/joints     (N,12)  float32
/right_hand/joints    (N,12)  float32
/timestamps           (N,)    float64
```

**camera_data.h5 结构**：
```
/cam_0/images      (M_0,)  vlen uint8 JPEG
/cam_0/timestamps   (M_0,)  float64
/cam_1/images      (M_1,)  ...
/cam_2/images      (M_2,)  ...
```

---

## 五、后处理

```bash
# 1. 时间对齐（30Hz 相机基准，最近邻搜索）
python3 -m nero_dual_data_collect.align_data \
    --data_dir ~/xnero_data/Data/my_task
# 可选: --resize_width 448 --resize_height 256 --downsample 2

# 2. 转 LeRobot 格式
python3 -m nero_dual_data_collect.convert_to_lerobot \
    --data_dir ~/xnero_data/Data/my_task \
    --repo_id my_task_lerobot \
    --task_description "dual arm manipulation"

# 3. 回放验证（先停遥操释放 CAN）
python3 -m nero_dual_data_collect.replay_episode \
    --episode_dir ~/xnero_data/Data/my_task/episode0
```

---

## 六、所有可配置参数

### 6.1 遥操参数

| 参数 | 默认值 | 说明 | 配置文件 |
|------|--------|------|----------|
| `control_rate` | 80.0 | 控制频率 (Hz) | `nero_teleop_left.yaml` |
| `motion_scale` | 0.65 | VR→臂运动缩放 | `nero_teleop_left.yaml` |
| `pos_smoothing` | 0.5 | 位置 EMA 平滑 | `nero_teleop_left.yaml` |
| `rot_smoothing` | 0.8 | 姿态 SLERP 平滑 | `nero_teleop_left.yaml` |
| `max_joint_vel` | 0.05 | 单步速度限幅 (rad) | `nero_teleop_left.yaml` |
| `workspace_radius` | 0.58 | 工作空间球半径 (m) | `nero_teleop_left.yaml` |
| `data_timeout` | 5.0 | VR 超时 hold (s) | `nero_teleop_left.yaml` |
| `init_pose` | `[-0.41, 1.28, -0.96, 1.31, 2.68, -0.31, -0.16]` | 初始化位姿 | `nero_teleop_left.yaml` |
| `require_clench_to_start` | true | 握拳启动模式 | `nero_teleop_left.yaml` |
| `dry_run` | false | 测试模式（臂不实际动） | `nero_teleop_left.yaml` |
| `print_metrics` | false | 延时全量打印 | `nero_teleop_left.yaml` |
| `print_ik_stats` | false | IK 统计打印 | `nero_teleop_left.yaml` |

### 6.2 手部参数

| 参数 | 默认值 | 说明 | 配置文件 |
|------|--------|------|----------|
| `smoothing_alpha` | 0.7 | 关节平滑系数 | `retargeting_params.yaml` |
| `enable_thumb_fix` | false | 拇指矫正 | `retargeting_params.yaml` |
| `clench_angle_threshold` | 40.0 | 握拳检测角度 (°) | `retargeting_params.yaml` |
| `clench_debounce_frames` | 30 | 握拳去抖帧数 | `retargeting_params.yaml` |
| `require_clench_to_start` | true | 握拳启动模式 | 命令行覆盖 |
| `print_metrics` | false | 延时打印 | `retargeting_params.yaml` |

### 6.3 数据采集参数

| 参数 | 默认值 | 说明 | 传递方式 |
|------|--------|------|----------|
| `task_name` | default_task | 任务名（子目录） | launch arg |
| `data_base_path` | ~/xnero_data | 数据根目录 | launch arg |
| `collect_left` | true | 采集左手左臂 | launch arg |
| `collect_right` | true | 采集右手右臂 | launch arg |
| `use_cam_0` | true | 采 overhead 335 | launch arg |
| `use_cam_1` | true | 采左腕 305 | launch arg |
| `use_cam_2` | true | 采右腕 305 | launch arg |

相机分辨率和帧率在 `data_collect.yaml` 的 `camera_configs` 中修改，改后直接生效无需编译。

---

## 七、常见问题

| 问题 | 解决 |
|------|------|
| 手不动 | `ls /dev/ttyUSB*` 检查串口，`sudo chmod 777 /dev/ttyUSB*` |
| 臂不动 | `bash find_all_can_port.sh && bash can_muti_activate.sh` |
| 相机打不开 | `for dev in /dev/bus/usb/*/*; do sudo chmod 777 $dev; done` |
| 握拳不触发 | 降低 `clench_angle_threshold` 到 25-30° |
| 很卡顿 | 提 `control_rate`，检查 WiFi 信号 |
| 归零被 SIGKILL | 提 `init_speed_percent` 或降归零步骤超时 |
| CRC 错误 | 换短 USB 线或加磁环 |
| 单臂模式握拳卡住 | 加 `require_clench_to_start:=false` |
