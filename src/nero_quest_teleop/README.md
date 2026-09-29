# Nero Quest Teleop

基于 Meta Quest 3 手部追踪的松灵 Nero 7-DOF 机械臂遥操系统。

使用**解析 DH 几何逆解算**（臂角参数化 + 连续性分支选择 + 1D QP 精修），`move_js` MIT 透传模式低延时控制，包含多层安全过滤。

---

## 目录

- [系统架构](#系统架构)
- [环境依赖](#环境依赖)
- [编译与安装](#编译与安装)
- [快速启动](#快速启动)
- [键盘模拟器使用](#键盘模拟器使用)
- [坐标系说明](#坐标系说明)
- [配置参数说明](#配置参数说明)
- [测试说明](#测试说明)
- [安全机制说明](#安全机制说明)
- [文件结构](#文件结构)
- [常见问题排查](#常见问题排查)

---

## 系统架构

```
Quest3 (Unity App)
    │ UDP (port 9000)
    ▼
┌─────────────────────┐
│  quest3_udp_mocap   │  复用已有包 — 发布腕部 PoseStamped
└─────────┬───────────┘
          │ quest3/{left|right}_wrist_pose
          ▼
┌──────────────────────────────────────────────┐
│           nero_teleop_node (核心)             │
│                                              │
│  1. VR 位姿预处理 (坐标映射 + EMA/SLERP 滤波)  │
│  2. 夹爪目标 → TCP offset → 法兰目标           │
│  3. IK 逆解 (解析 DH 臂角法, ~8ms, 100%成功率) │
│  4. 安全过滤 (关节限位 / 速度 / XYZ工作空间)     │
│  5. move_js 下发 (MIT 透传，无轨迹规划)         │
└──────────────────────────────────────────────┘
    │ CAN bus (pyAgxArm)
    ▼
  Nero 机械臂
```

**数据流关键路径：** VR 腕部位姿 → 坐标系映射 → 增量计算 + 平滑滤波 → 工作空间边界检查 → 夹爪 4x4 目标位姿 → TCP offset 转换为法兰目标 → IK 求解 → 关节速度/限位过滤 → `move_js` 下发

---

## 环境依赖

### 硬件

- Meta Quest 3 头显（安装手部追踪 Unity 应用）
- 松灵 Nero 7-DOF 机械臂
- CAN 适配器（连接 Nero 机械臂控制端）
- 运行 ROS 2 的主控电脑（与 Quest 3 在同一局域网）

### 软件

| 依赖 | 版本要求 | 说明 |
|------|---------|------|
| ROS 2 | Humble / Jazzy | |
| Python | >= 3.8 | |
| pyAgxArm | >= 最新版 | Nero 机械臂 SDK |
| scipy | >= 1.7 | 数值优化（1D QP 精修） |
| numpy | >= 1.20 | 矩阵运算 |

> **不再依赖 Pinocchio 和 CasADI。** 逆解算使用纯 numpy + scipy 的解析 DH 几何方法。

### ROS 2 包依赖

- `agx_arm_description` — Nero URDF 模型（预留接口，当前解析解不依赖）
- `agx_arm_msgs` — 自定义消息
- `quest3_hand_mocap` — Quest3 手部数据采集节点

---

## 编译与安装

```bash
# 进入工作空间
cd ~/Documents/xnero_ws

# 1. 先编译依赖包
colcon build --packages-select agx_arm_description agx_arm_msgs

# 2. 安装 Python 依赖
pip install numpy scipy

# 3. 安装 pyAgxArm
cd pyAgxArm && pip install -e . && cd ..

# 4. 编译本包
colcon build --packages-select nero_quest_teleop

# 5. 加载环境
source install/setup.bash
```

---

## 快速启动

> ### ⚠️ Quest 3 HTS 启动时的双手摆放姿势
>
> 启动 Quest 3 手部追踪（HTS）时，双手的初始摆放姿势直接影响遥操的坐标系对齐精度。
> **每次启动 HTS 时务必按以下姿态摆放双手，姿态不对会导致遥操跟手方向错误**：
>
> | | 左手 | 右手 |
> |---|------|------|
> | **手掌朝向** | 朝前偏内（朝身体中线） | 指向斜右侧（约右前方 45 度） |
> | **手腕指向** | 朝前偏内 | **指向身体中间方向** |
> | **手臂姿态** | 整体向中间倾斜，肘部内收 | 手臂内收，手向右前方倾斜 |
>
> **简单记：左手往中间倒，右手向外斜但手腕指向中间。**

### 前提条件检查

**每次启动前必须按顺序执行以下检查：**

#### 1. XHand 手部串口检查

```bash
# 查看串口设备是否连接
ls /dev/ttyUSB*

# 给串口权限
sudo chmod 777 /dev/ttyUSB*
```

正常情况下应看到 `/dev/ttyUSB0`（左手）和 `/dev/ttyUSB1`（右手）。

#### 2. CAN 转 USB 接口检查

```bash
# 查看 CAN 端口是否识别
bash find_all_can_port.sh

# 激活并重命名 CAN 接口
bash can_muti_activate.sh
```

正常应看到 `can_nero_left` 和 `can_nero_right` 两个接口。

#### 3. Quest3 数据传输方式

三种模式可选，通过 `protocol` 参数切换：

| 模式 | 参数值 | Quest3 端 | PC 端前置操作 | 延迟 |
|------|--------|----------|-------------|:---:|
| UDP（默认） | `udp` | UDP → PC_IP:9000 | 无 | 低 |
| TCP 有线 | `tcp_wired` | TCP → localhost:8000 | USB 连接 + adb 端口映射 + 关防火墙 | 最低 |
| TCP 无线 | `tcp_wireless` | TCP → PC_IP:8000 | 无 | 中 |

**TCP 有线模式详细设置（推荐，延迟最低）：**

```bash
# 1. Quest3 USB 连 PC，戴上 Quest3
#    在弹出的"允许 USB 调试"对话框中点击"允许"
adb devices
# 应输出：
#   List of devices attached
#   XXXXXXXXXX    device
# （如果显示 unauthorized，说明 Quest3 端尚未允许 USB 调试）

# 2. 端口映射（Quest3 8000 → PC 8000）
adb reverse tcp:8000 tcp:8000
# 第一次执行输出：8000
# 再次执行无任何输出 → 端口映射已成功（"没有下文即可"）
# 如果反复输出 8000，说明之前映射已失效，重新执行直到无输出

# 3. 关闭防火墙（否则 TCP 连接可能被拦截）
sudo ufw disable

# 4. Quest3 HTS 应用选择 TCP，地址 localhost，端口 8000

# 5. PC 端启动
ros2 launch ... protocol:=tcp_wired
```

#### 4. 机械臂上电

CAN 线连接正常，机械臂处于安全姿态（无碰撞风险）。

### 单臂遥操

```bash
# 左臂遥操（默认）
ros2 launch nero_quest_teleop teleop_single_nero.launch.py

# 右臂遥操
ros2 launch nero_quest_teleop teleop_single_nero.launch.py \
    arm_side:=right can_channel:=can_nero_right

# 自定义参数
ros2 launch nero_quest_teleop teleop_single_nero.launch.py \
    arm_side:=left \
    can_channel:=can_nero_left \
    control_rate:=50.0 \
    motion_scale:=0.8
```

### 双臂遥操

```bash
ros2 launch nero_quest_teleop teleop_dual_nero.launch.py \
    left_can:=can_nero_left \
    right_can:=can_nero_right
```

### 运行时操作

1. 启动后，节点会连接机械臂并使能
2. 机械臂以 5% 低速移动到初始遥操姿态
3. IK 求解器进行预热（首次全局扫描 ~400ms，后续稳定在 ~8ms）
4. **戴上 Quest 3，伸出指定手**，系统自动校准零点
5. 移动手部，机械臂跟随运动
6. **摘下头显或遮挡手部 > 0.5 秒**，机械臂自动保持当前位置
7. 重新追踪到手部后，自动重新校准零点后继续跟手

---

## 键盘模拟器使用

如果暂时没有 Quest 3，可以用键盘模拟 VR 手部数据来测试系统。

```bash
# 启动键盘模拟器（默认同时发布左右手）
ros2 run nero_quest_teleop keyboard_vr_sim

# 只发布左手或右手
ros2 run nero_quest_teleop keyboard_vr_sim --ros-args -p arm_side:=left
```

### 键盘控制说明

**手部位置（VR 坐标系）：**

| 按键 | 方向 | VR 轴 |
|------|------|-------|
| `W` | 手往前 | VR X+ |
| `S` | 手往后 | VR X- |
| `A` | 手往左（左手）/ 往右（右手） | VR Y- |
| `D` | 手往右（左手）/ 往左（右手） | VR Y+ |
| `Q` | 手往上 | VR Z- |
| `E` | 手往下 | VR Z+ |

> 注意：由于左右手 VR 坐标系的 Y 轴方向不同（左手 Y+ = 左手外侧，右手 Y+ = 右手外侧），A/D 的实际运动方向在左右手之间是镜像的。

**手部旋转（VR 局部坐标系）：**

| 按键 | 旋转 | 绕 VR 轴 |
|------|------|----------|
| `I` / `K` | pitch 俯仰 | VR X（前向轴） |
| `J` / `L` | yaw 偏航 | VR Z（垂直轴） |
| `U` / `O` | roll 翻滚 | VR Y（外侧轴） |

**其他：**

| 按键 | 功能 |
|------|------|
| `R` | 重置手部位姿到零点 |
| `SHIFT` | 按住操作右手，松开操作左手 |
| `ESC` | 退出模拟器 |

### 使用键盘模拟器验证遥操

```bash
# 终端1：IK 求解节点
ros2 run nero_quest_teleop ik_solver_node

# 终端2：遥操节点（dry run 模式，不接机械臂）
ros2 run nero_quest_teleop test_dataflow --ros-args -p arm_side:=left

# 终端3：键盘模拟器
ros2 run nero_quest_teleop keyboard_vr_sim --ros-args -p arm_side:=left

# 按 W/A/S/D/Q/E 移动手部，观察 test_dataflow 输出：
#   - 按 W（手往前），Target EE 的 Y 坐标应该增大（机械臂往前）
#   - 按 D（左手往外），Target EE 的 Z 坐标应该增大（机械臂往外）
#   - 按 E（手往下），Target EE 的 X 坐标应该减小（机械臂往下）
```

---

## 坐标系说明

### VR 手部坐标系

手自然下垂贴裤缝时的初始姿态：

| 手 | X+ | Y+ | Z+ |
|----|----|----|-----|
| 左手 | 往前（远离身体） | 左手外侧（左） | 垂直下方 |
| 右手 | 往前（远离身体） | 右手外侧（右） | 垂直下方 |

### 机械臂基座坐标系

| 臂 | X+ | Y+ | Z+ |
|----|----|----|-----|
| 左臂 | 上方 | 前方 | 左手外侧 |
| 右臂 | 上方 | 后方 | 右臂外侧 |

### VR → 机械臂 旋转矩阵

**左臂** (`vr_to_arm_rot`)：

```
VR X+(forward) → Arm Y+(forward)
VR Y+(outward) → Arm Z+(outward)
VR Z+(down)    → Arm X-(down)

R_left = [[ 0,  0, -1],
          [ 1,  0,  0],
          [ 0,  1,  0]]
```

**右臂** (`vr_to_arm_rot`)：

```
VR X+(forward) → Arm Y-(forward, 因臂 Y+ 朝后)
VR Y+(right)   → Arm Z+(outward)
VR Z+(down)    → Arm X-(down)

R_right = [[ 0,  0, -1],
           [-1,  0,  0],
           [ 0,  1,  0]]
```

### TCP Offset

`tcp_offset: [x, y, z, roll, pitch, yaw]` 是旧版独立遥操节点可选的法兰到工具 TCP 变换。Nero 当前左右臂配置均为 `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`：Quest3 提供的是 `link7` 法兰位姿，IK 也以 `link7` 为目标，无需额外 TCP 偏移。

只有输入目标明确是工具 TCP 时，才应配置非零偏移并转换为法兰目标：

```
T_flange_target = T_gripper_target @ inv(T_flange_to_tcp)
```

---

## 配置参数说明

主配置文件：`config/nero_teleop_left.yaml`（左臂）、`config/nero_teleop_right.yaml`（右臂覆写）

### 机械臂连接

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `arm_side` | `left` | 控制哪条臂 `left` / `right` |
| `can_channel` | `can_nero_left` | CAN 通道名（左臂） |
| `firmware_version` | `default` | 固件版本 `default`(≤1.10) / `v111`(≥1.11) |
| `can_interface` | `socketcan` | CAN 接口类型 `socketcan`(Linux) / `slcan`(macOS) |

### 控制频率

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `control_rate: 100.0 Hz)，建议 50-100（默认100Hz） |

### IK 逆解

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `solver_type` | `analytic_dh` | 逆解方式：`analytic_dh`(当前) / `urdf`(预留) |
| `ik_service_name` | `/ik_solver/solve_ik` | IK 节点服务名（使用独立IK节点时） |

> URDF 相关参数（`urdf_package`、`urdf_relative_path`）已预留接口，当 `solver_type: "urdf"` 时启用。

### VR 位姿处理

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `vr_to_arm_rot` | 3×3 矩阵 | VR 坐标系到机械臂坐标系的旋转矩阵（行主序平铺） |
| `vr_to_arm_rpy` | `[0,0,0]` | 已弃用的 RPY 格式，仅在 `vr_to_arm_rot` 未配置时回退使用 |
| `pos_smoothing` | `0.5` | 位置 EMA 滤波系数（0=全平滑, 1=无平滑） |
| `rot_smoothing` | `0.7` | 姿态 SLERP 滤波系数 |
| `motion_scale` | `1.0` | VR 到机械臂的运动缩放因子 |
| `flip_pitch` | `true` | 是否翻转 VR 的 pitch 轴 |

### TCP 偏移

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `tcp_offset` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | 可选的法兰到工具 TCP 变换 `[x,y,z,roll,pitch,yaw]` (m/rad)；当前 Nero 法兰目标不使用偏移 |

### 安全参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_joint_vel` | `0.15` | 单步最大关节角变化 (rad/step) |
| `workspace_radius` | `0.58` | 末端工作空间球半径 (m)，Nero 臂展约 580mm |
| `workspace_z_min` | `0.0` | 末端 Z 轴下限 (m) |
| `workspace_z_max` | `0.8` | 末端 Z 轴上限 (m) |
| `workspace_x_min` / `_max` | （未启用） | X 轴盒式边界，设定后激活 |
| `workspace_y_min` / `_max` | （未启用） | Y 轴盒式边界，设定后激活 |
| `data_timeout` | `0.5` | VR 数据超时时间 (s) |

### 启动初始化

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `init_pose` | 左/右臂初始姿态 | 7 个关节角 (rad)，启动时低速移动到该姿态 |
| `init_speed_percent` | `5` | 初始逼近速度百分比 |

---

## 测试说明

**强烈建议按以下顺序逐步测试。**

### 测试1：IK 离线验证 (`test_ik_solver`)

**目的：** 验证 IK 求解器数学正确性，不需要机械臂、不需要 ROS、不需要 Quest3。

```bash
ros2 run nero_quest_teleop test_ik_solver
```

| 测试项 | 验证内容 |
|--------|---------|
| `ik_from_home` | 从零位姿态求解到一个目标位姿 |
| `fk_ik_roundtrip` | FK(关节角→位姿) → IK(位姿→关节角) 往返一致性，位置误差 < 10mm |
| `smooth_trajectory` | 沿圆形轨迹连续求解 50 步，验证收敛率和关节平滑性 |
| `safety_filter` | 关节限位 clamp、角速度限幅、工作空间边界约束 |

预期 4/4 tests passed，位置误差 0.000mm。

### 测试2：VR 坐标系映射验证 (`test_vr_mapping`)

**目的：** 验证 VR→机械臂旋转矩阵正确，手部运动和机械臂运动方向一致。

```bash
ros2 run nero_quest_teleop test_vr_mapping
```

| 测试项 | 验证内容 |
|--------|---------|
| `left_position_map` | 左手 6 方向（前后/内外/上下）→ 左臂坐标映射 |
| `right_position_map` | 右手 6 方向 → 右臂坐标映射 |
| `tcp_roundtrip` | TCP offset 夹爪→法兰→夹爪 往返一致性 |
| `ik_tcp_chain` | IK 链中 TCP offset 补偿正确性 |
| `e2e_left` | 左臂端到端：VR 增量 → 臂增量 → IK → FK |
| `e2e_right` | 右臂端到端：VR 增量 → 臂增量 → IK → FK |

预期 6/6 tests passed。

### 测试3：数据流端到端验证 (`test_dataflow`)

**目的：** 使用真实 Quest3 数据或键盘模拟器，验证完整 VR→IK→安全过滤流水线，不需要机械臂。

```bash
# 方式一：用键盘模拟器
ros2 run nero_quest_teleop keyboard_vr_sim --ros-args -p arm_side:=left
ros2 run nero_quest_teleop test_dataflow --ros-args -p arm_side:=left

# 方式二：用真实 Quest3
ros2 run quest3_hand_mocap quest3_udp_mocap --ros-args -p arm_side:=left
ros2 run nero_quest_teleop test_dataflow --ros-args -p arm_side:=left
```

**观察要点：**
1. IK 成功率应 > 95%
2. 帧率应接近 50 FPS
3. 关节角度在 Nero 限位范围内
4. Safety violations 为 0

### 测试4：安全遥操实机测试 (`test_safe_teleop`)

**目的：** 连接真实 Nero 机械臂，使用非常保守的参数验证臂真实跟随。

**阶段 A — Dry Run（只看数据不动臂）：**

```bash
ros2 run nero_quest_teleop test_safe_teleop --ros-args \
    -p arm_side:=left \
    -p can_channel:=can_nero_left
```

**阶段 B — Live Run（低速实际驱动）：**

```bash
ros2 run nero_quest_teleop test_safe_teleop --ros-args \
    -p arm_side:=left \
    -p can_channel:=can_nero_left \
    -p dry_run:=false
```

内置保守参数（比正式遥操安全得多）：

| 参数 | 安全测试值 | 正式值 |
|------|-----------|--------|
| `max_joint_vel` | 0.03 | 0.15 |
| `motion_scale` | 0.3 | 1.0 |
| `workspace_radius` | 0.55 | 0.58 |
| `pos_smoothing` | 0.6 | 0.5 |

---

## 安全机制说明

1. **关节限位** — IK 内置限位约束 + 安全过滤器二次 clamp
2. **关节角速度限幅** — 单步变化量不超过 `max_joint_vel`，超出时等比例缩放
3. **笛卡尔工作空间** — 球半径 + 可选的 XYZ 盒式边界（XYZ 三个方向独立可配）
4. **VR 数据超时保护** — 数据中断 > `data_timeout` 后保持当前位置
5. **VR 数据有效性检查** — 四元数范数检查 + 位置突变自动重校准
6. **电子急停** — 节点内置 `emergency_stop()`，也可使用机械臂物理急停按钮

---

## 文件结构

```
nero_quest_teleop/
├── package.xml                                    # ROS 2 包描述
├── setup.py                                       # Python 构建配置 (entry points)
├── setup.cfg                                      # 安装路径配置
├── CMakeLists.txt                                 # （保留，用于未来自定义服务接口）
├── README.md                                      # 本文件
├── resource/
│   └── nero_quest_teleop                          # 包标记文件
├── config/
│   ├── nero_teleop_left.yaml                           # 左臂主配置
│   └── nero_teleop_right.yaml                     # 右臂参数覆写
├── launch/
│   ├── teleop_single_nero.launch.py               # 单臂遥操 launch
│   └── teleop_dual_nero.launch.py                 # 双臂遥操 launch
└── nero_quest_teleop/
    ├── __init__.py
    ├── ik_solver.py                               # 解析 DH 几何 IK 求解器（核心）
    ├── ik_solver_node.py                          # 独立 IK 求解 ROS2 节点（可选）
    ├── pose_processor.py                          # VR 位姿坐标映射与滤波
    ├── safety_filter.py                           # 多层安全过滤
    ├── nero_teleop_node.py                        # 核心遥操控制节点
    ├── keyboard_vr_sim.py                         # 键盘 VR 手部数据模拟器
    ├── test_ik_solver.py                          # 测试：IK 离线验证
    ├── test_vr_mapping.py                         # 测试：VR→机械臂坐标映射验证
    ├── test_dataflow.py                           # 测试：数据流端到端验证
    └── test_safe_teleop.py                        # 测试：安全遥操实机测试
```

### 各模块职责

| 模块 | 职责 |
|------|------|
| `ik_solver.py` | **核心** — 解析 DH 几何逆解算。臂角参数化 + 连续性分支选择 + 1D QP 精修。包含 `IKSolver` 类（solve/fk/sync_state）、`NeroParams`（DH 参数）、`ContinuityParams`（连续性参数）。纯 numpy+scipy，无 Pinocchio/CasADI 依赖。 |
| `ik_solver_node.py` | 独立 IK 求解节点。订阅目标法兰位姿话题，发布关节角解算结果。当需要 CPU 隔离运行时使用。 |
| `pose_processor.py` | VR 位姿预处理：VR→机械臂坐标映射（旋转矩阵）、增量计算、EMA 位置平滑、SLERP 旋转平滑、motion_scale 缩放。 |
| `safety_filter.py` | 多层安全过滤：关节限位 clamp、角速度限幅、球半径 + XYZ 盒式工作空间边界。 |
| `nero_teleop_node.py` | 核心遥操节点：50Hz 控制循环，整合 PoseProcessor → TCP offset → IKSolver → SafetyFilter → move_js。支持左右臂、急停、VR 超时保护。 |
| `keyboard_vr_sim.py` | 键盘模拟器：用 W/A/S/D/Q/E 控制手部位移，I/J/K/L/U/O 控制手部旋转，模拟 Quest3 发布 PoseStamped。用于无 Quest3 时的开发和测试。 |

### 各测试文件用途

| 测试文件 | 用途 | 需要硬件 |
|---------|------|---------|
| `test_ik_solver.py` | IK 数学正确性验证（FK→IK 往返、轨迹连续性、安全过滤） | 无 |
| `test_vr_mapping.py` | VR→机械臂坐标映射矩阵正确性验证（6方向、TCP offset、端到端） | 无 |
| `test_dataflow.py` | 完整数据流测试：VR数据→IK→安全过滤，打印统计信息 | Quest3 或键盘模拟器 |
| `test_safe_teleop.py` | 安全实机遥操测试：保守参数低速跟随，支持 dry_run 模式 | Nero 机械臂 + Quest3/键盘模拟器 |

---

## 常见问题排查

### 编译问题

**Q: `import nero_quest_teleop` 失败**

A: 需要先 colcon build 并 source：
```bash
colcon build --packages-select nero_quest_teleop
source install/setup.bash
```

### 运行时问题

**Q: VR 数据不到达 / FPS 为 0**

A: 检查：
1. Quest3 应用是否启动并与电脑同网络
2. UDP 端口是否匹配（默认 9000）
3. 键盘模拟器是否正常运行

**Q: 机械臂运动方向与手不一致**

A: 运行 `test_vr_mapping` 验证矩阵正确性，然后调整 `config/nero_teleop_left.yaml` 中的 `vr_to_arm_rot` 矩阵。矩阵中的 1/-1 对应 VR 轴到机械臂轴的映射方向。

**Q: 机械臂抖动**

A: 降低 `pos_smoothing` / `rot_smoothing`（增强滤波），或降低 `max_joint_vel`。

**Q: IK 求解失败率高**

A: 检查目标位姿是否超出工作空间（`workspace_radius: 0.58`），降低 `motion_scale`。

**Q: 机械臂响应延迟大**

A: 确保使用 `move_js` 模式，CAN 通信正常，控制频率 ≥ 50Hz。

### 急停

- **物理急停**：按下机械臂上的急停按钮
- **软件急停**：`Ctrl+C` 停止节点
- 急停后恢复：重新启动节点
