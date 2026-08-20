# Changelog（astral_ws）

Quest3 → Astral 双臂 + Wuji 双手。从 `xnero_ws-main` 迁入。各包 README 内亦有对应条目。

时间均为北京时间。

## 2026-08-20

适配 Quest **astral-tracking** 新推流：Mixed 下手柄与手可同时有数据；IOBT 开时发全身关节，且头/手/手柄已在 hips 身体系。`quest3_udp_mocap` 重新解析 `Left/Right controller:` 与 `body iobt`。

- Mixed：同侧 Quest 只发手柄或手（手柄优先）；左右可一边 ctrl 一边 hand。`controller_as_wrist` 默认把手柄写到 `quest3/{side}_wrist_pose`，臂 IK 不用改。另发 `quest3/{side}_controller_pose`、`quest3/input_mix`。
- IOBT：`quest3/hips_pose`（世界系）+ `quest3/body_joints`（转到 hips 系）+ `quest3/body_joint_names`。头/腕/手柄的 `frame_id` 在收到身体包时切到 `robot_body`/`vr_body`。landmark 仍是腕局部。
- TCP `listen` 提到 8，覆盖手×2 + 头 + 手柄 + 身体多路连接。
- 独立脚本 `quest3_mocap_standalone.py` 同步解析与可视化。

新增 **`quest3_video_streamer`**：把 PC 相机画面经 WebRTC 推到 Quest 3，遥操时看机器人第一视角。与 `quest3_hand_mocap` 独立（信令 `:8765`，mocap 仍是 `:8000`）。自 `hand-tracking-sdk` 视频部分拆出，不依赖该 SDK、不用 venv。

- 单条 peer connection 多路 track：默认 D435i 1080p30（v4l2 `/dev/video8`）+ 左右腕 USB 720p30。
- 可调全在 `config/params.yaml`（相机列表、source/device/preset/fov/layout）；launch 只加载 yaml。
- 延迟：源端 `next_frame()` 节流、webcam 后台线程抓帧、D435i 默认真连 v4l2（可选 `d435i_source:=ros` 走 realsense 节点）。
- Quest 用 `video_config` 的 `fov_h_deg` + layout 摆 3D 面板。码率下限/默认/上限 3/5/12 Mbps。
- `quest3_udp_mocap` 增加 `[Mocap Downlink] kbps`，与视频上行 `total_kbps` 可对照链路占用。
- 与 HTS 同跑时 `enable_mocap_tcp` 保持 false，避免抢 TCP 8000。

```bash
ros2 launch quest3_video_streamer multi_camera.launch.py
# Quest：astral-tracking 填 PC 信令（Wi-Fi 或 adb reverse tcp:8765 tcp:8765）后 Start Stream
```

## 2026-08-19

相对 `f6300cf` 的未提交改动一并入库。当日另做：

- **移出 WebXR**：`astral_webxr` 不再在本工作空间内（现位于 `../astral_webxr`）。臂/手 pipeline 去掉 `input_source:=webxr`，只走 HTS `quest3_udp_mocap`。
- **撤回 PC 端 IOBT/混控**：`quest3_udp_mocap` 不再解析 `body iobt` / `controller` 行，也不再发 `quest3/body_joints`、`hips_pose`。Quest HTS 工程里的 IOBT/混控代码未改。

## 2026-08-17

相对 `f6300cf` 之后、当日完成的遥操改动（此前未 commit）：

### 跟手调参 UI（`teleop_tune_plot`）

- 布局、中文字体、按钮颜色；横轴改为「过去 8 s → 现在」。
- 热改滑条 / DH↔URDF 热切仍沿用 08-14 的话题与参数（见下文 08-14 节）。

### 配置改走 yaml

- 臂 IK 默认改为 **`urdf_numerical`**（`astral_teleop_{left,right}.yaml`）。
- `protocol` 以 `quest3_mocap.yaml` 为准（默认 `tcp_wired`）。
- `astral_sim_pipeline` / `astral_dual_arm_teleop` 的 `solver_type` / `protocol` / `convert_to_robot` 留空则**不覆盖** yaml。

### 删除旧单进程入口

- 去掉 `astral_teleop_node.py`、`astral_teleop.yaml`、`astral_quest_teleop.launch.py`、`astral_real_pipeline.launch.py`。
- 真机只留 `astral_dual_arm_teleop`（2× `astral_teleop_arm_node`）。
- `setup.py` 用 `_existing()` 过滤 glob，避免删 yaml 后 colcon 仍拷已失效的安装链接。

### 真机启动走到 init_pose

- `move_to_init_pose` 默认开：从当前 `joint_states` 按 `init_speed_percent`（10% × `max_joint_vel`）关节空间插值到 yaml `init_pose`，到位（0.05 rad）或超时 15 s 后再跟手。
- `init_speed_percent` 是本节点相对 `max_joint_vel` 的比例，**不是** SDK `set_speed_percent` / `move_j`。
- `dry_run` 跳过 homing。

## 2026-08-14

此版本使用 Quest3，逆解使用 DH 以及 URDF，双臂遥操成功。

- 输入：Quest3（`quest3_hand_mocap`，`convert_to_robot:=true`）
- 臂 IK：`analytic_dh` 与 `urdf_numerical` 均可
- 双臂：`astral_teleop_{left,right}` 进程并行 → `/{side}_arm/joint_commands`

### 延迟测量与 150 Hz 控制环

在 mocap / 单臂遥操 / MuJoCo 仿真加周期性 `[Latency]`（约 2 s，`print_latency` / `latency_print_interval`）。

| 日志 | 含义 |
|------|------|
| VR `recv_to_pub` | PC 收到 Quest 文本到发出 ROS |
| VR `wrist_gap.{left,right}` | 同侧腕包间隔（跟踪率，不是 300 Hz 行率） |
| 臂 `vr_rx` / `vr_age` / `ik` / `loop` / `ik_fail` | 收腕、等到控制拍、IK、整圈、无解次数 |
| 仿真 `e2e.{side}` | VR 收包时间戳 → 仿真拿到 `joint_commands` |
| 仿真 `apply.{side}` | 回调入队 → 写入 `qpos` |
| `[VR Data Rate]` | 所有文本行（头+双腕+landmark）约 300–360 Hz |
| `[Sim Cmd FPS]` | 仿真实际收到的关节指令频率 |

Quest 头显内部时钟未对齐，设备→PC 的空中延迟测不到。`[VR Data Rate]` 不是臂跟踪率；臂跟踪看 `wrist_gap` ≈ 14 ms ≈ **72 Hz**。

- 遥操 `control_rate`：50 → **150 Hz**（yaml）。真机驱动 `control_rate` / `ctrl_hz` **仍为 50 Hz**，未改板卡发送。
- `max_joint_vel`：由「每拍 rad」改为 **rad/s**（默认 4.0，相当于旧 0.08 rad/tick @ 50 Hz）。`SafetyFilter` 每拍上限 = `max_joint_vel * dt`。
- `pos_smoothing` / `rot_smoothing`：仍为 0–1，按 50 Hz 的 α 标定成时间常数 τ，每拍 `α(dt)=exp(-dt/τ)`。改 `control_rate` 不再改变手感。
- 仿真循环原先每圈只 `spin_once` 一次，150 Hz 指令会被丢到 `[Sim Cmd FPS] ~50`。改为 `_spin_drain`（最多 24 次 `spin_once`），订阅 QoS depth 50。修好后 Cmd FPS ≈ **150 Hz**，`e2e` 约 10–12 ms。仿真只保留最新 `q` 再写入 `qpos`：收得比发得慢会跳过中间拍，看起来像冲一截；收得更快则零阶保持，不会因此冲。

有线 TCP + URDF 数值 IK 的仿真对比（150 Hz、drain 之后）：均值 `e2e` ~10–12 ms；DH 的 IK 约 +1 ms；UDP 主要伤尾部；**DH+UDP 左臂曾大量 `ik_fail`、指令中断**。

### 跟手调参曲线（`teleop_tune_plot`）

```bash
ros2 run astral_quest_teleop teleop_tune_plot --ros-args -p arm_side:=right
```

- 遥操默认发 `/teleop/{side}/tune/`：`ee_vr`（未平滑的 VR 映射）、`ee_filt`（滤波目标）、`ee_cmd`（指令 FK）、`xyz`（三条打成一条）。
- 窗口叠画 mm 曲线 + RMS / p95 / hold jitter / 互相关 lag。键位在**曲线窗口**（不是 launch 终端）：`z` 把当前末端设为图零点，`s` 存 CSV，`q` 退出。
- 滑动条热写 ROS 参数（默认左右同步）：手感 `pos_smoothing` / `rot_smoothing` / `motion_scale` / `max_joint_vel`；URDF LM：`ik_w_pos` / `ik_w_ori` / `ik_w_reg` / `ik_max_iter` / `ik_tol`。
- 左下角可热切换 `analytic_dh` ↔ `urdf_numerical`（重建求解器、按当前 `q` 重定 VR 零点）。DH 闭式忽略 LM 权重。仿真 launch 即使以 DH 启动也会带上 `urdf_path`，以便热切到 URDF。
- yaml 只是下次启动的初值；热改不写回文件。`solver_type` / `urdf_path` / `init_pose` / `control_rate` 等不可随意热改（`control_rate` 仍需重启）。

## 2026-08

### Wuji 手

- 包：`wuji_glove`、`wujihand_retargeting`、`wujihand_mujoco_sim`、`wujihand_control`（+ vendored `wujihandros2`）
- 统一话题：`hand_landmarks/{side}` → `/{side}_hand/joint_commands` → driver / MuJoCo
- 三后端：`official` / `wuji_retargeting` / `dexpilot`
- TuningViewer 三层骨架（橙/青/白）+ retarget yaml 热重载
- Quest3 专用 yaml：`retarget_wuji_lib_quest3_{left,right}.yaml`

#### Bug fixes

| ID | 现象 | 根因 | 修复 |
|----|------|------|------|
| Q1 | Quest3 小指呈 **Z 形** | `adaptive_retargeting_xhand` 顺序拉伸 | 默认 `enable_xhand_pinky_adapt:=false`；Wuji launch 强制关闭 |
| Q2 | Quest3 相对手套 **MCP / 掌部偏移** | Quest 已做腕帧+MANO，Retargeter 再变换一次 | `landmark_preprocess:=raw`；手套节点不做 MANO |
| Q3 | Quest3 调参橙/白 MCP 仍偏 | Adaptive 不约束 MCP | quest3 yaml：`snap_mcp_to_robot: true` |
| Q4 | 真机 **收不到** `joint_commands` | BEST_EFFORT vs RELIABLE | 对齐 SensorDataQoS BEST_EFFORT |
| Q5 | 真机未用上调参 yaml | real pipeline 仍指向手套 yaml | `input_source:=quest3` 自动传 quest3 yaml |
| Q6 | dexpilot/official 仿真握拳、不跟手 | MJCF `kp` 过小 | tuning/sim 写 `qpos`+`mj_forward` |

#### 操作提示

- 真机与 MuJoCo **二选一**，勿抢同一 `joint_commands`
- `*_serial` 可空（按 `hand_side` 连）
- USB：`0483` 需 udev `MODE="0666"`
