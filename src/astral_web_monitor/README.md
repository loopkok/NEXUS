# astral_web_monitor

非侵入式 Web 监控面板：通过浏览器启动/暂停/停止遥操作栈，实时查看关节状态、指令频率、健康巡检与实时图表。

**不修改任何现有功能包**。只做四件事：

1. **只读订阅**现有关节话题（`joint_states` / `joint_commands`）
2. **发布到已有控制话题** `/teleop/armed`、`/teleop/disarm`（暂停/恢复/急停）、`/teleop/start`（外部启动闸门）
3. **subprocess 启停** `ros2 launch`（与命令行操作等价）
4. **只读健康巡检**：估算各源状态流/指令流频率与新鲜度，不下发任何命令

```text
┌─────────────── astral_ws 现有功能包（不修改）──────────────┐
│  quest3_mocap → arm_teleop → /{side}_arm/joint_commands   │
│                          → /teleop/armed ◄──┐              │
│                          → /teleop/disarm ◄─┤  (已有话题)   │
│  pinch_gripper → /left_gripper/command     │              │
│  wujihand_retarget → /right_hand/joint_commands            │
│  driver → /{side}_arm/joint_states, /astral/joint_states  │
└────────────────────────────────────────────────────────────┘
                        │ 订阅(只读)        ▲ 发布(disarm/arm)
                        ▼                  │
┌─────────────── astral_web_monitor (本包) ──────────────────┐
│  monitor_node (rclpy)                                      │
│   ├─ Sub: joint_states ×5, joint_commands ×4              │
│   ├─ Pub: /teleop/armed, /teleop/disarm  (仅暂停/恢复时发)  │
│   ├─ RateCounter: EWMA 指令 Hz 估计                        │
│   └─ LaunchManager: subprocess ros2 launch ...             │
│         │                                                  │
│         ▼  WebSocket 30Hz                                  │
│  web_server (FastAPI + uvicorn :8080)                     │
│   ├─ REST /api/v1/*  → 启停/暂停/恢复/预设/健康              │
│   ├─ WS   /ws/telemetry → 实时推送关节/HZ/日志              │
│   └─ Static / → web/dist (React SPA)                      │
└────────────────────────────────────────────────────────────┘
                        │ WebSocket
                        ▼
                  浏览器 React UI
```

## 架构

| 层 | 技术 | 说明 |
|----|------|------|
| ROS 节点 | rclpy（后台线程） | 只读订阅 + 线程安全快照 + pause/resume 发布 |
| Web 后端 | FastAPI + uvicorn | REST 控制 + WebSocket 实时推送 + SPA 静态托管 |
| Web 前端 | React 18 + Vite | 单例 WebSocket + `useSyncExternalStore`（30Hz 不穿透重渲染） |
| 进程模型 | 单进程 | rclpy spin 线程 + uvicorn 主线程，无需多进程 IPC |

## 话题契约

### 只读订阅（不干扰数据流）

| 话题 | 类型 | 用途 |
|------|------|------|
| `/left_arm/joint_states` | JointState | 左臂 7-DoF 显示 |
| `/right_arm/joint_states` | JointState | 右臂 7-DoF 显示 |
| `/left_gripper/joint_states` | JointState | 左夹爪状态（driver 回显的指令角） |
| `/right_hand/joint_states` | JointState | 右灵巧手 20-DoF |
| `/head/joint_states` | JointState | 头部 yaw/pitch（摇杆遥操回显/真机 OBS） |
| `/astral/joint_states` | JointState | 全身 18-DoF（含腰） |
| `/left_arm/joint_commands` | JointState | 左臂指令 Hz |
| `/right_arm/joint_commands` | JointState | 右臂指令 Hz |
| `/left_gripper/command` | Float64 | 左夹爪闭合比指令 Hz（rad 映射在 driver） |
| `/right_hand/joint_commands` | JointState | 右手指令 Hz |
| `/head/joint_commands` | JointState | 头部指令 Hz（head_teleop_node） |

- QoS：**BEST_EFFORT**（SensorData）
- 灵巧手话题名可配置（`ASTRAL_WEB_MONITOR_HAND_NAME`，默认 `right_hand`）

### 发布（暂停/恢复/外部启动，已有话题）

| 话题 | 类型 | 触发 |
|------|------|------|
| `/teleop/disarm` | Bool(True) | 点"暂停" |
| `/teleop/armed` | Bool(True) | 点"恢复" |
| `/teleop/start` | Bool(True) | 点"开始遥操"（一次性，记录 `vr_init` 并 arm） |

- QoS：`/teleop/armed`、`/teleop/disarm` 为 **RELIABLE + TRANSIENT_LOCAL**（latched，晚启动的臂节点也能收到）
- `/teleop/start` 为 **RELIABLE + VOLATILE**（**非** latched 一次性触发，避免晚加入的臂节点收到旧 start 自动开始）
- **暂停只影响臂**：`/teleop/disarm` 只作用于 `astral_arm_teleop_node`，夹爪和灵巧手节点无 disarm 接口，继续运行
- **开始遥操**：配合 `astral_arm_teleop` 的 `require_start_signal:=true`——启动预设后臂节点只跟踪 `vr_current` 不记零点；手摆好初始位姿后点此按钮，臂节点用当前 pose 记 `vr_init` 并 arm。再点一次 = 重新记零点（re-center）
- **无条件发送**：`/api/v1/teleop/start` 始终发布 `/teleop/start`，**不**检查 launch 是否经本监控启动。臂节点是唯一裁判：homing 中或无 VR pose 时会忽略并告警。因此无论遥操由本监控的预设启动还是从外部 CLI 启动，此按钮均可用

## REST API

所有响应信封：`{ok, message, data}`，冲突返回 HTTP 409 + `{"detail": "..."}`。

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/api/v1/health` | ROS 节点状态、launch 状态、PID、运行时长、WS 客户数、health 汇总 |
| GET | `/api/v1/presets` | 启动预设列表 |
| GET | `/api/v1/state` | 当前快照（同步，含 health + state_rates_hz） |
| POST | `/api/v1/start` | `{preset: "..."}` 启动指定预设 |
| POST | `/api/v1/stop` | SIGINT 停止 launch（30s 超时 SIGKILL） |
| POST | `/api/v1/pause` | 发 `/teleop/disarm`（软暂停，节点保持运行） |
| POST | `/api/v1/resume` | 发 `/teleop/armed`（恢复） |
| POST | `/api/v1/teleop/start` | 发 `/teleop/start`（一次性，记录 `vr_init` 并 arm；配合 `require_start_signal`；无条件发送，臂节点自行判断有效性） |
| POST | `/api/v1/restart` | 重启当前预设（停止后重新启动；仅对经本监控启动的预设有效） |
| POST | `/api/v1/robot/ready` | 调 driver `~/ready`（one_click_ready，上电+零位） |
| POST | `/api/v1/robot/home` | 调 driver `~/home`（全关节归零） |
| POST | `/api/v1/robot/position` | 调 driver `~/position`（motion_mode=1，位置保持） |
| POST | `/api/v1/robot/damping` | 调 driver `~/damping`（motion_mode=0，阻尼释放，可手动拖拽） |
| POST | `/api/v1/robot/estop` | **真急停**：调 driver `~/estop` → SDK `e_stop()`/`disable()` 断电，臂失去保持力 |

## WebSocket

端点：`ws://<host>:8080/ws/telemetry`

- 推送频率：**30Hz**（`ASTRAL_WEB_MONITOR_PUSH_HZ`）
- 无客户端时跳过广播
- 连接后立即发一帧快照
- 支持 `ping` → 回 `{"type":"pong","ts":...}`

```json
{
  "type": "ui_state",
  "ts": 1692878400.123,
  "teleop": {"state": "running", "preset": "Full real (Quest)", "uptime_s": 120.5, "pid": 12345},
  "joints": {
    "left_arm":      {"values": [0.1, ...7], "ts": 1692878400.1, "stale": false},
    "right_arm":     {"values": [0.1, ...7], "ts": 1692878400.1, "stale": false},
    "left_gripper":  {"values": [0.35],     "ts": 1692878400.1, "stale": false},
    "right_hand":    {"values": [...20],    "ts": 1692878400.0, "stale": true},
    "head":          {"values": [0.0, 0.0], "ts": 1692878400.1, "stale": false}
  },
  "rates_hz": {
    "left_arm_cmd": 149.8, "right_arm_cmd": 149.9,
    "left_gripper_cmd": 50.0, "right_hand_cmd": 30.1, "head_cmd": 50.0
  },
  "state_rates_hz": {
    "left_arm_state": 200.0, "right_arm_state": 200.0,
    "left_gripper_state": 100.0, "right_hand_state": 100.0,
    "head_state": 100.0,
    "full_body_state": 100.0
  },
  "health": {
    "overall": "ok",
    "entities": {
      "left_arm": {"stale": false, "state_hz": 200.0, "cmd_hz": 149.8, "expected_hz": 30.0, "slow": false, "status": "ok"}
    }
  },
  "log_tail": ["[INFO] mocap connected ...", "..."]
}
```

## 状态机

```
stopped ──start──► starting ──2s暖机──► running
   ▲                                       │
   │                                  pause │ resume
   │                                       ▼
   │                                    paused
   │                                       │
   └────────── stop(SIGINT) ◄──────────────┘
```

## Web UI 功能

顶栏：标题 + WS/ROS/数据 三色状态徽标 + ControlBar（急停[真断电] / 阻尼释放 / 开始遥操 / 暂停 / 恢复）。
下方三 Tab：

| Tab | 内容 |
|-----|------|
| 监控 | 4 个关节面板（左/右臂、左夹爪、右灵巧手）+ 指令频率 chips + 实时折线图（指令 Hz、臂关节0 角度） |
| 健康 | 总体徽标 + 每实体卡片（状态流 Hz / 指令流 Hz / 期望 / 数据龄期 / ok·slow·stale） |
| 系统 | 预设管理（启动/停止/重启）+ **机器人模式**（一键就绪/归零/位置保持/阻尼释放/急停断电）+ Launch 日志控制台 |

- **急停（真断电）**：红色常驻按钮，确认后调 driver `~/estop` → SDK `e_stop()`/`disable()`，臂失去保持力；恢复需重新「一键就绪」。区别于「暂停」（软 disarm，臂仍上电保持位姿）
- **阻尼释放**：调 driver `~/damping` → `motion_mode=0`，电机仍上电、关节可手动拖拽。典型流程：遥操中 → 停止（臂保持末位姿）→ 阻尼释放（手动拖回 home）→ 位置保持/归零

> **机器人模式按钮的可用时机（重要）**：急停/阻尼释放/位置保持/一键就绪/归零 这五个按钮调的都是 **driver 节点（astral_robot_driver）的 ROS 服务**，driver 随遥操栈启停：
>
> | 遥操栈状态 | 按钮表现 |
> |-----------|---------|
> | 运行中（启动后） | ✅ 可用 |
> | 已暂停（软 disarm） | ✅ 可用（driver 还活着，只是不收遥操指令） |
> | 已停止 | ❌ 503「driver service 未就绪」——driver 进程已随栈退出，属**预期行为**，先点「启动」再操作 |
>
> 即：**先启动栈，再点机器人模式按钮**；停止栈之后任何机器人按钮都不会生效。若运行中仍 503，说明 driver 没起来（`with_arm_driver:=false` 的预设/sim）或 Jetson 端 driver 是旧代码——在 Jetson 上 `ros2 service list \| grep astral_robot_driver` 应列出 5 个 Trigger 服务。

- **Toast 通知**：操作成功/失败以右上角浮窗提示（替代 alert），自动消失
- **实时图表**：手写 SVG 折线（无第三方图表库），环形缓冲 200 样本（≈6.7s @ 30Hz）
- **头部通道**：监视 tab 显示「头部 (yaw/pitch)」面板 + yaw/pitch 实时折线；健康巡检含头部（仅新鲜度判断——头部指令在启动前/手柄掉线时合法为 0Hz，故不设 cmd 频率下限）
- **健康巡检**：`expected_hz` 阈值经 `ASTRAL_WEB_MONITOR_EXPECTED_HZ` 配置（默认 30Hz），只读估算

## 启动预设

见 `config/presets.yaml`，可直接编辑无需改代码：

| 预设 | 包 | launch | 说明 |
|------|----|--------|------|
| Arms only (sim, no driver) | astral_arm_teleop | astral_dual_arm_teleop | 双臂+Quest，无真机 |
| Full real (Quest right hand) | astral_teleop | full_teleop | 双臂+左夹爪+右手灵巧手，Quest |
| Full real (Glove right hand) | astral_teleop | full_teleop | 同上，右手来自 Wuji Glove |
| MuJoCo sim pipeline | astral_mujoco_sim | astral_sim_pipeline | MuJoCo 仿真全链路 |

## 依赖

```bash
# Python 依赖（用系统 python3，不是 conda）
python3 -m pip install fastapi uvicorn

# 构建
cd /path/to/astral_ws
export PATH=/usr/bin:$PATH          # 避免 conda python 抢占
source /opt/ros/humble/setup.bash
colcon build --packages-select astral_web_monitor --symlink-install
source install/setup.bash
```

## 启动

```bash
# 生产模式：后端托管已构建前端
# 注意：ros2 pkg prefix 返回 install/<pkg>，需 ../../ 回到工作区根再进 src
ASTRAL_WEB_MONITOR_DIST=$(ros2 pkg prefix astral_web_monitor)/../../src/astral_web_monitor/web/dist \
  ros2 launch astral_web_monitor web_monitor.launch.py

# 浏览器访问 http://<host>:8080
# 若 8080 被占（如 Cursor IDE），换端口：
# ASTRAL_WEB_MONITOR_PORT=8090 ros2 launch astral_web_monitor web_monitor.launch.py
```

开发模式（前端热更新）：

```bash
# 终端1：后端
ros2 launch astral_web_monitor web_monitor.launch.py

# 终端2：前端 dev server（:5173，代理 /api /ws → :8080）
cd src/astral_web_monitor/web && npm install && npm run dev

# 浏览器访问 http://localhost:5173
```

## 参数 / 环境变量

| 变量 | 默认 | 说明 |
|------|------|------|
| `ASTRAL_WEB_MONITOR_HOST` | `0.0.0.0` | 绑定地址 |
| `ASTRAL_WEB_MONITOR_PORT` | `8080` | 端口 |
| `ASTRAL_WEB_MONITOR_DIST` | `""` | 前端构建产物路径（空=不托管 SPA） |
| `ASTRAL_WEB_MONITOR_PUSH_HZ` | `30` | WebSocket 推送频率 |
| `ASTRAL_WEB_MONITOR_STALE_S` | `2.0` | 关节陈旧阈值（秒） |
| `ASTRAL_WEB_MONITOR_WARMUP_S` | `2.0` | 启动暖机时间 |
| `ASTRAL_WEB_MONITOR_STOP_TIMEOUT_S` | `30` | SIGINT 超时 |
| `ASTRAL_WEB_MONITOR_HAND_NAME` | `right_hand` | 灵巧手话题命名空间 |

## 前端构建

```bash
cd src/astral_web_monitor/web
npm install
npm run build    # → web/dist/
```

## 非侵入性保证

| 交互 | 机制 | 修改现有包？ |
|------|------|-------------|
| 监控关节 | 订阅现有话题 | 否 |
| 指令 Hz | 订阅现有指令话题 | 否 |
| 启动 | subprocess `ros2 launch` | 否 |
| 停止 | SIGINT subprocess | 否 |
| 暂停 | 发布 `/teleop/disarm`（已有话题） | 否 |
| 恢复 | 发布 `/teleop/armed`（已有话题） | 否 |
| 开始遥操 | 发布 `/teleop/start`（已有话题） | 否 |
| 重启 | stop + start 同一预设（subprocess） | 否 |
| 急停（真断电） | 调 driver 已有服务 `~/estop`（SDK `e_stop`） | 否（仅调用 driver 已暴露的服务） |
| 阻尼释放 | 调 driver 已有服务 `~/damping`（`motion_mode=0`） | 否 |
| 位置保持 | 调 driver 已有服务 `~/position`（`motion_mode=1`） | 否 |
| 一键就绪 / 归零 | 调 driver 已有服务 `~/ready` / `~/home` | 否 |
| 健康巡检 | 订阅现有 state/command 话题估算频率 | 否 |
| 日志 | 采集 subprocess stdout | 否 |

> 注：机器人模式按钮调用 `astral_robot_control` driver **已暴露**的 Trigger 服务（`~/ready`/`~/home`/`~/position`/`~/damping`/`~/estop`）。driver 是硬件权威，monitor 仅作为服务客户端调用——不新增硬件写入路径、不修改 driver 逻辑。若 driver 未启动，端点返回 503。

## 包结构

```
astral_web_monitor/
├── astral_web_monitor/
│   ├── monitor_node.py      # rclpy 节点：只读订阅 + 快照 + pause/resume
│   ├── launch_manager.py    # subprocess 启停 + 孤儿检测
│   ├── rate_counter.py      # EWMA Hz 估计
│   ├── web_server.py        # FastAPI + WebSocket + 静态托管
│   ├── schemas.py           # Pydantic 模型
│   └── config.py            # 话题名/阈值/端口
├── config/presets.yaml      # 启动预设
├── launch/web_monitor.launch.py
└── web/                     # React + Vite 前端
    ├── src/
    │   ├── hooks/useRealtime.ts   # 单例 WebSocket + 连接状态
    │   ├── hooks/useToast.ts      # Toast 通知 store
    │   ├── hooks/historyStore.ts  # 图表环形缓冲
    │   ├── components/            # ControlBar, MonitorTab, HealthPanel,
    │   │                          # SystemTab, ChartPanel, ToastHost,
    │   │                          # Tabs, JointPanel, LogConsole, StatusBadge
    │   └── lib/mapUiState.ts      # 数据归一化
    └── dist/                # 构建产物
```
