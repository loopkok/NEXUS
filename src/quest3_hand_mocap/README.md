# quest3_hand_mocap

Quest3 VR 手部追踪 + 头部位姿数据接收节点。支持 UDP / TCP 有线 / TCP 无线。

---

## 功能

接收 Quest3 HTS 的手部 landmark（21×3）、腕部位姿、头部（HMD）位姿，发布 ROS2 话题。

**下游（本工作空间 `astral_ws`）：**

- **Astral 臂**：订 `quest3/{side}_wrist_pose`（`convert_to_robot:=true` → `robot_world`）
- **Wuji 手**：Wuji launch 强制 `landmark_preprocess:=raw`（Unity→机器人轴后相对腕心），由 `wuji_retargeting` 再做一次 MANO，避免双重变换

默认 `landmark_preprocess:=mano` 是给旧 XHand 链路用的；本仓库 Wuji pipeline 会覆盖为 `raw`。

**遥操坐标系：**

```text
robot_world|vr_world (convert_to_robot 时为 X左 Y后 Z上)
 ├── head          ← quest3/head_pose
 ├── wrist         ← quest3/{side}_wrist_pose（与 head 同世界系，不再相对头）
 └── landmarks    ← hand_landmarks/{side}（腕局部 + raw|mano + EMA）
```

**Unity → 机器人轴（`convert_to_robot:=true` 时）：**

| Unity (LH) | 机器人 |
|------------|--------|
| X 右 | **X 左** = `-X_u` |
| Y 上 | **Z 上** = `Y_u` |
| Z 前 | **Y 后** = `-Z_u` |

即 \((x,y,z)_r = (-x_u,\ -z_u,\ y_u)\)，姿态用 \(R_r = M R_u M^\top\)。

> 建议 HTS 开启 head tracking（若要用头控底盘）；腕不再依赖 head，无 head 也可发腕。

## 数据传输方式

| 模式 | 参数值 | Quest3 端配置 | PC 端 | 延迟 | 需要 WiFi 同网 |
|------|--------|-------------|-------|------|:---:|
| UDP | `udp` | UDP, 端口 9000, PC_IP | 无额外操作 | 低 | 是 |
| TCP 有线 | `tcp_wired`（yaml 默认） | TCP, localhost:8000 | `adb reverse tcp:8000 tcp:8000` | 最低 | 否 |
| TCP 无线 | `tcp_wireless` | TCP, PC_IP:8000 | 无额外操作 | 中 | 是 |

### TCP 有线（推荐）

1. Quest3 USB 连 PC，允许 USB 调试（`adb devices` 为 `device`）
2. `adb reverse tcp:8000 tcp:8000`
3. HTS 选 TCP，地址 `localhost`，端口 `8000`，并开启 head tracking
4. PC：`protocol:=tcp_wired`

## 相对 VR 原始数据的处理

| 数据 | VR 原始 | 本节点处理 | 最终 `frame_id` |
|------|---------|------------|-----------------|
| head | Unity 追踪世界 | 可选 Unity→机器人轴 | `robot_world` / `vr_world` |
| wrist | Unity 追踪世界 | 可选 Unity→机器人轴（与 head 同系，不相对头） | 同 head |
| landmarks | Unity 腕局部 21×3 | ① Unity→机器人轴或旧版翻 X ② `raw`/`mano` ③ EMA | `hand_{side}` |

## 数据流

```text
Quest3 HTS (Unity LH)
  ├── head    → [convert_to_robot?] → quest3/head_pose     [robot_world|vr_world]
  ├── wrist   → [convert_to_robot?] → quest3/{side}_wrist  [同上]
  └── landmarks
        → Unity→robot 或翻 X → process_landmarks()
             ├── raw  → 相对腕心（Wuji）
             └── mano → SVD + OPERATOR2MANO (+ 可选小指)
                        → EMA → hand_landmarks/{left,right}
```

## 话题

| 话题 | 类型 | `frame_id` | 说明 |
|------|------|------------|------|
| `hand_landmarks/left` | PoseArray | `hand_left` | 左手 21 点（腕局部） |
| `hand_landmarks/right` | PoseArray | `hand_right` | 右手 21 点（腕局部） |
| `quest3/left_wrist_pose` | PoseStamped | `robot_world`/`vr_world` | 左手腕，世界系 |
| `quest3/right_wrist_pose` | PoseStamped | `robot_world`/`vr_world` | 右手腕，世界系 |
| `quest3/head_pose` | PoseStamped | `robot_world`/`vr_world` | HMD，世界系 |
| `quest3/{side}_hand_markers` | MarkerArray | `world` | RViz（可选） |

## 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `protocol` | `tcp_wired` | `udp` / `tcp_wired` / `tcp_wireless` |
| `udp_port` | `9000` | UDP 端口 |
| `tcp_port` | `8000` | TCP 端口 |
| `arm_side` | `right` | `left` / `right` / `both` |
| `ema_alpha` | `0.7` | landmarks EMA 平滑 |
| `viz` | `true` | RViz markers |
| `landmark_preprocess` | `mano` | `mano`（XHand）或 `raw`（Wuji） |
| `enable_xhand_pinky_adapt` | `false` | XHand 小指顺序拉伸；Wuji 必须 `false` |
| `convert_to_robot` | `true` | `true`：Unity→机器人轴（X左 Y后 Z上）；`false`：保留 Unity 轴（landmarks 仅旧版翻 X） |

Wuji 相关 launch（`wujihand_tuning` / `sim_pipeline` / `real_pipeline`）已写死：
`landmark_preprocess:=raw`，`enable_xhand_pinky_adapt:=False`。

## 启动

### ROS2 节点

```bash
# UDP
ros2 run quest3_hand_mocap quest3_udp_mocap --ros-args -p arm_side:=both -p protocol:=udp

# TCP 有线
ros2 run quest3_hand_mocap quest3_udp_mocap --ros-args \
  -p arm_side:=both -p protocol:=tcp_wired

# Wuji 兼容
ros2 run quest3_hand_mocap quest3_udp_mocap --ros-args \
  -p arm_side:=both -p protocol:=tcp_wired \
  -p landmark_preprocess:=raw -p enable_xhand_pinky_adapt:=false \
  -p convert_to_robot:=true

# 关闭机器人轴转换（保持 Unity 轴）
ros2 run quest3_hand_mocap quest3_udp_mocap --ros-args \
  -p arm_side:=both -p protocol:=tcp_wired -p convert_to_robot:=false
```

### 无 ROS 独立脚本

与本包处理链路一致（含可选 Unity→机器人轴），不依赖 `rclpy`。仅需 `numpy`；`--viz` 可选 `matplotlib`。

先准备 TCP 有线：

```bash
adb reverse tcp:8000 tcp:8000
# HTS：TCP + localhost:8000，开启 head tracking
```

**推荐：TCP 有线 + Wuji(raw) + 机器人轴转换 + 可视化 + 落盘**

```bash
cd /home/robot/loopkok/quest3/quest3_hand_mocap
python3 scripts/quest3_mocap_standalone.py \
  --protocol tcp_wired \
  --arm-side both \
  --landmark-preprocess raw \
  --convert-to-robot \
  --viz \
  --jsonl /tmp/mocap.jsonl
```

关闭轴转换时加 `--no-convert-to-robot`。

其他示例：

```bash
# 仅 TCP 有线打印（默认已开启 convert_to_robot）
python3 scripts/quest3_mocap_standalone.py --protocol tcp_wired --arm-side both

# 保持 Unity 轴
python3 scripts/quest3_mocap_standalone.py --protocol tcp_wired --no-convert-to-robot

# UDP + 可视化
python3 scripts/quest3_mocap_standalone.py --protocol udp --port 9000 --viz
```

也可在代码里直接用（把 `scripts` 加入 `PYTHONPATH`，或按文件路径导入）：

```python
from quest3_mocap_standalone import MocapConfig, Quest3MocapStandalone

cfg = MocapConfig(protocol="tcp_wired", arm_side="both", landmark_preprocess="raw")
rx = Quest3MocapStandalone(cfg, on_wrist=lambda side, pose: print(side, pose.position))
rx.start()
# ...
rx.stop()
```

## 在系统中的位置

```text
quest3_udp_mocap
  ├── head_pose (世界系) → 可选：人形 base / map
  ├── wrist_pose (世界系，同 head) → 双臂 IK / Nero
  ├── XHand:  mano landmarks → xhand_retargeting
  └── Wuji:   raw landmarks → wujihand_retargeting
```

## Changelog / Bug fixes

| 日期 | 项 | 说明 |
|------|----|------|
| 2026-08 | 腕改回世界系 | 取消 wrist 相对 head；head 与 wrist 同属 `robot_world`/`vr_world` |
| 2026-08 | convert_to_robot | 启动可选 Unity→机器人轴 |
详见顶层 [README](../../README.md#wuji-hand) 与 [WUJI_CHANGELOG.md](../../WUJI_CHANGELOG.md)。
