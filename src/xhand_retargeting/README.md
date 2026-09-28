# xhand_retargeting

手部 landmark → XHand 关节角重定向节点。使用 DexPilot 算法。

---

## 功能

订阅 `quest3_hand_mocap` 发布的 21 点手部 landmark（MANO 坐标系），通过 DexPilot 重定向算法映射到 XHand 12 关节角，发布 XHandCommand 供硬件驱动执行。

## 数据流

```
/hand_landmarks/{left,right} (PoseArray)
    │
    ▼
retarget_hand()
  ├── DexPilot: 手指间参考向量 → XHand URDF 关节空间优化
  ├── URDF 关节名 → XHand 控制名映射
  └── 输出原始关节角
    │
    ▼
_postprocess_thumb()  ← 拇指矫正
    │
    ▼
EMA 关节级平滑 (smoothing_alpha)
    │
    ▼
/{left,right}_hand/xhand_command (XHandCommand)
/{left,right}_hand/joint_states (JointState, viz)
```

## 话题

| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `hand_landmarks/left` | sub | PoseArray | 左手关键点输入 |
| `hand_landmarks/right` | sub | PoseArray | 右手关键点输入 |
| `/left_hand/xhand_command` | pub | XHandCommand | 左手控制指令 |
| `/right_hand/xhand_command` | pub | XHandCommand | 右手控制指令 |
| `/left_hand/joint_states` | pub | JointState | RViz 可视化 |
| `/right_hand/joint_states` | pub | JointState | RViz 可视化 |
| `~/metrics/left_hand` | pub | Float64MultiArray | 左手延时指标 |
| `~/metrics/right_hand` | pub | Float64MultiArray | 右手延时指标 |
| `~/home_left` | srv | Trigger | 左手回零 |
| `~/home_right` | srv | Trigger | 右手回零 |

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `input_topic` | hand_landmarks | 输入话题前缀 |
| `smoothing_alpha` | 0.7 | 关节级平滑 |
| `enable_thumb_fix` | true | 拇指伸直矫正 |
| `viz` | false | 发布 JointState |

## 配置文件

- `config/xhand_right_dexpilot.yml` — 右手 DexPilot 参数
- `config/xhand_left_dexpilot.yml` — 左手 DexPilot 参数
- URDF: `urdf/xhand_right.urdf`, `urdf/xhand_left.urdf`

## 启动

```bash
# 默认
ros2 run xhand_retargeting xhand_dex_retargeting_node

# Launch（含 quest3 + retargeting + hand drivers）
ros2 launch xhand_retargeting quest3_xhand_teleop.launch.py
```

## 在系统中的位置

```
quest3_udp_mocap → xhand_retargeting → xhand_control_ros2 → XHand 电机
```

详见顶层 [README](../README.md)。
