# Wuji Hand — 变更日志（astral_ws）

从 `xnero_ws-main` 迁入本工作空间后的 Wuji 集成修复。各包 README 内亦有对应条目。

## 2026-08-14

此版本使用 Quest3，逆解使用 DH 以及 URDF，双臂遥操成功。

- 输入：Quest3（`quest3_hand_mocap`，`convert_to_robot:=true`）
- 臂 IK：`analytic_dh`（默认）与 `urdf_numerical` 均可
- 双臂：`astral_teleop_{left,right}` 进程并行 → `/{side}_arm/joint_commands`

## 2026-08

### 功能

- 包：`wuji_glove`、`wujihand_retargeting`、`wujihand_mujoco_sim`、`wujihand_control`（+ vendored `wujihandros2`）
- 统一话题：`hand_landmarks/{side}` → `/{side}_hand/joint_commands` → driver / MuJoCo
- 三后端：`official` / `wuji_retargeting` / `dexpilot`
- TuningViewer 三层骨架（橙/青/白）+ retarget yaml 热重载
- Quest3 专用 yaml：`retarget_wuji_lib_quest3_{left,right}.yaml`

### Bug fixes

| ID | 现象 | 根因 | 修复 |
|----|------|------|------|
| Q1 | Quest3 小指呈 **Z 形** | `adaptive_retargeting_xhand` 顺序拉伸 | 默认 `enable_xhand_pinky_adapt:=false`；Wuji launch 强制关闭 |
| Q2 | Quest3 相对手套 **MCP / 掌部偏移** | Quest 已做腕帧+MANO，Retargeter 再变换一次 | `landmark_preprocess:=raw`；手套节点不做 MANO |
| Q3 | Quest3 调参橙/白 MCP 仍偏 | Adaptive 不约束 MCP | quest3 yaml：`snap_mcp_to_robot: true` |
| Q4 | 真机 **收不到** `joint_commands` | BEST_EFFORT vs RELIABLE | 对齐 SensorDataQoS BEST_EFFORT |
| Q5 | 真机未用上调参 yaml | real pipeline 仍指向手套 yaml | `input_source:=quest3` 自动传 quest3 yaml |
| Q6 | dexpilot/official 仿真握拳、不跟手 | MJCF `kp` 过小 | tuning/sim 写 `qpos`+`mj_forward` |

### 操作提示

- 真机与 MuJoCo **二选一**，勿抢同一 `joint_commands`
- `*_serial` 可空（按 `hand_side` 连）
- USB：`0483` 需 udev `MODE="0666"`
