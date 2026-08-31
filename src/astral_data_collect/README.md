# astral_data_collect — VLA 数据采集包

为 OpenPI pi0.5 等 VLA 模型采集训练数据。与遥操作链路并行运行，四段式：
**采集（raw HDF5）→ 离线对齐（固定 fps 网格）→ 清洗校验 → LeRobot v2.1 导出**，
外加 Rerun 可视化回放。设计文档见 `docs/superpowers/specs/2026-08-28-astral-data-collect-design.md`。

```
遥操话题 ──┐
           ├─► data_collect_node ─► ~/astral_data/{session}/episode{N:06d}/
streamer 抽头 ┘     ▲                    robot_data.h5 + camera_data.h5 + meta.json
        keyboard_controller（热键→ /data_collect/control）

离线：align_data → validate_data → convert_to_lerobot → replay_rerun
```

## 1. 前置条件

1. 遥操作链路运行中（`astral_teleop` / 真机或仿真）
2. `quest3_video_streamer` 运行中且 `collect_tap: true`（默认开）——
   抽头向 `/quest3_video_streamer/collect/{label}` 发全分辨率 JPEG，
   **仅在有订阅者时编码**，不采集时零开销
3. 依赖：`pip install h5py pyarrow`（采集只需 h5py；导出需 pyarrow+av；回放需 rerun-sdk）

## 2. 采集前必查（schema 冻结进每段 meta.json）

`config/data_collect.yaml` 或 launch 参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `arms` | `[left, right]` | 启用臂侧：`["left"]` 仅左臂 / `["right"]` 仅右臂 / 双臂；launch 里用逗号串 `arms:=left` |
| `end_effector_left/right` | `gripper` | 左右末端：`gripper` / `wuji` / `none`（跟随所属臂，臂未启用不生效） |
| `include_waist` | `false` | 腰 2 维（来自 `/astral/joint_states[14:16]`） |
| `include_head` | `false` | 头 2 维（body_state[16:18]，无 body 时回退 `/head/joint_states`） |
| `cameras` | `[d435i, wrist_left, wrist_right]` | 相机 label，须与 streamer 实际生效值一致；**第一个 = 对齐参考相机**（episode 区间由其首尾帧界定）。auto_scan 下 label 是 `videoN`——直接把 d435i 那路的 videoN 写第一个即可；**videoN 编号不稳定（重启/重插会重排），每次开录前在 web 视频卡片（sysfs 名含 RealSense）或 `gate_state` 核对一次** |
| `dataset_fps` | `30` | 对齐网格与 LeRobot fps |
| `action_source` | `next_state` | `next_state`：action[t]=state[t+1]；`command`：指令流采样 |
| `hold_frames` | `10` | 对齐时末尾追加的保持帧 |
| `save_root` / `session` | `~/astral_data` / `default_task` | 存储位置 |

state 向量布局（顺序固定，仅含启用项）：`[left_arm(7)?, right_arm(7)?, left_ee?, right_ee?, waist(2)?, head(2)?]`
- `arms` 决定录哪些臂（单臂/双臂）；末端块跟随所属臂
- 末端=gripper：1 维闭合比（0=开 1=合，同 pi0.5 dim6 约定）。**夹爪无反馈，state 用指令回显**。
- 末端=wuji：20 维关节反馈；command 模式下 action 用 `/left|right_hand/joint_commands`。
- 腰/头在采集侧无独立指令话题，`command` 模式下这两块仍是 next_state 语义。

维度速查：双 gripper=16；gripper+wuji+头=37；双 wuji+腰+头=58；单臂 gripper=8；单臂 wuji+腰+头=31。

## 3. 采集操作

```bash
# 终端 1：采集节点
ros2 launch astral_data_collect data_collect.launch.py \
    session:=pick_place end_effector_right:=wuji include_head:=true

# 终端 2：键盘（也可只发话题，见下）
ros2 run astral_data_collect keyboard_controller
```

**web 端操作（推荐）**：`astral_web_monitor`「系统」tab 启动 **Data collect** 预设
（或 CLI 启动节点均可），「监控」tab 顶部的**数据采集卡片**提供完整控制：
开始/停止保存/下一段/暂停继续/丢弃按钮 + 下一段任务文本 + 实时状态徽标与流率。
三种控制面（web 卡片 / 键盘 / 话题）完全等价，可混用。

热键：`s` 开始 / `q` 停止保存 / `d` 丢弃当前段 / `n` 保存并开新段 /
`p` 暂停继续 / `t` 输入下一段任务文本 / `ESC` 退出键盘（不影响采集）。

等价话题接口（UI/脚本可接）：

```bash
ros2 topic pub --once /data_collect/control std_msgs/msg/String "{data: 'start'}"
ros2 topic pub --once /data_collect/task std_msgs/msg/String "{data: '把方块放进盒子'}"
ros2 topic echo /data_collect/state   # latched JSON：状态/段号/各流频率/丢弃计数
```

## 4. 离线流水线

```bash
# ① 对齐（raw → aligned_data.h5，严格 1/fps 网格）
ros2 run astral_data_collect align_data -- --session ~/astral_data/pick_place

# ② 校验（规则体检 + 报告；--apply 把 fail 段移入 quarantine/，移动不删除）
ros2 run astral_data_collect validate_data -- --session ~/astral_data/pick_place --apply

# ③ 导出 LeRobot v2.1（OpenPI 直接可读）
ros2 run astral_data_collect convert_to_lerobot -- \
    --session ~/astral_data/pick_place --output ~/astral_data/lerobot/pick_place

# ④ 回放（Rerun：图像 + 关节曲线 + 时间轴）
ros2 run astral_data_collect replay_rerun -- --session ~/astral_data/pick_place --episode 0
```

校验规则（fail 必须处理 / warn 人工过目）：F1 文件缺失、F2 流缺失或为空、
F3 NaN/Inf、F4 时间戳非递增、F5 时长 <2s；W1 频率不足、W2 采样空洞、
W3 关节跳变 >0.5rad、W4 JPEG 不可解码、W5 armed 覆盖率 <50%、W6 对齐偏差大。
报告写在 `{session}/validation_report.json`。

## 5. LeRobot v2.1 导出格式

布局与 `VLA/lerobot @0cf86487`（OpenPI pyproject 钉死提交）逐字段一致：
`meta/info.json`（codebase_version=v2.1、features、data/video_path 模板）、
`meta/{tasks,episodes,episodes_stats}.jsonl`、`meta/stats.json`、
`data/chunk-000/episode_XXXXXX.parquet`、`videos/chunk-000/{key}/episode_XXXXXX.mp4`。

- parquet 视频列 = `struct{path: string, timestamp: float32}`（VideoFrame 同构）
- 视频编码默认 libsvtav1（crf=30, g=2, yuv420p），不可用自动回退 h264
- 图像原始分辨率入库，resize 到 224×224 由 openpi 训练 transform 完成

OpenPI 侧配置示例（state/action 维度 = 你的 schema 实维，padding 在 openpi config 里做）：

```python
# src/openpi/training/config.py 你的 TrainConfig：
data=LeRobotAlohaDataConfig(
    repo_id="<local path or pushed repo>",
    default_prompt="pick up the cube",
    # repack/transform 里把 state/action pad 到模型 max_dim（如 32）；
    # 若 schema 维度 > max_dim（如双 wuji 58 维），需调大模型 max_action_dim。
),
```

验证装载（在 openpi 环境内，金标准检查）：

```bash
cd VLA/openpi && uv run python -c "
from openpi.training.data_loader import create_data_loader
... # 或直接用 lerobot LeRobotDataset(root=...) 抽帧比对
"
```

## 6. 数据格式（raw）

```
episode{N:06d}/
├── robot_data.h5   /streams/{name}/values (N,D) f4 + timestamps (N,) f8
├── camera_data.h5  /{cam}/images (N,) vlen u8 JPEG + timestamps f8
└── meta.json       schema 快照 / task / events（armed 跃迁、pause）/ 流统计 / 丢弃计数
```

时间戳统一 PC 墙钟秒：`header.stamp` 有效（sec>0）优先，否则到达时刻。
所有话题同机同源同钟，跨流对齐误差 ≪ 1/fps。

aligned_data.h5：`observation/state (T,D)`、`action (T,D)`、`timestamps`、
`quality`（bit0=图像偏远 bit1=数值空洞）、`{cam}/{images,src_timestamps,src_offsets}`。

## 7. 测试

```bash
cd src/astral_data_collect/test
/usr/bin/python3 -m pytest test_schema.py test_align.py test_validate.py \
    test_convert_lerobot.py -p no:anyio            # 离线 35 项
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m pytest test_collect_smoke.py -p no:anyio   # 采集冒烟（假话题全流程）
```

注意用 `/usr/bin/python3`（系统 3.10 有 rclpy）；conda python3.13 无法 import rclpy。
