# astral_policy_inference — 策略部署 / 数据真机回放 / 人在环路（HITL）

一个 ROS 2 Humble 功能包，对 **遥操数采（`astral_arm_teleop` + `astral_data_collect`）→
训练（OpenPI pi0.5 / LeRobot ACT）** 的上游做推理闭环：

- **策略部署**：VLA 策略跑在真实机器人/仿真上，模型无关换权重（后端抽象）。
- **推理节奏**：同步 `queue_sync`、异步 `queue_async`、实时分块 `rtc` 三种引擎模式。
- **全链路适配**：与数采同一 `CollectSchema` → 观测订阅 / state 向量 / 指令拆分全自动，
  换机器人配置 = 换和采集时一样的 yaml 段落。
- **数据真机回放**：已采集 episode（`aligned_data.h5` 或 LeRobot v2.1 目录）绝对动作回放，
  训练好的模型和采集的数据走同一条执行链。
- **人在环路**：策略暂停 → VR 遥操**增量接管**（自动重新记录机器人原点/VR 位姿）→
  控制权返回策略/回放，全程可重复。

```text
              ┌──────────── 数采/训练上游（本包不生成数据）─────────────┐
   astral_arm_teleop  + astral_data_collect  ──►  LeRobot v2.1 / aligned_data.h5
              ▲            │                                    │
              │ ~/reanchor/ disarm/start                         ▼
              │ (增量接管)                              ┌──────────────────┐
              │                                        │  astral_policy_inference │
              │          joint_states / gripper_ratio  │  policy_node       │
   robot ◄────┼────────────────────────────────────────│      │ keyboard    │
   (arm driver│  /left_arm/joint_commands ...          └──┬──────────┬─────┘
    / sim) ◄──┘                                           │          │
                                   PolicyBackend ◄────────┘          │
                          openpi websocket | lerobot ACT | stub     replay
```

## 目录 / 包内部结构

```text
astral_policy_inference/
├── robot_io.py     ObsLayout：由 CollectSchema 生成订阅表 / 观测向量 / 指令拆分
├── executor.py     SafeExecutor：NaN/限位/速率安全层（纯逻辑，可单测）
├── backend.py      PolicyBackend：openpi(远程 ws) / act(进程内 lerobot) / stub
├── engine.py       ActionEngine：queue_sync / queue_async / rtc 三种分块引擎
├── controller.py   Controller：IDLE/POLICY(_PAUSED)/PLAYBACK(_PAUSED)/HUMAN FSM
├── replay.py       读取 aligned_data.h5 / LeRobot v2.1 目录，PlaybackSession 步进
├── node.py         policy_node：装配以上全部 + HITL 仲裁 + 控制定时器
├── keyboard.py     键盘驱动（s/y/空格/n/h/g/x）
├── config/policy_inference.yaml
└── launch/policy_inference.launch.py
```

## 快速开始

### 1) 运行节点

```bash
cd /home/robot/loopkok/sdk/astral_ws
ros2 launch astral_policy_inference policy_inference.launch.py \
    backend_type:=stub \
    keyboard:=true
```

不带 `keyboard:=true` 时，可用以下任意途径发命令：

```bash
ros2 topic pub /policy_inference/cmd std_msgs/msg/String "data: 'policy'" -1
ros2 topic pub /policy_inference/cmd std_msgs/msg/String "data: 'playback'" -1
ros2 topic pub /policy_inference/cmd std_msgs/msg/String "data: 'pause'" -1
```

节点状态 JSON 发布在 `/policy_inference/state`（latch）：

```bash
ros2 topic echo /policy_inference/state
```

### 2) 换模型 = 只改 backend 参数（其余配置零改动）

| 后端 | 改什么 | 说明 |
|---|---|---|
| `backend_type: "act"` | `checkpoint_dir` 指向新 checkpoint 目录 | 进程内 `PreTrainedPolicy.from_pretrained`；换 ACT 权重/归一化/分块即换目录 |
| `backend_type: "openpi"` | GPU 主机上换 `serve_policy.py` 加载的 checkpoint | 机器人端只填 `host/port`，**不碰**；server 换模型即可 |
| `backend_type: "stub"` | — | 无网络冒烟 / 开发用，勿上真机 |

换**机器人配置**（加右臂/换灵巧手/加腰头）时，改 `config/policy_inference.yaml` 顶部 robot
段，使其与采集当时的 `data_collect.yaml` 一致——观测布局/指令拆分自动跟随。

### 3) 键盘操作一览

| 键 | 动作 |
|---|---|
| `s` | 启动策略（需新鲜观测；缺图会告警不阻断） |
| `y` | 启动回放（用 `replay_source`/`replay_episode` 参数）；`y <path>[:<ep>]` 指定来源 |
| `t <文本>` | 设置语言指令 |
| `空格` | 暂停（机器人保持最后指令） |
| `n` | 恢复（策略：按当前实况重新规划；回放：重锚定偏移后续播） |
| `h` | 人在环路接管（对每个遥操节点调 `~/reanchor`，成功后切 HUMAN） |
| `g` | 交还控制权（回策略/回放继续） |
| `x` | 停止 → IDLE |
| `q` | 退出键盘 |

**接管语义**：`h` 先执行 re-anchor（机器人原点 ← FK(当前关节)，VR 零点 ← 当前 VR 位姿），
遥操以增量方式从当前姿态接管——不会跳变；任一遥操节点 re-anchor 失败则**不进入** HUMAN，
策略继续运行（接管为事件驱动：node 的定时器轮询 re-anchor 服务结果，成功后原子提交 HUMAN，
中途任何时刻失败都会重新 disarm 已武装的遥操节点，不会出现双写）。`g` 返回：若之前是策略 →
按当前实况重新规划续跑；若是回放 → 从实际位姿重锚定 offset 继续剩余帧。

**仲裁 / disarm 电平语义**：`/teleop/disarm` 是**电平闩锁**——`true` = 外部所有者（策略/Web 暂停）
占用指令流，`false` = 放行。policy_node 进入 POLICY/PLAYBACK 发 `true`，进入 HUMAN 或停止回
IDLE 发 `false`。arm/head 遥操与 web 都只在 `true` 时 disarm（`false` 不会误伤刚 re-anchor
武装的遥操），夹爪遥操门控同时监听 `disarm`(true 停发) 与 `/teleop/armed`(true 恢复)，因此
Web 的 pause/resume 与策略接管/交还互不踩踏。

**夹爪仲裁**：policy 运行期间若夹爪遥操（`astral_gripper_teleop`）也在跑，其
`gripper_teleop.yaml` 默认已设 `disarm_topic: "/teleop/disarm"` + `arm_topic: "/teleop/armed"`
——policy 持有指令话题时夹爪遥操停发；进入 HUMAN/IDLE 后 policy_node 发 disarm=false 放行
真人捏合/扳机，Web 恢复（armed=true）同样放行。

### 4) 回放采集数据

```bash
# LeRobot v2.1 数据集目录（语法 playback:<source>[:<episode>]）
ros2 topic pub /policy_inference/cmd std_msgs/msg/String \
    "data: 'playback:/data/datasets/xxx:0'" -1
# 或 aligned_data.h5 单文件（对齐流水线产物）
ros2 topic pub /policy_inference/cmd std_msgs/msg/String \
    "data: 'playback:/data/aligned/aligned_data.h5'" -1
```

回放帧率 = `dataset_fps`；播完保持末位 `replay_hold_s` 秒后自动回 IDLE。进入回放前会先校验
`action_dim` 与（h5/lerobot meta 带 schema 时）state 布局命名顺序，不一致直接拒绝并保持
当前活动——不会出现“总维数相同但块顺序不同”的静默错放。

### 5) MuJoCo 真话题端到端冒烟（无需模型/相机）

`astral_mujoco_sim` + `policy_node`（stub 后端）全链路验证仲裁与回放（覆盖 policy 驱动、
pause 夹持、resume 恢复、h5 回放逐帧一致与播完自动 IDLE）：

```bash
cd /home/robot/loopkok/sdk/astral_ws
source /opt/ros/humble/setup.bash && source install/local_setup.bash

# 先做一次 /tmp/e2e/replay.h5（h5py 写 action + attrs[fps]=30 即可，参考脚本注释）
ros2 launch astral_mujoco_sim astral_mujoco_sim.launch.py enable_viewer:=false &
install/astral_policy_inference/bin/policy_node --ros-args \
    --params-file install/astral_policy_inference/share/astral_policy_inference/config/policy_inference.yaml \
    -p backend_type:=stub -p engine_mode:=queue_sync &
/usr/bin/python3 scripts/run_inference_sim_smoke.py --replay /tmp/e2e/replay.h5
```

`ros2 run astral_policy_inference policy_node` 若报 “No executable found”，是该 colcon 布局把
console script 放进 `install/astral_policy_inference/bin/` 而无 resource index 所致，直接跑
上面 `bin/policy_node` 路径即可。

## 关键设计

- **观测/动作布局 = `astral_data_collect.schema.CollectSchema`**。state 向量按 `meta.json`
  冻结顺序拼装；`split_action` 把策略输出的绝对动作行拆回 arm/gripper/head 独立话题；
  waist 块无 v1 指令通道 → 显式 `UnsupportedActionBlockError`，拒绝猜测。
- **绝对动作行**。pi0.5/ACT 输出与采集的 next-state 同为绝对关节目标，因此 chunk 无缝替换
  无需混合；`engine` 只在“仍在执行中换新 chunk”时做延迟补偿跳行。
- **安全兜底独立于策略**：`SafeExecutor` 拦截 NaN/Inf、夹爪比值 [0,1]、关节限位与
  `max_joint_vel` 速率限制后才发话题（`max_joint_vel<=0` 关闭限速）。
- **模型无关**：`ObsBatch`（state+images+prompt）为中性输入；后端只负责输出与
  `state_dim` 同维的绝对 chunk。
- **仲裁先效应后提交（事件驱动）**：进入 HUMAN 前先对全部遥操节点发起 `~/reanchor`（效应），
  控制定时器轮询全部成功后原子提交状态机（提交）；任一失败就地保持原状态与策略引擎并重新
  disarm 已武装节点（事务性回滚）。`~/cmd` 经队列由控制定时器串行消费，与 `_tick` 共享一把
  RLock——仲裁回调与控制循环不可能交错双写。
- **运行期观测门控**：POLICY 运行中 state 反馈缺失超过 `obs_stale_stop_s` 自动暂停（保持当前
  目标），而不是继续用陈旧观测盲推盲发。

## 测试

```bash
cd /home/robot/loopkok/sdk/astral_ws
src/astral_policy_inference/scripts/run_policy_inference_tests.sh
```

覆盖：robot_io（布局/组装/拆分/图像）、executor（NaN/限位/限速/reset）、backend（openpi
payload、错误处理、工厂）、engine（三种模式、后台规划、RTC 尾切、线程安全）、controller
（FSM 全部合法/非法迁移）、replay（读取两来源、PlaybackSession 步进/重锚）、node_flow
（进程内 mock 机器人 + stub 后端 + 真实 Trigger 服务的端到端状态流、HITL 成功/失败路径）。

仿真冒烟（无需模型/相机，stub 后端 + sim 提供 joint_states/命令消费即可）。
