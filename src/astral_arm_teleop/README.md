# astral_arm_teleop

Quest3 腕部 → 双臂 IK（DH / 几何闭式 / URDF 数值）→ `/left_arm|/right_arm/joint_commands`。  
**不** import `astral_robot_sdk`；硬件由 `astral_robot_control` 负责。

当前架构是 **B：模型不动，控制层换算**。SolidWorks 的 `astral_robot_description` URDF、旧 MJCF、真机关节符号都保持原约定；闭式 DH 在「翻转轴 + 干净 MDH 基座」里求解，teleop 边界用 `flip_q` 和 `R_baseᵀ` 对接。

## 数据流（2× 单臂节点）

```text
quest3_udp_mocap (convert_to_robot:=true)
  Mixed → robot_world (X left, Y back, Z up)
  IOBT  → robot_body  (hips 系；+X右/+Y上/+Z前)
  # mocap 保证整段流 frame_id 稳定（sticky latch + hold gate）：
  # 见过 body 包即不回退世界系；启动后 head/wrist/controller 在帧确定前不发布。
  # 本节点按 "当前 - vr_init" 算增量；稳定系下增量有界、IK 可达。
  # IOBT 下增量即"手相对躯干"的位移，正是以髋为父、手身解耦的语义。

astral_arm_teleop_{left|right}                 # 进程并行
  PoseProcessor × vr_to_arm_rot
      analytic_dh: yaml(I) 再左乘 R_baseᵀ → 干净 MDH 基座
      urdf_numerical / geometric: yaml(I) 原样 → SW `left_base_link` / `right_base_link`
        （Pinocchio `T_base⁻¹ T_ee`，不再用 universe/躯干）
  → workspace_radius 球约束（IK 帧 *_base_link 原点；≤0 关闭）
  → IK
      analytic_dh: 翻转约定 q_dh（α=+90 闭式）
      geometric: 硬件约定 q_hw（URDF 几何 + POE/PK 闭式）
      urdf_numerical: 硬件约定 q_hw（Pinocchio LM）
  → SafetyFilter（关节限位 / 速度；与求解器同一约定）
  → flip_q（仅 analytic_dh）→ q_hw
  → /{side}_arm/joint_commands             # 始终旧约定
      → 真机 astral_robot_control
      → 或仿真 astral_mujoco_sim（旧 MJCF，无 flip）
```

```bash
ros2 run quest3_hand_mocap quest3_udp_mocap --ros-args \
  -p convert_to_robot:=true -p arm_side:=both -p protocol:=tcp_wired
```

## 两套约定

| | DH / analytic IK | 真机 / 旧 MJCF / URDF 数值 |
|--|--|--|
| 基座 | 干净 MDH：\(z_0 \parallel\) joint1 轴 | SolidWorks `*_base_link`（joint1 轴 ≈ −X） |
| 关节符号 | 左翻 2/3/4，右翻 2/4，使 \(\alpha=+90^\circ\) | URDF `axis xyz="0 0 1"` 原方向 |
| 谁转换 | teleop：`R_baseᵀ` + `flip_q` | 仿真/驱动直接吃 `joint_commands` |

yaml 里 `init_pose` / `vr_to_arm_rot` 都按 **旧约定 / robot_world** 写。  
单臂节点在 `solver_type=analytic_dh` 时自动：`R = R_baseᵀ @ yaml`，并对 init / seed / 输出做 `flip_q`。

## 双 IK（包内 `ik/`）

```text
astral_arm_teleop/
  ik/
    analytic.py         ← 闭式臂角 DH（AstralParams + 公式修复）
    geometric.py        ← 免 DH 臂角闭式（S/E/W 空间几何，POE + Paden-Kahan）
    urdf_solver.py      ← Pinocchio LM
    factory.py          ← make_ik_solver / make_single_arm_ik
  astral_arm_teleop_node.py   ← 自动 R_baseᵀ + flip_q
```

| `solver_type` | 实现 | 位姿帧 | 输出 q | 默认模型 |
|---------------|------|--------|--------|----------|
| `analytic_dh` | Nero 臂角闭式，硬编码 MDH | 干净 MDH 基座 | 翻转约定，发布前 flip | 无 URDF |
| `geometric`（yaml 默认） | 免 DH 臂角闭式：URDF 提取肩/肘/腕中心与关节轴，POE + Paden-Kahan；支持 `use_human_elbow` 人臂肘先验 | SW `*_base_link` | 已是硬件约定，不 flip | `astral_robot_description/urdf/astral_robot.pin.urdf` |
| `urdf_numerical` | Pinocchio + scipy LM | SW `*_base_link` | 已是硬件约定，不 flip | `astral_robot_description/urdf/astral_robot.pin.urdf` |

单臂节点 `urdf_path` 为空 → `default_astral_urdf_path()` = **`astral_robot.pin.urdf`**（与 MuJoCo 同源）。  
支持 RobotMain 命名（`Joint_la_*`）或 Astral 命名（`left_joint*`）。

### 人臂肘先验（`use_human_elbow`，仅 `geometric`）

Quest IOBT 身体追踪经 `quest3/body_joints` 发布 `{side}-arm-upper`（肩球头）/
`{side}-arm-lower`（肘）关节位置（`robot_body` 系，推算但可用）。节点取大臂
方向 `d = elbow − shoulder`，用与腕部增量相同的 `vr_to_arm_rot` 旋到机械臂基座系
（纯方向，与 zero point / motion_scale / 人臂长无关），投影到机器人 S–W 轴垂面
得到人臂角 `psi_ref`，作为软先验传入 `solve(T, psi_ref)`：

- 局部 ψ 扫描窗口以 `psi_ref` 为中心，评分加 `w_psi_ref·|ψ − psi_ref|`
  （`human_elbow_weight`，默认 2.0，须主导 w_vel=1.0 / w_theta0=0.15）；
- 效果：人转腕带动肘平面转 → 机器人肘跟转；人只转腕 → 肩肘不动；
- 鲁棒：`human_elbow_timeout`（0.3 s）外或关节名缺失自动回退纯连续性；
  肘方向 EMA（`human_elbow_smoothing_tau` 0.15 s）抑制 IOBT 噪声；
  `psi_ref` 落在关节限位不可行区时走全局扫描 + 连续性，不会卡死；
- **伸直度门控**（`human_elbow_min_sin`，默认 0.15）：人臂接近伸直时肘尖
  几乎在肩腕连线上，ψ 不可观测 —— IOBT 骨架的系统性肘偏置会被归一化
  放大成满权重先验，把机器人肘拧到固定错误角度（典型症状：手臂放下时
  肘部固定偏 ~90°）。低于门限自动关闭先验回退纯连续性，被门控的帧计入
  `[Latency]` 行的 `psi_off`。0.15 ≈ 肘尖离线 4 cm；正常弯曲手势
  sin ≥ 0.5，远离门限。

```bash
# 求解器见 astral_arm_teleop_{left,right}.yaml（默认 geometric + use_human_elbow）
ros2 launch astral_arm_teleop astral_dual_arm_teleop.launch.py dry_run:=true
# 一次性覆盖：solver_type:=analytic_dh
```

### 肘伸直奇异（`reach_margin`，仅 `geometric`）

肘完全伸直时 S/E/W 共线：肘圆半径 → 0，臂角 ψ 失去定义，且 q4（本机
`[-2.26, 0]`，0 = 伸直）正好撞限位 —— IK 在可达边界带内无解，节点保持上一帧
指令，恢复时臂会"卡一下"。两层防护：

- **软墙**：腕目标径向钳制在肩心 `l_se + l_ew - reach_margin`（默认 0.01 m）内，
  方向不变，手感到的是一堵软墙而不是卡顿；`reach_margin <= 0` 关闭，
  可热改，钳制次数计入 `[Latency]` 行的 `reach_clip`；
- **ψ 冻结**：`sin α < 0.05`（肘圆半径 < ~1.2 cm）时 ψ 在数值上无意义
  （此时解本就与 ψ 无关），扫描网格塌缩为 `θ0_prev` 单候选，杜绝肩部噪声跳变。

### 大幅 roll 时的肩部抽搐（逃逸迟滞，仅 `geometric`）

末端 roll 幅度超过腕关节行程（q5/q6/q7 各 ±100°，纯 roll ~180° 需多关节
分担）时，**当前臂角下**腕部可能翻不到目标姿态：局部窗候选全撞限位被弃，
旧行为同帧全局逃逸到远处可行 ψ（肩部猛甩），下一帧 `psi_ref` 窗恢复可行
又拉回 —— 在可行边界逐帧振荡，真机上表现为肩部"抽搐一下又拉回来"。

逃逸迟滞（`ContinuityParams.escape_after_frames` 默认 4、
`return_after_frames` 默认 20，~150 Hz 下 ≈ 27 ms / 133 ms）：

- 局部窗**连续空 4 帧**才允许全局逃逸，边界单帧闪烁不再甩肩；
- 逃逸后驻留在逃逸区（连续性窗跟随，期间 `w_psi_ref` 把臂角向人臂方向
  缓慢牵回），探针确认人臂角**连续 20 帧**可行才正式返回 —— 一次大 roll
  最多甩一次、回一次，且都被速度限幅摊成平滑过渡；
- 冷启动（无 `θ0_prev`）不受影响，立即全局求解；`1/1` 可复现旧行为。

### 按求解器切换的 `vr_to_arm_rot` 与 `flip_q`

推荐路径 **`astral_arm_teleop_node`**（左右 yaml 里 `vr_to_arm_rot` 仍是 **I**）：

| | `analytic_dh` | `urdf_numerical` / `geometric` |
|--|--|--|
| 有效 `vr_to_arm_rot` | `_R_BASE_T @ yaml` | yaml（默认 I） |
| `flip_q` | 左 2/3/4、右 2/4 取反（自反） | 恒等 |
| Safety 限位 | DH 限位 | URDF 限位 |

`_R_BASE_T`（\(R_\text{base}^\top\)，行优先）：

```text
[ 0  0  1 ]
[ 0  1  0 ]
[-1  0  0 ]
```

把 `robot_world` / SW `*_base_link` 下的增量转到「joint1 轴 = +Z」的干净基座。与关节翻转无关。

`flip_q` 边界（仅 DH）：yaml `init_pose`（旧约定）→ flip 入 IK；warm-start `flip(q_cmd)`；输出 `flip(safe)` 再发 `joint_commands`。

## DH 参数与闭式公式（`ik/analytic.py`）

闭式解要求 SRS + \(a_i=0\) + \(\alpha=+90^\circ\)。Astral CAD 轴符号混杂，用翻转约定把链统一成 +90°，而不是改 URDF。

### 参数（相对早期照抄 Nero）

| | 改前 | 改后 |
|--|------|------|
| 左 `d_i` | `[0.11655, 0, 0.246, 0, 0.23058, 0, 0.03474]` | `[-0.11655, 0, 0.246, 0, 0.2265, 0, 0.0036]` |
| 右 `d_i` | 与左相同 | `[0.11655, 0, -0.246, 0, 0.2265, 0, 0.0036]` |
| 左 \(\theta_\text{offset}\)（deg） | Nero `[0,-180,-180,-180,90,90,0]` | `[180,90,90,180,180,-90,-90]` |
| 右 \(\theta_\text{offset}\) | 左臂别名 | `[180,-90,-90,0,180,-90,-90]` |
| `apply_base_fixed` | 默认 True（joint1 rpy hack） | **False**（干净基座由 `R_baseᵀ` 承担） |

`d4=0.2265` 是前臂长度；旧 `0.23058` 量错。`d6=0.0036` 是沿腕轴；旧 `0.03474` 把横向视觉偏置误当 MDH \(d\)。

### 公式修复（与 `_dh_A` 的 \(\alpha=+90^\circ\) 对齐）

1. **`_solve_q123_from_swe`**：`c2=(d0-Ez)/d2`；`c1/s1` 去负号；`th3=atan2(w[1]/s4, w[0]/s4)`；返回满角。Nero 曾靠 offset=−180° 碰巧盖住 \(\alpha\) 符号错配。
2. **`_solve_theta4_from_triangle`**：按 `d_i[0]` 符号分左右——左 `-acos(c4)`，右 `+acos(c4)`（镜像肘）。

在「±q + 干净基座」映射下，DH FK 与 URDF FK 约 0.05 mm / 0.001°；SELF_IK 自洽。直接在 SW `*_base_link` 同 q 对比会差数百 mm，那是约定不一致，不是杆长算错。

### DH 限位（翻转约定，已放宽）

左右臂统一：

| J1 | J2 | J3 | J4 | J5 | J6 | J7 |
|----|----|----|----|----|----|----|
| [-2, 2] | [-2, 2] | [-1.57, 1.57] | [-1.012, 2.26] | [-1.57, 1.57] | [-0.8, 0.85] | [-1.57, 1.57] |

`init_pose` 仍为旧约定，例如左 `[0.32, 0.11, -0.53, -0.80, 0.28, 0, 0]`。  
注意：DH 限位 flip 到硬件后，J2/J4 可能宽于旧 URDF/MJCF（左 J2 `[-0.3,2]`、J4 `[-2.26,0]`）。Safety 只在 DH 空间钳位。仿真/真机仍受旧 `ctrlrange` 约束。

## 仿真 / URDF / MJCF —— **未改运行时模型**

| 文件 | 是否改控制闭环 |
|------|----------------|
| `astral_robot_description`（`astral_robot.pin.urdf`） | **否**，仍 SW 原轴 |
| `astral_mujoco_sim` 默认 `assets/mjcf/astral_dual.xml` | **否** |
| `astral_mujoco_sim.yaml` 的 `init_pose_*` | **否**（旧约定，与 teleop yaml 一致） |

仿真吃的是 flip 之后的 `q_hw`，自己不再 flip。  
**不要**把默认 `mjcf_path` 指到 `astral_dual_clean.xml`，除非整条链都改成干净约定。

## 可选包与工具（不影响默认闭环）

| | 作用 |
|--|--|
| `astral_arm_clean_description` | 干净单臂/双臂 URDF（关节原点=MDH，mesh 用 visual origin 重定位）。架构 B 下 **不接入** teleop/仿真 |
| `astral_mujoco_sim/assets/mjcf/astral_dual_clean.xml` | 对应干净 MJCF；默认不用 |
| `derive_clean_mdh.py` | 从 URDF 抽 MDH 诊断 |
| `generate_clean_urdf.py` | 生成干净 URDF |
| `generate_clean_mjcf.py` | 生成干净 MJCF |
| `fit_dh_from_urdf.py` | DH 拟合审计 |

```bash
python3 -m astral_arm_teleop.fit_dh_from_urdf
python3 -m astral_arm_teleop.derive_clean_mdh
python3 -m astral_arm_teleop.generate_clean_urdf
```

## 依赖

```bash
# URDF IK:
# pip install pin

cd /home/robot/loopkok/sdk/astral_ws
export PATH=/usr/bin:$PATH
source /opt/ros/humble/setup.bash
colcon build --packages-select astral_robot_description astral_robot_control astral_arm_teleop --symlink-install
source install/setup.bash
```

## 启动

```bash
# 2× 单臂 teleop（自动 R_baseᵀ + flip_q）
ros2 launch astral_arm_teleop astral_dual_arm_teleop.launch.py dry_run:=true
ros2 launch astral_arm_teleop astral_dual_arm_teleop.launch.py \
  with_driver:=true control_board_ip:=192.168.10.2
# 单左臂：arm_side:=left（right 对称）。整机左臂+夹爪见 astral_teleop arm_side:=left
# 纯臂：不含夹爪 / 灵巧手。整机编排见 astral_teleop/full_teleop.launch.py

# 仿真管线（solver / protocol 见 yaml，默认 geometric + tcp_wired）
ros2 launch astral_mujoco_sim astral_sim_pipeline.launch.py
# 一次性覆盖：solver_type:=analytic_dh  protocol:=udp
```

Quest3 有线：`adb reverse tcp:8000 tcp:8000`。  
**不要**与 `astral_robot_control` 和 `astral_mujoco_sim` 同时订同一 `joint_commands`。

左手夹爪（独立包 `astral_gripper_teleop`）：`hand_landmarks/left` 拇指–食指距离 → `/left_gripper/command`（0 开 1 合）。真机由驱动 `set_gripper_angle` 下发。本包不再 include 夹爪；整机编排见 `astral_teleop`，单独跑夹爪用 `ros2 launch astral_gripper_teleop gripper_teleop.launch.py`。

## 外部启动闸门（`require_start_signal`）

**问题**：Quest3 端点 "start stream" 时手得抬起来点按钮，推流一来第一帧腕姿就在按钮位置，旧逻辑把第一帧当 `vr_init`（零点），于是零点错位、机器人一上手就偏。

**新流程**（`require_start_signal:=true`）：

1. 节点启动后**不**自动记 `vr_init`、**不** arm，只跟踪 `vr_current`（PoseProcessor `auto_calibrate=false`）。
2. homing 走完后机器人停在 `init_pose`；你把手摆到与机器人一致的初始位姿。
3. **外部发一个 start 信号** → 节点用**当前** `vr_current` 记 `vr_init` 并 arm，遥操开始。

信号入口（任选）：

```bash
# 全局，双臂同启（推荐）
ros2 topic pub --once /teleop/start std_msgs/msg/Bool '{data: true}'

# 单臂 + 带反馈
ros2 service call /astral_arm_teleop_left/start  std_srvs/srv/Trigger
ros2 service call /astral_arm_teleop_right/start std_srvs/srv/Trigger
```

再发一次 `/teleop/start` = 用当前 pose **重新记零点**（re-center，不解除 arm）。`/teleop/disarm` 仍可暂停，`/teleop/armed` 只 arm 不重记零点（未标定时仍不发指令）。

启动时若仍在 homing 或还没收到腕姿，start 会被拒并告警（等 homing 完成、Quest 推流后再发）。

```bash
# 真机整机：外部启动闸门（yaml 已默认 true，此处 require_start_signal:=true 可省略）
ros2 launch astral_teleop full_teleop.launch.py \
  with_arm_driver:=true with_hand_driver:=true \
  right_hand_source:=quest3
# 手摆好后：
ros2 topic pub --once /teleop/start std_msgs/msg/Bool '{data: true}'
```

默认值：**真机 yaml `require_start_signal: true`**（启动推流后手摆好再发 start）；**sim 同样默认 true**（launch arg 默认 `""` 透传 yaml）。yaml `require_start_signal` 或 launch arg 都可覆盖（launch arg 非空时覆盖 yaml）。

## 频率

默认遥操 / 驱动均为 **50 Hz**。有效落板频率 ≈ `min(teleop, driver)`。

## 测试

| 命令 | 作用 | 依赖 |
|------|------|------|
| `ros2 run astral_arm_teleop test_ik_solver` | DH/URDF 离线 FK↔IK、双臂、安全滤波 | 无（URDF 需 pin） |
| `ros2 run astral_arm_teleop test_geometric_ik` | 几何闭式 vs Pinocchio FK、回环、限位、工厂 | pinocchio |
| `ros2 run astral_arm_teleop test_dh_urdf_fk` | 同 q、SW `*_base_link` 下 DH vs URDF（未映射会很大） | pinocchio |
| `ros2 run astral_arm_teleop fit_dh_from_urdf` | 轴几何 / MDH 拟合 | pinocchio |
| `ros2 run astral_arm_teleop test_vr_mapping` | PoseProcessor、TCP | 无 |
| `ros2 run astral_arm_teleop test_dataflow` | 订腕姿跑管线，FPS/IK 统计 | Quest 或键盘 |
| `ros2 run astral_arm_teleop test_safe_teleop` | 保守参数发 `joint_commands` | 可选驱动 |
| `ros2 run astral_arm_teleop keyboard_vr_sim` | 键盘发 `quest3/*_wrist_pose` | 无 |
| `ros2 run astral_arm_teleop body_joints_sim` | 假人臂：发 `quest3/body_joints`+腕姿，测 `use_human_elbow`（scenario: elbow_circle / wrist_spin / combined） | 无 |

对齐 DH 与 URDF 时必须带：**关节 ±q（左 2/3/4、右 2/4）** 和 **干净基座 `Tbinv`**，不能只跑 `test_dh_urdf_fk` 的同 q 直比。

```bash
ros2 run astral_arm_teleop test_ik_solver
ros2 run astral_arm_teleop keyboard_vr_sim --ros-args -p arm_side:=left
ros2 run astral_arm_teleop test_dataflow --ros-args -p arm_side:=left
```

## 小范围跟手调参（曲线）

遥操节点默认发 `/teleop/{left|right}/tune/`：

| 话题 | 含义 |
|------|------|
| `ee_vr` | 手：VR 增量 × `motion_scale`，**未**平滑 |
| `ee_filt` | 滤波后的 IK 目标 |
| `ee_cmd` | 指令关节的 FK 末端 |
| `xyz` | 上面三个位姿打成一条（绘图用） |

仿真起来后另开终端：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run astral_arm_teleop teleop_tune_plot --ros-args -p arm_side:=right
```

窗口里：**蓝=手、橙虚线=滤波、绿=末端**。先静止按 `z` 清零，再做 1–2 cm 来回。

左下角 **DH analytic / URDF LM** 可热切换（切完重定 VR 零点，臂停在当前姿态）。  
左侧滑条两套 IK 都生效：`pos_smooth` / `rot_smooth` / `motion_scale` / `max_vel` / `workspace_radius`。  
右侧滑条仅 URDF LM：`ik_w_pos` / `ik_w_ori` / `ik_max_iter` / `ik_tol` / `ik_w_reg` / `ik_q4_max` / `ik_w_limit`。DH 是闭式，改右侧无效。

| 曲线 | 说明 |
|------|------|
| 绿贴蓝 | 跟手 |
| 静持时绿抖、蓝平 | IK/控制抖 |
| 蓝自己抖 | Quest 噪声 |
| 绿圆滑但落后蓝 | `pos_smoothing` 太大 |
| 蓝幅值对、绿跟不上 | 限速 / IK |
| URDF 位置跟、姿态飘 | 加大 `ik_w_ori` |
| URDF 整臂拧出怪姿势 | 减小 `ik_w_ori`，或先确认 `ik_w_reg` 够大 |
| URDF 发黏、小动作被吃 | 减小 `ik_w_reg` |
| URDF 构型乱跳、姿势怪 | 加大 `ik_w_reg`（默认 1e-4；贴上一帧时曾用 0.02） |

右侧数字：`pos err RMS/p95`、静持 `hold jitter`、`lag`。`s` 存 CSV 到 `/tmp`。yaml 只是下次启动的初值。

也可用 PlotJuggler 订 `ee_vr` / `ee_cmd` 的 `pose.position.{x,y,z}`。

## 参数

单臂：`config/astral_arm_teleop_{left,right}.yaml`。

常用：`solver_type`、`urdf_path`、`motion_scale`（默认 0.65）、`vr_to_arm_rot`（yaml 默认 I；DH 时节点自动乘 `R_baseᵀ`）、`init_pose`（旧约定）、`init_waypoints`（扁平 7×N 途经点，硬件约定，先按序走到再去 `init_pose`；空=直接到 init）、`move_to_init_pose`（启动低速走到 `init_pose`，默认开）、`init_speed_percent`（默认 10，相对 `max_joint_vel`）、`max_joint_vel`（rad/s）、`workspace_radius`（末端相对 `*_base_link` 原点的球半径，m；yaml 默认 0.55，≤0 关闭；可热改）、`reach_margin`（肘伸直软墙，腕目标钳在肩心 l_se+l_ew−margin，默认 0.01 m，≤0 关闭；可热改）、`ik_q4_max`（URDF IK 肘关节上限，默认 −0.45；URDF 硬件上限是 0=伸直奇异，≥0 关闭此帽）、`ik_w_fold` / `ik_q4_fold`（单向折肘，比 fold 更直才罚）、`ik_w_limit`（近限位软约束；短边按到 0 的余量缩 margin）、`ik_w_ori`（姿态权重，默认 0.3）、`ik_w_reg`（贴上一帧，默认 1e-4）、`ik_dq_max`（每拍关节步进盒 rad，默认 0=关）、`ik_w_pref`（冗余时往 init_pose 靠，默认 0=关）、`pos_smoothing` / `rot_smoothing`（0–1，按 50 Hz 标定，与 `control_rate` 无关）、`require_start_signal`（真机 yaml 默认 true、sim launch 默认 false；true 时等外部 `/teleop/start` 或 `~/start` 服务记 `vr_init` 并 arm，见上节）。

`use_joint_state_seed`：单臂节点会订 `joint_states` 但控制环目前仍用 `q_cmd` 做 warm-start（开环种子）。数值 IK 同样用上一帧 `q` 作 LM 初值。
