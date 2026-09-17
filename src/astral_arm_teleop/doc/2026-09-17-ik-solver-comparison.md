# 2026-09-17 Astral 逆解求解器对比测试（geometric vs urdf_numerical）

> 测试记录归档：`/home/robot/loopkok/sdk/astral_test_logs/2026-09-17_ik_solver_comparison/`
> （数据、分析脚本、摘要）；本文件为详细分析结论。

## 1. 背景与现象

真机遥操分别用 `solver_type:=geometric` 和 `:=urdf_numerical` 各测一段，操作者肉眼观察：

| | geometric（默认） | urdf_numerical |
|---|---|---|
| 快速移动 | 流畅 | 流畅 |
| **慢速小范围移动** | **一顿一顿（肉眼明显）** | 相对平滑 |
| 前伸 | 有时"伸不直"（肘被限制） | 能伸（但位置误差更大） |
| 肘部 | 贴人臂角（好） | **乱跑**（但末端跟得上） |

**核心问题**：同样是慢速移动，为什么 geometric 一顿一顿而 urdf 平滑？

## 2. 测试方法演进

排查分四层，逐步缩小范围：

1. **上游指令层**：检查 Quest→遥操的腕位流质量（遥操 JSONL 的 `kind=wrist`）。
2. **遥操链路层**：遥操 JSONL 逐级目标（`vr→filt→cmd`）求速度/误差。
3. **驱动层（SDK 直连）**：绕过 ROS/IK，用 `astral_robot_sdk` 直接对单个关节发
   **干净平滑轨迹**（`repro_stickslip.py`），排除上游一切因素，确认驱动层行为。
4. **求解器对比层**：同段遥操分别用两种 solver 录制（本测试），量化差异。

配套工具（均已在 astral_robot_sdk / astral_arm_teleop 落地）：
`repro_stickslip.py`（驱动复现）、`read_gains.py`（读电机 PID/增益）、`set_pid.py`（写增益）、
遥操 JSONL 诊断日志（`pos_err/ori_err/psi_err/vr_vel/db_nudge` + metrics `track`）。

## 3. 量化指标定义

| 指标 | 定义 | 含义 |
|---|---|---|
| `pos_err` (mm) | \|命令 FK 位 − 滤波目标位\| | 位置跟踪误差 |
| `ori_err` (°) | 命令 FK 与滤波目标姿态夹角 | 姿态跟踪误差 |
| `psi_err` (rad) | \|θ0_sel − psi_ref\| | 臂角跟随误差（无人肘先验=null） |
| `vr_vel` (mm/s) | VR 目标速度 | 慢/快帧打标 |
| `track.slow/fast_frac` | vr_vel<40mm/s 占比 | 慢速占比 |
| `track.{pos,ori}_err_{slow,fast}_p95` | 慢/快段误差 p95 | 分段跟踪质量 |
| 关节每拍步长 (mrad) | \|Δq_j\| 每控制拍 | 是否低于电机死区 |
| 死区下占比 | 步长<1mrad 的拍占比 | 电机不执行的比例 |
| `ik_ms` | 求解耗时 | 数值 vs 解析延迟 |
| 边界事件 | ik_fail/reach_clip/hard_fallback | 求解器"停/钳/回退"次数 |

## 4. 测试数据与文件

| 文件 | 内容 |
|---|---|
| `astral_ws/inference_test_logs/teleop/20260917-144153_urdf_numeric/teleop_teleop_left.jsonl` | urdf_numerical 遥操（74.6s） |
| `astral_ws/inference_test_logs/teleop/20260917-144526_geometric/teleop_teleop_left.jsonl` | geometric 遥操（82.5s） |
| `astral_robot_sdk/demos/repro_stickslip.py` | 驱动层复现（速度扫描/增益 A/B） |
| `astral_test_logs/2026-09-17_ik_solver_comparison/analysis/analyze_solver_compare.py` | 本测试的完整分析脚本 |

## 5. 详细分析结果（含量化）

### 5.1 指令侧精度（两求解器都很"准"，但准的方向不同）

| 指标 | urdf_numerical | geometric |
|---|---|---|
| `pos_err` p95（慢速段） | **1.33 mm** | **0.03 mm** |
| `ori_err` p95 | 0.00° | 0.00° |
| `psi_err` p95 | **null（不追人肘）** | 0.0000 rad（硬跟人肘） |
| `ik_ms` 中位 | **7.23 ms**（迭代） | **1.11 ms**（闭式） |

→ geometric **精确解析**（位置/姿态/臂角全严格满足）；urdf **LM 加权**（位置权重 1.0、
姿态 0.3），位置误差 1.33mm、臂角完全不受控（=肘乱跑的数字证据）。`ori_err` 两都≈0 是因为
目标可达时 LM 也能收敛姿态。

### 5.2 慢速死区交互（root cause：为什么 geometric 一顿一顿）

**电机最小可靠步长 ≈ 1 mrad**（驱动层实测：`repro_stickslip` 速度扫描，调 speed 增益
0.04→0.1 无效 → 锁定机械/固件死区，非增益可解）。

匹配速度段（vr 20-35mm/s，两 log 前 40%）对比关节指令每拍步长：

| | urdf_numerical | geometric |
|---|---|---|
| 全关节每拍总步长 | **123 mrad** | **10 mrad** |
| j0 肩pitch / j3 肘pitch | 17.8 / 29.9 mrad | 0.98 / 1.45 mrad |
| 各关节 | 全 >5 mrad（远超死区） | **全 0.9~1.7 mrad（压死区边缘）** |

死区下（<1mrad）占比（慢速帧，整段）：

| | urdf_numerical | geometric |
|---|---|---|
| **整臂 7 关节全 <1mrad**（电机一步不走） | **7%** | **24%** |
| 至少一个关节 <1mrad | 70% | 90% |
| 单关节 <1mrad（最差 j5） | 34% | **69%** |
| 各关节步长中位 | 1.8~5.5 mrad | 0.55~0.97 mrad |

**机制**：geometric 精确解 + 连续性评分**最小化关节速度** → 慢速目标时把指令优化到
每关节仅 ~1 mrad/拍，正好压在电机死区上 → 死区以下电机不执行 → 攒误差 → 跳一下 →
**一顿一顿**。urdf 的 LM 无臂角约束 + 收敛差（ik_sat=79）→ 为同一目标**挥霍 12 倍关节
运动** → 每拍远超死区 → 电机持续动 → **平滑**（但肘乱跑、误差 1.33mm、关节指令躁）。

### 5.3 边界行为（前伸"伸不直" + 慢速边界卡）

geometric 慢速前伸到肘伸直边界时，防护机制轮流触发（**81 次事件，全部集中在慢速帧
7mm/s vs 非事件 64mm/s**）：

| 事件 | 次数 | 含义 |
|---|---|---|
| `reach_clip` | 26 | 腕目标钳在可达球内（**事件帧腕距肩 474mm ≥ 满伸 473mm**=伸直边界），径向停住 |
| `ik_fail` | 9 | 无解 → 保持上一帧（硬停） |
| `hard_fallback` | 46 | 近伸直人肘不可行 → 回退（肘"拧一下"） |

`ik_q4_max=-0.45`（肘 pitch 上限，0=伸直）让肘永远至少弯 0.45 rad，**丢 ~12mm 行程** →
"伸不直"。urdf 无钳制、无硬人肘钉 → 目标超界也照跟 → 平滑（靠 1.33mm 误差吞掉）。

### 5.4 前置排查结论（为什么排除其他因素）

| 排查项 | 结论 |
|---|---|
| Quest 腕位 F4 量化 0.1mm | 真实缺陷（指令侧），但 0.1mm < 执行精度，**非主因** |
| OBS 测量量化 1 mrad（100Hz） | 放大"停-跳"表象（速度测量量子=100mrad/s），非主因 |
| 0x95 速度前馈 | 无效（板卡 MIT 未接入电机环，MIDCTRL 增益全 0） |
| 0xC4 摩擦补偿 | 无效（且 cfg 写入疑似未生效，读回 0） |
| 电机 speed 环 kp 0.04→0.1 | 无效 → 死区非增益可解（机械/固件） |
| 速度扫描 0.05→0.5 rad/s | 滞后 84→63 mrad 缓慢下降、**无死区断崖** → 不是指令量化阈值 |

## 6. 结论

1. **geometric 慢速一顿一顿 = 电机死区（~1 mrad）与 geometric"最小关节速度"优化的交互**：
   慢速指令正好压在死区边缘（24% 整臂 <1mrad），电机不执行 → 停-跳。
2. **urdf 平滑 = 以"浪费"换"动"**：关节运动 12 倍于实际需要，每拍超死区 → 电机持续动，
   代价是肘乱跑、1.33mm 误差、关节指令躁、ik_ms 7ms。
3. 这不是 IK 精度问题（geometric 更准），是**驱动层死区**被 geometric 的"高效"暴露。
4. 死区在机械/固件层（调增益无效），SDK 层无法根治，只能缓解。

## 7. 修复与验证（`cmd_deadband_mrad`）

方案：**指令最小步长地板**——慢速运动（VR 速度 ≥ `cmd_deadband_min_vel` 10mm/s）中，把
`0<|dq|<floor` 的关节步抬到 floor（方向保持），让电机每拍都有步长去跟。`0=关`。

| 验证（节点 dry_run） | floor=0（默认） | floor=1.0 |
|---|---|---|
| 非零关节步 <1mrad | 98%（原行为） | **0.2%** |
| 最小非零步长 | <1mrad | 正好 1.000 mrad |
| `db_nudge` | 0 拍 | 59/60 拍 |

副作用（文档明示）：慢速带轻微恒定微动（"蠕"）、TCP 可能略超目标速度。真机 A/B 建议
1.0→1.5 找手感。

## 8. 遗留

- 电机死区本身在机械/固件层 → 需联系板卡厂商（调 speed 环无效已锁定，疑似减速器间隙或
  固件指令死区）。
- 数值求解器仍无臂角约束（肘乱跑）→ 可加 `w_psi·|psi−psi_ref|` 正则（LM 侧），作为
  geometric 精确解失效时的兜底。
- 边界处理（reach_clip/ik_q4_max/hard_fallback）仍为启发式参数，未统一理论。
- 板端限位（CfgKey 0x0012/13）与 URDF 未核对同步。
