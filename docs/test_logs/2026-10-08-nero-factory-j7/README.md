# Nero 左臂原厂 J7 对照与 SDK 隔离工具 — 2026-10-08

## 现场结果（用户报告）

用户先使用原厂 Web 将左臂移动到 NEXUS 配置的初始位置，再单独操作 J7：相对当前位置增加一度、减少一度时，实际腕部正常运动，网页实际角度正常反馈。本次没有提供原厂角度轨迹或视频，属于用户现场观察；右臂尚未报告对应结果。

这证明左臂在该姿态、该负载及原厂控制链下具备小幅正反向运动能力。持续抱闸、完全不能转动或单纯通过增大 J7 归位容差解决的解释不再成立。不能据此证明其他姿态、速度与加速度下的负载能力，也不能直接定位某一 SDK 代码行。

## 软件检查

- NEXUS 归位调用计划关节模式 `move_j`，提交七轴目标一次。J7 通过 SDK `0x170` 发送有符号毫度；前六轴使用 `0x155 / 0x156 / 0x157`。
- 当前与原 xnero SDK 同样包含 J7。厂商当前公开驱动也使用七轴目标及该帧映射。现有内存传输测试能证明编码与传输调用，不能证明控制器实际执行。
- 尚没有证据可以确定是帧顺序、控制器锁存或某个模式切换导致。未更改帧顺序、自动重发、改用 JS 追赶、改固件或放宽 0.05 rad 归位容差。

厂商参考：[Nero 驱动](https://github.com/agilexrobotics/pyAgxArm/blob/master/pyAgxArm/protocols/can_protocol/drivers/nero/default/driver.py)、[Nero 编解码器](https://github.com/agilexrobotics/pyAgxArm/blob/master/pyAgxArm/protocols/can_protocol/drivers/nero/default/parser.py)。

## 主机只读检查

14:58 左右 NEXUS 驱动进程仍在运行，PID 为 2585725 / 2585727，均处于 `enabled=false, homing=false`。诊断服务返回的控制器和驱动缓存已超过 719 秒，不能用其旧角度推断原厂测试后的当前位置。CAN 接口处于 ERROR-ACTIVE，统计中无 bus-off、bus-errors 或 TX errors。

没有停止进程、开启 CAN 推送、切换模式或发送运动命令。进程存在本身不证明它们在原厂测试时发送了控制命令。

## 新增独立工具

`scripts/nexus_nero_j7_probe.py`：

- 默认只读，通过实际 SDK 接收连接观察原始位置帧和 SDK 解码角度；传输层阻止所有发送。不自动开启 CAN 推送。
- 分别检查四路位置帧，避免前六轴的新鲜数据掩盖 J7 的旧缓存。
- 只有现场显式 `--execute` 才切到 CAN/J 控制；要求七轴原已使能、健康、新鲜，且无未完成运动。
- 拒绝在已知 Nero ROS 驱动仍运行时执行。
- 从当前实测位置构造目标，J1–J6 不变，J7 相对位移不超过 1°，速度不超过 10%；默认 1° / 5%。一次 `move_j`，没有自动返回或重试。
- 记录模式与关节发送帧及传输返回、四路位置帧年龄、控制器及七轴驱动状态、目标和连续实测角度。传输成功明确不等于控制器确认。
- 执行时模式异常、反馈中断、其他关节偏移超过 0.5°或 J7 未在 5 秒内达到 0.2°范围，将请求 SDK 阻尼急停；物理急停由现场操作员保障。试验位移只允许 0.5–1°，避免小于到位容差的目标把不动误判为通过；该工具的试验容差独立于 NEXUS 归位 0.05 rad 规则。
- 成功后不失能、不自动恢复原厂模式；断开只关闭本工具的通信。测试最终姿态与恢复由现场操作员管理。

## 验证与未完成项

10 项本地隔离测试通过，覆盖只读零发送、意外发送阻止、在运行驱动互斥、缺失/过期 J7、未使能拒绝、控制模式不匹配、前六轴异常偏移、成功的一次 J7 目标、冻结 J7 失败且不重试，以及角度/限位边界。见 [隔离测试](isolated_tests.log)。这些测试未打开真实 CAN。

15:16 主机在 `f4e284f` 上也运行 10 项隔离测试并通过：[主机测试](host_tests.log)。分别对两臂仅执行工具默认只读模式，均因四路位置反馈不可用而拒绝继续，工具发送计数为零；Linux CAN TX 包计数左右分别保持 67121 / 67114 不变。原始日志和统计见 [主机结果](host_result.json)、[左臂](left_read_only.jsonl)、[右臂](right_read_only.jsonl)。这是缺流拒绝与零发送验证，不是运动或闭环通过；未自动开启 CAN 推送。

**NEXUS 真机归位根因与修复仍待 SDK 单轴对照；原厂左臂通过不等于 NEXUS 真机归位已通过。** 不要求再次完整归位来重复同样的失败日志。

### 15:29 现场工具结果：反馈正常，但尚未使能

用户默认只读运行：[原始记录](nero_j7_left_20261008_152923_875539.jsonl)。四路位置帧年龄约 0.4–1.2ms，SDK 与原始 CAN 解码一致。控制器 `ctrl_mode=3`（以太网控制），`arm_status=6`（关节抱闸未打开），`err_code=0`；七轴驱动使能均为 false。

随后显式执行：[原始记录](nero_j7_left_20261008_152949_051872.jsonl)。实际日志只有 start / initial_feedback / summary，`tx_count=0`，在初始健康检查时拒绝，未发送模式切换、运动或急停。**这不是“SDK 已发 J7 目标但不动”的测试结果，当前尚未完成该对照。** 控制器状态 6 与使能 false 只解释本次拒绝，不能用来反推之前已使能、状态正常的归位故障。

厂商状态定义：[ArmStatus.JOINT_BRAKE_NOT_RELEASED = 0x06](https://github.com/agilexrobotics/pyAgxArm/blob/master/pyAgxArm/protocols/can_protocol/msgs/nero/default/feedback/arm_feedback_status.py)。

工具改进：只读结果增加 `ready_for_trial` 与 `blocking_reason`，将有效反馈与可执行条件区分；错误显示实际 arm_status / err_code，状态 6 明确标注抱闸未释放。保留原有使能/状态检查。新增未使能且状态 6 时零运动/零停止，以及只读成功但准备未通过的回归；12 项隔离测试通过：[测试日志](readiness_tests.log)。

该段实测姿态也不等于配置初始位，例如 J4 约 -3.10°，配置目标约 75.11°。如需与原厂测试保持同姿态，应由现场操作员在原厂页面低速恢复原测试姿态，保持七轴已使能，再运行只读检查。工具按执行时的当前实测位置生成目标，不执行整臂归位。

## 由现场操作员执行的下一步

先在 NEXUS 网页停止机器人会话；不要只关闭浏览器。原厂页面确认机器人静止、七轴已使能，开启 CAN 反馈推送（如当前没有 CAN 数据），随后停止在原厂页面发送运动命令。一次测试一条臂。

```bash
cd /home/loopkok/NEXUS
source /opt/ros/humble/setup.bash
source install/setup.bash

# 只读；应看到 ready_for_trial: true，命令会打印日志路径。
python3 scripts/nexus_nero_j7_probe.py --side left

# 由现场操作员执行：切 CAN/J 模式，一次低速 J7 +1°，其他轴保持实测值。
python3 scripts/nexus_nero_j7_probe.py --side left --execute --delta-deg 1
```

执行模式不自动使能。若没有 CAN 反馈、存在未完成运动或未使能，会拒绝运动并打印原因。先按原因处理，不重复点击整臂归位。只有日志存在 failure_stop_requested 的执行失败才请求了阻尼急停；本次 preflight 拒绝没有触发急停，不需要为此额外做故障复位。

- 独立 SDK 测试通过：继续检查 NEXUS 的 JS→J 模式转换、仲裁与生命周期；仍不能直接认为定位完成。
- 独立 SDK 测试失败而原厂同姿态正常：聚焦 CAN/J 命令的实际执行差异，结合日志中的 `0x151 / 0x170` 和模式反馈定位，必要时交厂家核对。

将打印路径对应的 JSONL 日志提供回来；无需手工粘贴几千行。
