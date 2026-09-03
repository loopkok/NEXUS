# CLAUDE.md — astral_policy_inference 策略推理工程

策略部署（pi0.5/ACT）+ 采集数据真机回放 + 人在环路仲裁。先读本文件 → `README.md`
（用法）→ `docs/specs/2026-09-03-astral-policy-inference-design.md`（设计决策）→
`astral_ws/CHANGELOG.md`（症状→根因→修法）。改动本包前请确认不破坏下述不变量。

## 架构不变量

1. **CollectSchema 是唯一布局真源**。`robot_io.ObsLayout` 从 `astral_data_collect.schema`
   生成：订阅表、state 向量顺序、`split_action` 指令拆分。改机器人配置（臂/手/腰头/相机）
   只能通过给到 policy_node 的 schema 参数发生——别在推理侧写死维度或话题名。
2. **动作/状态都是“绝对量”**（arm rad + gripper 0..1，与采集 next-state 同序）。chunk
   换尾因此无需混合：`ActionEngine._install` 仅在“替换仍在执行中的 chunk”时按
   `latency_ms×policy_fps` 跳行补偿。回放同样发绝对动作，中断后续播用 offset 重锚定。
3. **仲裁先效应后提交，且事件驱动**。`~/cmd` 只入队，由控制定时器串行消费；所有 `_cmd_*`
   与 `_tick` 共享一把 `RLock`（仲裁与控制不可能交错）。`takeover` 先异步发起全部
   `~/reanchor`（效应），定时器轮询全部成功才提交 HUMAN；任一失败/超时**事务性回滚**：
   重新 disarm 已武装的遥操节点、恢复被冻结的引擎，就地留在原状态。任何分支都禁止“半接管”。
4. **暂停≠拆引擎，恢复必须重规划**。暂停只是控制器转 `*_PAUSED` 并 hold 最后指令；但
   暂停期间策略的“未来行”按旧观测已过期（机器人原地没跟），因此恢复（POLICY）一律
   `_start_policy()` 重建 chunk；恢复（PLAYBACK）一律按当前实况 `reanchor(offset)`。
5. **发布即仲裁权 + disarm 电平语义**：仅在 POLICY/PLAYBACK 及其暂停态发送指令
   （`_last_cmds`），进入时 `_disarm_teleop()`（true）；`/teleop/disarm` 是电平闩锁——
   进 HUMAN 或停止回 IDLE 发 false 放行遥操/夹爪。arm/head 遥操 `_on_disarm` 只认
   `Bool(true)`（false 不 disarm 刚 re-anchor 武装的节点），夹爪门控同时认 `armed=true`
   恢复（web pause/resume 不踩踏）。HUMAN 期间 policy_node 静默。
6. **QoS 与上游一致**：状态/图像/指令话题 `BEST_EFFORT depth=1`（`_SENSOR_QOS`）；
   遥操 disarm、state、task 用 RELIABLE+TRANSIENT_LOCAL（`_LATCHED_QOS`）保证新订阅者
   拿到当前模式/指令。

## 已解决的坑（症状 → 根因 → 防护）

| 症状 | 根因 | 防护 |
|---|---|---|
| takeover 卡死（服务没起也 hang） | `service_is_ready()` 误判就绪后走 `client.call()` 阻塞等响应 | 事件驱动：`call_async` 发起后由控制定时器轮询，3s 超时整体回滚（re-disarm + 恢复引擎） |
| 节点一启动 `_backend_cfg` 空 → make_backend 报缺参 | `__init__` 在 `_load_robot_params()` 之后又把 `_backend_cfg={}` 清空 | 删掉重复赋值；构造顺序：declare → load(填充 cfg) |
| rclpy 直接声明 dict 参数报 “not one of allowed types” | Humble rclpy 无 struct 参数类型，Python 侧默认值只能标量/数组 | `camera_map` 改 JSON 字符串参数（`_parse_camera_map`），yaml 里写 JSON 文本 |
| pause 后 resume，策略行陈旧造成跳变 | 暂停期间机器人 hold，未消费的旧 chunk 行按旧观测过期 | `_cmd_resume` 对 POLICY 重建引擎、对 PLAYBACK reanchor |
| 回放中 pause→takeover 顺序 | 状态机允许从 `*_PAUSED` 直接 takeover（FSM 语义） | `release` 会恢复 paused 标志；节点 hold 当前姿态等 `resume` |
| HUMAN 后遥操臂“假接管”（动不了） | policy 进 HUMAN 发 disarm=false，而 arm/head 的 `_on_disarm` 对**任何** Bool 消息都 disarm，把刚 re-anchor 武装的遥操又拆了 | arm/head `_on_disarm` 只认 `msg.data is True`（电平语义），与 web/pinch 消费方对齐 |
| policy stop → IDLE 后夹爪/遥操再也发不了 | `_disarm_teleop` 是 TRANSIENT_LOCAL 闩锁，没人再发 false | `_cmd_stop` 也 `_release_teleop_gate()`（false）开闸；HUMAN/IDLE 同样开闸 |
| pinch 门控回归 web pause→resume | pinch 把 disarm 当电平门，web resume 只发 `/teleop/armed`=true 从不发 disarm=false | pinch 新增 `arm_topic`（armed=true 恢复）；yaml 默认同时设 disarm+arm |
| `_cmd_playback` 先拆引擎后加载失败 → 状态“POLICY”引擎为 None 静默冻结 | 拆除/teardown 在 load+校验之前 | 先 load + `action_dim`/`state_names` 校验，失败原地保持；全部通过才 request→disarm→teardown→装 session |
| `_start_policy` 失败后资源没恢复（回放会话丢、disarm 闩死、interrupted 丢） | 先改共享资源再校验/建引擎 | `_start_policy` 先验证 obs → 冻结旧引擎 → 建新引擎成功后 disarm+换新；失败 `revert()` 且 Controller 快照含 `_interrupted` |
| 运行中丢 joint_states，策略继续用陈旧观测盲推盲发 | obs 门只在“进入策略”时检查，运行期无门 | POLICY 中 state 连续缺失 > `obs_stale_stop_s` → 自动 `_cmd_pause()`（保持+停引擎） |
| `_on_cmd`(MultiThreaded) 与 `_tick` 并发改 FSM/引擎 | 无共享锁，`_tick` 可能在 stop/takeover 置 `_engine=None` 后解引用 | `~/cmd` 入队 → 定时器 `_tick` 持 `RLock` 串行 drain+dispatch；传感器回调保持无锁只写缓冲 |
| stop/接管瞬间恰逢 queue_sync 边界推理 | 另一线程 `stop()` 直接 close 后端，与正在 `infer` 的线程并发 | `_run_plan` 整段持锁推理；`stop()` 先 join 后台线程再**锁内** close 后端 |
| 第二次连续回放不保持末帧 | `_hold_end` 只在 `_start_policy` 清，播完自动 stop 后残留旧时间戳 | `_cmd_playback` 与 `_cmd_stop` 都置 `_hold_end=None` |
| 回放总维数相同但块序不同会静默错放 | 只校验 `action_dim` | 带 schema（h5/lerobot meta）时比对 `ep.schema.state_names()` 与机器人 schema，不符拒绝 |
| 策略与夹爪遥操抢写 `/left_gripper/command` | `astral_gripper_teleop` 原来不看 disarm 一直发 pinch 比值 | pinch 节点 `disarm_topic`+`arm_topic` 仲裁门；policy 进 POLICY/PLAYBACK 发 disarm=true、进 HUMAN/IDLE 发 false |

## 代码路径速查

```text
node.py      policy_node：参数/schema → 布局/后端 → 引擎 → 100Hz 控制定时器 _tick
             _policy_tick / _playback_tick / _emit_target(插值) / _send_targets_from(安全层)
             _cmd_{policy,playback,pause,resume,takeover,release,stop}  仲裁入口
engine.py    queue_sync 阻塞重填；queue_async 后台预取；rtc 后台滚切尾段（min_tail）
backend.py   make_backend(openpi|act|stub)；重依赖（websockets/torch/lerobot）在 open() 懒加载
replay.py    PlaybackSession：target()/advance()/reanchor(offset)/done
controller.py 纯 FSM：request/revert/snapshot；_TABLE 显式迁移表
```

## 测试与验证（改后必须全绿）

```bash
cd /home/robot/loopkok/sdk/astral_ws
src/astral_policy_inference/scripts/run_policy_inference_tests.sh   # 7 个文件 ~79 例
# 单跑节点流（需 ROS 环境）：
source /opt/ros/humble/setup.bash
PYTHONPATH=src/astral_data_collect:src/astral_policy_inference \
  /usr/bin/python3 src/astral_policy_inference/tests/test_node_flow.py -v
```

- `test_engine.py` 刻意用 `autostart=False` + 直接 `_run_plan()` 把后台线程时序改成确定性，
  别把新用例写回 `time.sleep` 赌调度。
- 改推理控制流必须补 node_flow 用例（进程内 stub 后端 + 真实 Trigger 服务，
  覆盖成功/失败接管、暂停恢复重规划、回放重锚）。
- `astral_arm_teleop` 的 `~/reanchor` 服务另在 `astral_arm_teleop/test_reanchor_teleop.py` 有单测。

## 修改约定

1. 新能力先写失败用例复现，再修复转正（同 astral_arm_teleop 风格）。
2. 安全相关默认“平滑降级 + 可观测”：宁可 keep-last/hold，不跳变不静默。
3. 每个实质改动同步更新 `astral_ws/CHANGELOG.md`（当日日期下加粗条目：改动—动机—
   数值—验证）+ 本包 README 相关段落。
4. 显式 `/usr/bin/python3` + `source /opt/ros/humble/setup.bash`（miniconda python 会
   抢 import rclpy）。

## 遗留事项

- `queue_sync` 的阻塞重填发生在控制线程（同步模式下换 chunk 会有一次整块阻塞）；
  ACT 本地推理够快通常无感，超长推理建议 queue_async/rtc。
- 后端 `infer()` 尚无独立超时：openpi server 若“接受连接但永不应答”，首块规划/queue_sync
  重填会一直阻塞该调用方（async/rtc 只阻塞后台 planner 线程，控制定时器不受影响）。真机
  部署前可在 backend 加 deadline 并抛 `PolicyError`。
- openpi server 端机器人零改动，但 serve 进程与 checkpoint 加载在 GPU 主机（无断线自动重连
  前的“server 重启需重启节点”仍是操作注意事项）。
- 未有真实相机/驱动参与的本机联调；真机前先在 MuJoCo sim 用 stub→真后端过一遍
  astral_sim_pipeline（topic 仲裁、disarm、takeover 全链路）。
