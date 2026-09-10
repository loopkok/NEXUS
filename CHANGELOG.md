# Changelog（astral_ws）

Quest3 → Astral 双臂 + Wuji 双手。从 `xnero_ws-main` 迁入。各包 README 内亦有对应条目。

时间均为北京时间。

## 2026-09-09

**推理包重构：传输/模型解耦命名 + 统一 serve.py + 自包含 client**。`backend_type` 原来用
`"openpi"|"act"` 把传输和模型名混在一起（openpi 实为远程传输、act 实为进程内加载，且都支持
任意模型），与后续多模型扩展冲突。**做法**：① `backend_type` 改为表达**传输**（`remote` 连
serve.py / `inproc` 进程内 / `stub` 冒烟），新增 `model` 表达**模型族**（`act` / `pi05` / 后续，
决定 serve/inproc 加载路径）；类重命名 `OpenPiServerBackend→RemoteBackend`、
`LerobotActBackend→InprocBackend`（体现本地非远程），**不留** "openpi"/"act" 旧别名；
② **统一 serve**：`scripts/serve.py --model act|pi05` 单入口替代 serve_act.py/serve_policy.py
分离（act=lerobot InprocBackend，pi05=openpi create_trained_policy，同一 websocket 协议）；
③ **client 自包含**：新增 `protocol.py`（vendor openpi `__ndarray__` 序列化器）+ `client.py`
（websocket client），节点 RemoteBackend 不再依赖外部 openpi_client——**Jetson 无需再装
openpi-client**，也去掉 serve_act 的 `_OPENPI_CLIENT_SRC` sys.path hack。④ node/yaml/launch
同步：`backend_type: remote` + `model: act`，launch 新增 `model` 透传。**验证**：90 例单测全绿
（新增 protocol round-trip + wire-key 锁定、make_backend remote/inproc/stub、RemoteBackend
payload）；`serve.py --model act` + RemoteBackend(自包含 client) 真实 ACT 推理返回 (50,8) 绝对。

**移除非 ROS 推理实现（决策：主用 ROS 侧）**。删除：`runner.py`/`session.py`/`hw_io.py`/
`scripts/runner_demo.py`/`scripts/act_inference.py`/`scripts/robot_session_cli.py`/
`config/robot_session.yaml` + `tests/test_hw_io.py`/`tests/test_session.py`（共 9 文件），
并清 CLAUDE.md/README 的非 ROS 章节。包回到纯 ROS 侧结构（node/keyboard/backend/engine/
executor/controller/replay/robot_io + scripts/serve_act.py）。`s` 键盘问题（launch 子进程
stdin=/dev/null）按「单独终端 `ros2 run astral_policy_inference policy_keyboard`」解决。

**真机推理"policy blocked: missing left_gripper_ratio"修复——夹爪状态三级种子**——
`astral_policy_inference`。**症状**：机器人侧纯 POLICY 部署（无遥操在发）按 policy 直接被拒
`incomplete obs, missing=['left_gripper_ratio']`。**根因**：夹爪观测订 `/left_gripper/command`
（命令回显约定），节点对 ratio 有两级种子（收到过命令 / 自己发过命令），但**首次进 POLICY 时
两者皆空**（teleop 不在 → 无人发命令；节点还没输出过动作）→ 鸡蛋问题死锁。**做法**：加第三级
种子——订阅 driver **每拍都发**的 `/{side}_gripper/joint_states`（last-commanded rad 回显，
从未命令过则 0.0），按 rad↔ratio 线性映射换算回 ratio（新参数 `gripper_open_rad/closed_rad`，
**须与 driver yaml 一致**，本机 2.5/0.0）；优先级 = 收到命令 > 自身命令 > driver 回显。
**验证**：test_node_flow 12→**14 例**全绿（driver 回显 rad=0.4→ratio 0.5 且完整进 POLICY、
无任何夹爪源仍拒绝进策略），全套件 **106 例**全绿；yaml 同步 cameras=[video8,video0] 与新参数。
**部署**：机器人侧同步包代码 + colcon build 后重启 policy_node，确认 driver yaml 的
open/closed_rad 与节点参数一致。

**夹爪冷启动种子语义修正——"从未命令"回显 open_rad（全开）而非 0.0（全合）**——
`astral_robot_control`（driver）。**动机**：用户指出整机上电时夹爪实际是**全开**的；原 driver
对"从未命令过"的夹爪回显 0.0 rad，经 policy_node 的 rad→ratio 换算得 1.0=全合——与物理现实
相反；且 0.0 rad 在话题层与"真命令到全合"无法区分。**做法**：driver state 定时器的夹爪回显
改为"从未命令 → 回显 `open_rad`（按侧取 `*_gripper_open_rad`，本机 2.5）"，节点换算自然得
ratio 0.0=开；命令过则照旧回显真实最后命令。**验证**：`test_driver_services.py` 20 例全绿；
policy_node 侧换算链路（open_rad→ratio 0.0）由 node_flow 的 driver 回显种子用例覆盖。**部署**：
driver 与 policy_inference 两个包都要同步 + colcon build。


**新增 `astral_policy_inference` 非 ROS 真机推理 Session（`hw_io.py` + `session.py` +
`scripts/robot_session_cli.py` + `config/robot_session.yaml`）——脱离 ROS2 完成完整真机推理 Session。**
**动机**：`runner.py` 已是纯线程的非 ROS 完整编排（FSM/引擎/安全层/回放/HITL），但输入输出没接
真实硬件——`runner_demo.py` 只喂合成观测、把动作丢进列表，无法实机使用。**做法**：新增三层 I/O
装配——① `hw_io.py`：`RobotIO`（`astral_robot_sdk` 适配：读 18 维关节反馈按 schema 拼状态、
夹爪 ratio 回显、绝对动作行按块下发 `set_target_positions(左臂 motor ids)`/`set_gripper_angle`
（ratio→rad 映射，单臂不命令右臂）、阻尼/位置 HITL 带 `seed_from_current` 防回跳、
`dry_run` 无硬件模式）+ `CameraIO`（每相机后台线程：V4L2/pyrealsense2，帧 letterbox 到模型
尺寸，**单相机失败不拖垮 Session**）；② `session.py`：`RobotSession`（泵线程按 fps 喂观测 +
`on_action` 下发/记录 + HITL 阻尼接管映射 + estop 兜底，SDK 调用异常不杀控制线程）+ 
`Hdf5SessionRecorder`（写 **aligned_data.h5 兼容格式** `/action`+attrs{fps,schema}，可直接
`load_replay` 回放本次 Session）+ `SessionConfig`/`load_session_config`/`build_robot_session`
（yaml 装配，schema 段必须与 `data_collect.yaml` 一致）；③ `scripts/robot_session_cli.py`：
键盘 CLI（s/y/t/空格/n/h/g/x/e/q + 1s 状态行到 stderr）。**后端两选**：openpi 远程（大 VLA
走 GPU 主机 serve_policy/serve_act，机器人侧只连 websocket）/ act 进程内（Jetson py3.12
lerobot env，需装 astral_robot_sdk + pyrealsense2）。**数值**（dry-run + stub 冒烟）：策略
30Hz 动作下发、引擎 loop 0.5-0.6ms、POLICY→PAUSED→POLICY→IDLE 全通、HITL 阻尼→位置切换
正确、56 帧记录可被 `load_replay` 回读（action_dim 8/fps 30/schema 齐全）。**顺带修一个潜伏
bug**：`robot_io.assemble_state` 对 waist/head 用 `body[(14,16)]` 当二元索引（必 IndexError，
现有部署关腰/头从未触发），改 `slice(*body_slice)`（先红后绿，新增用例
`test_waist_head_sliced_from_body_state`）。单测 104 例全绿（新增 18 例 hermetic：RobotIO
状态拼装/单臂只发左/ratio→rad/HITL/dry-run、CameraIO letterbox、记录→回读一致、
RobotSession 编排/HITL、config 解析）。**部署注意**：真机参数（控制板/相机）本机无法验证，
Jetson 侧需装 `astral_robot_sdk`（act 进程内还要 + pyrealsense2 + lerobot env）；`camera_map`
与 ACT 图像键就是 `observation.images.video8/video0`（`make_backend` label→label 路径正确）。

**修复 `astral_policy_inference` ACT 后端加载真实 checkpoint 的 bug + 本机全链路验证**——
`LerobotActBackend.open()` 原用 `PreTrainedPolicy.from_pretrained()` 直接加载，但 lerobot 0.6.2
（`/home/robot/loopkok/lerobot` 与 `VLA/lerobot` 两个 checkout 行为一致）里 `PreTrainedPolicy`
是**抽象基类**，直接调用必挂 `Can't instantiate abstract class`。**根因**：正确用法是经
`factory.get_policy_class(config.json["type"])` 解析出具体类 `ACTPolicy` 再
`from_pretrained`；`test_backend.py` 从未用真实 checkpoint 覆盖过 `LerobotActBackend.open()`
（只测 openpi fake client / stub / factory），这条路径零覆盖。**修法**：`open()` 改读
`checkpoint_dir/config.json` 的 `type` → `get_policy_class` → 具体类 `from_pretrained`
（`pre/post` 处理器加载不变）；新增 2 例 hermetic 回归（`sys.modules` 伪造 lerobot，无
torch/无网络）：`TestLerobotActBackendLoad::test_open_resolves_concrete_policy_via_factory`
（先红后绿）+ `test_open_unknown_type_raises_policy_error`。**验证**（本机 4090D + ROS
humble + conda lerobot env）：① 后端层——真实 checkpoint
`astral_ckpt/pickup_act_480/checkpoints/080000/pretrained_model` 加载 0.5s 到 cuda:0，合成
480×480 图像推理出 8 维绝对动作（夹爪 [0.33,0.33]）；② 引擎层——50 tick 产出 50 行平滑
绝对动作（最大步长 0.0085 rad，夹爪 [0.37,0.43]），ACT 内部 50 行队列只真推理 1 次（236ms）
其余缓存弹出（≤5ms）；reset 清空 ACT 队列后 tick 从新观测重规划（队列 49=新 50-chunk）；
③ 节点层——colcon 可执行文件 + 参数文件（含 JSON `camera_map`）在本机 ROS 下启动成功
（schema=8D arms=['left']），cmd→POLICY→指令流→pause 夹持（Δ=0）→resume 重规划→stop→IDLE
全 PASS，顺带实测了运行期观测门控（关节停发 >1s 自动暂停、resume 缺新鲜观测被拦，均正确）。
单测 81 例全绿（69 非 ROS + 12 node_flow）。**注意**：ACT 模型 preprocessor 无 resize，图像
须 480×480（节点 `camera_image_size` 须设 480，非默认 224）。

**环境硬墙 + ACT 远程 serve 化（`scripts/serve_act.py`）——真实 ACT 端到端跑通**。**动机**：
「完整节点 + ACT 单进程」在本机不可行——lerobot 0.6.2 要求 py3.12（代码真用了 PEP 695 泛型
`def f[T](...)`，py3.10 解析期 SyntaxError，shim 无法绕过），而 rclpy（ROS Humble）只有 py3.10
绑定；版本约束不可调和。**做法**：新增 `scripts/serve_act.py`，在 py3.12 lerobot 环境里复用
修好的 `LerobotActBackend` 加载 checkpoint，起一个**与 openpi websocket 协议完全兼容**的
server（msgpack_numpy、连接先发 metadata、请求 state+images+prompt、响应 absolute actions、
字符串响应即错误）；node 侧**零改动**直接 `backend_type=openpi` + `host/port` 连它
（`OpenPiServerBackend` 现成）。**踩坑**：①`msgpack_numpy.Packer` 无 `unpackb`（模块级函数）；
②openpi_client **自带 vendored msgpack_numpy**（键 `__ndarray__`），与 pip 版（键 `nd`）线上
不兼容——server 必须用与 client 同源的 vendored 版，否则解出 dict；③编辑残留重复的
`_make_packer` 定义导致旧 pip 版覆盖新 vendored 版（后定义覆盖前定义）。**验证**（本机
4090D）：py3.10 `OpenPiServerBackend` client → serve_act（py3.12 + cu130 torch）真实推理回传
8 维绝对动作（首推 187ms）；完整节点端到端——policy_node（backend_type=openpi,
camera_image_size=480）+ 合成关节/夹爪/480² 图像驱动，**真实 ACT 驱动指令流 123 条/99 个不同值、
夹爪比值 [0.25,0.5]、pause 夹持 Δ=0、resume 重规划、stop→IDLE，全 PASS**。环境处置：torch
2.11.0+cu130 + torchvision 0.26.0 装入 ros2 conda env（与训练 env 同版本，满足 lerobot `<2.12`
约束）；ros2 env 里 pip 强装的 lerobot 因 py3.10 无法解析已卸载。**部署模型**：ACT = serve_act
进程（py3.12, GPU 主机）+ 节点（py3.10, 机器人侧）远程连；pi0.5 走原 openpi server 同理。
**验证脚本收拢进仓库**（原 /tmp 脚本会丢）：`astral_ws/scripts/run_act_integration.py`
（模型侧：后端 get_policy_class 加载 + 推理 + 引擎分块/队列/reset，py3.12 环境跑）与
`astral_ws/scripts/run_act_e2e.py`（节点侧全链路 + `--check-server` 预检，py3.10 + ROS 跑），
均已实测 ALL PASS；包 CLAUDE.md/README、顶层 README、根 CLAUDE.md 均补交接文档。

**完整端到端 + 推理正确性 + 远程链路指标（真实数据集 + 真实模型，本机 4090D）**。新增
`astral_ws/scripts/run_act_correctness.py`（正确性）与 `run_act_benchmark.py`（链路基准）。
**正确性方法**：从 `astral_data/act/pick up and place_480` 读真实 episode（observation.state +
video8/video0 视频帧，用 PyAV seek 按需解码，**禁止整段解码——单文件含全部 50 段 ~1 万帧 ≈15GB/相机，
全量解码会 OOM 卡死**，已踩过），喂训练好的 checkpoint 对比录制的 next-state 动作。
**踩坑（关键）**：`LerobotActBackend.infer()` 走 `select_action` 内部 50 行 action 队列——
不清队列时后续 infer 只吐上一 chunk 缓存行、不重新推理。**逐帧评估必须每帧先
`backend.reset()`**（节点运行时此行为正确=ACT 自管节奏，已验证；但逐帧正确性测量会被缓存行
污染，MAE 0.20→0.004）。**结果**：ep0/ep5/ep20/ep46 全 ALL PASS，整体 MAE **0.004~0.005 rad**
（≈0.3°，对齐偏移 0 确认 next-state 语义），与训练 L1=0.025 吻合——**模型能在训练数据上以
亚毫弧度精度复现录制轨迹，推理结果正确**。**链路指标**（250 请求远程推理）：0 失败、
吞吐 198 req/s（1.38MB/请求 → 274MB/s 上行）、稳态单请求 RTT **4.3ms**、冷启动真推理
**201ms**（warm 后 <50ms）、GPU 显存 ~957MiB；节点端到端**实测控制率 29.6Hz**（引擎
pops/4s，~30Hz 达标）、engine last_plan_ms 4.4ms、pause 夹持 Δ=0、resume 重规划、stop→IDLE。
**serve_act 优雅断开**：客户端正常关闭 ws 时 `_handle` 未捕获 ConnectionClosed → 误报
"handler failed" 刷屏，已加 try/except 捕获（0 error）。

**真实部署脚本审计——发现并修复 5 个运行时会踩的坑（本机 4090D 实测）**。
① **陈旧队列跨会话（critical）**：serve_act 的 ACT 后端常驻，`select_action` 的 50 行队列
跨客户端连接残留——节点 HITL 接管或 stop→重启后重新进 POLICY，会先重放最多 ~1.7s（50×33ms）
**接管前的陈旧动作**，可能造成危险跳变。实测：客户端 B 用完全不同的观测连上，第一条响应与
A 会话残留只差 0.0007（拿到的是 A 的缓存行）。**修法**：serve_act 每个新连接建立时
`backend.reset()`（openpi 协议无 reset 消息；节点每次 POLICY 会话正好一个新连接），修复后
B 与 A 残留差 0.1156（新推理）。② **launch 不透传 `camera_image_size`**：`ros2 launch`
部署 ACT 会静默用 yaml 默认 224，与模型 480 不匹配直接崩——launch 补 `camera_image_size`
透传。③ **launch 不透传 `engine_mode` + ACT 必须 queue_sync**：ACT 后端每次返回 1 行
（select_action 队列），queue_async 预取节流把控制率压到 **13Hz**（实测 pops/s），
queue_sync 才是 **29.6Hz**——launch 补 `engine_mode` 透传，文档/yaml 明示 ACT 用 queue_sync。
④ **console script 进 `bin/` 而非 `lib/<pkg>/`**：缺 `setup.cfg`，`ros2 run`/`ros2 launch`
找不到可执行文件（README 里早有的 "No executable found" 坑根因）——补 setup.cfg
（`[install] install_scripts=$base/lib/<pkg>`，与 astral_data_collect 一致），launch 部署验证
通过。⑤ **openpi_client 需手动 PYTHONPATH**：`pip install --user -e openpi-client --no-deps`
（其 numpy<2 约束过旧，numpy 2.2.6 实测兼容）免 PYTHONPATH。另：backend.py 的
"lerobot not importable" 错误改提示 serve_act 路径（py3.10 跑进程内 ACT 的运行时护栏）；
yaml 加醒目注释（本机 ACT 必须远程 + camera_image_size 480）。**验证**：`ros2 launch
... backend_type:=openpi camera_image_size:=480 engine_mode:=queue_sync` + serve_act +
run_act_e2e 全 PASS（控制率 29.6Hz、preflight 30ms、pause Δ=0）。

**远程链路指标补全埋点**——所有指标可观测、有数值。① **服务端分项**：`LerobotActBackend.infer`
加 `last_timing`（prep/pre/infer/post/total ms），serve_act 随响应返回 `server_timing`，
`OpenPiServerBackend` 捕获 `last_server_timing`；② **节点端到端**：node `_policy_tick` 测
`loop_ms`（新观测→指令发布处理耗时）与 `obs_age`（策略作用的真实关节反馈龄期，排除夹爪
自回显污染），入 `/policy_inference/state` 的 `latency_ms`；③ **基准脚本补全**：握手耗时、
本地序列化（vendored msgpack pack/unpack）、网络 vs 模型分离（RTT−server_total）、线缆探测
（小消息 RTT 下限）、RTT 抖动（std/跨度）；④ serve_act 加连接计数（断线/重连观测）。
**实测（本机 4090D，250 请求）**：RTT p50 3.9ms / p95 5.1ms / p99 12.1ms；服务端分项
prep 1.68 + pre 0.22 + infer 1.16（median 0.33，真推理抬均值）+ post 0.07 ≈ total 3.14ms；
**网络+序列化 p50 1.94ms**（线缆下限 0.14ms，大头是 1.38MB 上传）；本地序列化 pack 0.26ms /
unpack 0.04ms；握手 39ms；吞吐 207 req/s；GPU mem ~1GiB util mean 6-10%；**节点端到端
obs_age avg 16.5ms + loop avg 7.9ms ≈ 观测→指令 ~24ms**（受 33ms 控制周期约束）。

**方案 A（ACT 返回完整 chunk）+ 引擎锁饥饿修复——queue_async 达 30Hz、网络降 96%**。**方案 A**：
`LerobotActBackend.infer()` 从 `select_action`（每次 1 行 + 内部 50 行队列）改为
`predict_action_chunk`（一次前向返回完整 n_action_steps 行），引擎拿回分块权；
`temporal_ensemble_coeff` 非 None 时回退 `select_action`（保留时序融合）。**收益**：①
queue_async 从 13Hz 提到 30Hz（引擎按 50 行 chunk 预取）；② 图像上传从"每行一次"变
"每 chunk 一次"→ **网络降 ~96%**（e2e 实测 plans=5/4s=1.25 chunk/s vs 原 30 次/s）；③
节点 loop avg 从 7.9ms 降到 **1.26ms**（引擎本地弹 chunk，无逐 tick 网络）；④ 周期性
200ms 真推理停顿消失（warm infer 仅 8ms，且 queue_async 后台预取）。**对抗性审查发现并修复
引擎锁饥饿 bug（critical）**：`_planner_loop` 把 `self._stop.wait(0.005)` 写在
`with self._lock` **内**——planner 空闲时几乎 100% 持有引擎锁（5ms 等待+立即重获），控制线程
`tick()` 在锁上饿死 → queue_async 即使瞬时推理也只有 ~15Hz。此前被 select_action 的 1 行
chunk（planner 一直在重填循环）掩盖，方案 A 让它进入空闲路径后彻底暴露（真实 ACT 只有
0.5Hz）。**修法**：`Event.wait` 移出锁外（空闲判定在锁内、等待在锁外）。验证：stub 50 行
chunk 15→**4900Hz**、真实 ACT 0.5→**3000Hz**、e2e queue_async **30.0Hz**、loop 1.26ms、
正确性 MAE 0.0036 不变；新增回归 `test_queue_async_control_thread_not_starved_by_planner`
（0.5s >200 pops，修复前 ~7）。基准：3888 actions/s（130× 余量）、RTT p50 15.3ms std 3.5ms、
每 chunk 服务端 10.4ms、GPU util 23%。**部署变化**：ACT 不再要求 `engine_mode:=queue_sync`，
默认 queue_async 即 30Hz（launch 无需传 engine_mode）。

**绝对动作语义守卫（`abs_action_min_scale`）——对抗性审查发现信任边界后加固**。审查结论：
Jetson 侧对两后端（openpi 内部 delta / ACT 绝对）统一按绝对动作处理，delta↔绝对转换完全在
server 层——但节点**原先无任何"返回的是绝对"断言**（信任 backend 契约）。若 openpi server 漏
`AbsoluteActions` 输出变换或错 checkpoint，会静默返回 delta → 机器人到错误目标。**加固**：
引擎 `_check_absolute_semantics`：机器人明显离开零位（max|state|>1，即臂关节）时，chunk 首行
目标值须保持量级（`arm_scale` = 在这些维上 max|action| ≥ 阈值 0.5），否则 `PolicyError` →
引擎报错 → 节点安全 stop（不静默跳到错误目标）。只查 |state|>1 的维（夹爪 state∈[0,1] 自动
排除），接近零位 fail-open。**对抗迭代**：初版用「|action-state| 必须小」判别，被真实模型的
OOD 输入误报（合成 state 让模型预测向均值，|action-state| 达 1.05）；改为「离开零位时动作
量级不得塌缩」后，真实模型在所有真实量级 state 下 arm_scale≈1.7（3.4× 余量）、delta≈0.03
（16× 余量）。集成/e2e 改用真实位姿（含 d3≈-1.9）喂守卫并全部通过；测试合成 OOD 状态禁用
守卫（部署中状态恒在分布内）。新增 3 例单测（拒 delta / 收绝对 / 关闭）。`abs_action_min_scale`
yaml 可配（默认 0.5，≤0 关闭）。单测 85 例全绿。

**独立 ACT 推理脚本 `scripts/act_inference.py`（无 ROS、可移植）**。单文件、**只依赖
lerobot**（`get_policy_class` / `make_pre_post_processors` / `prepare_observation_for_inference`），
不 import 任何 ROS/astral 包——`PYTHONPATH=VLA/lerobot/src` 或任意 lerobot 环境可跑，参考
VLA/lerobot 官方推理路径（`lerobot_eval.py`）。`ActPolicy` 类：load（get_policy_class +
from_pretrained，含方案 A 的时序融合守卫）→ `predict_chunk`（一次前向完整 n_action_steps 行，
绝对量）/ `select_action`（单行，官方逐行路径，独立观测需 reset）；全部参数（action_dim /
image_keys / image_size / n_action_steps / temporal_ensemble）从 checkpoint config.json 的
input/output_features **自动推导**，CLI 可覆盖。三模式 CLI：`--scratch`（合成冒烟）/
`--state`+`--image label=path`（单次推理，可存 .npy）/ `--dataset-dir`（真实数据批量 +
逐帧 reset 对比录制动作 MAE）。**验证**（PYTHONPATH=VLA/lerobot/src）：三模式全通过，ep5/46
MAE 0.0052/0.0036 与正确性脚本一致。**对抗性审查修复**：① `device` 参数与同名 property 冲突
→ 删 property；② preprocessor 用 checkpoint 保存的 device（cuda）→ `--device cpu` 时观测被
pre 搬回 cuda 与 cpu 策略冲突 → 加 `preprocessor_overrides={"device_processor":{...}}` 强制
一致（参考 lerobot_eval.py），cpu 实测通过。

**非 ROS 完整推理包 `runner.PolicyRunner` + `runner_demo.py`**。推理包的编排层（node.py）依赖
rclpy，但核心（backend/engine/executor/controller/replay/robot_io）本就纯 Python——新增
`PolicyRunner` 用**纯线程控制循环**替代 ROS 定时器/话题/回调，复用全部纯核心，覆盖推理包全部
功能：三种引擎节奏、绝对动作 + 语义守卫、SafeExecutor、FSM（POLICY/PLAYBACK/PAUSED/HUMAN）、
回放（h5/v2.1）、暂停恢复重规划、HITL 接管交还、延迟/引擎指标。**I/O 可插拔**：`feed_observation
(state, images, prompt)` 喂观测（任意频率）、`on_action(row)` 收**安全处理后的完整绝对动作行**
（你下发真机/仿真）、`on_acquire_control/on_release_control` 镜像 disarm 电平门（外部遥操可接）、
`request("policy|playback|pause|resume|takeover|release|stop")` 命令。**真机/无 ROS 都能跑**：
demo 用真实 ACT 进程内后端（lerobot 环境，无需 serve_act）全链路 PASS——policy 30Hz →
pause 夹持 → resume 重规划 → takeover HUMAN 静默 → release 回 POLICY → playback 回放 →
stop；stub 后端（任意 python，无模型）同样全 PASS。**对抗性审查修复**：① `_resend_last` 把
逐流 CmdTarget（7/1 维）发给 on_action → 改为重发最近一次完整行；② `on_action` 给原始行而非
安全层处理值 → 按 schema 块序把 clip/slew 后的流拼回完整行再回调（消费者拿到实际下发值）；
③ 控制率测量用 on_state tick 计数（避免 resend 干扰）；④ demo 断言按 FSM 实际语义修正
（release 回到被打断的活动）。

## 2026-09-07

**左手柄 X 键「段间回位」+ web 数采卡片同功能按钮（episode 间免手摆放物品）**——
`astral_teleop`（新 `controller_workpos_gate` 闸门）× `astral_web_monitor`（数采卡片按钮）×
`astral_mujoco_sim`（sim launch 同启）。**动机**：单人数采的段与段之间，人要离开手柄去重新摆放
物品——需要一只手柄按键"停止跟随 VR 并回到工作位"；web 侧同功能按钮方便不戴头盔时操作。
**做法**：①新闸门节点镜像 `controller_start_gate`（grip→start）对称——左手柄 X（primary,
`buttons[0]`，此前完全空闲）按下沿 → 发 `/teleop/disarm` + `/teleop/init`（即 web「工作位」的
信号序列：停止跟随 + 沿 init_waypoints→init_pose 回位，臂侧接口全现成，零改动）；②**录制保护**：
闸门订 `/data_collect/state`（latched），RECORDING/PAUSED/SAVING 时按 X 忽略并节流告警（防手臂
回位毁掉正在录的段）；无数采节点运行（纯遥操）时照常生效；③`full_teleop.launch.py` 无条件接入、
sim pipeline 随 `with_start_gate` 接入；④web 数采卡片按钮组加「段间回位」（`POST
/api/v1/teleop/workpos` 复用既有端点），仅遥操 RUNNING/PAUSED 且**非录制中**可用（录制保护与 X 键闸门一致）+ confirm 弹窗，键位小抄补
"左手柄 X"。纯决策拆 `controller_workpos_logic.py`（无 ROS）。**对抗性审查发现并修复**：①
**disarm/init 跨话题无保序竞态**——`/teleop/disarm` 会取消进行中的 homing（arm_teleop 刻意行为），
背靠背连发可能"先收 init 启动回位、再收 disarm 取消回位"致 X 静默失效；闸门改为 **disarm 立即发 +
0.1s 一次性定时器发 init**（镜像 web workpos 的 disarm→sleep(0.1)→init 时序），连按 X 取消旧待发
init。②depth=1 BEST_EFFORT 订阅连发不消费时中间跃迁被最新帧覆盖（真实 DDS 行为）；③
**rclpy 同进程 intra-process 双投**：闸门在 spin 回调内发布时同进程订阅会收到 2 份
（intra 快路径 + DDS 环回各一，~6ms 间隔）——纯测试环境怪癖，非生产缺陷（跨进程单收）。
**验证**：纯逻辑 pytest 10 例 + 节点集成 6 例（rclpy，本机 ROS humble 实测**16/16 全绿 ×2 连跑**；
节点测试用独立 client 捕获节点 + 直接驱动 `_on_joy`（回调内发布会双投，主线程调用单投，确定性），
含 disarm 先于 init 的时序断言、连按取消待发 init）；`colcon build astral_teleop` 后**跨进程
DDS 冒烟 ×3 稳定 PASS**（子进程精确 PID 启停，零残留）——独立闸门进程 + rclpy 客户端：
IDLE+X → disarm×1+init×1（0.1s 间隔）、RECORDING+X → 忽略并节流 WARN；前端 `npm run build`
（tsc+vite）通过；smoke_test.sh 加 workpos 端点探测（遥操停止时 409 非 404）。**待办（机器人
侧）**：实机冒烟——按 X 回位 → 再 grip 重标定开始下一段；录制中按 X 确认被忽略。

**遥操完整启动命令 + 操作步骤文档化并全树同步**——`astral_teleop/README.md`（主）× 顶层
`CLAUDE.md`（采集→训练速查）× `astral_web_monitor/README.md` × `astral_data_collect/CLAUDE.md`。
**动机**：用户要求把"完整遥操启动（web + CLI 两条路径）与遥操步骤"写成文档，一处权威、各处引用。
**做法**：①`astral_teleop/README.md` 新增「完整遥操启动（web/CLI）」+「遥操操作步骤（数采 episode
循环）」两节——web 路径（起监控 → 预设启动 → 数采卡片启节点/设目录/任务 → 工作位）、CLI 路径
（单左臂/双臂/双夹爪/仿真/数采五条命令）、七步 episode 循环表（grip→A→任务→B→X→摆物→grip）、
键位速查（VR/web/键盘/CLI 四端等价）、顺序要求（先 grip 再 A 防 W5；B 后等 IDLE 再 X；回位完再
grip）、安全收尾（急停/暂停/HOME）；②顶层 `CLAUDE.md` 采集→训练速查加遥操启动两条路径 + episode
循环摘要；③`astral_web_monitor/README.md` 启动节后加「完整数采流程」指引；④`astral_data_collect/
CLAUDE.md` 新增「数采完整流程」节（含 X 与数据的关系：`_accepting` 门控 + `dq.clear` + 录制拦截
三保险）。**验证**：文档与代码契约逐条核对（命令与 presets.yaml/launch 参数一致、键位与
vr_collect_logic/controller_start_gate/workpos_gate 一致）；无代码改动，无需跑测试。

**段间回位改直达（X / 数采卡片「段间回位」不经途径点）+ 工作位保持途经点**——
`astral_arm_teleop` × `astral_teleop` × `astral_web_monitor`。**动机**：真机测试后用户要求——
段间回位（左 X / 数采卡片）现在先回 init_waypoints 再到 init_pose，改为**直接**关节空间直线
插补回工作位（不经途经点）；而系统 tab「工作位」按钮保持途经点路径。**做法**：①臂节点新增
`/teleop/init_direct` 订阅 + `~/init_direct` 服务，`_go_init(direct=True)` 时 `_homing_path` 仅
`[init_pose]`（直达），`direct=False`（原 `/teleop/init`/工作位）保持 `init_waypoints → init_pose`
——两路共用同一 homing 慢速轨迹机（限速/限位/到点判定相同），disarm→0.1s→init 时序沿用；
②`controller_workpos_gate` 的 `init_topic` 默认改 `/teleop/init_direct`（X 走直达）；③web 新增
`POST /api/v1/teleop/workpos/direct`（disarm→0.1s→发 init_direct），数采卡片「段间回位」按钮改调
直达端点，系统 tab「工作位」保持 `/api/v1/teleop/workpos`（途经点）；④`config.py` 加
`TOPIC_INIT_DIRECT`、`monitor_node` 加 `publish_init_direct`、client.ts 加 `teleopWorkposDirect`。
**验证**：新增臂节点级测试 `test_arm_teleop_node_init.py` **7/7 全绿**（直达路径=1 点=init_pose、
途经点路径=途经点+init_pose=2 点、回调触发、level guard、homing 中拒绝、只读参数 init_waypoints
经 `rclpy.init(args)` 构造期注入）；astral_teleop **16/16**（闸门节点测试改订 `/teleop/init_direct`）；
跨进程 DDS 冒烟 **×3 PASS**（IDLE+X → disarm×1 + init_direct×1；RECORDING+X → 忽略；零残留）；
前端 tsc+vite build 通过；`colcon build astral_teleop astral_arm_teleop` 后冒烟。**待办（机器人
侧）**：同步构建三包后实机冒烟——X 直达回位、工作位途经点回位、录制中按 X 忽略。

## 2026-09-04

**AV1 并行度可配：`ASTRAL_AV1_LP` 环境变量（默认 2 保内存红线，20 核机可放开）**——
`astral_data_collect`。**动机**：用户问"为什么转换这么慢"——实测 20 核机器只用了 2 核
（SVT `Level of Parallelism: 2`，内存红线），AV1-224 ≈78 帧/秒、720 ≈8 帧/秒；要求针对本机
放开核数。**做法**：`encode_video` 的 `lp=` 从硬编码改为 `_svt_av1_params()` 读
`ASTRAL_AV1_LP`（默认 2；非法值回退 2；>2 时 stderr 提示 ~0.6GB/lp 峰值内存），
`lookahead=16` 不变；所有入口（脚本/模块直调）自动生效。**验证**：新增
`test_svt_av1_params_env_override`（默认 lp=2 / 放开 lp=8 / 非法回退）通过；CLAUDE.md
不变式 6 与 README §5 补说明。**注意**：正在运行的后台转换进程不受影响（旧代码已加载），
配置对下次运行生效；本机当前 IDE 占 10GB（可用 ~4GB），放开建议 lp≤4（~2.4GB），
要 lp=8 需先关 IDE/opencode。

**convert_to_act 分辨率选项化：224/480/720/原生（0）+ 原生同 shape 预检**——
`astral_data_collect` × `scripts/vla_process_act.sh`。**动机**：用户要求 ACT 转换输出分辨率可
选（224/480/720/原生）。机制本已存在（`--image-size` 任意整数、0=原生），真正的坑是**原生模式下
各相机分辨率不同**（video8=1080p、video0=720p）会破坏 ACT 的"全部相机同 shape"要求，此前要到结构
自检才报通用错。**做法**：①`convert_to_act` 新增 `_probe_native_shapes`——原生模式（image_size=0）
转换前先解各相机首帧 JPEG 探测分辨率，不一致即 `ValueError` 并提示改用 224|480|720 或统一采集
分辨率；②`run_pipeline` 内部把 `image_size==0` 归一化为 `None`（此前只有 CLI 路径做了，直连调用会
漏判）；③help/docstring/脚本注释写明 224 默认/480/720/0=原生选项。**验证**：`test_convert_act.py`
6→**8 例**全绿——新增 原生不一致提前报错（注入 32×32 vs 64×48 两相机）、480 端到端（结构自检 0
issue + 产物视频实测 480×480）；`bash -n` 通过；README §5b 补分辨率选项说明。真实数据 480 全链路
+ conda lerobot 深度自检后台运行中（5 段 2 相机）。

**数据目录三文件夹重组（raw/ pi/ act/）+ 剔除 video2 + 清理旧转换产物**——
`astral_data` × `astral_data_collect` × `scripts/`。**动机**：用户要求——不要 video2（原生+转换都
不要）、删掉之前几次转换产物、统一到 astral_data 下建 原始/pi/act 三个文件夹、后续按 session 各进各的。
**做法**：①5 段 raw（061-065）剔除 `camera_data.h5` 的 video2 组 + meta.json schema.cameras 同步为
[video8,video0] + 删旧 3 相机 aligned_data.h5（对齐按新 schema 重建）；②删除全部旧转换产物
（astral_data_lerobot/v3、astral_data_act/v2、default_task_openpi/act 共 6 目录）；③建三目录
`astral_data/{raw,pi,act}/`，raw 移入 `raw/default_task/`；④`data_collect.yaml` `save_root` 改
`~/astral_data/raw`（未来采集直接落 raw/{session}）；⑤README §4 加目录约定块、§2 save_root 行、
§5b 示例路径、包/顶层 CLAUDE.md 同步。**验证**：从新 raw（2 相机）端到端重建——
`vla_process_openpi.sh raw/default_task pi/default_task`（v2.1, `cams=2`）与
`vla_process_act.sh raw/default_task act/default_task --check-python lerobot`
（v3, conda lerobot 深度自检 `OK episodes=5 frames=1391 cams=[video0,video8]`）全绿；
全树 `find -iname "*video2*"` 零残留。**遗留**：`astral_data/default_task_ep58-65.tar.gz`（ep58-65
备份包）与 `~/astral_data`（home 根旧 junk：default_task quarantine + verify_*）未动，确认后另清。

**openpi/ACT 一键脚本拆分：`vla_process_openpi.sh` + `vla_process_act.sh`（同 raw、独立输出）**——
`scripts/`。**动机**：用户要求"处理脚本与 sh 脚本都要清晰：从同一个采集原始数据文件夹来，
openpi 和 ACT 分开，转换后的文件夹也分开"。**做法**：删掉合并的 `vla_process_session.sh`，
拆成两个脚本，**入参都是同一个 raw 会话目录**，输出独立文件夹（建议 `<session>_openpi` /
`<session>_act`）：①`vla_process_openpi.sh <raw> <openpi输出>`——对齐→校验(默认隔离)→转 v2.1
（openpi 直接消费）；②`vla_process_act.sh <raw> <act输出>`——对齐→校验(默认隔离)→
`convert_to_act`（v3 + 两级自检），带 `--check-python`/`--keep-v21`/`--overwrite`；两脚本共享
同样的"坏段默认隔离 + 隔离后中止"门禁。引用同步：`openpi_train.sh` 提示、包 CLAUDE.md/README
一键脚本段、`convert_to_act` docstring。**验证**：两脚本 `bash -n` 通过；**同一 raw**
（default_task，5 段 1391 帧）端到端各跑一遍——openpi → `default_task_openpi`（v2.1，5 段）、
act → `default_task_act`（v3，conda lerobot 深度自检 `OK episodes=5 frames=1391`）全绿。

**convert_to_act 增 `--v21-root` 快速入口 + vla_process_session.sh 的 --act-output 改走 convert_to_act**——
`astral_data_collect` × `scripts/vla_process_session.sh`。**动机**：用户问"keep-v21 中间产物是什么、
与 openpi 转换什么关系、脚本同步改"——明确 openpi/ACT 共享 raw→v2.1 中间层：openpi 直接消费 v2.1，
ACT 在 v2.1 之上升版 v3 + 自检。**做法**：①`convert_to_act` 输入改为 `--session`/`--v21-root` 二选一
（`--v21-root` 跳过对齐/重编码，直接升版+自检，秒级）；②`vla_process_session.sh` 的 `--act-output` 从
直接调 `convert_to_lerobot_v3` 改为调 `convert_to_act --v21-root $OUTPUT`（复用本步 v2.1，不重复编码），
新增 `--act-check-python` 透传深度自检，`--overwrite-v3` 透传为 `--overwrite`；③README §5b 补 v21-root
用法。**验证**：`test_convert_act.py` 4→**6 例**全绿（新增 v21-root 复用路径、--session/--v21-root 互斥）；
离线回归 59 例全绿；**真实端到端** v21-root 复用现有 v2.1 → `/home/robot/loopkok/sdk/astral_data_act_v2`
仅 **7.6s**（对比全链路重编码 ~2min），conda lerobot 深度自检 `OK episodes=5 frames=1391 cams=[video0,video2,video8]`；
`bash -n` 语法通过。

**新增 ACT 专属转换 `convert_to_act.py`：raw 会话目录 → 官方 lerobot ACT 可训数据集（内置两级自检）**——
`astral_data_collect`。**动机**：用户要求"转 ACT 不要写通用 v3 升版，要专门的 ACT 转换，且保证无缝接
官方 lerobot ACT 训练"。**前提澄清**：官方 `lerobot-train --policy.type=act` 的输入格式就是 LeRobot
v3（无独立于 v3 的 ACT 专用格式），所以"专属"体现在**硬保证 + 自检**而非新格式。**做法**：新模块
`convert_to_act.py`——输入 raw session（含 episode*），内部复用 align_data → convert_to_lerobot(v2.1,
临时目录, --keep-v21 可留) → convert_to_lerobot_v3 → **结构级自检**（stats.json 含
observation.images.{每路}+state+action 的 mean/std（ACT VISUAL/STATE/ACTION MEAN_STD 硬需求）、
各相机同 shape、tasks 非空、parquet+视频可读可解码，不过即退出）；**深度级自检**
（`--check-python <现代lerobot python>`）：用该解释器真装载 `LeRobotDataset` + 构建 ACT 预处理管线 +
逐帧解码。输出 = 官方 v3 布局，脚本结尾打印 `lerobot-train --dataset.root=... --policy.type=act` 命令。
**验证**：`test_convert_act.py` 4 例全绿（全链路转出布局关键件齐 + 结构级自检 0 issue + 深度自检源码
可编译、stats 缺图像统计负例报错、tasks 空负例报错、无 --overwrite 拒绝覆盖输出）；离线回归 35 例全绿；
**真实数据端到端**（default_task 5 段 1391 帧）→ `/home/robot/loopkok/sdk/astral_data_act`，conda
lerobot 深度自检输出 `OK episodes=5 frames=1391 cams=[video0,video2,video8]`——现代 lerobot 真装载 +
预处理管线构建成功，即"无缝衔接官方 ACT"的实测证明。

**数采 web 端可运行期切换录制目录（session）——换目录不再重启节点**——
`astral_data_collect` × `astral_web_monitor`。**动机**：web 泳道 launch 把
`session:=default_task` 写死，所有段都进 `~/astral_data/default_task/`——用户按 A 录完
几段后在自己的 `~` 下"找不到数据集"（其实都在 default_task 里），换目录只能改 launch
重启节点（pending task 也丢）。**做法**：① 节点新增 latched 话题 `/data_collect/session`
（与 `/data_collect/task` 同模式）——目录名安全校验（非空/无路径分隔符/非 `..`/不以
`.` 开头/≤64 字符），**仅 IDLE 生效**（录制/暂停/保存中拒绝并计入 `ignored` 的
`set_session`）；切换后 state 立即 latched 重发（卡片头部 session 实时更新），
**meta.json 新增 `session` 字段**（段级溯源，离线侧无视）；launch/yaml 的 session 仍是
初始值（重启回落 default_task）。② web 桥：`DC_TOPIC_SESSION` → monitor 发布器
`publish_dc_session` → REST `POST /api/v1/collect/session {text}` → `api.collectSession`；
卡片任务文本同排新增「录到目录（当前: xxx，仅空闲可换）」输入 + 「设定目录」按钮
（成功了 toast、输入清空）。**验证**：`test_node_guards.py` 12→**14 例**全绿——IDLE
切换后段落新 session 目录（段号独立从 000000 起）且 meta.json 带 session、录制中切换
与 6 种非法名（a/b、a\b、..、空、.hidden、65 字符）被拒且计入 ignored、IDLE 再切成功；
`npm run build`（tsc+vite）通过 dist 重建；后端 py_compile 通过。机器人侧需同步
`astral_data_collect`（节点话题）与 web dist。

**工作位/HOME 后夹爪无响应——latched disarm 把 pinch 仲裁门关死，「开始遥操」补发 /teleop/armed 开门**——
`astral_web_monitor` × `astral_teleop`。**症状**：no-right-arm 预设 → 点工作位 → 开始遥操，
手柄对夹爪无响应（此前正常）。**根因**：工作位/HOME 流程发 **latched** `/teleop/disarm`
（门控语义），pinch 仲裁门只认 `/teleop/armed=true` 重开；而「开始遥操」（web 按钮 / 左手
gripClick 闸门）只发 `/teleop/start`，从不发 armed——点过工作位后门永久关死。**做法**：
① web `POST /api/v1/teleop/start` 先发 `/teleop/armed=true` 再发 start（臂节点未校准会忽略
armed，无害）；② `controller_start_gate`（gripClick 入口）同步一并发 armed（新参数
`armed_topic` 默认 /teleop/armed）。验证：py_compile、web monitor 4 例全绿。

**单臂预设下缺席臂被拽向零位——driver 缺侧"补零"改为只发有指令的一侧**——
`astral_robot_control`。**现象**：web 预设选 no-right-arm（单左臂）启动后点「工作位」，
**右臂也抽了一下**（右臂不在预设里、停在某非零位姿，却被命令拽向零）。**根因**：driver
控制定时器发现"左臂有新鲜指令、右臂 None"时给缺失侧**补零**（`right=[0.0]*7` 后
`move_arm_js(left, zeros)`）——工作位/HOME/遥操一发流（150Hz），缺席侧实体臂就被 100Hz
零目标持续拽向零位；双臂模式平时两侧都在发所以不显形。**做法**：控制定时器 arm-only 路径
改为**只向有新鲜指令的一侧下发**——双侧都新鲜仍走 `move_arm_js` 一次下发（不变）；仅单侧
新鲜走新 `_send_arm_side()`（SDK `set_target_positions` 按该侧电机 ID 子集下发，与
`move_arm_js` 同一 0x90 指令通道）。缺席侧不发命令 = 板端位置保持维持原目标（原位保持），
与预设搭配：no-right-arm 时右臂不再被任何左臂轨迹打扰。**验证**：`test_driver_services.py`
16→**20 例**全绿——新增控制定时器 4 例：左新鲜只发左电机 ID 集（无 move_arm_js）、右新鲜
只发右、双侧新鲜仍 move_arm_js 双发、无新鲜零下发。README 话题契约补"缺侧不补零"。

**fix：monitor 节点补 import `TOPIC_INIT`——上条「工作位」条目遗留的启动 NameError**——
`astral_web_monitor`。**症状**：monitor_node `__init__` 里 `create_publisher(Bool,
TOPIC_INIT, ...)` 引用的常量未在 import 区导入（上条改动只加了 config 定义与
`publish_init` 调用，import 列表漏 `TOPIC_INIT`），monitor 节点启动即抛
`NameError: name 'TOPIC_INIT' is not defined`，web 整体不可用。**做法**：import 区补
`TOPIC_INIT`（config.py 中已定义，无其他缺失）。**验证**：模块可导入（rclpy 环境）。

**web「工作位」按钮：启动不再自动归位，回初始位改手动触发**——`astral_arm_teleop` ×
`astral_web_monitor`。**动机**：web 启动会拉起节点并自动走 init_waypoints → init_pose，
机器人动不动不该由"启动栈"决定，且操作员常要先把臂摆开/上电再决定回位时机；改为启动只
拉节点、回工作位由按钮显式触发。**做法**：① 节点新增 `_go_init()`（与 `_go_home` 对称：
disarm → init_waypoints **正序** → init_pose，启动同款 init 轨迹机，不加 park 门控），
入口 = 全局一次性 `/teleop/init` + 单臂 `~/init`（VOLATILE，迟到不触发）；`_finish_homing`
init arrived 置 `_at_init_pose` 标志；② **`/teleop/start` 守卫**：臂不在工作位时先
`_anchor_origin_to_measured()`（从 `_reanchor_teleop` 提取的共享重锚 helper）把原点锚到
当前实测再 arm——从任意位姿 start 不再向启动位 FK 锚点跳变，遥操纯增量开始；③ 左右 yaml
`move_to_init_pose: false`（注释说明；CLI 需要可 `:=true` 覆盖）；④ web：`POST
/api/v1/teleop/workpos`（disarm + 0.1s + 发 `/teleop/init`，**不调 `~/enable`**——按用户
要求）＋ SystemTab 预设卡片「工作位」按钮（sky 色，HOME 旁，confirm 弹窗）；前端重建
dist。**验证**：`test_home_park.py` 16→**21 例**全绿（新增 _go_init 正向路径构建/disarm、
busy 拒绝、init arrived 置位/超时与 park 不置位、start 不在工作位时重锚+告警、在工作位时
不重锚）；reanchor 5 例全绿（共享 helper 重构回归）；web monitor 4 例、py_compile 通过。
README（arm_teleop 工作位节 + web REST/话题表）与 CHANGELOG 同步。机器人侧需重新
colcon build（astral_arm_teleop + astral_web_monitor）。

**HOME/启动途经点调整：init_waypoints 改远摆位 [-1.6, ±0.2, 0, -1.92, ∓0.2, 0, 0]**——
`astral_arm_teleop` 配置。**动机**：HOME 收回时臂到途经点的摆臂动作不明显（原途经点
[-1.0, 0, 0, -2.2, 0, 0.46, 0] 与 init_pose 的 j1 只差 0.65 rad），实机确认需要更明显的
"到过折叠途经点"行程。**做法**：左右 yaml `init_waypoints` 改为：左
`[-1.6, 0.2, 0.0, -1.92, -0.2, 0.0, 0.00]`、右按 init_pose 镜像规律（j2/j5 反号）
`[-1.6, -0.2, 0.0, -1.92, 0.2, 0.0, 0.00]`。数值全在 URDF 限位内（J1 -1.6 < ±2.0、
J4 -1.92 < [-2.26, 0]），不会触发 clip。**影响**：该参数同时作用于启动 init 与 HOME park
（同一 `_parse_init_waypoints`）；HOME 段 init_pose→途经点 j1 差 0.65→**1.25 rad**，
行程 1.05→**~2.1 s**（摆臂更明显）；w1→零 j4 1.92 rad（原 2.2）略快。**机器人侧需重新
colcon build 生效**（astral_arm_teleop 的 yaml 非 symlink 构建为拷贝）。

**日志降噪：夹爪仲裁门 disarm 提示只打状态变化；capture driver-side fps 稳定时不再 5s 刷屏**——
`astral_gripper_teleop` × `quest3_video_streamer`。**动机**：HOME/暂停期间夹爪仲裁门长期关闭，
pinch 节点每秒打一条 "disarmed by arbitration gate"（3 相机 x 5s 窗口的 capture fps 同理），
正常状态刷屏。**做法**：① pinch 新增 `_set_gate()`——门状态**变化**才打日志（进入 disarm 打
"not publishing (waiting for open)"，重开打 "publishing resumed"），删 1s 节流重复；②
`webcam_source._capture_loop` 的 driver-side fps 改为 5s 统计窗口 + **偏差 >15% 才报**（掉速/
降级立即可见）+ **≥60s 心跳保底**（证明线程存活），稳定满帧不再刷屏。**验证**：两文件
py_compile；streamer 纯测试 24 通过（1 例环境性失败：uv venv 无 aiortc，与本次无关）。

**HOME 途经点放行日志加"实测距途经点"误差**——`astral_arm_teleop`。HOME/init 途经点放行的
`Via N/N reached` WARN 追加实测关节距刚放行途经点的 max 误差（`.3f rad`），实机核对
"实体到没到过途经点"无需再看动作——日志直接给数字。`test_home_park` 放行断言同步。

**HOME 归位仍"没到过 init_waypoints 姿态"——实机日志定案：门限秒放行 + 只擦过不停下；改"到位 + 稳定驻留"放行**——
`astral_arm_teleop`。**现象**（实机 HOME 日志）：`Homing 3 segment(s)` 路径含 w1 无误，但
`Via 1/2`(t=0.01s) → `Via 2/2`(t=1.06s)——命令 1.05 s 走完 init_pose→w1 后**立即反向**，
无任何驻留；用户观察：臂"先到另一处再回零"，从没真正到过 w1（[-1.0,0,0,-2.2,0,0.46,0]）
的关节姿态。**根因**：上一版门限 = "实测进入 `homing_via_tol`(0.12) 即放行"太松且不判停——
实体 0.6 rad/s 运动中，命令到 w1 瞬间实测恰已擦过 0.12 边界 → **秒放行**；臂刚擦过 w1
（差最多 ~7°）就掉头下零，位置保持从未在 w1 停住。**做法**：放行条件改为**实测进入 tol
且连续稳定 `homing_via_settle_s`（新参数默认 0.5 s）**——命令全程钉在途经点，实体在位置
保持下真到位停下；运动中擦过 tol（err 再出 tol）连续计时清零重来；`homing_via_hold_s`
改为途经点总等待上限（默认 2 s），实测流不可用时按纯命令驻留 settle 时长放行。启动 init
不加门限不变。**验证**：`test_home_park.py` 14→**16 例**全绿（新增/改写：到位后不满
settle 不推进、中途离开 tol 清零重计、实测流缺失按命令驻留放行；hold 超时放行保留）；
闭环滞后模拟：实体距 w1 最近距离 0.033-0.047 rad→**0.000-0.025 rad**，命令在 w1 驻留
0.02 s(1 拍)→**0.52-0.68 s**（τ=0.05-0.35 s 全档）；reanchor 5 例全绿。左右 yaml 补
`homing_via_settle_s`，README 同步。机器人侧重新 colcon build 生效。

**数采卡片加「动态状态提示行 + 操作键位/门控小抄」**——`astral_web_monitor` 前端。
**动机**：卡片此前只给按钮不给"当前状态能做什么/为什么按键没反应"的语境——VR 摇杆按下=丢弃
（删文件不可逆）等门控语义只写在 README，操作者在 IDLE 下误按摇杆、SAVING 期按 A 只会看到
"没反应"。**做法**：①按钮行下方动态提示行（online 显示，按 state 出文案）——IDLE：可开始、
段号接续与复用说明；RECORDING：可停止保存/下一段/暂停/丢弃 + VR B=停止保存、摇杆=丢弃不可逆、
非法状态按键被忽略；PAUSED：缓冲已落盘可继续/停止/丢弃；SAVING：收尾写盘中按键会被忽略计入
「忽略指令」勿急（左侧琥珀色竖条强调）；②卡片底部键位小抄——VR A/B/摇杆 + 键盘
s/q/d/n/p/t 的键位与各自合法状态、门控忽略语义、"要确认弹窗的破坏性操作用卡片按钮"。
**验证**：`npm run build`（tsc+vite）通过，dist 重建（index-Dxq7FFA1.js）。

**VR 采集控制改键：A=开始 / B=停止保存 / 摇杆按下=丢弃**——`astral_data_collect`。
上一版键位（摇杆=start / A=next）实机试用手感不合，用户改配：**A 键(IDLE) → start；
B 键(录制中) → stop&save；摇杆按下(录制中) → discard**（删除当前段并回 IDLE，语义同
键盘 `d`）。上升沿 + 状态门控机制不变（同帧多键：低号键优先、非法跳过）；`discard` 是
破坏性操作（删文件无确认），门控保证 IDLE 下误按摇杆不动作。**验证**：`test_vr_collect_control.py`
重写 12 例全绿（A/IDLE=start、摇杆/RECORDING=discard、B 不变、三态门控、同帧 A+B→start/
stop 与 摇杆+A→discard 的优先级、长按不重复）；`test_vr_collect_node.py` 集成 3 例全绿
（IDLE: A→start、B/摇杆门控；RECORDING: 摇杆→discard、B→stop、A 门控）。README/CLAUDE.md/
launch 注释同步新键位。

**遥操加"硬跟人肘"模式（human_elbow_mode=hard）+ astral_pim_ik 录制转换器——"训练臂角复现人手姿态"链路打通**——
`astral_arm_teleop` × `astral_pim_ik`。**动机**：用户要最终训练出的臂角符合人手 Quest3 增量遥操姿态
（无肘跟踪也运行）；遥操原本的 `use_human_elbow` 只是 ψ_human 软先验（w=2.0 与 w_vel=1.0 连续性混合
+ 局部窗/逃逸迟滞），肘"半跟人半跟惯性"，正是"遥操姿态怪"的主要嫌疑。**做法**：① teleop
`GeometricIKSolver` 新增 `solve_hard(T, ψ)`——精确解在人臂角上（`_collect_solutions(T,ψ)` + 最小
warm-start 选支，无网格/1D QP/逃逸；镜像 pim_ik 的 `GeometricArmAngleSolver.solve`，因
pim_ik→teleop 依赖不可反向），成功同步连续性 state 保证回退平滑；② 节点新参数
`human_elbow_mode: soft|hard`（默认 soft 保兼容，左右 yaml 置 **hard**），人肘方向新鲜且伸直
门控通过时走 `solve_hard`，不可行/奇异/超龄期自动回退软路径，`[Latency]` 行加 `hard_follow`/
`hard_fallback` 计数，参数热改可用；③ pim_ik 新增 `convert_sessions.py`——遥操录制
`robot_data.h5` 的 `{side}_arm_state`（实测 q）→ FK 得 T_ee、`psi_from_config` 得标签
（schema 同 `generate_dataset`），meta pause/resume + 帧跳变 + 超 URDF 限位帧自动断段
（本机 verify/default_task 存量段全是等差假数据 q4 超限，已被正确拒绝），train/val **按整段会话
留出**（`--val_sessions`）；eval 加 `--no_split`。**验证**：`test_geometric_ik` 10→**11 例**
全绿——hard 保真：双臂各 12 样本肘方向偏差 L 0.0118°/R 0.0002°、位姿 <0.05mm、不可达→None、
确定性；pim_ik `test_deterministic_solver` 8→**9 例**——teleop `solve_hard` ≡ pim_ik `solve`
**逐位 0.00e+00**（双臂 25 样本）。整链排演（合成会话 → convert → 8 epochs 训练 →
`--no_split` 整段评估）跑通。**硬模式是录制前提**：执行的 q 的臂角 == 喂入的 ψ_human，故训练
标签零额外录制流。真机 runbook 见 pim_ik `CLAUDE.md` §7.1。机器人侧需重新 colcon build 生效。

**VR 采集控制：右手柄 摇杆按下=开始 / A=下一段 / B=停止保存**——`astral_data_collect`。
**动机**：单人采集时双手都在遥操，录段控制（开始/分段/停止）要伸手去键盘或 web，打断操作
节奏。**做法**：新增 `vr_collect_logic.py`（纯决策：上升沿 + 状态门控，无 ROS 可离线单测）
+ `vr_collect_control.py` 节点——订 `quest3/right_controller_joy`（mocap 已发布，
buttons=[primary(A)/secondary(B)/stickPress/...]）与 `/data_collect/state`（latched），
**摇杆按下(buttons[2]) 上升沿 + IDLE → start；A(buttons[0]) + RECORDING/PAUSED → next；
B(buttons[1]) + RECORDING/PAUSED → stop**。长按不重复；状态不合法/采集未运行（未收到
state）时按键静默忽略（info 日志说明），不给采集节点发无效命令刷 warning；与 web 卡片/
键盘控制器三面并存（离散一次性命令，采集节点状态机幂等）。随 `data_collect.launch.py`
同启（`vr_control:=false` 可关，同 keyboard 参数模式）。**验证**：新增 `test_vr_collect_control.py`
纯逻辑 12 例全绿（上升沿/长按不重复/释放再按/三种状态门控/state 未知/同帧多键/短数组不越界）；
`test_vr_collect_node.py` rclpy 集成 3 例全绿（真实话题契约：latched state 门控 → control
命令序列 start/next/stop、录制中摇杆被门控、长按不重复）；node_guards 9 例、离线套件 30 例
回归全绿。README/CLAUDE.md 控制面与文件地图同步（三面→四面）。

**HOME 归位"出了但没到 waypoint 就掉头"修复：途经点加实测到达门限 + park 路径改倒序**——
`astral_arm_teleop`。**现象**：点 web 紫色 HOME 能正常回到零点，但途经点与启动移动不成反向——
预期先到 init_pose、再经 init_waypoints、再到零；实际**伸向 waypoint 但没到位就转去零位**
（用户确认"出了但没到就掉头"），且**经过位置与启动路径不是反向**。**根因**（两个独立缺陷）：
① **切角**——park 轨迹纯开环，命令 q_cmd 领先实测（板端跟踪误差+链路延迟）；途经点推进只看
命令误差（<`init_arrive_tol` 0.05 即走下一段），命令一到点立刻 180° 反向，实体会被"切角"在
waypoint 之前掉头（waypoint 是防刮桌安全走廊位姿，切角 = 走廊失效）；冻结守卫只兜 >0.25 rad
的失能/堵转，0.05~0.25 的常规滞后静默切角。② **顺序**——`_go_home` 把 init_waypoints **正序**
拼在 init_pose 与零之间，而 README 明示收回应**倒序逐点**（启动路径的反向）；单一途经点配置
（当前左右臂 yaml 各 1 个）掩盖了顺序错误。**做法**：① park 途经点推进改双条件——命令到点后
**钉住重发，等实测也进入 `homing_via_tol`（新参数默认 0.12 rad）再推进下一段**；实测流不可用
或等满 `homing_via_hold_s`（新参数默认 2.0 s，防负载静差拖死归零）则照旧放行。段间运动仍纯
开环——不退回"每拍从实测迈步"的阶梯采样（c3941c5 教训）。启动 init 不加门限（实测没动不能
卡启动）。② `_go_home` 路径改 `_parse_init_waypoints()[::-1]`（倒序），docstring/注释同步。
**验证**：`test_home_park.py` 11→**14 例**全绿——新增 park 途经点等实测到位再推进（钉住重发
断言）、实测长期不到超 `homing_via_hold_s` 放行、init 不加门限三例；路径构建测试断言改为
**倒序**（[init_pose, WAY2, WAY1, 零]）；闭环滞后模拟：旧行为命令在 waypoint 只停留 1 tick
(0.02s) 即反向，修复后钉住等实体进入 0.12 rad 才走下一段；reanchor 5 例全绿。左/右臂 yaml 补
两参数；**机器人侧需重新 colcon build 生效**（本机 install/ 为 9/3 旧构建，连 /teleop/home 都无）。

**astral_pim_ik 收尾：首次真实执行 torch 路径暴露并修复 3 个真 bug + F1 兜底落地 + F2 关闭 + 小规模训练闭环打通**——
`astral_pim_ik`。**背景**：该功能包此前在无 torch 机器上开发（文档引用的 `/opt/anaconda3/envs/MujocoSim`
本机不存在），`network.py`/`kinematics.py`/`train.py` 只过语法 + 静态审查、从未执行。本机实测
`arm_sdk`（pinocchio 3.8 + torch 2.13 cpu）与 `lerobot`（pinocchio 3.4 + torch 2.11 cu130 + 4090D）
均有 torch，全部未完成项可补。**现象①（9D 编码行列序 bug）**：首次跑 `test_network.py` 即 2 FAIL——
`transform_to_9d` 把 (3,2) 旋转块行主序拉平得 `[r00,r01,...]`，与 docstring/`rotation_6d_to_matrix`
的列主序 `[r00,r10,r20,r01,r11,r21]` 不符，恒等位姿编码成退化矩阵。**修**：拉平前 transpose +
非恒等旋转 round-trip 断言。**现象②（L_elbow 变量遮蔽）**：真跑 `train.py` loss 卡 ~14 不降、网络
几乎不学 ψ——`PhysicsInformedLoss.forward` 里 `B, W, _ = pred_psi.shape` 把形参 `W`（腕部张量）
遮蔽成窗口长度 int，肘误差对 int 广播成 ~13 m 假项（同批数据手动肘误差 mean 0.11 m vs `loss_fn`
报 12.97）。**修**：解包改 `T` + 回归测试「perfect ψ → L_elbow<1e-3」（实测 1e-4 m）。
**现象③（IID 数据不可学）**：IID 随机位形数据 ψ 学不动（误差 52.8°、loss 平台 ~1.15）——单帧
T_ee 不决定 ψ（肘在 S-W 轨道圆上自由），窗口网络需要帧间运动信号。**修**：新增
`generate_trajectory_dataset`（平滑关节漂移，`travel`/`wobble` 控速，schema 同
`generate_dataset`），文档明确训练必须用轨迹数据。**F1 兜底**：`solve(fallback_scan=True, n_scan=36)`
网格扫最近可行 ψ（默认关保持 None 契约，救援计入 `fallback_count`），`solve_trajectory` 透传，
test_deterministic_solver 新增用例 8/8。**F2 关闭**：torch/numpy 肘点一致性 **9.58e-07 m**（<1e-5）。
**训练闭环实测**（lerobot/4090D，时间相干轨迹 20k 帧/40 ep）：ψ err 27.1°（median 19°）、
Joint MAE 10.0°、solve 率 74%、位姿 <0.06 mm；oracle（真值 ψ）100% solve——stage-2 几何 IK 精确，
残差为合成数据宽肘先验的信息瓶颈，真机遥操录制是生产路径。5k 帧 seed 对照 ψ err 15.2°（方差来自
小 val 集）。**包布局修复**：`package.xml`/`setup.py` 原埋在 `src/astral_pim_ik/src/astral_pim_ik/`
（多一层 src），上移到包根 `src/astral_pim_ik/`——与其余 20 包一致，colcon 可发现，PYTHONPATH 与
文档对上了。**验证**：test_network 4/4、test_deterministic_solver 8/8、test_dataset 3/3、
test_pipeline 2/2、test_geometric_ik 10/10（回归）；全部实测数字见包 `CLAUDE.md` §5.1 与
`docs/adversarial_review.md`（F9/F10/F11 三条新发现）。

**阻尼释放后再就绪/归零仍回跳——根因在板端陈旧目标，补"切回 POSITION 先重写实测位姿"**——
`astral_robot_control` × `astral_robot_sdk`。**现象**：启动后点「阻尼释放」正常，手动拖臂后
再点「一键就绪」或「归零」，臂仍**抽一下快速回到阻尼释放前的位置**（上轮"陈旧命令流抢占"
三层修复后依旧）。**根因**：上轮修的是 **ROS 侧**（driver 缓存 `_clear_cmd_cache` + 臂节点
disarm 停流），但漏了**板端内部的目标寄存器**——阻尼（motion_mode=0）只是让板端忽略 0x90
位置指令，并不清空其内部保存的"上一次目标"（= 阻尼前位姿 P）；手动拖臂只改实测、不改板端
目标。`~/ready`（SDK `one_click_ready` 内部 `set_motion_mode(POSITION)`）与 `~/home`
（显式 `set_motion_mode(1)`）一切回 POSITION，板端立刻重新追踪 P → 臂抽回旧位姿；此时
`enable()` 轮询最长 3s，期间臂就停在 P 上，归零要么迟到要么因电源位未确认被跳过。**做法**：
① SDK `one_click_ready(..., seed_from_current=True)`——POSITION 切换后、enable 轮询前，用最近
一帧实测关节角 `set_target_positions` 重写板端目标，POSITION 进入即保持当前位置，随后归零
照常下发（默认 False 保持旧行为）；② driver 新增 `_seed_target_from_current()`（`_read_q18` →
`split_full_q` → `move_arm_js` 双臂），在 `~/position` 与 `~/home` 的 `set_motion_mode(1)`
之后、显式目标之前调用——「位置保持」真正保持被拖拽后的当前位置，「归零」先定住再收回；
③ `~/ready` 传 `seed_from_current=True`。**验证**：`test_driver_services.py` 13→**16 例**全绿
（新增 home 阻尼切回先重写实测位姿再归零的顺序断言、position 阻尼/非阻尼两例都重写目标）；
SDK `one_click_ready` 签名向后兼容（旧调用不带参行为不变）。

**真机遥操手感/可达调优（当前 yaml 生效值；配置项，无 A/B 数字，留待实机复核）**——
`astral_arm_teleop`。左右 yaml 各四处：`workspace_radius` 0.55→**0.65**（腕目标球约束
放宽，末端可达范围更大）；`reach_margin` 0.01→**0.001**（肘伸直软墙基本放平，逼近 URDF
伸直极限才挡）；`motion_scale` 0.8→**1.0**（VR 位姿增量 1:1 无缩放）；`rot_smoothing`
0.25→**0.35**（旋转低通加强，τ≈14→19 ms @50 Hz）。`arm_teleop` README 参数默认值说明
同步为当前值。

**数采"坏段默认隔离 + 空录/吞指令在线告警"——堵坏数据进训练集的通路**——
`astral_data_collect` × `astral_web_monitor` × `scripts/vla_process_session.sh`。
**动机**：对抗性审查 start→stop→start→stop 边界时序（逐拍验证端头/端尾/接缝均干净：
尾部先 drain 后 close 不丢帧、头部清缓冲+门控不收旧数据、缝内帧按段独立对齐零伪影），
真正毒化通路在别处——①一条龙 validate **默认不隔离** fail 段（align 的 too-short 阈值
仅 `n_frames < 2`≈33ms，0.5~2s 短段/体检差段直接进转换产物进训练集）；②源未就绪空录
无即时告警（low_fps 只管参考相机半速，管不到数值/图像全 0）；③SAVING 期被吞的 start
无留痕（web 按钮有 disabled 视觉，键盘/VR 没有——"以为开了实际没录"）。**做法**：
①`vla_process_session.sh` validate 默认 `--apply`（fail 段移入 `session/quarantine/`，
移动不删除可逆；旧 `--apply-quarantine` 兼容 no-op），隔离后仍有 fail 即 `exit 1`
中止转换；`--no-quarantine` 显式放行旧行为；②节点录制启动 ~2s 空录检查（段级一次性，
阈值对齐 F5）：数值/图像全 0→"未收到任何样本"、无数值→teleop 未发布、无图像→抽头
未发布，进 state JSON `empty_warning` + 节点 `EMPTY-REC` WARN + web 红条（begin 复位/
end 清空）；③状态机拒绝的指令计数入 state JSON `ignored`（start/stop/discard/next/
pause/resume 六键；pause/resume 原静默 no-op 现也计数留痕）；④web 卡片新增
`empty_warning` 红条 + `ignored` 忽略指令 chip，`types.ts` 同步（`mapUiState` 整对象
透传无需改）。**验证**：`test_node_guards.py` 9→**12 例**全绿——空录全空 e2e 订阅
`/data_collect/state` 断言 JSON 带 `empty_warning` 且丢段后复位、部分缺失只告警缺失侧
（有数值无图像→图像文案）、数值+图像流入不误报（`_quiet_start_checked` 确认检查真执行）、
吞指令计数 `{"start":1,"pause":1,"stop":1}` 进 JSON；`test_collect_smoke.py` 2 例通过；
前端 `npm run build`（tsc+vite）通过、dist 重建。机器人侧无需重装（脚本/话题/前端改动），
web 新 dist 需同步部署机。

## 2026-09-03

**HOME 归零卡顿/抽动修复：park 从"贴实测迈步"改纯开环 + 冻结守卫**——`astral_arm_teleop`。
**现象**：上一轮修好"电机未使能"后 HOME 第一次真正跑起来，但臂**慢慢的一卡一卡**地动，还
**突然抽一下**，和当初启动归位被贴实测钳制时的异常很像。**根因**：HOME park 分支每拍
`base = 实测关节 state_q`、`q_cmd = 实测 + 一个 tick 步长`——命令永远只领先实测一个 tick 步长，
等于把平滑位置控制改成"等实测挪一步才挪一步"的**阶梯采样**：实测经板端读码→UDP→ROS→节点有
几十 ms 延迟，q_cmd 实际推进被实测采样节拍门控 → 速度被压到远低于设定的 0.6 rad/s（慢）；
命令按实测帧节拍一跳一跳 → 粘滑/卡顿，静摩擦突破时"抽一下"。这套"贴实测"钳制当初为启动 init
引入、因同样症状已回退成开环，但 **park 保留至今，而 HOME 此前从未真正成功跑过**（先报未使能、
再被陈旧命令流抢目标），直到这次才暴露。**做法**：① park 与 init 一致改**纯开环匀速推进**
（base = q_cmd，唯一被实机验证的平滑路径）；② 反馈只做**跟随便用**——新增参数
`homing_follow_tol`（默认 0.25 rad）：park 运行中实测落后指令超过阈值（电机失能/堵转没在跟随）
→ **冻结轨迹**，q_cmd 不空跑一路领先到零位，领先被钳在 tol 内；实测追近（恢复跟随）自动解除
继续收回，不会猛扑；正常跟随稳态滞后远小于 tol，守卫不介入；③ 状态字段 `_park_frozen` 每次
`_go_home` 重置。**验证**：`test_home_park.py` 7→**11 例**全绿——新增健康跟随开环平滑推进、
实测停住轨迹冻结（领先钳在 tol 内）、实测追近后解冻继续收回三例；reanchor 5 例全绿；README
`homing_track_state` 语义说明同步。

**阻尼/归零/一键就绪"陈旧命令流抢占"修复**——`astral_robot_control` × `astral_arm_teleop`
× `astral_web_monitor`。**现象**：启动后点「阻尼释放」正常，手动拖臂后再点「一键就绪」，
臂直接回到**阻尼释放前的位姿**；阻尼释放后点「归零」**无效**。**根因**（非重复使能）：
① 阻尼 = 板端 motion_mode=0，位置指令被忽略——`~/home` 只发一次性 `set_all_joints_zero`，
在阻尼态静默无效；② 归零/一键就绪只是**一次性目标**，而 100Hz 控制定时器在
`command_timeout_s` 新鲜窗口内持续重发缓存的"阻尼前目标 P"，或遥操臂节点仍在 armed/
启动归位 homing 持续发流——一旦运动模式切回 POSITION，陈旧 P 立刻重新接管、覆盖刚下发的
归零 → 臂回到阻尼前位姿。**做法**（三层）：① driver `_srv_home` **先切回 POSITION**
（motion_mode=1）再归零（阻尼后归零生效），未上电明确报"先 ~/ready"而非静默；② driver
新增 `_clear_cmd_cache()`——`~/ready`/`~/home`/`~/damping`/`~/estop`/`~/enable` 执行时清掉
缓存目标，100Hz 重发只复读显式目标（零位），不再把陈旧 P 顶回去；③ 臂节点 **operator
disarm 取消进行中的 homing/park**（`_on_disarm` 中 `_homing=False`，否则取消命令流仍持续
下发覆盖后续归零）；④ web 五个硬件模式端点（急停/阻尼/一键就绪/归零/位置保持）**先自动
发布 `/teleop/disarm`** 再调 driver（与 HOME 端点同款）——手动模式即交还控制，之后用
「开始遥操」重新捕获 vr_init。**验证**：`test_driver_services.py` 7→**13 例**全绿（阻尼中
归零先切 POSITION/缓存清理/未上电报错）；`test_home_park.py` 7→**9 例**全绿（disarm 取消
进行中 homing）；reanchor 5 例、web monitor 4 例全绿；前端提示同步并重编 dist。

**HOME 使能修复：~/enable 幂等化 + 板端在线即算下发成功**——`astral_robot_control` ×
`astral_web_monitor`。**现象**：启动正常到设定位置（电机已使能）后点 web **HOME**，报
"电机未使能"、HOME 不下发。**根因**：HOME 端点先调 driver `~/enable`，而 SDK `enable()`
靠轮询板端 obs 帧的 `robot_powered` 位确认——该位在此板子常不置位（SDK demo 同款
"enable 未确认仍继续"）；auto_ready/一键就绪已使能后再 enable = **重复使能 + 确认位失灵**
→ 服务回 "enable not confirmed" → web 把已使能误报成未使能。**做法**：① driver `_srv_enable`
**幂等化**——`_robot_powered` 已 True 直接成功返回（跳过 WORK→POSITION→enable 重复下发，
仅当运动模式非位置时补切 position）；② 未确认但**板端在线**（obs 帧持续）也算下发成功
（命令已送达，对齐 demo「未确认仍继续读反馈」），仅真正离线才硬失败；③ web
`_ensure_driver_enabled` 注释/文档同步（HOME 先 enable 再 disarm+park 语义不变）。
**验证**：新增 `test_driver_services.py` 7 例全绿（已上电零下发/阻尼补位置/确认 OK/在线未确认
=成功/离线=失败/dry_run/未连接），arm home-park 回归 12 例、web monitor 4 例全绿。

**web 启动流程回退（急停恢复方案收敛到 HOME 按钮）**——`astral_web_monitor` ×
    `astral_arm_teleop`。**现象**：上一条「启动/重启自动调 `~/enable` + 启动归位贴实测」实机
验证无效——web「停止→启动」后直接 **503 service** 且臂不动。**根因**：① `/start` `/restart`
在 launch 起来后同步等 `~/enable` 确认（12s 预算），driver 尚未 ready 时超时抛 503——启动本应
"只把栈拉起来"；② 启动 init 归位也被 `homing_track_state` 钳到实测关节，电机一旦没立刻跟上，
q_cmd 逐拍被实测拽回 → 臂"原地不动"。**做法（回退 + 保留）**：① `/start` `/restart` **回退**
到改前——不再自动调 `~/enable`、不再 503；`~/enable` 服务本身保留，仅 **HOME** 端点使用
（先使能再 disarm+收回零位）；② 臂节点 `_homing_tick` 的贴实测钳制**收窄到 HOME park 专用**
（`self._homing_mode == "park"`），启动 init 归位恢复改前**开环逐拍推进**——web 启动只拉栈，
电机使能交给 driver auto_ready/一键就绪，臂正常往 init 走；急停后的"使能+收回零位"统一走 HOME
按钮。**验证**：`test_home_park.py` 由 5 例扩到 7 例——新增 `test_init_homing_advances_even_
when_state_frozen`（实测冻结时 init 仍开环推进，50 tick 前进 ~0.1 rad）与 `test_park_homing_
holds_until_robot_moves`（park 模式下实测不动 q_cmd 不空跑，50 tick 误差不动），全绿；arm teleop
回归 13 例通过；web monitor 单测 4 例通过。

**真机遥操手感调参：两级低通串联改单级**——`astral_arm_teleop` × `astral_robot_control`。
**现象**：遥操小动作"先顿后冲"（先迟钝一下再冲上去），静止又有发颤。**原因**：teleop
`pos/rot_smoothing` 与 driver 板卡 `lpf` 是**两级串联低通**，时间常数叠加、噪声抑制与跟手
矛盾。**做法**：噪声抑制收敛到 teleop 一级，板卡近直通——teleop `pos/rot_smoothing` 0.4→
**0.25**（按 50Hz 标定 ≈ τ14ms）；driver `lpf_alpha` 0.35→**0.85**（100Hz ≈ ~2ms 近直通）。
注释记录回退阶梯：静止发颤回 0.35~0.4 / `lpf 0.6~0.7`；大动作仍觉肉再降 0.15~0.2 / 升 0.95
（`lpf_enable:false` = 全直通）。三处 yaml（左右臂 + `astral_robot.yaml`）同步。

**web 急停后恢复不归位修复 + HOME（归位到零）按钮**——`astral_robot_control` ×
`astral_arm_teleop` × `astral_web_monitor`。**现象**：web 端急停（真断电 disable）→ 停止 →
再启动，机器人不回零，直接锁在当前位置。**原因**：急停后电机失能；重启时 driver `auto_ready`
的 one_click_ready 未可靠确认上电，且启动仅起节点、无"使能=遥操版一键就绪"步骤，臂节点的
归位轨迹在电机失能期"内部空跑"领先实体。**做法**：① driver 新增 `~/enable` Trigger 服务
（WORK→POSITION→enable，**不回零**——遥操恢复归位路径专用，避免与 teleop 正在发布的
joint_commands 轨迹目标抢）；② 启动/重启带真机 driver 的预设后，monitor 等待 `~/enable`
服务出现并幂等调用（`DRIVER_ENABLE_WAIT_S` 12s 预算，无 driver/sim 自动跳过）——
`_preset_has_driver` 按 `with_arm_driver` 或 package 判定（**实机 503/臂不动后已回退**：
web 启动只拉栈、不再自动 enable，见当日顶部条目）；③ 臂节点归位贴实测 joint_states
（新参数 `homing_track_state` 默认 true）：fresh 且贴近 q_cmd 时每拍从实测位姿迈步，电机未使能
不动时 q_cmd 不再空跑，使能晚到不会猛扑（**启动 init 路径已回退开环**，此钳制仅保留 HOME park，
见当日顶部条目）；④ 新增 **HOME / park-to-zero**：臂节点订阅
`/teleop/home`（一次性 volatile，语义同 `/teleop/start`）+ `~/home` 服务，`_go_home` disarm
后沿 **init_pose → init_waypoints → 零位** 慢速收回，到零位把机器人原点锚到 FK(零)，超时保持
当前 q_cmd 绝不硬发零目标（新单臂/双臂通用测试 `test_home_park.py` 5 例全绿）；⑤ web monitor
`config.py` 增 `TOPIC_HOME`/`DRIVER_SRV_ENABLE`，monitor_node 发布 `publish_home` 并把
`enable` 纳入 `_DRIVER_SERVICES`；⑥ `web_server.py` 新增 `POST /api/v1/teleop/home`（先使能
后 disarm+home，仅 RUNNING/PAUSED 可用），前端预设管理区「启动/停止/重启」旁加 **HOME** 按钮
（`api.teleopHome`，purple，与急停区分）。真机流程：急停→停止→启动（拉起栈，不再自动使能）→
按需点 **HOME**（自动 enable + 收回零位）→ 一键就绪/`/teleop/start` 重新遥操。

**新增推理功能包 `astral_policy_inference`（策略部署 / 数据真机回放 / 人在环路）**。
本包接遥操数采与训练上游做推理闭环：模型无关后端（openpi 远程 ws `host/port` / lerobot ACT 进程内
`checkpoint_dir` / stub 冒烟），**换模型=只换后端参数**；三种引擎模式 `queue_sync`（阻塞重填）/
`queue_async`（后台预取）/ `rtc`（后台滚切+min_tail+延迟补偿跳行）；布局真源
`astral_data_collect.schema.CollectSchema`——观测订阅/state 向量/`split_action` 指令拆分全自动，
换机器人配置只改同 schema 参数。回放读 LeRobot v2.1 目录或 `aligned_data.h5`，`PlaybackSession`
步进 + 中断续播 offset 重锚。HITL：`_cmd_takeover` 先对每个遥操节点调 `~/reanchor`（效应）全部成功
才提交 HUMAN（仲裁先效应后提交）；暂停→增量 VR 接管→交还策略/回放；release/resume 时策略按实况
重规划、回放按实况重锚，避免陈旧 chunk 跳变。`astral_arm_teleop` 新增 `~/reanchor` 服务（同步重记
机器人原点=FK(当前关节) 与 VR 零点=当前 VR 位姿）。安全兜底 `SafeExecutor`（NaN/比值 [0,1]/
关节限位/`max_joint_vel` 限速）独立于策略。修复三个易踩坑：rclpy 无 struct 参数 →
`camera_map` 用 JSON 字符串；`__init__` 顺序清空 `_backend_cfg` 导致 make_backend 缺参；re-anchor
`service_is_ready` 误判就绪后阻塞等响应 → `call_async`+3s 轮询。验证：最初 74 例单测全绿（含进程内
mock 机器人 + stub 后端 + 真实 Trigger 服务的 node_flow：策略起停/暂停恢复重规划/回放两路径/
接管成功与失败路径），另有 `astral_arm_teleop/test_reanchor_teleop.py` 覆盖 `~/reanchor`。
实现期后续修复/补强（同批次）：① `pinch_gripper_node` 新增 `disarm_topic` 仲裁门
（默认 `/teleop/disarm`，空串=旧行为），策略在 POLICY/PLAYBACK 期间独占夹爪指令话题、进入
HUMAN 时 policy_node 发 disarm=False 放行真人捏合/扳机——否则夹爪遥操与策略抢写
`/left_gripper/command`；② `ActionEngine.stop()` 在锁内 close backend、queue_sync 边界重填
整段持锁推理，消除"控制线程推理中另一线程停引擎→并发关后端"竞态；③ 刚进播放立刻暂停且尚无
指令时 `_cmd_pause` 兜底 `_hold_current()`（以实测姿态冻结）；④ 引擎/节点恢复语义：暂停后
resume 策略重建 chunk、回放重锚 offset（防陈旧行跳变）。MuJoCo 真话题端到端冒烟
`scripts/run_inference_sim_smoke.py`（astral_mujoco_sim + policy_node stub 全链路：policy 驱动
arm/gripper→pause 夹持值稳定→resume 恢复变化→h5 回放逐帧一致→播完自动 IDLE）**全部通过**。

**对抗性审查修复（首轮 14 项，本轮全处理）**——`astral_policy_inference` × `astral_arm_teleop`
× `astral_gripper_teleop`。① **`/teleop/disarm` 电平语义冲突（critical）**：arm/head 遥操
`_on_disarm` 对任何 Bool 都 disarm，而 policy 进 HUMAN/IDLE 发 disarm=false 开闸，把刚
re-anchor 武装的遥操又拆掉 → arm/head 只认 `Bool(true)`，与 web/pinch 消费方对齐；② stop→
IDLE 发 disarm=false 重开仲裁门（TRANSIENT_LOCAL 闩锁不再永久关死夹爪门）；③ pinch 门控回归
web pause→resume：pinch 新增 `arm_topic`（`/teleop/armed`=true 也恢复），默认配置同时设
disarm+arm；④ `_cmd_playback` 先 load+`action_dim`/`state_names` 校验再 request→teardown，
失败原地保持（不再"POLICY+引擎 None 静默冻结"）；⑤ `_start_policy` 先验证 obs→冻结旧引擎→
新引擎成功后 disarm+换新，失败 `revert()` 恢复（Controller 快照含 `_interrupted`）；⑥ POLICY
运行期 state 连续缺失 >`obs_stale_stop_s` 自动暂停，杜绝陈旧观测盲推；⑦ `~/cmd` 入队 + 控制
定时器持 `RLock` 串行 drain+dispatch（仲裁回调与控制循环不再交错双写，接管改为事件驱动轮询
re-anchor、失败事务性 re-disarm）；⑧ 接管先验新鲜 state，避免陈旧 q_cmd 锚点跳变；⑨
`_run_plan` 整段持锁推理，`stop()` 锁内 close，消除 infer/close 竞态；⑩ 二次回放不再跳过
末帧保持（`_hold_end` 在 playback/stop 清理）；⑪ 回放带 schema 时校验布局命名顺序（同维不同
块序拒绝）；⑫ smoke 驱动反馈泵按 disarm 闩锁门控、断言策略夹爪比值且不再被 0.5 回显污染、
回放 leg 断言轨迹末帧=记录末行。验证：全套 79 例单测全绿（新增 obs 陈旧自动暂停/disarm 电平
顺序/接管失败事务性回滚/回放 schema 拒绝/连续回放保持 等 node_flow 回归），
`astral_arm_teleop/test_reanchor_teleop.py` 5 例全绿。

**LeRobot v2.1→v3.0 升版转换器（现代 lerobot ACT 直接训练）**——`astral_data_collect`
新增 `convert_to_lerobot_v3.py`：把 openpi 用的 v2.1 数据集**原样升版**为 v3.0，
供 `VLA/lerobot`（>=0.6，ACT 与其它策略共用读取管线）直接训练——其读取侧对 v2.1
直接 `raise BackwardCompatibilityError`。布局/数值语义逐字段镜像官方
`scripts/convert_dataset_v21_to_v30.py`：data/episodes 按 chunk 分段合并
`file-*.parquet`、同 chunk 的段视频用 PyAV `ffconcat` **流拷贝串接不重编码**、
时间窗按各段实测时长叠加、tasks.jsonl→`tasks.parquet`、episodes.jsonl→
`meta/episodes/chunk-000/file-000.parquet`、v2.1 专用 info 字段清掉换成 v3.0
模板；`stats.json` 原样沿用（无需重算）。**源 v2.1 目录只读**，输出独立目录默认
拒覆写（`--overwrite` 重建）。验证：与官方脚本同源产物逐文件一致；现代
`LeRobotDataset` 加载 3 段 1828 帧、逐帧解码逐像素与官方产物一致；`lerobot-train
--policy.type=act` 冒烟 2 步在 CPU 上真实跑通并落 checkpoint（前向/反向/保存全过）。
`vla_process_session.sh` 增加 `--act-output <dir>`（可加 `--overwrite-v3`）一条龙
串接升版。新增 `test_convert_lerobot_v3.py` 7 例回归：目录布局/info 字段迁移、
tasks.parquet、数据 parquet 行数+非视频列集不变、episodes 元数据区间/tasks/
length、视频合成帧数=各段之和+时间窗首尾相接、stats 原样拷贝、守卫（覆写拒绝/
非 v2.1 源/输出在源内）与源目录只读。离线回归全套 59 例通过。

**改采集配置后同目录重转的静默污染修复**——`convert_to_lerobot.py` ×
`convert_to_lerobot_v3.py`。对抗审查发现：改 `data_collect.yaml`（加右臂/换手/
段数变少）后把 v2.1 重转进**同一输出目录**，旧 `data/`、`videos/` 的 episode
文件不会被删（只截断 meta jsonl），而 v3 升版按目录遍历文件 → 把**旧布局的行**
（如单臂 8 维）与新高维行混进同一个 parquet，静默产出损坏数据。修复两层：
①`convert_session` 视输出目录为数据集专属目录，开始前清掉旧 `data/`、`videos/`
（与官方"输出即替换"语义一致）；②`convert_to_lerobot_v3` 以
`meta/episodes.jsonl` 为权威 episode 清单，数据/视频文件集与清单不一致即响亮报错
（列出多余残留下标与缺失下标并提示换新目录），不再静默跳过或合并。新增 2 例回归
（重转清残留后升版一致性 / 脏源拒绝），离线回归全套 **61 例**通过。

**sim README 对齐当前默认 + 关节 3/5 限位放宽**——`astral_mujoco_sim` × `astral_arm_teleop` × `astral_robot_description`。①README：默认求解器描述从 `urdf_numerical` 修正为 **`geometric`**（yaml 实际默认，含 `use_human_elbow` 先验说明）、init_pose 描述改为「代码默认非零位、`astral_mujoco_sim.yaml` 覆盖为零位看 homing 过程」、MJCF 表补 `mjcf_path` 回退逻辑。②URDF/MJCF/`analytic.py` 四处一致放宽 joint3/5：±1.57 → **±2.2689 / ±1.7802**（`astral_arm.urdf`、`astral_robot.urdf` 及各自 `.pin.urdf`、内嵌 `RobotMain_URDF.pin.urdf`、`astral_dual.xml` 关节 range + actuator ctrlrange）。③`test_geometric_ik` 满伸 sweep 用例对齐逃逸迟滞设计：>0.15 rad 跳变仅当 `esc_active`（分支逃逸）时放行并打印 note，无逃逸的真跳变仍 FAIL；离散化与 seed 选择同为 5mm，避免更细步长撞进合法的窄姿态口袋。

## 2026-09-02

**roll 逃逸迟滞参数化 + 逃逸可观测**——`astral_arm_teleop`。`ik_escape_after_frames`（默认 4）/`ik_return_after_frames`（默认 20）从 `geometric.py` 常量提升为节点参数，工厂/求解器全链路透传，可经 `ros2 param set` 热改（调大 = 越不易甩肩但"卡住不跟"窗口越长；调小返回 = roll 结束更快回人臂角）。逃逸发生时打一次 WARN 并计入 `[Latency]` 行 `psi_escape` 计数——真机肩部突动先查这个，不再是无声的瞬间跳变。验证：`test_geometric_ik` 9 例全 PASS（含 `roll_escape_hysteresis`），工厂注入 7/33 生效、默认 4/20 不变。

**指纹查询 CLI：`python3 -m quest3_video_streamer.scan`**——在机器人上直接打印每路采集设备（节点名/设备路径/fourcc）的全部稳定指纹（by-id → by-path → sysfs），抄子串进 `label_aliases` 即可，替代手工 `ls -l /dev/v4l/by-id/` 再对节点。无采集设备时提示 no capture-capable device。无相机环境冒烟通过。

**label_aliases：auto_scan 换口/重插防漂移落地（别名表方案）**——`quest3_video_streamer`。在昨日「auto_scan: false + 命名块 + by-id」备选写法之上，补上两全方案：保持 `auto_scan: true` 的自动发现，新增 `label_aliases` 参数（字符串数组 `"指纹子串=稳定label"`），扫描完成后按设备指纹（`/dev/v4l/by-id` → `by-path` → sysfs 名，子串匹配，先列先赢）改写 label；改写发生在 per-label 覆盖块查找之前（`video8.preset` 等同名块自动生效）。语义细节：目标名=匹配设备当前内核名 → 合法 no-op（固化现状，且消耗该设备，后续规则不再对其生效）；目标名撞上**其他**设备的内核名 → 拒绝并告警（防止重名）；一条规则匹配多台 → 只改第一台并告警；规则零命中 → 告警（指纹写错或相机未插，开录前看日志一眼）。`scan.py` 新增 `stable_fingerprints()`（纯函数）与 `apply_label_aliases()`（支持预置 fingerprints 注入）；`streamer_node` 声明 `label_aliases` 参数并接入 `_build_sources` 的 auto_scan 分支。新增 `test_label_aliases.py` 12 例（指纹顺序/兜底 mock、命中/未命中/固化现状/多设备/冲突/畸形规则/集成断言——rclpy 缺失自动 skip），streamer 全套 32 例通过。使用：`params.yaml` 加 `label_aliases: ["Intel_R._RealSense=video8", ...]`，换口/重插/重启后 data_collect 与 openpi camera_map 永远零改动。

**VLA 一键脚本 + 相机 label 防漂移 + 默认配置切单左臂**——`scripts/` × `astral_data_collect` × `quest3_video_streamer`。①新增 `vla_process_session.sh`（对齐→校验→转 LeRobot 一条龙，走 openpi uv venv，`--image-size`/`--apply-quarantine` 可选）与 `openpi_train.sh`（norm stats 重算 → 训练，`lora|full` 二选一，前置检查数据集软链与 pi05_base 权重）。②`params.yaml` auto_scan 覆盖块加「换 USB 口防漂移」注释写法：`auto_scan: false` + `/dev/v4l/by-id/` 稳定路径 + label 沿用 `video8/video0/video2`，数据集 key 不变下游零改动。③`data_collect.yaml` 默认切实际采集配置：单左臂 + 左夹爪、`cameras: ["video8", "video0", "video2"]`（auto_scan 节点名，双臂配置注释保留）。回归：转换/schema/web 31 例 + node_guards 9 例 + 对抗配置 8 例（openpi uv 环境跑通）全绿。

## 2026-09-01

**换配置全链路对抗测试 + openpi 侧布局硬编码清除**——`astral_data_collect` × `VLA/openpi`。新增 `test_adversarial_configs.py`（8 例，独立临时 HF_LEROBOT_HOME，不碰真实数据集）：双臂+双夹爪+三相机（16 维）、左臂+wuji 灵巧手+腰+头（31 维，贴近上限）、双臂+腰头+15fps+非对称相机映射（右腕映射/左腕零填充）、NaN 段校验隔离后训练侧只剩健康段、31 维 norm stats 全量计算+归一化回归，以及三个负例（35 维超限必须拒 / camera_map 指向不存在相机必须拒 / 缺 meta 清晰报错）。每例在 openpi 侧逐维验证：delta 掩码与 schema 期望逐位一致、delta 语义为「相对 chunk 起始 state」（含 k≥1 帧）、`AstralOutputs` 截断维=state_dim、相机槽位按 conftest 颜色指纹验证接线正确、空 task 段 default_prompt 兜底分词与正常 task 段不同。同步修复：openpi `LeRobotAstralDataConfig` 原来写死 8 维截断、`(7,-1)` delta 掩码、video8/video0 相机映射——现改为 `create()` 时从数据集 `meta/info.json` 现读（state 维度/names 推导，`_ee_` 保持绝对其余转 delta，相机走 `camera_map`）；并修正环境变量口径：pinned lerobot 0.1.0 只认 `HF_LEROBOT_HOME`（设旧名 `LEROBOT_HOME` 会直接 raise），meta 解析与文档同步对齐。全套 53 例回归通过。


**转换期图像 letterbox 到 224×224**——`astral_data_collect`。`convert_session` 新增 `image_size`（默认 224，`--image-size 0` 保留原分辨率）：视频落盘前按 openpi `resize_with_pad` 的几何约定等比缩放 + 对称黑边（余数归下/右），训练时 `ResizeImages(224,224)` 变恒等操作，模型输入与"全分辨率入库 + 在线缩放"逐像素一致，但省掉加载侧解码大图 + 缩放的 CPU 开销；图像统计口径同步改为 letterbox 后的帧。存量 3 段重转后数据集 283MB → 20MB（-93%），openpi 管线加载验证通过（原生 224 直入）。新增 2 例测试（letterbox 几何对齐 openpi 约定 / 默认 224 端到端尺寸与黑边断言）。

**转换产物 parquet 去掉视频 struct 列（OpenPI 下游适配）**——`astral_data_collect`。v2.1 规范的 parquet 只含非视频列（视频帧由读取侧用 `timestamp` 列 + `meta/info.json` 的 `video_path` 模板解析，`get_hf_features_from_features` 对 `dtype=="video"` 直接跳过）；此前为"兼容性"多写的 `struct{path,timestamp}` 列会让 openpi 锁定的 lerobot 0.1.0 在 `hf_transform_to_torch` 的 `torch.tensor(dict)` 处崩溃（`Could not infer dtype of dict`）。转换器删列、存量 3 段 parquet 原地重写（未重编码视频）、schema 测试改为断言**不得**出现 `observation.images.*` 列。修复后 openpi 数据管线端到端验证通过：1828 样本加载、AV1 视频解码、action=next_state 语义保持（max diff 0.0）、pi05 三相机槽位映射 + 空 prompt 兜底分词正确。

**转换管线 OOM 修复：SVT-AV1 限内存 + 图像统计流式化**——`astral_data_collect`。实机 3 段 1080p 数据转换在小内存机（15GB）上被 OOM 杀掉（exit 137，峰值 RSS 6.3GB，且管道下游静默表现为"只转出 1 段、meta 全空、退出码 0"——教训：该命令勿接 `| tail` 看结果，要看退出码）。两处根因：① SVT-AV1 默认按核数并行 + 深 lookahead，1080p 内部缓冲数 GB → 加 `svtav1-params: lp=2:lookahead=16`（参数不识别则退化无参打开，再不行回退 h264）；② 图像统计 `np.stack(≤100 帧)` ≈620MB/相机 → 改 `ImageStatsAccumulator` 流式累加（每通道 sum/sumsq/min/max，输出与堆叠计算数值等价，新增等价性回归 2 例）。修复后峰值 RSS **6.3GB → 1.14GB**，3 段 1828 帧完整转出。

**cmd 流时间戳修复：改用到达时刻**——`astral_data_collect`。validate 对 9/1 上午实机 3 段数据报 F4 全灭：`left_arm_cmd` ~45% 样本戳完全重复（值不同）。根因：teleop `_publish_q` 把 `joint_commands` 的 `header.stamp` 打成上游 VR 输入戳（供 web 延迟面板算 E2E），IK 定时器（~60Hz）比 VR（~30Hz）快时同戳复用。采集端 `*_cmd` 流改为一律用到**达时刻**——指令样本时刻语义本就是「发出时刻」，且 VR 戳比真实发出早一个管线延迟，混用两时钟还会给 command 模式 action 对齐引入交错偏差。state 流仍用 header.stamp（驱动侧时钟）。新增 2 例回归（cmd 用到达时刻严格递增 / state 保留戳）。存量 3 段测试数据的 raw cmd 戳仍是旧的（next_state 模式不消费 cmd，不影响对齐/转换）。

**采集抽头从推送开关解耦**——`quest3_video_streamer`。此前 `~/collect/{label}` 与 preview 共用运行时门控：`push_enabled=false`（或相机被 `active_cameras` 静音）时采集流也断，「关了推送开关想省 CPU 却采不到图」。改为**唯一准入条件 = 话题有订阅者**（编码本来就按需，无订阅零开销），录不录由 `astral_data_collect` 状态机决定；preview/WebRTC 仍走门控不变。效果：推送开关现在只控制「看」（Quest + web 实时画面），与「录」完全正交。

**采集抽头限流器换代：令牌桶**——`quest3_video_streamer`。0.8 容差后仍有 2-3/s 误杀（捕获线程调度抖动偶发 20ms 间隔对）；改令牌桶（容量 2、速率=max_fps）：源≈目标时抖动被吸收、长跑 99%+ 透传，源更快仍限 30/s。仿真：30fps±5ms 抖动 99.4% 透传、60fps 源稳态 30.0/s。实机录制 ~28.8/27.6/27.3fps（中位 29-30）→ 修复后 tap 发布 29.8-29.9/s、rate_skip=0，采集支路 30fps 满速透传实锤。

**回传降载再减半：push_max_width 960→640**——`quest3_video_streamer`。960 宽下 WebRTC 实测 10-13fps/路；降 640（360p，编码像素较 1080p −84%）后 13-17fps/路。降载只作用于 WebRTC track（等比缩放 + 黑帧同尺寸保编码 context），采集抽头全分辨率全速不受影响。yaml 注释内调参阶梯同步更新。

**采集抽头限流器抖动误杀修复**——`quest3_video_streamer`。setter 修复后实机：capture 三路 30fps 钉死、queue_full=0，但 tap 只放行 18-20/s——限流器硬卡整周期（33.3ms），30fps 源到达间隔 33.3±5ms 抖动，早到几 ms 的帧被 rate_skip 误杀 1/3。改 0.8×period 容差：≤37.5fps 的源全通过，更快源仍限在目标附近。**同时记录里程碑**：修复后 WebRTC H264 回传达 10-13fps/路（5.5Mbps，此前 1-2fps），采集/捕获/推流首次三线并发满速。

**全案根因定案：rosidl `data` setter 逐字节校验吃光 GIL**——`quest3_video_streamer`。**py-spy 实锤**：三路 collect-tap 线程全部停在 `sensor_msgs/msg/_compressed_image.py:183-184` 的 genexpr——`msg.data = bytes` 触发 rosidl setter 的 `all(isinstance(v, int) ...)` + `all(0<=v<256 ...)` 两遍逐字节 Python 校验（200KB JPEG ≈ 40 万次迭代/帧，持 GIL 数百 ms）；3 路 × 30fps 把 GIL 占满，捕获线程/事件循环/WebRTC 全部饿死。**这解释了全部历史症状**：采集 video8 3fps、推送开关 ON/OFF 的 16↔30fps（无 Quest 也复现）、三种编码配置吞吐不变（编码器从来无罪——VP8/x264 修复仍保留，是真实加速只是非瓶颈）。**修复**：`msg.data = array.array('B', jpg.tobytes())` 走 setter 的 array 快路径（C memcpy），本地实测 10.1ms → 0.016ms（630×）。collect_tap + preview 两处都改。Jetson cv2 5.0 多线程扩展比 3.9×，GIL 释放正常，洗清嫌疑。

**相机 eager start：采集/预览与 Quest 视频解耦**——`quest3_video_streamer`。此前相机源仅在 WebRTC track 首帧时惰性打开——Quest 不开 video feed，采集抽头就拿不到任何图像。新增 `eager_start_sources`（yaml 默认 true）：service 启动即打开全部源，Quest 连接后 track 复用已开源（`WebcamVideoSource.start`/`RosImageSource.start` 补幂等守卫）。同时支撑"无 WebRTC 负载时采集支路满速"的对照实验（三组编码配置吞吐不变已证瓶颈非编码器，疑似 aiortc 媒体面 Python/GIL 开销拖垮全进程，待 py-spy 实证）。

**H264 上线后流塌缩修复：x264 线程爆炸**——`quest3_video_streamer`。**实机证据**：重排生效（`codec 首优 video/VP8 -> video/H264` ×3、协商 H264）后，流从 ~3fps 一路塌缩到 **0.0fps/0kbps**，且捕获/采集线程照旧被饿死。**根因**：x264 默认 `threads=1.5×核数`——12 核 Jetson 每编码器 18 线程、3 路共 54 线程在弱 ARM 核上 convoy（zerolatency 的 sliced-threads 同步开销在小核上被放大）；对比 VP8 基本单线程所以只是慢不是塌。**修复**：x264 options 加 `threads=2`（3 路共 6 线程，给捕获/采集留核）；本地 540p veryfast threads=2 实测 215fps（x86），Jetson 预期每路 15-30fps。另加补丁幂等守卫（模块重复 import 时不再自包装递归）。

**H264 强转失效根因定案与修复**——`quest3_video_streamer`。**实机证据**：`offer video codecs: ['VP8','rtx','VP9','H264','AV1','H265',...]`——Quest **明明提供 H264**，应答却仍是 VP8（`h264_forced` 空转实锤）。**根因（aiortc 1.15 源码实证）**：`setCodecPreferences` 在应答路径无效——`createAnswer` 直接用 `setRemoteDescription` 时算好的 `transceiver._codecs`（offer 顺序，VP8 在前），发送端 `RTCRtpSender` 以 `_codecs[0]` 建编码器。**修复**：弃用 setCodecPreferences，改为在 setRemoteDescription 之后、createAnswer 之前直接重排 `transceiver._codecs`（H264 及其 RTX 伴随提前；条目来自协商公共集即 offer 参数深拷贝，无 fmtp 不匹配风险）；公共集无 H264 时打日志保持 VP8。新增重排日志 `codec 首优 VP8 -> H264`。新增 2 例单测（重排序+RTX 伴随、无 H264 回退）。

**低帧率根因定案与修复：VP8 软编码挤爆嵌入式 CPU**——`quest3_video_streamer`。插桩证据链：tap `publish=3.4ms`（DDS 无罪）、编码线程有活跑不动（encode 20ms 吞吐仅 7.5/s）、捕获线程进程内 17-20fps（单跑 30）、**协商日志显示三路全是 VP8**（初判 Quest 未提供 H264；当日实机日志证伪——offer 含 H264，实为强转代码在应答路径无效，见上方 H264 条目）——aiortc 给 libvpx 设 `cpu-used=-6`（比默认更慢的画质向档位），三路软编在 Jetson 上吃光 CPU 并阻塞默认 executor，全进程线程互相饿死。修复三件套：① **VP8 `cpu-used=-6→8` 补丁**（realtime 档位上限提速，镜像 aiortc 1.15 原生 options 仅改此一项）；② **x264 preset veryfast 补丁**（dormant：Quest 端 app 开 H264 后立即受益）；③ **回传降载旋钮** `push_max_width`/`push_fps`（默认 960/30 ≈ 编码像素 -59%），track 级等比缩放 + 黑帧同尺寸保编码 context，采集抽头全帧全速不受影响。两补丁带 `encoder_patch_status()` 启动自证行（结构不符静默回退会显示 OFF）；`apply_offer` 新增 offer 编码列表日志（Quest 报了什么一眼可见）。新增 `test_push_throttle.py`（11 例，含补丁激活断言）。调参阶梯：960/30 → 960/15 → 采集时 mute 腕部轨。

## 2026-08-31

**E2E 延迟：QoS 只留最新帧**——腕位 / body / 关节状态 / 指令流改为 `BEST_EFFORT` `KEEP_LAST` **depth=1**。mocap 发布、teleop 订阅、driver/monitor 指令订阅对齐。旧 depth 10/20 会在 IK 跟不上时把旧腕位排队，表现为「手已经停了臂还在走」。Joy/头/hips/landmarks 同步改，避免 BEST_EFFORT 发布对不上 RELIABLE 订阅。

**Web Launch 日志**——环形缓冲 500→**8000**，WS 实时推尾 **800** 行；新增 `GET /api/v1/logs` 拉全量。系统 tab 日志窗 240px→**70vh**，复制/下载。

**运维清场 + 夹爪刷屏**——`jetson_teleop_start.sh` 强清补 `controller_start_gate`、`data_collect`、`keyboard_controller`、`ik_solver`。夹爪 `log_interval_s` 默认/yaml/launch 均 **0.0**，关掉每 2 秒一条的状态日志。

**左臂 TCP 偏置撤回**——`tcp_offset` Y **−0.147 → 0**（与右腕一致）。此前为贴 Quest 左手视觉中心，实测不必。

**数采段边界速率不为负**——writer 每段新建计数归零；`_prev_counts` 随段清零，delta 再 `max(0, …)`，避免下一段第一秒打出负 Hz。

**数采防护体系 + launch 参数优先级修复**——`astral_data_collect` / `astral_web_monitor`。①launch 参数改空串默认 + OpaqueFunction：**yaml 成为唯一默认来源**，此前 launch 默认值（如 `arms` 默认 `left,right`）静默盖掉 yaml——"改了 yaml 却采出旧 schema"即此机制；CLI/预设显式传参仍可覆盖。②**domain 单例锁**：`/data_collect/control` 谁订阅谁开录是双份数据的根因（残留节点 + 再启动 = 双开同录）；节点对 `/tmp/astral_data_collect_domain{N}.lock` 取排他锁，第二个节点构造即拒绝，session 不同也不放过。③**段号原子占位**：mkdir 抢号撞号让位——老残留进程不持锁，段号互斥由文件系统原子性兜底（此前一个 start 双节点各写 000000/000001）。④**多节点红条**：web 按 `/data_collect/state` 发布者计数 >1 即提示残留。⑤**低帧率告警**：录制中参考相机实率 < dataset_fps/2 → state `low_fps_warning` + 日志节流 WARN + 卡片红条。⑥web 重启端点等旧进程真退出再启（单例锁窗口）。另修：cameras 逗号字符串曾被逐字符拆解；jpeg_quality 参数此前被静默忽略。

**auto_scan MJPG 确定性优先（非帧率根因，实机已证伪带宽假设）**——`quest3_video_streamer`。MJPG/JPEG 提至 110 分消除平票依赖枚举序的不确定性。**实机验证**：d435i 彩色节点仅播 YUYV（无 MJPG 可优先）、三相机分属不同总线且 1080p YUYV 单跑满 30fps——采集低帧率非带宽问题；双订阅者收到完全相同帧集合证明丢失在 streamer 进程内部（抽头总共只发了那么多）。`test_scan_mjpg.py` 保留作为确定性回归。

**Web 数据采集卡片 + 独立泳道**——监控页顶部录制控制（开始/停/下一段/暂停/丢弃 + 任务文本）；数采 `LaunchManager` 与遥操预设解耦，互不挡启动。`/teleop/start` 订阅改 VOLATILE 才能收到 web 一次性触发。详见 8/28 VLA 条目补记。

**Web 预设：单左臂 + 左夹爪**——`presets.yaml`「Left arm + left gripper (no right arm)」。`full_teleop` / `astral_dual_arm_teleop` 新增 `arm_side:=left|right|both`（默认 both）；left 时不启右臂遥操节点，mocap 只发左腕。

**Web 管线延迟面板**——`astral_web_monitor` 系统页。只读：mocap 腕姿 stamp 龄期、`ik_solver_*/ik_status` 求解耗时、`joint_commands` stamp 作为 VR→指令端到端；左右各一列，带 Hz / FAILED / 过期灰显。

**左臂 TCP 偏置**——`astral_arm_teleop_left.yaml` `tcp_offset` Y **−0.147 m**（右腕仍为 0）。

**Quest 面板：D435i 缩回、勿挡左侧腕部**——`quest3_video_streamer`。分辨率仍是 1080p30，未改。

- 8/28 `x=0.05` / `size=0.88` 半宽 ≈1.09 m，盖住左上/左下腕部约一半。改回 `x=0.50` / `size=0.60`（间隙约 0.42 m）。腕部左侧叠放不动。重启 streamer 并让 Quest 重连。

**D435i IR 节点带 UYVY 也不当彩色**——`scan.py`。红外立体旁路常挂 UYVY（packed IR 不是 RGB），旧打分会当彩色、靠更低 video 号赢过 RGB 节点。节点上只要有 GREY/Y8I 等 IR fourcc 就打 10 分丢掉。

**大幅 roll 逃逸迟滞**——`geometric`。末端 roll 超腕行程时局部 ψ 窗闪空，旧行为同帧全局逃逸再被 `psi_ref` 拉回，肩部抽搐。

- 局部窗连续空 **4** 帧才允许全局逃逸；逃逸后驻留，home 窗连续可行 **20** 帧才返回（~150 Hz 下约 27 / 133 ms）。冷启动立即全局。`test_geometric_ik` 加 `roll_escape_hysteresis`。

**geometric 默认 + 人臂肘先验**——`astral_arm_teleop`。左右 yaml / 节点默认 `solver_type: geometric`，`use_human_elbow: true`。

- 现象：只跟腕 6D 时冗余臂角由连续性随便锁一支，人转肘平面机器人肘不动；人臂伸直时 IOBT 肘偏置会被当成满权重 ψ，放下手臂肘拧到固定错角。
- 订 `quest3/body_joints` 的 `{side}-arm-upper/-lower`，大臂方向旋到 `*_base_link` 得 `psi_ref`，局部 ψ 窗以它为中心、评分加 `w_psi_ref·|ψ−ψ_ref|`（默认 weight 2.0）。人转腕带肘 → 机器人肘跟转；人只转腕 → 肩肘不动。超时/缺名/不可行自动回退纯连续性。
- 伸直度门控 `human_elbow_min_sin=0.15`（肘尖离肩腕线 ≲4 cm）关先验；肘方向 EMA 0.15 s。Latency 行报 `psi_off`。
- 调参：`pos/rot_smoothing` 0.5/0.6→0.4，`max_joint_vel` 4→6。tune 图加「几何 臂角」按钮。假人臂 `body_joints_sim`（elbow_circle / wrist_spin / combined）。

**肘伸直软墙**——`geometric`。全伸展 S/E/W 共线、q4 撞限位 → IK 无解 → 臂卡一下。

- `reach_margin=0.01`：腕目标径向钳在肩心 `l_se+l_ew−margin` 内，手感是墙不是顿；`sin α < 0.05` 时 ψ 网格塌成上一帧单候选。Latency 行报 `reach_clip`。

**免 DH 几何臂角闭式 IK**——`astral_arm_teleop` 新增 `solver_type: geometric`（当时 yaml 默认仍是 `urdf_numerical`；同日稍后改为 geometric 默认，见上）。

- 现象：`analytic_dh` 靠硬编码 MDH + 关节翻转，和 SolidWorks URDF 对不齐；`urdf_numerical` 贴 URDF 但每拍 LM，遥操 150 Hz 偏重。
- `ik/geometric.py`：q=0 时用 Pinocchio 从 URDF 抽出肩/肘/腕中心与关节轴（`left_base_link` / `right_base_link`），POE 正运动学 + 臂角 ψ + Paden-Kahan 子问题闭式求 q。无 DH 表、无 theta 偏置、无轴翻转；输出已是硬件约定，节点不 `flip_q`。
- 连续解沿用 `analytic.py` 的局部 ψ 窗 + 迟滞 + 全局回退。工厂别名 `swe` / `poe` / `geometric_arm_angle`。
- 离线套件 `test_geometric_ik`：FK 对 Pinocchio 到浮点精度、FK→IK 回环、限位、不可达、轨迹连续性、双臂/单臂工厂。

## 2026-08-28

**Quest 面板：腕部靠左缩小，RealSense 居中放大**——`quest3_video_streamer`。

- 左上/左下腕部 `x -0.6→-0.95`、`size 0.38→0.28`；D435i `x 0.5→0.05`、`size 0.6→0.88`。
- auto_scan 无同名 yaml 块时按 fourcc：YUYV 居中 1080p，其余当腕部左侧叠放 720p（换 USB 口编号变了仍是这套布局）。

**D435i auto_scan 勿推红外**——`quest3_video_streamer`。

- 彩色节点常是 `VIDEO_CAPTURE_MPLANE`，扫描只 enum 了普通 capture，RGB 格式列表为空；同机红外 GREY 得分更高，推流变成红外。
- 同时 enum mplane；USB 接口 `1-2:1.0` / `1-2:1.3` 归到同一设备再比分；得分低于彩色阈值的红外/深度组直接丢掉。

**VLA 数据采集包**——新增 `astral_data_collect`；`quest3_video_streamer` 加采集抽头；`astral_web_monitor` 集成录制控制卡片。

- web 集成：`astral_web_monitor` 监控页新增**数据采集卡片**（开始/停止保存/下一段/暂停/丢弃 + 任务文本 + 录制状态徽标与流率/掉帧显示），REST `/api/v1/collect/{control,task}` 桥接 `/data_collect/*` 话题（纯话题接口，CLI 启动的采集节点同样可控）。
- **数采节点独立泳道（与遥操预设解耦）**：`web_server` 新增第二个 `LaunchManager` 专管数采 launch，`POST /api/v1/collect/launch/{start,stop,restart}` + ui_state `collect_launch`（state/preset/uptime/pid）；数采卡片右上角直接启停节点。数采是纯订阅者，启动跳过孤儿检测（`start(check_orphan=False)`）；`_find_orphan` 对 `astral_data_collect` 命令行对称豁免——两条泳道任意顺序起停互不阻塞。「系统」tab 遥操预设下拉过滤 `astral_data_collect` 条目（避免占用互斥的主泳道），其 presets.yaml 条目保留为泳道启动命令来源。
- 修复：数采节点遥操事件订阅 QoS 由 latched 改为 VOLATILE（DDS 订阅端 durability ≤ 发布端；`/teleop/start` 在 web_monitor 是刻意的 volatile 一次性触发，latched 订阅完全收不到——事件是跃迁语义，仅损失订阅前的历史值）。
- 修复：`replay_rerun` 适配 rerun ≥0.24 API（`set_time(timestamp=)`）+ h5py 数据集句柄逃逸（文件关闭后逐帧读图必崩，此前被缺包 skip 掩盖）。

- 目标：为 OpenPI pi0.5 采集训练数据。四段式：采集（raw HDF5，与遥操并行）→ 离线对齐（严格 1/fps 网格，action=state[t+1] 可切 command）→ 清洗校验（11 条规则 + quarantine 隔离）→ LeRobot **v2.1** 导出（逐字段对齐 `VLA/lerobot @0cf86487`，纯 pyarrow+av 手写、不依赖 lerobot 包）；另附 Rerun 回放。
- schema 采集前配置并冻结进 meta.json：**臂侧可选（`arms`: 左/右/双臂）**、左右末端独立 gripper/wuji/none（跟随所属臂）、腰/头可选纳入；state 布局 `[左臂7?, 右臂7?, 左EE?, 右EE?, 腰2?, 头2?]`，夹爪用闭合比（0..1，同 pi0.5 dim6 约定）。
- 控制：键盘热键（s/q/d/n/p/t）与 `/data_collect/control` 话题双通道；`/data_collect/state` latched JSON 报各流帧率/丢弃数；任务文本每段一条、录制中可改。
- streamer 抽头 `~/collect/{label}`：全分辨率 JPEG，**仅在有订阅者时编码**（不采集零开销），与 preview 同线程模型、跟随门控；参数 `collect_tap`/`collect_tap_fps`/`collect_tap_quality`。
- 测试：36 项离线单测（schema/align/validate/convert 逐字段校验 v2.1 布局）+ ROS 进程内冒烟（假话题 start/stop/discard/pause 全流程）。

**夹爪开合限速 + 扳机整形**——`astral_gripper_teleop`。

- 现象：手柄扳机控制夹爪"一扣到底"，中间态几乎不可见。链路（Unity `Axis1D` F3 模拟量 → Joy → ratio → 驱动每拍绝对角度 0x97/0x98）全程比例，但**没有任何速度整形**，夹爪伺服对每个目标全速赴位，快扣时宏观上就是直接合死。
- `pinch_gripper_node` 新增合并输出限速 `max_ratio_rate`（默认 **2.5 ratio/s**，全行程 ≥0.4 s；≤0 关闭），trigger/pinch 两路合并后的比值按拍斜坡下发；首拍直取，避免持握中恢复时先松一下。比例性保留：扳机停半路夹爪停半路。
- 扳机路径补整形链（此前无任何滤波）：死区重标定 `[trigger_deadzone,1]→[0,1]` → `trigger_gamma`（默认 **1.4**，前半程更细，便于半捏）→ `trigger_ema_alpha`（默认 **0.4**，去 Joy 抖动）。
- 参数在 `gripper_teleop.yaml`，节点启动时读取（无热改回调，改后需重启节点）。

**肘折偏置，方便内收**——`astral_arm_teleop` `urdf_numerical`。

- 现象：J4 卡死少了，但肘经常伸太直（`ik_q4_max=-0.25` 只剩约 14°），J3 对末端几乎没力臂，手难往内收。左 J2 内收侧 URDF 只有 0.3 rad，软限位 0.18 还吃掉大半。
- `ik_q4_max` −0.25→**−0.45**；新增单向折肘 `ik_w_fold=0.015` / `ik_q4_fold=-1.20`（比这更直才罚，约 2 mm 级位置代价；位置误差仍优先）。限位铰链按「到 0 的短边」缩 margin，J2 内收不再一碰就顶。

**暂时撤回「贴上一帧」默认值**——`astral_arm_teleop`。求解器仍支持这些项，默认改回改之前：`ik_w_ori=0.3`、`ik_w_reg=1e-4`、`ik_dq_max=0`、`ik_w_pref=0`。yaml 注释里留了 0.40 / 0.02 / 0.30 / 0.004 以便恢复。J4 帽和工作球不动。

**驱动 SESSION 低通 + 建议 100 Hz**——`astral_robot_control`。

- yaml 新增 `lpf_enable` / `lpf_alpha`，connect 与 `~/ready` 后调 SDK `set_lpf`（进 WORK 可能重置，故每次 ready 重写）。默认开、`alpha=0.35`（100 Hz 约 19 ms）。
- `obs_hz` / `ctrl_hz` / `control_rate` / `state_publish_rate` 50→**100**（与遥操 150 对齐的起步档；UDP 稳再升 150）。

**URDF IK 贴上一帧，少换怪构型**——`astral_arm_teleop` `urdf_numerical`。

- `ik_w_reg` 1e-4→**0.02**（贴 `q_prev`；肩/肘权重大、腕小）。另：`ik_dq_max=0.30` 每拍关节步进盒，挡住一次解跳到另一支 IK；`ik_w_pref=0.004` 冗余时往 `init_pose` 靠。**`ik_w_ori` 提到 0.40**（在强正则下跟 Quest 腕朝向；0.12 会漂）。位置权重不变。tune 正则滑条范围扩到 0.1。

**URDF IK 约束 Joint4 勿伸直卡死**——`astral_arm_teleop` `urdf_numerical`。

- 原因：`left/right_joint4` URDF 上限是 **0**（完全伸直）。LM 只有 `w_reg=1e-4` 贴上一帧，跟手外伸时 q4 顶死在 0，肘奇异，再收不回来。
- 不改 URDF。IK + SafetyFilter 使用 `ik_q4_max`（默认 **−0.25 rad**，约 14° 余弯）；另加近限位铰链残差 `ik_w_limit`。`ik_q4_max≥0` 关闭帽、回到 URDF 0。可热改 / tune 滑条。
- **边界跟手**：顶到 `ik_q4_max` 后不再因 `ik_ok=false` 丢解冻住。LM 有限解一律下发，q4 停在余弯、其余关节继续跟可达面上的手；肘贴上限时姿态权重降到 0.2。延迟日志 `ik_sat` 表示在边界上滑、`ik_fail` 才是真失败。

**`workspace_radius` 接到 IK 前**——`astral_arm_teleop`。

- 末端目标相对 `left_base_link` / `right_base_link` 原点超半径则径向收到球面再求解。yaml 默认 **0.55 m**（工作位 |p|≈0.44 m，零位下垂≈0.49 m，关节限位内最远≈0.59 m）。≤0 关闭。可 `ros2 param set` 热改。
- 延迟日志增加 `ee_r=…mm`（夹之前的目标距离）和 `ws_clip` 次数。

**URDF 数值 IK 改到臂基座系**——`astral_arm_teleop` `urdf_numerical`。

- 此前 Pinocchio `oMf` 在 universe（= `body_link` 躯干）求解。现 FK/IK 位姿改到 `left_base_link` / `right_base_link`（肩安装座），与 DH 解析解同一类臂基座系。
- `*_base_link` 相对躯干只有平移、无旋转，Quest `robot_world` 增量方向仍可与 `vr_to_arm_rot=I` 相加；`robot_init` 原点会从躯干原点变为肩原点（日志 EE0 的 z 不再含 0.412 m 肩高）。

## 2026-08-28（超时）

**超时统一 1.5s**——`astral_arm_teleop` `data_timeout`、`astral_robot_control` `command_timeout_s` 均改为 1.5s（代码默认 + yaml）。VR 断连后 teleop 再撑 1.5s 才 disarm；teleop 停发后 driver 再 1.5s 才停刷电机目标。

## 2026-08-27（四）

**Homing 途经点（先抬再伸，避桌）**——`astral_arm_teleop` / `astral_mujoco_sim`。

- 启动不再关节空间直达 `init_pose`（那条弧会刮桌）。新增 `init_waypoints`（扁平 7×N，硬件约定）：按序走完再到工作位。当前左右各一个途经点 `[-1, 0, 0, -2.20, 0, 0.46, 0]`。
- `init_timeout` 15→40s（两段慢速）。MuJoCo 默认起始改为零位，便于看 rest → via → init。

## 2026-08-27（三）

**视频扫描按像素格式挑彩色节点（修 RealSense 无画面）**——`quest3_video_streamer`。

- 问题：插上 D435i 后 auto_scan 按"最小编号"选中深度节点 `/dev/video4`（Z16），OpenCV 打不开 → 该轨反复黑帧重试刷屏、Quest 无画面。
- `scan.py`：同一物理设备多采集节点时，用 `VIDIOC_ENUM_FMT` ioctl 枚举**像素格式**打分（彩色 YUYV/MJPG/NV12=100 > 红外 GREY/Y8=10 > 深度 Z16=0），自动选中彩色节点（D435i → video8 一类）；深度-only 节点直接丢弃。`v4l2-ctl` 仅作补充，没有该命令也不退回选 video4。并输出 `force_mjpg` 建议（YUYV 节点强制 MJPG 会让 cv2 打不开）。sysfs 名含 RGB/Depth/Infrared 作为无 fourcc 时的回退。
- `streamer_node.py` `_scan_spec`：`force_mjpg` 默认采用扫描建议；新增 `{label}.device` 覆盖项。`params.yaml` 新增 `video8` 显式块（D435i 彩色、右侧布局）。
- 修复：`hook_gate_visibility()` 接线被误删导致 web 改勾选后 Quest 面板不跟随，已恢复。

## 2026-08-27（二）

**静音轨 Quest 面板缩小到边缘 / 可隐藏（track_visibility）**——`quest3_video_streamer` / `astral-tracking`。

- 需求：web 取消勾选后 Quest 面板显示黑色，希望"什么都不显示或变得很小"。视频帧本身无 alpha，改为 PC 经信令通道发 `track_visibility {enabled: [...]}` 消息、Quest 端按 label 处理被静音面板。
- `quest3_video_streamer`：`StreamGate` 新增 `on_change` 钩子；service 新增 `send_track_visibility()`——门控每次变化即推（经 `loop.call_soon_threadsafe` 跨线程调度），进入 playing 时也同步一次初值；总开关关闭 = enabled 空表（全部静音）。黑帧仍照常发（2fps 保活，恢复无重连）。
- `astral-tracking`（Quest app）：`VideoStreamManager.HandleTrackVisibility`——被静音面板默认**缩小为视野左下角的小条**（多路横向排开；`minimizeMutedPanels`/`minimizedScale`/`minimizedOrigin`/`minimizedSpacing` 可在 Inspector 调；关掉开关则**整个隐藏**）；恢复勾选时还原原布局。旧版 app 忽略该未知消息（行为不变=黑面板）；**新行为需在 Unity 重新构建 APK 并安装到头显后生效**。

## 2026-08-27（前条）

**安全修复：`/teleop/armed` 闸门收紧（审查 C2）**——`astral_arm_teleop` / `astral_web_monitor`。

- `_on_armed` 不再无视消息内容任意武装：仅 `Bool(true)` 生效；未标定（无 vr_init）拒绝并告警——顺带堵住 web「恢复」latched 残留让新启动节点自动武装的旁路。
- 新增 disarm 原因区分：操作员「暂停」→「恢复」可直接 armed（零点仍有效，体验不变）；**VR 看门狗超时 disarm（fault）→「恢复」被拒绝**，必须重新 `/teleop/start` 重标定 vr_init——断连后手可能已移动，沿用旧零点会产生计划外快速移动。fault 状态粘性：fault 后点「暂停」不会降级，只有 start 能清除。
- `astral_web_monitor` README 更新「恢复」按钮的生效条件。
- 冒烟（状态机桩件测试）：`data=false` 不武装 ✅；未标定拒绝 ✅；暂停→恢复 ✅；fault 拒绝恢复 ✅；fault 后暂停不降级 ✅；start 清除 fault 后暂停/恢复如常 ✅。

**安全修复：VR 断连即 disarm + 全链路 NaN 屏障（审查 C5/C6）**——`astral_arm_teleop`。

- **VR 看门狗改为 disarm**：`data_timeout` 超时不再只是静默停发——立即 `_armed=False` + ERROR 日志，恢复必须重新 `/teleop/start`（顺带重标定 vr_init、重置 EMA，消除重连跳变）。此前断连后 1.5s 内手臂仍按冻结的最后一帧完成在途动作，且重连后无重闸门直接恢复。`data_timeout <= 0` 可关闭（语义与驱动 `command_timeout_s` 一致，不推荐）。
- **`data_timeout` 默认 1.5→1.0s**（代码 + 左右 yaml）：容忍 VR 链路短暂抖动（亚秒级断连不误触发）；超时前 teleop 持续发冻结目标（driver 跟随），teleop 停发后 driver 的 `command_timeout_s`(0.5) 接管保持。运行中可热调（`ros2 param set`）。
- **NaN 屏障三处**（此前全链路无 isfinite 检查，一帧 NaN 永久污染）：① `_on_wrist` 拒绝非有限 position/四元数——此前 NaN 四元数穿过模长校验（`NaN > 0.1` 为 False）且 `Rotation.from_quat(NaN)` 会在回调里抛 ValueError；② `PoseProcessor.update_vr_pose` 入口拒绝非有限输入——EMA 吃进 NaN 后 `a·NaN+(1-a)·x=NaN` 永不恢复；③ `SafetyFilter.filter` 入口 isfinite 检查——非法帧保持上一合法指令、不更新 `prev_q`（此前 NaN 经 clip/速度门双双放行并写入 prev_q，恢复后第一帧速度限制失效），info 新增 `invalid` 标记。
- 冒烟：SafetyFilter NaN 保持 / 恢复后限速完好 / Inf 阻断 / 无历史回零 ✅；PoseProcessor 丢弃 NaN 位置与旋转、正常更新仍跟踪 ✅；三文件 py_compile ✅。

**Web 实时相机画面（JPEG 抽帧 + MJPEG 转发）**——`quest3_video_streamer` / `astral_web_monitor`。

- `quest3_video_streamer` 新增 `preview.py` `PreviewPublisher`：每路相机发布 `~/preview/{label}`（CompressedImage，BEST_EFFORT，默认 10fps/640宽/q65）。**延迟隔离设计**：捕获线程只做 gate 检查 + 速率限制 + maxsize=1 队列 `put_nowait`（忙则丢）；JPEG 编码在独立 daemon 线程——不进捕获线程（不拉长采集周期）、不进 asyncio 循环（不干扰 Quest RTP 节奏）；跟随门控（静音路不发）。参数 `web_preview`/`_fps`/`_width`/`_quality`。两个 source 适配器加 `preview_hook` 抽头（webcam 抽原生 BGR 免回转，ros 抽转换后 RGB）。
- `astral_web_monitor`：按 gate_state 相机列表动态订阅 preview 话题（BEST_EFFORT depth1）；新增 `GET /api/v1/video/feed/{label}`（MJPEG multipart，浏览器 `<img>` 原生支持）与 `GET /api/v1/video/snapshot/{label}`（单帧）；视频卡片新增「实时画面」开关 + 勾选相机的 320px 预览网格，取消勾选即停。
- 延迟预算：Quest 链路 ≈0 影响（共享读 + 独立编码线程）；控制链路 ≈0（Jetson 每路 ~2-5% 单核）；DDS ~0.3-0.6MB/s/路；web 预览自身延迟 ~0.2-0.4s（监控用途）。
- 冒烟：preview 话题建立✅；monitor 动态订阅✅；CLI 发测试 JPEG → snapshot 200 返回原图✅、feed multipart 帧格式正确✅、未知 label 404✅。

**视频设备自动扫描（web 免配置可选）+ 懒打开**——`quest3_video_streamer` / `astral_web_monitor`。

- `quest3_video_streamer` 新增 `scan.py` + `auto_scan` 参数（默认 true）：启动时枚举 `/dev/video*`（ioctl `VIDIOC_QUERYCAP` 查 `V4L2_CAP_VIDEO_CAPTURE` 跳过 metadata 节点，按物理设备 sysfs 父级去重），label = 节点名（`video0`…），面板一行网格自动排布——**不再写死设备节点/中文名**。同名 yaml 块可覆盖单路字段；一台没扫到回退 `cameras` 列表；`cameras` CLI 覆盖联动 `auto_scan:=false`。D435i 多采集节点取第一个不一定是彩色，需显式块（README 已注）。
- **懒打开 + 失败黑帧**：sender 启动不再打开全部相机；每轨首个未静音帧才 `source.start()`，打开/读帧失败 → 黑帧 + 1s 退避重试（设备没插/被占用不再拖垮 sender，插上即恢复）。
- `~/gate_state` JSON 增加 `cameras:[{label,device,source,preset,sysfs_name}]`——web 端不再解析 streamer 的 yaml。
- `astral_web_monitor`：`/video/status` 在线时改用 gate_state 的相机列表（exists/sysfs 每次轮询实时刷新）；**离线时 monitor 自行扫描主机设备，可预选**（latched `~/active_cameras` 在 streamer 启动后生效；总开关服务仍需在线）；在线判定加 `count_publishers` 活性检查（latched 残留不再误报在线）。VideoCard 离线时勾选/路数可用、仅总开关禁用。
- 冒烟：无相机主机 → auto_scan 回退 wrist 列表✅；gate_state 带 cameras✅；web 在线/离线切换正确✅；离线预选 latched 下发 → streamer 启动拾取并对未知 label 告警过滤✅。

## 2026-08-26

**视频回传并入完整遥操 + 运行时可门控（web 可选路/开关）**——`quest3_video_streamer` / `astral_teleop` / `astral_web_monitor`。

- `quest3_video_streamer` 新增**运行时推流门控**（`gate.py` `StreamGate`）：服务 `~/set_push_enabled`（SetBool 总开关）、latched 话题 `~/active_cameras`（逗号 label 子集，空=全部）、latched 状态 `~/gate_state`（JSON）。门控在 track 帧级生效：被关的轨改发 2fps 黑帧（Y=16/U=V=128，几乎不占带宽，Quest 面板变黑，恢复即时）——不做 SDP 重协商，相机集合仍在启动时固定。
- 修复 `StreamGate.set_active` 嵌套取非重入锁死锁（topic 回调卡死 spin 线程，服务也连带无响应）。
- 默认相机改为实际硬件：`cameras: ["wrist_left","wrist_right"]`（当前两路 USB，无 RealSense）；d435i 配置块保留。`multi_camera.launch.py` 新增 `cameras` CLI 覆盖；仅当 d435i 在列表且 source≠v4l2 才拉 realsense 节点。
- `full_teleop.launch.py`：`with_video:=true` **默认**带视频回传；启动时自动 `adb reverse tcp:8000/8765`（无 adb/无设备仅 WARN）；`video_cameras:=` 透传。仍需 Quest 端开启 video feed。
- `astral_web_monitor`：新增 `GET /api/v1/video/status`（配置相机列表 + `/dev/videoN` 在线/sysfs 名 + gate 实时状态）、`POST /video/push`、`POST /video/cameras`；monitor_node 订 `~/gate_state` 镜像进 `ui_state.video_gate`；系统页新增「视频回传」卡片（总开关 + 路数下拉(快捷勾前n路) + 逐路勾选 + 在线点，streamer 离线时禁用）。
- 冒烟：streamer 起 2 源 + gate 初始化✅；`set_push_enabled false`→gate_state push_enabled=false✅；`active_cameras:='wrist_right'`→active=[wrist_right]✅、空串恢复全部✅；web 三端点全通✅、`ui_state.video_gate` 随 WS 下发✅。

**夹爪开度 1.5→2.5 + web 机器人按钮可用时机文档化**：

- `astral_robot.yaml` 左右 `gripper_open_rad` 1.5→**2.5**（真机实测目标全开角；若顶死回弹则回调 ~2.0）。之前"2.0 开到最大马上闭合"实为双流竞态所致（已修），非硬止点过流。sim echo 配置同步 2.5。
- `astral_web_monitor` README 写清机器人模式按钮（急停/阻尼/位置保持/就绪/归零）可用时机：运行中✅、已暂停✅（driver 还活着）、已停止❌503 属预期（driver 随栈退出）——**先启动栈再点按钮**；运行中仍 503 则查 Jetson driver 是否旧代码（`ros2 service list | grep astral_robot_driver` 应列 5 个 Trigger）。
- 文档写明上电瞬态：ready 把夹爪归零(0)→遥操首条指令开到 open_rad，像"抽一下"，正常。

**真机修复：夹爪抽搐 + 开度不到位（双流竞态）**——`astral_gripper_teleop` / `astral_robot_control` / `astral_web_monitor` / `astral_teleop`。

- 根因：`pinch_gripper_node` 同时发 `/command`(Float64) 和 `/joint_commands`(JointState)，driver 两个都订且各自映射 rad（ pinch 节点配置 vs driver 配置），Jetson 上两份配置不一致（0.8 vs 1.5）→ 两路 50Hz 流交替覆盖同一目标 → 夹爪按控制频率抽搐，且 pinch 端旧配置 0.8 导致"张开但不到位"。
- 修复：pinch 节点**只发 Float64 闭合比**（0=开 1=合），rad 映射唯一权威收敛到 driver `astral_robot.yaml`（1.5/0.0）；`joint_commands` 话题保留给直接发弧度的适配器。`gripper_teleop.yaml` 删 `open_rad/closed_rad`。
- `astral_web_monitor`：左夹爪指令频率改听 `/left_gripper/command`（Float64），`TopicPair` 加 `cmd_kind`；状态显示仍走 driver 回显的 `/left_gripper/joint_states`（即 driver 实际用的 rad）。
- `head_teleop_node`/`controller_start_gate` 回退昨日多余的双 QoS 订阅（RELIABLE 发布↔BEST_EFFORT 订阅本就兼容，双订阅只会触发 incompatible QoS 警告噪音）；CLI `echo` BEST_EFFORT 话题用 `--qos-reliability best_effort`。
- 冒烟：pinch 只发 /command；driver dry_run 全程只见 rad=1.500 单值（无交替）；head 节点 0 条 QoS 警告、start 正确捕获 head_init=[0.1,0.05]、摇杆回中指令=[0.1,0.05]；web monitor 夹爪 cmd 49.4Hz health=ok。
- ⚠️ Jetson 注意：本修复要求 driver 端 `astral_robot.yaml` 的 `left/right_gripper_open_rad` 正确（1.5）；拉代码后需重新 `colcon build`（非 symlink 构建的 install 里 yaml 是构建时拷贝，旧值会残留）。

`astral_web_monitor`：接入头部通道（配合 `astral_teleop/head_teleop_node`）。

- `config.py` `TOPICS` 新增 `head`（`/head/joint_states` + `/head/joint_commands`）——snapshot/健康巡检自动携带头部；**不设 cmd 频率下限**（启动前/手柄掉线时头部指令合法为 0Hz，设下限会误报"偏慢"）。
- 监视 tab 新增「头部 (yaw/pitch)」JointPanel、指令频率序列加"头部"、新增「头部 yaw/pitch」实时折线图；健康巡检面板加 `head→头部` 标签。
- README 话题表/WS 负载示例/Web UI 功能同步；冒烟：snapshot 含 `head`（values/cmd_hz/state_hz/health=ok, expected_hz=0）。

`quest3_hand_mocap`：README 补手柄 Joy 量程/符号约定（实测）。

- 话题表后新增「手柄 Joy 量程与符号约定」小节：`axes=[trigger,grip,stickX,stickY]`，stickX 左−1/右+1、stickY 后−1/前+1，回中 0；buttons 6 位 mask 顺序。实测左右手均达 ±1.0 满量程、回中干净 0.000，建议下游死区 0.05~0.1。

`astral_teleop` / `astral_mujoco_sim`：右手柄摇杆 → 机器人头部（yaw/pitch），绝对位置控制，与臂遥操共用 `/teleop/start` 闸门。

- 新增 `astral_teleop/head_teleop_node`（`with_head_teleop:=true` 默认）：订 `quest3/right_controller_joy`，`axes[2]`=stickX（左=-1 右=+1）→ yaw、`axes[3]`=stickY（前=+1 后=-1）→ pitch。弹簧回中摇杆用**绝对控制**：`head_target = head_init + stick*scale`，摇杆回中→头回初始位，`/teleop/disarm`→停发由 driver 位置保持。
- **启动记 `head_init`**：`require_start_signal:=true`（默认）下等 `/teleop/start`，触发时读 `/head/joint_states`（真机为 OBS 真实头角）记为基准，避免跳变；无该话题回退 `init_head_*`（默认 0,0）。发 `/head/joint_commands` 给 `astral_robot_control`（driver 唯一硬件权威）。配置 `config/head_teleop.yaml`：`yaw_scale`/`pitch_scale`（带符号，方向反了改符号）、限幅、deadzone、publish_rate、input_timeout。
- 非侵入：只订已有话题、只发 driver 已在订的 `/head/joint_commands`，不动 driver/臂/夹爪/灵巧手。Joy 与 joint_states 改**双 QoS 订阅**（真机 BEST_EFFORT + CLI RELIABLE），`ros2 topic pub` 注入也能测；`controller_start_gate` 同步补 RELIABLE 订阅。
- sim：`astral_sim_pipeline.launch.py` 加 `with_head_teleop`；`mujoco_sim_node` 新增 head echo（订 `/head/joint_commands`→回显 `/head/joint_states`+每秒日志 `[Head] cmd yaw/pitch`），MJCF 无头关节故 viewer 不动头，用于验证映射。
- 冒烟：start 前 `/head/joint_commands` 无输出；start 后 stickX=+1→yaw 0.8、stickY=+1→pitch 0.4、stickX=-1 stickY=-1→[-0.8,-0.4]；sim echo `[Head] cmd yaw=-0.4 pitch=0.2` 正确。真机首测需确认 yaw/pitch 正负号（反了改 scale 符号）。

`astral_mujoco_sim`：sim 管线接入手柄遥操 + 外部启动开关，便于在仿真里测手柄功能。

- 新增 `controller_start_gate`（`with_start_gate:=true` 默认）：左手柄 grip 键 → `/teleop/start`。
- gripper include 注入 `controller_joy_topic=quest3/left_controller_joy`：左 trigger 模拟量 → 左夹爪（新鲜时优先，否则回退 pinch）。
- `with_gripper` 描述更新为 pinch/trigger；package.xml 加 `astral_teleop` exec_depend；README 补「手柄集成（sim）」。
- 冒烟：sim 启动后 controller_start_gate(grip→/teleop/start)、pinch_gripper_node(joy=...left_controller_joy axis=0)、arm 节点 require_start_signal=true(等待 start) 均正常，无报错。

`astral_teleop` / `astral_gripper_teleop`：手柄接入遥操——左 grip 键作外部启动开关，左右 trigger 模拟量控夹爪。

- 新增 `astral_teleop/controller_start_gate` 节点：订 `quest3/left_controller_joy`，左手柄 grip 键（mask bit 5 = gripClick）按下沿发 `/teleop/start`（Bool true，RELIABLE+VOLATILE 一次性），等价 web「开始遥操」/CLI `/teleop/start`；再按一次重新记零点。`full_teleop.launch.py` 默认启动它。
- `pinch_gripper_node` 新增 trigger 合并：参数 `controller_joy_topic`/`trigger_axis`(0)/`trigger_deadzone`(0.05)/`trigger_invert`。订 `Joy`，`axes[trigger_axis]`(0=开..1=合)作为闭合比；手柄 Joy 新鲜时优先 trigger，否则回退 pinch（同侧手柄/裸手互斥，二者互补）。保持单发布者到 `/{side}_gripper/*`，无冲突。
- `gripper_teleop.launch.py` 暴露 `controller_joy_topic` 参数；`full_teleop` 为左/右 gripper 分别注入 `quest3/{side}_controller_joy`。
- `astral_teleop` package.xml 加 `sensor_msgs`/`std_msgs`，setup.py 加 entry_point。
- 冒烟：左 grip 按下 → `/teleop/start` data:true ✅；trigger=0.8 → `src=trigger close=0.80` ✅。

`quest3_hand_mocap`：接收 Quest 端新增的手柄按键/摇杆推流，发布 `sensor_msgs/Joy`。

- 新增 `_process_buttons_line`：解析 ``{Left|Right} buttons:, trigger, grip, stickX, stickY, mask`` → `Joy`。
  - axes=[trigger, grip, stickX, stickY]；buttons=mask 6 位 [primary(X/A), secondary(Y/B), stickPress, menu, triggerClick, gripClick]。
  - 话题 `quest3/left_controller_joy` / `quest3/right_controller_joy`；在 side-filter 前分发，双侧手柄始终转发。
  - 手柄位姿行（`controller:`）原有逻辑已处理（`controller_as_wrist` 时镜像到 wrist_pose）。
- **修复 mask 解析 bug**：原 `len(parts) >= 6` 误判（parts 实为 5 字段）导致 mask 恒为 0、按键全丢；改为 `>= 5` 读 `parts[4]`。
- `package.xml` 加 `sensor_msgs` 依赖；README 话题表补 Joy 说明。

`astral_teleop` / `astral_gripper_teleop`：右手新增「捏合→右夹爪」模式，`right_hand_source` 加 `gripper` 选项。

- `full_teleop.launch.py`：`right_hand_source` 增加 `gripper`（右捏合→右夹爪、不起 Wuji）；新增右 `gripper_teleop.launch.py hand_side:=right`（条件 `right_hand_source==gripper`）；`use_wuji` 改为 `right_src in (quest3,glove)`，gripper/none 均不起 Wuji；mocap `publish_landmarks_right` 对 gripper 仍为 true（右 pinch 需要 landmarks）。
- `astral_gripper_teleop`：`pinch_gripper_node` 本就支持 `hand_side:=right`（订 `hand_landmarks/right`、发 `/right_gripper/*`），无需改节点代码；README 补左/右两侧用法。
- CLI：`right_hand_source:=gripper with_gripper:=true`；Web：`presets.yaml` 新增「双臂+双夹爪(无灵巧手)」预设。
- 冒烟：`right_hand_source:=gripper` 启动后左/右两个 `pinch_gripper_node` 均起（rad=[1.5,0.0]），Wuji 节点 0 个。

`astral_gripper_teleop` / `astral_robot_control`：夹爪全开角度 2.0→1.5 rad（左右均改）。

- 现象：`open_rad=2.0`（机械硬止点）时遥操手一张开，夹爪开到最大后立即闭合——持续顶死硬止点 → 堵转 → 过流保护触发 → 电机失力回弹至合拢。
- 修复：`open_rad` 回退到 1.5（安全开度，接近全开但不顶死）。`astral_gripper_teleop/config/gripper_teleop.yaml` 与 `astral_robot_control/config/astral_robot.yaml`（`left/right_gripper_open_rad`）同步改为 1.5。
- 更新两包 README 真机全开值描述。

`astral_arm_teleop`：外部启动闸门 `require_start_signal` 真机默认改为开启。

- `config/astral_arm_teleop_left.yaml` / `_right.yaml`：`require_start_signal: false` → `true`。真机启动推流后臂节点只跟踪 `vr_current`、不自动记零点/不 arm；手摆好初始位姿后由 web「开始遥操」或 `/teleop/start` 记 `vr_init` 并 arm。
- `astral_mujoco_sim/launch/astral_sim_pipeline.launch.py`：`require_start_signal` launch arg 默认保持 `""`（透传 yaml=true），sim 同样默认开启外部闸门，统一真机/仿真流程。
- launch arg 非空时覆盖 yaml；full_teleop / dual_arm 默认 `""` 透传 yaml（=true）。
- 更新 `astral_arm_teleop/README.md` 默认值描述。

## 2026-08-25（续）

`astral_robot_control` / `astral_web_monitor`：**真急停 + 阻尼释放**。原"急停"实为软 disarm（与暂停同效），改为硬件级断电；并补「阻尼释放」用于遥操收尾手动拖臂回 home。

- `astral_robot_control` driver 新增服务 `~/damping`（`motion_mode=0`，阻尼可拖拽）与 `~/position`（`motion_mode=1`，位置保持）；`~/estop` 语义明确为真断电（SDK `e_stop`/`disable`，臂失去保持力）。更新 driver docstring 与 README 服务表。
- `astral_web_monitor` 后端：`monitor_node` 新增 `call_driver_service(name)`（缓存 Trigger 客户端，后台 spin 线程完成 future，web 线程轮询 done）；`config.py` 新增 `DRIVER_NODE` + 5 个 driver 服务名（`ASTRAL_WEB_MONITOR_DRIVER_NODE` 可配）。`web_server` 新增 `POST /api/v1/robot/{ready,home,position,damping,estop}`（`asyncio.to_thread` 调用，driver 未就绪返回 503）；**移除**旧 `POST /api/v1/estop`（软 disarm，与 `/api/v1/pause` 重复）。
- `astral_web_monitor` 前端：`ControlBar` 急停按钮改为真断电（`api.robotEstop`，确认弹窗文案强调断电）；新增「阻尼释放」按钮（`api.robotDamping`）。`SystemTab` 新增「机器人模式」卡（一键就绪/归零/位置保持/阻尼释放/急停断电）。`client.ts` 移除 `estop`，新增 `robotReady/Home/Estop/Damping/Position`。
- 典型流程：遥操中 → 停止（臂保持末位姿）→ 阻尼释放（手动拖回 home）→ 位置保持/归零；急停=断电，恢复需重新一键就绪。
- 非侵入性：机器人模式按钮仅**调用 driver 已暴露的服务**，driver 是硬件权威，monitor 不新增硬件写入路径、不改 driver 逻辑。
- 冒烟测试：driver `dry_run:=true` + web_monitor 8091，`/api/v1/robot/{damping,position,ready,estop}` 均返回 `dry_run: skipped ...` 成功；旧 `/api/v1/estop` 已 405。
- 前端已 `npm run build` 重新生成 `web/dist`；两包 `colcon build --symlink-install` 通过；lint 无错误。

## 2026-08-25

`astral_web_monitor`：Web UI 大幅增强——急停、Toast 通知、实时图表、健康巡检面板、系统运维 tab。全程非侵入（只读已有话题、只发已有 `/teleop/disarm` 等控制话题、不动其他包）。

- **急停**：`POST /api/v1/estop` 无条件发 `/teleop/disarm`（软暂停）；前端红色常驻按钮 + 确认弹窗。与 launch 状态无关，CLI 启动的遥操也可急停。
- **重启**：`POST /api/v1/restart` 停止后重新启动当前预设（仅对经本监控启动的预设有效）。
- **健康巡检**：`monitor_node` 新增 state 话题频率计数 + `health` 汇总（每实体 stale/state_hz/cmd_hz/expected_hz/slow/status + 总体）；`config.py` 新增 `EXPECTED_RATES_HZ`（`ASTRAL_WEB_MONITOR_EXPECTED_HZ` 可配）。修复原 lambda 闭包捕获循环变量导致指令频率计数失效的 bug。
- **ui_state / health 端点**新增 `state_rates_hz` 与 `health` 字段。
- **Toast 通知**：`useToast` store + `ToastHost`，替换原 `alert()`，成功/失败浮窗自动消失。
- **实时图表**：`historyStore` 环形缓冲（200 样本）+ `ChartPanel` 手写 SVG 折线（指令 Hz、臂关节0 角度），无第三方图表库。
- **三 Tab 重构**：监控（关节面板 + 图表）/ 健康（HealthPanel）/ 系统（预设管理 + 重启 + 日志控制台）。`Tabs` 组件 + `MonitorTab`/`HealthPanel`/`SystemTab`。
- **顶栏状态徽标**：WS / ROS / 数据 三色 pill（`useWsConnected`）。
- 前端已 `npm run build` 重新生成 `web/dist`。

`astral_web_monitor`：接入 `astral_arm_teleop` 的外部启动闸门，Web 界面可手动发 `/teleop/start`。

- `config.py` 新增 `TOPIC_START="/teleop/start"`。
- `monitor_node.py` 新增 `publish_start()`（RELIABLE + VOLATILE，非 latched 一次性触发，避免晚加入臂节点收到旧 start 自动开始）。
- `web_server.py` 新增 `POST /api/v1/teleop/start`（**无条件发送** `/teleop/start`，不检查 launch 是否经本监控启动——臂节点是唯一裁判：homing 中或无 VR pose 时会忽略并告警。因此无论遥操由本监控预设启动还是从外部 CLI 启动，此端点均可用）。
- 前端 `ControlBar` 新增"开始遥操"按钮（琥珀色，running/paused 时可用）；`api.teleopStart()`。
- 前端已 `npm run build` 重新生成 `web/dist`。

`astral_arm_teleop`：新增**外部启动闸门** `require_start_signal`，修复 Quest3 "start stream" 时手抬着点按钮导致首帧 `vr_init` 记在错误位置、机器人一上手就偏的问题。

- `PoseProcessor`：新增 `auto_calibrate` 开关（默认 true=旧行为，首帧自动记零点）；新增 `calibrate_from_current()` 用当前 pose 记 `vr_init`。
- 节点：新增 `require_start_signal` 参数（默认 false，不破坏 sim/旧流程）。true 时启动不 arm、不自动记零，只跟踪 `vr_current`；订阅全局 `/teleop/start`（Bool true，双臂同启）+ 服务 `~/start`（Trigger，单臂带反馈）；收到信号时用当前 `vr_current` 记 `vr_init` 并 arm。homing 中或无腕姿时拒绝并告警。重发 start = 重新记零点（re-center）。
- yaml（left/right）新增 `require_start_signal: false`；`astral_dual_arm_teleop.launch.py` 与 `astral_teleop/full_teleop.launch.py` 暴露 `require_start_signal` launch arg。
- 真机用法：`require_start_signal:=true` 启动 → 手摆好 → `ros2 topic pub --once /teleop/start std_msgs/msg/Bool '{data: true}'`。

`astral_robot_sdk` / `astral_robot_control`：纠正电机 ID 语义——`0x31/0x32` 是**头部**（head_yaw/head_pitch），不是夹爪；机械夹爪只走专用命令 `0x97/0x98`（`set_gripper_angle`），在 18 轴关节流里没有槽位。

- SDK：`ROBOT_JOINT_NAMES` 末两位改为 `head_yaw/head_pitch`；新增 `HEAD_IDS=[0x31,0x32]`、`move_head_js(yaw,pitch)`、`move_waist_js(front,side)`；`GRIPPER_IDS` 保留为 `HEAD_IDS` 的兼容别名（旧导入不报错，但语义已是头）。
- `astral_robot_control`：`joint_layout` 末两位改 `head_yaw/head_pitch`，新增 `HEAD_JOINT_NAMES`/`HEAD_NS`（`GRIPPER_JOINT_NAMES` 保留为别名）；driver 新增 `/head/joint_commands`→`move_head_js`、`/head/joint_states`；18-DoF `/astral/joint_states` 末两位改为头；夹爪 `/joint_states` 改发最近一次指令角（夹爪无 OBS 槽位）；`_on_grip_js` 期望名改 `f"{side}_gripper"`。yaml 新增 `head_ns`/`enable_head_cmd`。
- 下游：`astral_mujoco_sim`（自带 14-DoF 臂名 + 独立夹爪 echo，不碰 18-DoF/头）、`astral_arm_teleop`（仅 14-DoF 臂）、`astral_web_monitor`（仅 launch 预设）、`astral_teleop` bringup 均无需改代码；无包再以 `GRIPPER_IDS/GRIPPER_JOINT_NAMES` 当夹爪电机 ID 下发。

`astral_gripper_teleop` / `astral_robot_control` / `astral_mujoco_sim`：修复夹爪两个问题。

- **极性相反**：真机夹爪电机方向为 0.8=张开、0.0=合拢，原 `open_rad/closed_rad=0.0/0.8` 导致捏合(close_ratio=1)→0.8→夹爪反而张开。翻转为 `0.8/0.0`：捏合→0.0(合)、张开→0.8(开)。`close_ratio` 语义不变(0=开,1=合)，`/left_gripper/command` Float64 仍 0=开 1=合。两条路径(pinch 节点 JointState 弧度 + driver Float64 比例)的 `open/closed_rad` 一并翻转，避免互相覆盖；sim 同步。
- **行程太小**：原 `open_dist_m/close_dist_m=0.08/0.015` 固定映射常与用户实际捏合距离范围不符，只用到中间一小段。新增 `auto_range`（默认开）：带遗忘地跟踪实测捏合距离 min/max，把当前距离映射到真实范围，用满夹爪行程；`open/close_dist` 退化为先验/回退，关掉则回退固定映射。新增 `close_ratio_from_range`；`_update_envelope` 指数遗忘包络 + `min_span` 兜底；日志加 `range=[lo,hi]mm` 便于观察。

`quest3_hand_mocap`：修复 IOBT 开启后下游遥操 IK 失败（`ik_fail`、`vr_rx`/`vr_age` 延迟爆炸）。根因是 `frame_id` 在 `robot_world`↔`robot_body` 间切换导致 `astral_arm_teleop` 的 `PoseProcessor` 跨系做 delta（零点在世界系、当前在躯干系 → 米级跳变 → 不可达目标）。在源头（mocap）解决，`astral_arm_teleop` 不再需要身体系特判（已回退 `7c77555`）。

- **sticky latch**：`_iobt_active()` 一旦见过 body 包即置 `_body_ever_seen=True`，此后即使 body 包瞬时丢包（>0.4s）也不再回退世界系，消除运行中 `frame_id` 抖动（实测 26 次切换）。
- **hold gate**：新增 `_pose_frame_settled()` 与 `_WRIST_SETTLE_S=1.0`。启动后 head/wrist/controller 在"帧确定"前不发布：收到首个 body 包 → 躯干系；或 1s 内无 body 包 → 世界系。从源头消除启动期 world→body 翻转，下游第一帧 wrist 即在正确系，`vr_init` 不会被错系污染。landmarks 为腕局部系、与参考系无关，不受闸门影响。
- 独立脚本 `quest3_mocap_standalone.py` 同步 sticky latch。

## 2026-08-24

重命名：`astral_quest_teleop` → **`astral_arm_teleop`**（纯臂遥操 + IK，去掉夹爪 include）；`astral_bringup` → **`astral_teleop`**（整机编排：mocap + 双臂 + 左夹爪 + 右 Wuji）。夹爪 include 从臂包移到 `astral_teleop`。

`astral_arm_teleop` 内部命名统一：节点 `astral_teleop_arm_node` → `astral_arm_teleop_node`，节点名 `astral_teleop_{left,right}` → `astral_arm_teleop_{left,right}`，配置 `astral_teleop_{left,right}.yaml` → `astral_arm_teleop_{left,right}.yaml`（避免与新 `astral_teleop` 包混淆）。

新增 **`astral_teleop`**（原 `astral_bringup`）：整机遥操只拼现有包（双臂 + 左夹爪 + 右 Wuji）。`right_hand_source:=quest3|glove`；手套时 mocap 不再发 `hand_landmarks/right`。`astral_dual_arm_teleop` 增加 `with_mocap`，避免两份 mocap。

新增 **`astral_gripper_teleop`**：Quest3 左手捏合 → 左夹爪，与 mocap / 臂 IK 解耦。

- `pinch_gripper_node` 订 `hand_landmarks/left`，拇指尖–食指尖距离映射为 `/left_gripper/command`（`Float64`，0=开 1=合）和 `/left_gripper/joint_commands`（弧度）。
- `astral_robot_control` 收上述话题，调用 SDK `set_gripper_angle`（CMD 0x97/0x98，非电机 ID）；`move_arm_js` 不带动夹爪。
- 夹爪 include 原在 `astral_dual_arm_teleop.launch.py`（`with_gripper:=true`），现移到 `astral_teleop/full_teleop.launch.py` 编排；臂包不再含夹爪。以后换硬件：新节点发同一 `command` 即可。

## 2026-08-21

`quest3_hand_mocap`：配合 Quest **astral-tracking** 坐标系修复，下游不再二次转换参考系。

- Quest 端把 `FullBody_Hips` 骨头系（+X下/+Y前/+Z左）用常量四元数 `(-0.5,0.5,0.5,0.5)` 重定向为躯干系（+X右/+Y上/+Z前）；`_lastBodyReferencePose` 与 body 包里的 hips 朝向均用重定向后的值。
- Quest 端 `BuildAndSendPacket` 对非 hips 关节直接 `ToBodyRelative`，body 包里非 hips 关节已是 hips 相对；hips 仍发世界系。
- `quest3_udp_mocap._process_body_line` 移除 `pose_in_parent_frame`：hips 发世界（`quest3/hips_pose`），其余关节直接 `unity_pose_to_robot`，PoseArray 中 hips 为 identity 根。整棵树统一在躯干系，轴映射 `unity_pose_to_robot` 由"用错轴"变为正确。
- 独立脚本 `quest3_mocap_standalone.py` 同步去掉二次 `pose_in_parent_frame`，避免可视化把已是 hips 相对的关节再转一遍。

## 2026-08-20

适配 Quest **astral-tracking** 新推流：Mixed 下手柄与手可同时有数据；IOBT 开时发全身关节，且头/手/手柄已在 hips 身体系。`quest3_udp_mocap` 重新解析 `Left/Right controller:` 与 `body iobt`。

- Mixed：同侧 Quest 只发手柄或手（手柄优先）；左右可一边 ctrl 一边 hand。`controller_as_wrist` 默认把手柄写到 `quest3/{side}_wrist_pose`，臂 IK 不用改。另发 `quest3/{side}_controller_pose`、`quest3/input_mix`。
- IOBT：`quest3/hips_pose`（世界系）+ `quest3/body_joints`（转到 hips 系）+ `quest3/body_joint_names`。头/腕/手柄的 `frame_id` 在收到身体包时切到 `robot_body`/`vr_body`。landmark 仍是腕局部。
- TCP `listen` 提到 8，覆盖手×2 + 头 + 手柄 + 身体多路连接。
- 独立脚本 `quest3_mocap_standalone.py` 同步解析与可视化。

新增 **`quest3_video_streamer`**：把 PC 相机画面经 WebRTC 推到 Quest 3，遥操时看机器人第一视角。与 `quest3_hand_mocap` 独立（信令 `:8765`，mocap 仍是 `:8000`）。自 `hand-tracking-sdk` 视频部分拆出，不依赖该 SDK、不用 venv。

- 单条 peer connection 多路 track：默认 D435i 1080p30（v4l2 `/dev/video8`）+ 左右腕 USB 720p30。
- 可调全在 `config/params.yaml`（相机列表、source/device/preset/fov/layout）；launch 只加载 yaml。
- 延迟：源端 `next_frame()` 节流、webcam 后台线程抓帧、D435i 默认真连 v4l2（可选 `d435i_source:=ros` 走 realsense 节点）。
- Quest 用 `video_config` 的 `fov_h_deg` + layout 摆 3D 面板。码率下限/默认/上限 3/5/12 Mbps。
- `quest3_udp_mocap` 增加 `[Mocap Downlink] kbps`，与视频上行 `total_kbps` 可对照链路占用。
- 与 HTS 同跑时 `enable_mocap_tcp` 保持 false，避免抢 TCP 8000。

```bash
ros2 launch quest3_video_streamer multi_camera.launch.py
# Quest：astral-tracking 填 PC 信令（Wi-Fi 或 adb reverse tcp:8765 tcp:8765）后 Start Stream
```

## 2026-08-19

相对 `f6300cf` 的未提交改动一并入库。当日另做：

- **移出 WebXR**：`astral_webxr` 不再在本工作空间内（现位于 `../astral_webxr`）。臂/手 pipeline 去掉 `input_source:=webxr`，只走 HTS `quest3_udp_mocap`。
- **撤回 PC 端 IOBT/混控**：`quest3_udp_mocap` 不再解析 `body iobt` / `controller` 行，也不再发 `quest3/body_joints`、`hips_pose`。Quest HTS 工程里的 IOBT/混控代码未改。

## 2026-08-17

相对 `f6300cf` 之后、当日完成的遥操改动（此前未 commit）：

### 跟手调参 UI（`teleop_tune_plot`）

- 布局、中文字体、按钮颜色；横轴改为「过去 8 s → 现在」。
- 热改滑条 / DH↔URDF 热切仍沿用 08-14 的话题与参数（见下文 08-14 节）。

### 配置改走 yaml

- 臂 IK 默认改为 **`urdf_numerical`**（`astral_teleop_{left,right}.yaml`）。
- `protocol` 以 `quest3_mocap.yaml` 为准（默认 `tcp_wired`）。
- `astral_sim_pipeline` / `astral_dual_arm_teleop` 的 `solver_type` / `protocol` / `convert_to_robot` 留空则**不覆盖** yaml。

### 删除旧单进程入口

- 去掉 `astral_teleop_node.py`、`astral_teleop.yaml`、`astral_quest_teleop.launch.py`、`astral_real_pipeline.launch.py`。
- 真机只留 `astral_dual_arm_teleop`（2× `astral_teleop_arm_node`）。
- `setup.py` 用 `_existing()` 过滤 glob，避免删 yaml 后 colcon 仍拷已失效的安装链接。

### 真机启动走到 init_pose

- `move_to_init_pose` 默认开：从当前 `joint_states` 按 `init_speed_percent`（10% × `max_joint_vel`）关节空间插值到 yaml `init_pose`，到位（0.05 rad）或超时 15 s 后再跟手。
- `init_speed_percent` 是本节点相对 `max_joint_vel` 的比例，**不是** SDK `set_speed_percent` / `move_j`。
- `dry_run` 跳过 homing。

## 2026-08-14

此版本使用 Quest3，逆解使用 DH 以及 URDF，双臂遥操成功。

- 输入：Quest3（`quest3_hand_mocap`，`convert_to_robot:=true`）
- 臂 IK：`analytic_dh` 与 `urdf_numerical` 均可
- 双臂：`astral_teleop_{left,right}` 进程并行 → `/{side}_arm/joint_commands`

### 延迟测量与 150 Hz 控制环

在 mocap / 单臂遥操 / MuJoCo 仿真加周期性 `[Latency]`（约 2 s，`print_latency` / `latency_print_interval`）。

| 日志 | 含义 |
|------|------|
| VR `recv_to_pub` | PC 收到 Quest 文本到发出 ROS |
| VR `wrist_gap.{left,right}` | 同侧腕包间隔（跟踪率，不是 300 Hz 行率） |
| 臂 `vr_rx` / `vr_age` / `ik` / `loop` / `ik_fail` | 收腕、等到控制拍、IK、整圈、无解次数 |
| 仿真 `e2e.{side}` | VR 收包时间戳 → 仿真拿到 `joint_commands` |
| 仿真 `apply.{side}` | 回调入队 → 写入 `qpos` |
| `[VR Data Rate]` | 所有文本行（头+双腕+landmark）约 300–360 Hz |
| `[Sim Cmd FPS]` | 仿真实际收到的关节指令频率 |

Quest 头显内部时钟未对齐，设备→PC 的空中延迟测不到。`[VR Data Rate]` 不是臂跟踪率；臂跟踪看 `wrist_gap` ≈ 14 ms ≈ **72 Hz**。

- 遥操 `control_rate`：50 → **150 Hz**（yaml）。真机驱动 `control_rate` / `ctrl_hz` **仍为 50 Hz**，未改板卡发送。
- `max_joint_vel`：由「每拍 rad」改为 **rad/s**（默认 4.0，相当于旧 0.08 rad/tick @ 50 Hz）。`SafetyFilter` 每拍上限 = `max_joint_vel * dt`。
- `pos_smoothing` / `rot_smoothing`：仍为 0–1，按 50 Hz 的 α 标定成时间常数 τ，每拍 `α(dt)=exp(-dt/τ)`。改 `control_rate` 不再改变手感。
- 仿真循环原先每圈只 `spin_once` 一次，150 Hz 指令会被丢到 `[Sim Cmd FPS] ~50`。改为 `_spin_drain`（最多 24 次 `spin_once`），订阅 QoS depth 50。修好后 Cmd FPS ≈ **150 Hz**，`e2e` 约 10–12 ms。仿真只保留最新 `q` 再写入 `qpos`：收得比发得慢会跳过中间拍，看起来像冲一截；收得更快则零阶保持，不会因此冲。

有线 TCP + URDF 数值 IK 的仿真对比（150 Hz、drain 之后）：均值 `e2e` ~10–12 ms；DH 的 IK 约 +1 ms；UDP 主要伤尾部；**DH+UDP 左臂曾大量 `ik_fail`、指令中断**。

### 跟手调参曲线（`teleop_tune_plot`）

```bash
ros2 run astral_arm_teleop teleop_tune_plot --ros-args -p arm_side:=right
```

- 遥操默认发 `/teleop/{side}/tune/`：`ee_vr`（未平滑的 VR 映射）、`ee_filt`（滤波目标）、`ee_cmd`（指令 FK）、`xyz`（三条打成一条）。
- 窗口叠画 mm 曲线 + RMS / p95 / hold jitter / 互相关 lag。键位在**曲线窗口**（不是 launch 终端）：`z` 把当前末端设为图零点，`s` 存 CSV，`q` 退出。
- 滑动条热写 ROS 参数（默认左右同步）：手感 `pos_smoothing` / `rot_smoothing` / `motion_scale` / `max_joint_vel`；URDF LM：`ik_w_pos` / `ik_w_ori` / `ik_w_reg` / `ik_max_iter` / `ik_tol`。
- 左下角可热切换 `analytic_dh` ↔ `urdf_numerical`（重建求解器、按当前 `q` 重定 VR 零点）。DH 闭式忽略 LM 权重。仿真 launch 即使以 DH 启动也会带上 `urdf_path`，以便热切到 URDF。
- yaml 只是下次启动的初值；热改不写回文件。`solver_type` / `urdf_path` / `init_pose` / `control_rate` 等不可随意热改（`control_rate` 仍需重启）。

## 2026-08

### Wuji 手

- 包：`wuji_glove`、`wujihand_retargeting`、`wujihand_mujoco_sim`、`wujihand_control`（+ vendored `wujihandros2`）
- 统一话题：`hand_landmarks/{side}` → `/{side}_hand/joint_commands` → driver / MuJoCo
- 三后端：`official` / `wuji_retargeting` / `dexpilot`
- TuningViewer 三层骨架（橙/青/白）+ retarget yaml 热重载
- Quest3 专用 yaml：`retarget_wuji_lib_quest3_{left,right}.yaml`

#### Bug fixes

| ID | 现象 | 根因 | 修复 |
|----|------|------|------|
| Q1 | Quest3 小指呈 **Z 形** | `adaptive_retargeting_xhand` 顺序拉伸 | 默认 `enable_xhand_pinky_adapt:=false`；Wuji launch 强制关闭 |
| Q2 | Quest3 相对手套 **MCP / 掌部偏移** | Quest 已做腕帧+MANO，Retargeter 再变换一次 | `landmark_preprocess:=raw`；手套节点不做 MANO |
| Q3 | Quest3 调参橙/白 MCP 仍偏 | Adaptive 不约束 MCP | quest3 yaml：`snap_mcp_to_robot: true` |
| Q4 | 真机 **收不到** `joint_commands` | BEST_EFFORT vs RELIABLE | 对齐 SensorDataQoS BEST_EFFORT |
| Q5 | 真机未用上调参 yaml | real pipeline 仍指向手套 yaml | `input_source:=quest3` 自动传 quest3 yaml |
| Q6 | dexpilot/official 仿真握拳、不跟手 | MJCF `kp` 过小 | tuning/sim 写 `qpos`+`mj_forward` |

#### 操作提示

- 真机与 MuJoCo **二选一**，勿抢同一 `joint_commands`
- `*_serial` 可空（按 `hand_side` 连）
- USB：`0483` 需 udev `MODE="0666"`
