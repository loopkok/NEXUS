# Changelog（astral_ws）

Quest3 → Astral 双臂 + Wuji 双手。从 `xnero_ws-main` 迁入。各包 README 内亦有对应条目。

时间均为北京时间。

## 2026-08-31

**E2E 延迟：QoS 只留最新帧**——腕位 / body / 关节状态 / 指令流改为 `BEST_EFFORT` `KEEP_LAST` **depth=1**。mocap 发布、teleop 订阅、driver/monitor 指令订阅对齐。旧 depth 10/20 会在 IK 跟不上时把旧腕位排队，表现为「手已经停了臂还在走」。Joy/头/hips/landmarks 同步改，避免 BEST_EFFORT 发布对不上 RELIABLE 订阅。

**Web Launch 日志**——环形缓冲 500→**8000**，WS 实时推尾 **800** 行；新增 `GET /api/v1/logs` 拉全量。系统 tab 日志窗 240px→**70vh**，复制/下载。

**运维清场 + 夹爪刷屏**——`jetson_teleop_start.sh` 强清补 `controller_start_gate`、`data_collect`、`keyboard_controller`、`ik_solver`。夹爪 `log_interval_s` 默认/yaml/launch 均 **0.0**，关掉每 2 秒一条的状态日志。

**左臂 TCP 偏置撤回**——`tcp_offset` Y **−0.147 → 0**（与右腕一致）。此前为贴 Quest 左手视觉中心，实测不必。

**数采段边界速率不为负**——writer 每段新建计数归零；`_prev_counts` 随段清零，delta 再 `max(0, …)`，避免下一段第一秒打出负 Hz。

**数采防护体系 + launch 参数优先级修复**——`astral_data_collect` / `astral_web_monitor`。①launch 参数改空串默认 + OpaqueFunction：**yaml 成为唯一默认来源**，此前 launch 默认值（如 `arms` 默认 `left,right`）静默盖掉 yaml——"改了 yaml 却采出旧 schema"即此机制；CLI/预设显式传参仍可覆盖。②**domain 单例锁**：`/data_collect/control` 谁订阅谁开录是双份数据的根因（残留节点 + 再启动 = 双开同录）；节点对 `/tmp/astral_data_collect_domain{N}.lock` 取排他锁，第二个节点构造即拒绝，session 不同也不放过。③**段号原子占位**：mkdir 抢号撞号让位——老残留进程不持锁，段号互斥由文件系统原子性兜底（此前一个 start 双节点各写 000000/000001）。④**多节点红条**：web 按 `/data_collect/state` 发布者计数 >1 即提示残留。⑤**低帧率告警**：录制中参考相机实率 < dataset_fps/2 → state `low_fps_warning` + 日志节流 WARN + 卡片红条。⑥web 重启端点等旧进程真退出再启（单例锁窗口）。另修：cameras 逗号字符串曾被逐字符拆解；jpeg_quality 参数此前被静默忽略。

**auto_scan MJPG 确定性优先（非帧率根因，实机已证伪带宽假设）**——`quest3_video_streamer`。MJPG/JPEG 提至 110 分消除平票依赖枚举序的不确定性。**实机验证**：d435i 彩色节点仅播 YUYV（无 MJPG 可优先）、三相机分属不同总线且 1080p YUYV 单跑满 30fps——采集低帧率非带宽问题；双订阅者收到完全相同帧集合证明丢失在 streamer 进程内部（抽头总共只发了那么多）。`test_scan_mjpg.py` 保留作为确定性回归。

**H264 强转失效根因定案与修复**——`quest3_video_streamer`。**实机证据**：`offer video codecs: ['VP8','rtx','VP9','H264','AV1','H265',...]`——Quest **明明提供 H264**，应答却仍是 VP8（`h264_forced` 空转实锤）。**根因（aiortc 1.15 源码实证）**：`setCodecPreferences` 在应答路径无效——`createAnswer` 直接用 `setRemoteDescription` 时算好的 `transceiver._codecs`（offer 顺序，VP8 在前），发送端 `RTCRtpSender` 以 `_codecs[0]` 建编码器。**修复**：弃用 setCodecPreferences，改为在 setRemoteDescription 之后、createAnswer 之前直接重排 `transceiver._codecs`（H264 及其 RTX 伴随提前；条目来自协商公共集即 offer 参数深拷贝，无 fmtp 不匹配风险）；公共集无 H264 时打日志保持 VP8。新增重排日志 `codec 首优 VP8 -> H264`。新增 2 例单测（重排序+RTX 伴随、无 H264 回退）。

**相机 eager start：采集/预览与 Quest 视频解耦**——`quest3_video_streamer`。此前相机源仅在 WebRTC track 首帧时惰性打开——Quest 不开 video feed，采集抽头就拿不到任何图像。新增 `eager_start_sources`（yaml 默认 true）：service 启动即打开全部源，Quest 连接后 track 复用已开源（`WebcamVideoSource.start`/`RosImageSource.start` 补幂等守卫）。同时支撑"无 WebRTC 负载时采集支路满速"的对照实验（三组编码配置吞吐不变已证瓶颈非编码器，疑似 aiortc 媒体面 Python/GIL 开销拖垮全进程，待 py-spy 实证）。

**H264 上线后流塌缩修复：x264 线程爆炸**——`quest3_video_streamer`。**实机证据**：重排生效（`codec 首优 video/VP8 -> video/H264` ×3、协商 H264）后，流从 ~3fps 一路塌缩到 **0.0fps/0kbps**，且捕获/采集线程照旧被饿死。**根因**：x264 默认 `threads=1.5×核数`——12 核 Jetson 每编码器 18 线程、3 路共 54 线程在弱 ARM 核上 convoy（zerolatency 的 sliced-threads 同步开销在小核上被放大）；对比 VP8 基本单线程所以只是慢不是塌。**修复**：x264 options 加 `threads=2`（3 路共 6 线程，给捕获/采集留核）；本地 540p veryfast threads=2 实测 215fps（x86），Jetson 预期每路 15-30fps。另加补丁幂等守卫（模块重复 import 时不再自包装递归）。

**低帧率根因定案与修复：VP8 软编码挤爆嵌入式 CPU**——`quest3_video_streamer`。插桩证据链：tap `publish=3.4ms`（DDS 无罪）、编码线程有活跑不动（encode 20ms 吞吐仅 7.5/s）、捕获线程进程内 17-20fps（单跑 30）、**协商日志显示三路全是 VP8**（初判 Quest 未提供 H264；次日实机日志证伪——offer 含 H264，实为强转代码在应答路径无效，见上方 H264 条目）——aiortc 给 libvpx 设 `cpu-used=-6`（比默认更慢的画质向档位），三路软编在 Jetson 上吃光 CPU 并阻塞默认 executor，全进程线程互相饿死。修复三件套：① **VP8 `cpu-used=-6→8` 补丁**（realtime 档位上限提速，镜像 aiortc 1.15 原生 options 仅改此一项）；② **x264 preset veryfast 补丁**（dormant：Quest 端 app 开 H264 后立即受益）；③ **回传降载旋钮** `push_max_width`/`push_fps`（默认 960/30 ≈ 编码像素 -59%），track 级等比缩放 + 黑帧同尺寸保编码 context，采集抽头全帧全速不受影响。两补丁带 `encoder_patch_status()` 启动自证行（结构不符静默回退会显示 OFF）；`apply_offer` 新增 offer 编码列表日志（Quest 报了什么一眼可见）。新增 `test_push_throttle.py`（11 例，含补丁激活断言）。调参阶梯：960/30 → 960/15 → 采集时 mute 腕部轨。

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
