# 相机采集 / 视频回传性能排障（"三路相机把整进程拖到 3fps"全案）

> 2026-08-20~09-21 反复排障，2026-09-22 整理归档。
> 给新会话/接手者：相机链路"帧率掉、整进程卡"的排查顺序，以及为什么性能瓶颈要 py-spy 实证别猜。
> 母本：`src/quest3_video_streamer/README.md`（帧率排障插桩/常见问题）+ `astral_ws/CHANGELOG.md`
> （08-20~09-21 各条）。相机 label 语义化见 [camera-label-semanticization.md](camera-label-semanticization.md)。

## TL;DR（一分钟结论）

低帧率/整进程卡顿的**历史性根因链**，按排查顺序：

| 坑 | 症状 | 根因 | 修法 | 数值 |
|---|---|---|---|---|
| **GIL（全案定案）** | 三路采集把整进程拖到 3fps、捕获/推送全饿死 | rosidl `msg.data = bytes` setter 逐字节 Python 校验（200KB JPEG ≈40 万次迭代/帧）持 GIL | `msg.data = array.array('B', jpg.tobytes())` 走 C memcpy 快路径 | 10.1ms → **0.016ms**（630×），collect_tap + preview 两处 |
| x264 线程爆炸 | H264 上线后流从 ~3fps 塌到 0.0fps | x264 默认 `threads=1.5×核数`，12 核 Jetson 每编码器 18 线程、3 路 54 线程弱核 convoy | x264 options `threads=2` | 3 路共 6 线程 |
| VP8 cpu-used | 三路软编吃光 CPU 阻塞 executor | aiortc 给 libvpx 设 `cpu-used=-6`（画质向最慢档） | 补丁 `cpu-used=-6→8`（realtime 档） | — |
| H264 强转失效 | offer 含 H264 应答仍 VP8 | `setCodecPreferences` 在应答路径无效（aiortc 1.15 源码实证） | setRemoteDescription 后直接重排 `transceiver._codecs`（H264 提前） | 有日志 `codec 首优 VP8 -> H264` |
| 限流误杀 | 源≈目标时 2-3/s 被 rate_skip 误杀 | 硬卡整周期，早到几 ms 的帧被当超速 | 0.8×period 容差 → 后换**令牌桶**（容量 2、速率=max_fps） | 实机 29.8-29.9/s、rate_skip=0 |
| 相机源不启 | Quest 不开 video feed 采不到图 | 源惰性打开（等 WebRTC track 首帧） | `eager_start_sources: true`（service 启动即开源） | — |
| USB3 枚举假30 | 配置720p30却实采约90--120fps，伴随JPEG损坏 | 相机的SuperSpeed UVC描述符只声明120fps，驱动不能凭配置创造30fps | 正式运行严格拒绝；相机走USB2、升级固件或换设备 | testcam: 27.9 vs 95.5fps |

## USB2 有 720p30、USB3 只有 120fps：不是 launch 把 collect 改成了90

`testcam` 的启动配置仍然是 `left_wrist/right_wrist 1280x720@30`，collect tap 也明确为
`max 30.0fps`。但 V4L2 枚举显示：USB2 `/dev/video0` 的 MJPG 720p 支持 30/60/120fps；USB3
`/dev/video8` 的同模式只有一个离散的 120fps interval。V4L2 的离散 frame interval 列表就是设备/驱动
对该格式和尺寸声明的可用集合，设置一个未声明的30fps可能失败或被钳到120。实测 capture 日志对应为
left_wrist 27.9fps、right_wrist 95.5fps；后者低于标称120是主机解码/调度后的实吞吐，不代表协商成功
为90fps。

原链路中 capture 每读到一帧就调用 collect hook，而 collect tap 自身用30 token/s的令牌桶抽取最新帧，
离线 align 再建立严格30Hz网格，因此模型数据不会直接变成90fps。但仍有四项实质风险：

1. 传感器工作在120fps曝光时序，曝光时间、增益、运动模糊和30fps训练画面不同；简单四取一不能恢复
   真正的30fps成像域；
2. USB传输和OpenCV MJPEG解码仍按约95--120fps付费，会挤占双相机、DDS、推理与控制线程资源；
3. 日志已有连续 `Corrupt JPEG data`。该libjpeg信息不含设备标签，不能仅凭日志把每次损坏都归给
   `/dev/video8`，但可确定至少一路MJPEG在进入模型/采集前已损坏；解码后的坏画面会被抽头重新编码成
   结构合法JPEG，普通“能否decode”校验可能漏检；
4. 旧 collect header stamp 在JPEG编码完成后生成，高负载下把编码排队延迟伪装成采集时刻，恶化
   state/双相机最近邻对齐的可解释性。

当前修复：`WebcamSourceAdapter.start()` 在设置 FOURCC/尺寸/FPS 后读取并记录 negotiated 值；
`strict_capture_fps=true`（腕部相机默认）且偏差超过 `capture_fps_tolerance=0.15` 时释放设备、明确报错并
拒绝启动。这样正式采集和推理不会在用户不知情时使用错误模式。严格模式显式关闭时，捕获线程仍连续
排空物理流，容量2的令牌桶只把最新目标帧率交给 preview/collect/WebRTC，并在 diagnostics 同时记录
`fps`（驱动读取）与 `delivered_fps`（下游交付）；它只是诊断兼容模式，不能修复USB/MJPEG损坏或120fps
曝光域。collect timestamp 已提前到抽头准入时固定，编码时延不再污染图像时刻。

正式处理顺序：

1. 用 `v4l2-ctl -d /dev/videoN --set-fmt-video=width=1280,height=720,pixelformat=MJPG --set-parm=30
   --get-fmt-video --get-parm` 验证实际协商；若返回120或拒绝，软件不能强制该描述符产生30fps；
2. 让该相机经 USB2-only 口/集线器/线缆连接，并用 `lsusb -t` 确认 `480M`，再确认 MJPG 720p30；两台
   相机尽量分散到不同根控制器/Hub；
3. 若必须USB3，升级相机固件或更换在SuperSpeed描述符中真实列出30fps的型号；
4. 重新启动后必须看到 negotiated `1280x720@30 MJPG`、capture/delivered/collect 均约30fps，且至少
   持续录制5--10分钟无 `Corrupt JPEG data`，再用于正式采集或推理；已有可疑数据应抽帧可视化复核，
   不能只看HDF5/JPEG能否打开。

## 全案根因：rosidl `data` setter 逐字节校验吃光 GIL（最贵的教训）

**症状史**：采集 video8 3fps；推送开关 ON/OFF 的 16↔30fps（无 Quest 也复现）；三种编码配置
吞吐不变（编码器从来无罪）。**py-spy 实锤**：三路 collect-tap 线程全部停在
`sensor_msgs/msg/_compressed_image.py` 的 genexpr——`msg.data = bytes` 触发 rosidl setter 的
`all(isinstance(v, int)...)` + `all(0<=v<256...)` 两遍逐字节 Python 校验，持 GIL 数百 ms；
3 路 × 30fps 把 GIL 占满，捕获线程/事件循环/WebRTC 全部饿死。
**修复**：`msg.data = array.array('B', jpg.tobytes())` 走 setter 的 array 快路径（C memcpy），
本地实测 **10.1ms → 0.016ms（630×）**。collect_tap + preview 两处都改。
**教训**：嵌入式 CPU 上每帧 Python 开销都要过 GIL 这根弦；瓶颈判断用 py-spy 插桩实证，
别猜（编码器/x264 曾先被冤枉）。

## 编码器/线程坑

1. **H264 上线后流塌缩（x264 线程爆炸）**：`threads=1.5×核数` → 12 核 Jetson 54 线程弱核 convoy
   （zerolatency 的 sliced-threads 同步开销在小核上被放大）；VP8 基本单线程所以只是慢不是塌。
   修：`threads=2`。
2. **VP8 软编码挤爆 CPU**：aiortc 给 libvpx 设 `cpu-used=-6`（比默认更慢的画质向档位），三路软编
   吃光 CPU 并阻塞默认 executor。修：补丁 `cpu-used=-6→8`。两个补丁带 `encoder_patch_status()`
   启动自证行（结构不符静默回退会显示 OFF）。
3. **H264 强转失效（setCodecPreferences 应答路径无效）**：`createAnswer` 直接用 setRemoteDescription
   算好的 `transceiver._codecs`（offer 顺序 VP8 在前）。修：setRemoteDescription 之后、createAnswer
   之前直接重排 `_codecs`（H264 及其 RTX 伴随提前；条目来自公共集深拷贝）。`apply_offer` 加编码列表日志。

## 限流/采集抽头坑

1. **采集抽头限流器抖动误杀**：硬卡整周期（33.3ms）时 30fps 源 ±5ms 抖动，早到几 ms 的帧被
   rate_skip 误杀 1/3。修：先 0.8×period 容差，后换**令牌桶**（容量 2、速率=max_fps）——源≈目标
   抖动被吸收、长跑 99%+ 透传。实机 tap 发布 29.8-29.9/s、rate_skip=0。
2. **采集与推送解耦**：`push_enabled`/`active_cameras` 只管"看"（Quest + web 实时画面），采集抽头
   唯一准入条件 = **话题有订阅者**（编码本来就按需、无订阅零开销）——"关推送省 CPU"不能影响"录"。
3. **回传降载旋钮**：`push_max_width`（默认 960）/`push_fps`（默认 30）只降 WebRTC track
   （等比缩放 + 黑帧同尺寸保编码 context），采集抽头全帧全速不受影响。960→640 后 13-17fps/路。

## 设备发现/指纹坑

1. **auto_scan 选错节点**：D435i 多采集节点，按"最小编号"会选到深度 `/dev/video4`（Z16）黑帧。
   修：`scan.py` 按像素格式打分（彩色 YUYV/MJPG/NV12=100 > 红外 GREY/Y8=10 > 深度 Z16=0），
   IR 节点只要带 GREY/Y8I 等 fourcc 就丢；同物理设备多接口（1-2:1.0/1.3）归同一设备再比分；
   同时 enum mplane（彩色常是 VIDEO_CAPTURE_MPLANE）。
2. **label 漂移**：换口/重插后 `/dev/videoN` 重排，videoN 是"内核号"不是"角色"。修：`label_aliases`
   按硬件指纹钉稳定 label（**混合策略**：realsense 走 by-id 序列号+`-video-index0` 精确钉彩色；
   廉价 USB 相机 by-id 序列号是假的 [同款全同] → 按 by-path 物理口位钉）。详见
   [camera-label-semanticization.md](camera-label-semanticization.md)。
3. **D435i by-id index 串位**：复合 USB 设备（彩色+IR+depth 多接口）内核编 by-id 时 video-index0
   落到深度节点（`v4l2-ctl` 实锤 Z16），真正彩色（YUYV）在 by-id 无条目 → 序列号 alias 钉不上
   彩色。修：base 改 by-path 彩色口位。`python3 -m quest3_video_streamer.scan` 打印全部稳定指纹。

## 帧率排障插桩（定位帧丢在哪一段）

采集激活时每 5 秒打两类 INFO 日志：

- `[capture <label>] driver-side N fps`——捕获线程从驱动读帧的实际速率。这里低 = 捕获段慢；
  满 30 而下方 published 低 = 抽头/发布段慢。
- `[tap <label>] submit=N/s rate_skip=.. queue_full=.. encoded=N/s published=N/s encode=Xms publish=Xms`
  ——`queue_full` 高 = 编码/发布跟不上（看 `encode` 大还是 `publish` 大区分 JPEG 编码瓶颈与 DDS 发布阻塞）。

2026-09-21 起升级为 `/quest3_video_streamer/diagnostics`（每 5s 按相机发布 capture/tap 结构化
统计 + `max_frame_gap_ms`），推理侧 `log_dir` 自动合并进 `camera_diagnostics.jsonl`（`policy_rx`
逐帧 gap/payload/decode ms），与 metrics/cmd/control 同一时间轴——定位 left_wrist 超时在
`V4L2 capture → collect tap → DDS → policy callback` 哪一段（详见
[2026-09-21-pi05-inference-investigation.md](2026-09-21-pi05-inference-investigation.md) C 轮）。

## 排障顺序（checklist）

1. 低帧率/卡顿先看 tap INFO 日志（capture 满不满、queue_full 高不高），定位段；
2. `py-spy` 挂到 streamer 进程看线程栈（GIL 类问题一锤定音）；
3. 怀疑编码器 → 看 `apply_offer` 编码列表日志 + `encoder_patch_status()`（补丁是否静默回退）；
4. 换口/重插后帧率/画面异常 → `scan.py` 打印指纹，核对 label_aliases；
5. 别一开始就调相机会话参数/分辨率（那是掩盖，不是定位）。
