# quest3_video_streamer

把 PC 上的相机画面（Intel RealSense D435i 彩色流 + USB 腕部相机）通过 WebRTC 推送到 Meta Quest 3，用于遥操作 teleop 的视频回传。自包含、不依赖 `hand-tracking-sdk`、不使用虚拟环境，直接跑在系统 Python3 + ROS 2 Humble 上。

一个 WebRTC peer connection 承载多路视频 track（一路一个相机），Quest 端按 `video_config` 消息把每路画面放到 3D 空间的独立面板上。

---

## 目录

- [架构总览](#架构总览)
- [数据流与线程模型](#数据流与线程模型)
- [模块说明](#模块说明)
- [配置（params.yaml 驱动）](#配置paramsyaml-驱动)
- [Launch 文件](#launch-文件)
- [信令协议](#信令协议)
- [多路 WebRTC 与延迟设计](#多路-webrtc-与延迟设计)
- [统计输出](#统计输出)
- [依赖与安装](#依赖与安装)
- [使用示例](#使用示例)
- [常见问题](#常见问题)
- [更新日志](#更新日志)

---

## 架构总览

```
        相机                          PC (本包)                              Quest 3
 ┌───────────────┐   ┌──────────────────────────────────────┐   ┌──────────────────┐
 │ D435i 彩色    │──▶│ source adapter (v4l2 / ros)           │   │                  │
 │ USB 腕部相机  │──▶│   └─ next_frame() 节流到采集速率      │   │  Unity WebRTC    │
 └───────────────┘   │ VideoWebRTCSender (aiortc)            │   │  多个 RawImage    │
                     │   └─ 每路一个 VideoStreamTrack        │◀──│  面板 (3D 布局)   │
                     │ Quest3VideoService (信令+编排+统计)    │   │                  │
                     │   └─ WebSocket 信令 :8765              │   │  Start Stream    │
                     └──────────────────────────────────────┘   └──────────────────┘
                                     ▲ 信令 (WebSocket)  ──────────────▶▲
                                     └ RTP/SRTP 视频 (WebRTC, DTLS/SRTP) ─▶│
```

- **信令**：WebSocket（默认 `0.0.0.0:8765`），用于 hello / video_config / offer/answer / ICE 交换。
- **媒体**：WebRTC over SRTP，每路相机一个 outbound video track，走单条 peer connection。
- **mocap 下行**：Quest→PC 的手部/身体数据由 `quest3_hand_mocap` 包独立接收（TCP `:8000`）。本包默认不碰 8000，避免冲突；仅在单独跑本包且需要让 Quest 进入 streaming 阶段时，可开 `enable_mocap_tcp` 起一个 drain sink。

## 数据流与线程模型

两个线程，一个 asyncio 事件循环：

- **rclpy spin 线程**（daemon）：跑 `rclpy.spin(node)`，在 ROS 相机话题回调里把帧转 numpy，再 `loop.call_soon_threadsafe` 投递到 asyncio 队列。
- **主线程 asyncio 循环**：跑信令服务器 + WebRTC sender + 统计循环。

关键点：**所有 `next_frame()` 都按各自采集速率节流**（阻塞到新帧才返回），避免 aiortc 的 RTP 发送循环空转编码重复帧、抢占共享事件循环。这是多路低延迟的核心（见[延迟设计](#多路-webrtc-与延迟设计)）。

## 模块说明

| 文件 | 职责 |
|---|---|
| `streamer_node.py` | ROS 2 可执行节点。读 `params.yaml`（`cameras` 列表 + 每相机嵌套块）构建源列表，启动 rclpy spin 线程 + asyncio 服务。 |
| `service.py` | `Quest3VideoService`：信令会话编排、hello/video_config/offer/ice 处理、统计循环、`video_config` 下发（含每路布局）。 |
| `signaling.py` | WebSocket 信令服务器 + 连接封装。 |
| `schemas.py` | 信令消息信封（`SignalingMessage`）的序列化/解析。 |
| `webrtc_sender.py` | `VideoWebRTCSender`：aiortc peer connection，多路 track，H264/VP8 编码码率补丁，每路统计（fps/码率）。 |
| `source_base.py` | `VideoSourceAdapter` 协议 + `VideoFormat`（含 `fov_h_deg`、`label`）。 |
| `ros_source.py` | `RosImageSourceAdapter`：订阅 `sensor_msgs/Image` → `av.VideoFrame`，回调线程安全投递，队列 maxsize=1 丢旧留新。 |
| `webcam_source.py` | `WebcamSourceAdapter`：cv2 V4L2 直读，后台 daemon 线程做 `cv2.read()`，`next_frame()` 用 asyncio.Event 节流到采集速率。 |
| `config/params.yaml` | 唯一可调来源：相机列表、每相机 source/device/preset/fov/label/force_mjpg/layout + 公共参数。 |
| `launch/*.launch.py` | 只加载 yaml + 按 `d435i.source` 决定是否启动 `realsense2_camera_node`，不写死可调参数。 |

## 配置（params.yaml 驱动）

所有可调项集中在 `config/params.yaml`，结构为 `quest3_video_streamer.ros__parameters`：

```yaml
quest3_video_streamer:
  ros__parameters:
    cameras: ["d435i", "wrist_left", "wrist_right"]   # 推哪些相机 + track 顺序

    d435i:
      source: "v4l2"            # v4l2=cv2直连(低延迟) / ros=realsense2_camera_node
      device: "/dev/video8"     # v4l2 设备节点（ros 模式忽略）
      topic: "/camera/camera/color/image_raw"   # ros 模式话题
      preset: "1080p30"         # 480p30 / 720p30 / 1080p30
      fov_h_deg: 69.0           # 水平视场角，Quest 据此按真实尺度缩放面板
      label: "d435i"            # track 标签
      force_mjpg: false         # D435i UVC 彩色是 YUYV，不强制 MJPG
      layout:
        position: [0.5, -0.1, 1.8]   # 米，相对眼睛（z=面板距离）
        distance: 1.8
        size_multiplier: 0.6         # 面板缩放（1.0=按 fov 自然尺度）

    wrist_left:  { source: "webcam", device: "/dev/video0", preset: "720p30", fov_h_deg: 60.0, ... }
    wrist_right: { source: "webcam", device: "/dev/video2", preset: "720p30", fov_h_deg: 60.0, ... }

    signaling_host: "0.0.0.0"
    signaling_port: 8765
    enable_mocap_tcp: false    # 与 hand_mocap 同跑时保持 false
    verbose: false
```

**改任何参数只需编辑这个 yaml**，不用动 launch 和代码。加相机：新增一个同名块 + 加进 `cameras` 列表。

### 每相机字段含义

| 字段 | 作用 |
|---|---|
| `source` | `v4l2`（cv2 直连，最低延迟，不走 ROS/DDS）/ `ros`（订阅 sensor_msgs/Image）/ `webcam`（同 v4l2） |
| `device` | v4l2/webcam 模式的 `/dev/videoN` |
| `topic` | ros 模式的图像话题 |
| `preset` | 采集分辨率+帧率，决定 RTP time_base 与编码目标尺寸 |
| `fov_h_deg` | 相机水平视场角。Quest 用它把面板缩放到真实尺度，修"放大镜"问题 |
| `label` | track 标签，用于 Quest 端识别和 PC 端统计 |
| `force_mjpg` | USB 腕部相机 `true`（MJPG 高分辨率）；D435i UVC 彩色 `false`（YUYV） |
| `layout.position` | 面板在 3D 空间的位置（米，相对眼睛） |
| `layout.distance` | 面板距离 |
| `layout.size_multiplier` | 面板缩放系数 |

## Launch 文件

| Launch | 用途 | 默认相机 |
|---|---|---|
| `multi_camera.launch.py` | D435i + 2 腕部相机三路推流 | yaml 全部 3 个 |
| `realsense.launch.py` | 单 D435i 推流（覆盖 `cameras: ["d435i"]`） | 仅 d435i |
| `usb_camera.launch.py` | 单 USB 相机推流（legacy，参数式，未走 yaml） | 单 usb_cam |

`multi_camera` / `realsense` 都从 yaml 读配置；`d435i_source` CLI 参数可临时覆盖 yaml 的 `d435i.source`：

```bash
ros2 launch quest3_video_streamer multi_camera.launch.py d435i_source:=ros   # 强制走 ROS 节点
```

## 信令协议

WebSocket 文本帧，JSON 信封 `{type, session_id, payload}`。握手流程：

```
Quest                          PC (本包)
  │── hello ────────────────────▶│
  │◀──────────── hello_ack ─────│
  │◀──────── video_config ──────│   (tracks[]: index/label/w/h/fps/fov_h/layout)
  │── start_video ──────────────▶│
  │── offer (SDP) ───────────────▶│   (N 个 recv-only video transceiver)
  │◀──────── answer (SDP) ───────│
  │◀─/── ice_candidate ──/────▶│   (双向)
  │◀──────── video_state=playing│
  │◀──────── stats (周期) ──────│   (total + per-track fps/kbps)
  │── ping ─────────────────────▶│   (可选保活)
  │◀──────── pong ──────────────│
```

`video_config` 的 `tracks` 数组告诉 Quest：有几路 track、每路的尺寸/视场/3D 布局。Quest 据此创建对应数量的 recv transceiver 并摆放面板。PC 收到 offer 后按 track 顺序 `addTrack`，answer 回去。

## 多路 WebRTC 与延迟设计

多路相机共享一个 asyncio 事件循环 + 一条 peer connection。延迟相关的几个关键设计：

1. **源端节流（核心）**：每个 `next_frame()` 阻塞到"新帧"才返回（ROS 源用 `asyncio.Queue.get()`，webcam 源用 `asyncio.Event`）。否则 aiortc 的 `_run_rtp` 会全速空转、反复编码同一帧，把事件循环占满，饿死其他 track。这是早期"D435i 多路就卡、单路不卡"的根因。
2. **webcam 后台线程抓帧**：`cv2.read()` 是阻塞调用，放在 daemon 线程里持续抓最新帧，`next_frame()` 非阻塞取最新，避免阻塞 asyncio 循环。
3. **D435i 走 v4l2 直连**：D435i 彩色流是标准 UVC（`/dev/video8`，YUYV），用 cv2 直读绕开 `realsense2_camera_node` + DDS 的多层缓冲（realsense 节点内部缓冲 + DDS + 队列，2~3 帧延迟），延迟降到和腕部相机一致。`ros` 模式保留为回退。
4. **编码已由 aiortc 丢线程池**：aiortc 的 `encode` 走 `run_in_executor`，不直接阻塞循环；多路并行编码靠线程池，20 核足够。
5. **码率补丁**：aiortc VP8 默认 500kbps / 上限 1.5Mbps，Quest 的 REMB 会压得更低。本包把 `DEFAULT/MIN/MAX` 抬到 5/3/12 Mbps，保证 1080p 清晰。

### 码率（在 `webrtc_sender.py` 顶部）

```python
_BITRATE_MIN = 3_000_000      # 3 Mbps 下限
_BITRATE_DEFAULT = 5_000_000 # 5 Mbps 起步
_BITRATE_MAX = 12_000_000    # 12 Mbps 上限（1080p 锐）
```

想更锐就抬 `DEFAULT`/`MAX`，总上行会涨（USB 3.0 仍够）。

## 统计输出

每秒一行，含每路和总量：

```
stats session=... total_fps=89.9 total_kbps=15000 rtt_ms=None drops=0 |
  [0:d435i fps=30.0 5000kbps] [1:wrist_left fps=30.0 4900kbps] [2:wrist_right fps=30.0 5000kbps]
```

- `fps`：在 `AdapterTrack.recv()` 里按 track 计帧（aiortc 的 outbound-rtp 没有 `framesSent` 字段，故自计）。
- `bitrate_kbps`：从 outbound-rtp 的 `bytesSent` 增量按时间算。
- `rtt_ms`：aiortc 1.15 不填 `candidate-pair.currentRoundTripTime`，故常为 `None`（链路健康可忽略）。
- `drops`：`recv()` 返回 None 的次数。

mocap 下行统计在 `quest3_hand_mocap` 包：`[Mocap Downlink] X kbps` + `[Latency][VR] recv_to_pub`。

**Quest↔PC 线总带宽** = 视频上行 `total_kbps` + mocap 下行 `[Mocap Downlink]` kbps（两个终端各看一行相加）。

## 依赖与安装

ROS 2 Humble + 系统 Python3（不要虚拟环境）。

```bash
# ROS 依赖
sudo apt install ros-humble-sensor-msgs ros-humble-ament-index-python python3-yaml
# 可选相机驱动（ros 模式才需要）
sudo apt install ros-humble-realsense2-camera ros-humble-usb-cam

# Python pip 依赖（aiortc/av/numpy/opencv/websockets 非 rosdep 可解析，pip 装到系统）
python3 -m pip install aiortc av numpy opencv-python websockets
```

构建：

```bash
cd <workspace>
colcon build --packages-select quest3_video_streamer --symlink-install
source install/setup.bash
```

USB 相机权限：把用户加进 `video` 组（`sudo usermod -aG video $USER`，重新登录生效）。

## 使用示例

```bash
# 三路（D435i 1080p + 2×腕部 720p），默认 v4l2 直连
ros2 launch quest3_video_streamer multi_camera.launch.py

# 单 D435i
ros2 launch quest3_video_streamer realsense.launch.py

# 单 D435i 走 ROS 节点（回退）
ros2 launch quest3_video_streamer realsense.launch.py d435i_source:=ros

# 与 hand_mocap 一起跑（典型 teleop）
# 终端1：ros2 run quest3_hand_mocap quest3_udp_mocap --ros-args -p protocol:=tcp_wireless
# 终端2：ros2 launch quest3_video_streamer multi_camera.launch.py
```

Quest 端：在 astral-tracking app 里填 PC 的信令地址（WiFi 或 `adb reverse tcp:8765 tcp:8765`），Start Stream。

## 常见问题

- **画面放大像放大镜**：`fov_h_deg` 没设对。D435i 彩色 ~69°，USB 相机 ~60°，设成相机真实视场角即可。
- **D435i 多路时卡、单路不卡**：已由源端节流 + v4l2 直连解决；若仍卡，检查是否用了旧版（webcam `next_frame` 不节流）。
- **stats 里 fps=0 但有画面**：升级到按 track 自计帧的版本（早期版本误读 aiortc 的 `framesSent`）。
- **`rtt_ms=None`**：aiortc 1.15 限制，非链路问题。
- **v4l2 模式 D435i 打不开**：确认 `/dev/video8` 存在且用户在 `video` 组；D435i 彩色是 YUYV，`force_mjpg` 必须为 `false`。

## 更新日志

### v0.3 — yaml 驱动 + 统计完善
- **配置集中化**：所有可调项（相机列表、source/device/preset/fov/label/force_mjpg/layout）移入 `config/params.yaml`，结构为 `quest3_video_streamer.ros__parameters` 嵌套块。
- **launch 不再写死参数**：`multi_camera.launch.py` / `realsense.launch.py` 改为只加载 yaml + 按 `d435i.source` 决定是否启动 `realsense2_camera_node`；CLI `d435i_source` 可临时覆盖。
- **节点重构**：`streamer_node.py` 按 `cameras` 列表读每相机嵌套块（点分参数）构建源；`allow_undeclared_parameters` + `automatically_declare_parameters_from_overrides` 自动接收 yaml 嵌套参数，`_safe_declare` 防重复声明。
- **每路统计修复**：aiortc 1.15 的 outbound-rtp 没有 `framesSent` 字段，改为在 `AdapterTrack.recv()` 里按 track 自计帧 → `fps` 真实；`bitrate_kbps` 从 `bytesSent` 增量算；聚合直接对所有 outbound-rtp 求和，不再依赖 trackId 匹配。
- **默认分辨率**：D435i 1080p30、腕部 720p30；码率 `DEFAULT 5 / MIN 3 / MAX 12 Mbps`。

### v0.2 — 多路 WebRTC + 延迟架构修复
- **多路 track**：单 peer connection 承载 N 路相机，`video_config` 下发每路布局；Quest 端按 transceiver.Mid 路由纹理到对应面板。
- **源端节流（核心延迟修复）**：webcam `next_frame()` 改用 `asyncio.Event` 阻塞到新帧才返回，避免 aiortc 发送循环空转编码重复帧、饿死 D435i。
- **webcam 后台线程**：`cv2.read()` 移到 daemon 线程，`next_frame()` 非阻塞取最新帧。
- **D435i v4l2 直连**：新增 `d435i_source` 开关，v4l2 模式用 cv2 直读 `/dev/video8`（YUYV）绕开 realsense 节点 + DDS，延迟降到和腕部相机一致；`ros` 模式保留回退。
- **`force_mjpg` 开关**：D435i UVC 彩色是 YUYV，不强制 MJPG；USB 腕部相机仍强制 MJPG。
- **mocap 下行统计**：`quest3_hand_mocap` 加 `BandwidthMeter`，UDP/TCP 接收路径按字节统计 `[Mocap Downlink] X kbps`。
- **布局调整**：主视角右移放大、腕部相机往中间靠。

### v0.1 — 初始自包含包
- 从 `hand-tracking-sdk` 视频部分独立成新 ROS 2 包 `quest3_video_streamer`，不依赖 hand-tracking-sdk、不用虚拟环境，跑在系统 Python3。
- 自研信令（`schemas.py` / `signaling.py`）+ WebRTC sender（`webrtc_sender.py`）+ 源适配器（`ros_source.py` / `webcam_source.py`）。
- `RosImageSourceAdapter`：sensor_msgs/Image → numpy → av.VideoFrame，无 cv_bridge 依赖（规避 ABI 不匹配）。
- `WebcamSourceAdapter`：cv2 V4L2 直读 USB 相机，MJPG FOURCC。
- launch：`realsense.launch.py`（D435i）、`usb_camera.launch.py`（USB）、`multi_camera.launch.py`（三路）。
- 码率补丁：抬高 aiortc VP8/H264 编码码率上下限，防 REMB 压码率导致糊。
- `fov_h_deg` / `label` 加入 `VideoFormat`，Quest 据视场角按真实尺度缩放面板，修"放大镜"问题。
