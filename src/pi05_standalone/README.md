# 独立 Astral pi0.5 推理

只有本目录脚本参与推理；不导入 `astral_ws` 其他包，也不启动 ROS 2。
机器人端依赖 `astral_robot_sdk`、`opencv-python`、`numpy`、`PyYAML`、
`msgpack`、`websockets>=11`。GPU 端依赖 `VLA/openpi` 的运行环境以及
`msgpack`、`websockets>=11`。GPU 与机器人端可使用不同 Python 环境。

## 输入和动作约定

- 胸口 RealSense 彩色图像 → `base_0_rgb`；左腕 USB 彩色图像 → `left_wrist_0_rgb`。
- 两路图像均为 RGB uint8 224×224，等比例缩放后补黑边。
- 状态为左臂 7 个实测关节角（rad）加左夹爪闭合比例（0 张开、1 闭合）。
  SDK 无夹爪实际位置反馈，最后一维是启动时指定的初始值或本脚本最近下发值。
- 模型返回的是 8 维绝对目标，前 7 维仅送左臂电机；最后一维映射成
  左夹爪 `2.5*(1-ratio)` rad。不得使用 Piper 的 6 关节/CAN/prev_actions 格式。

## GPU 端

在 OpenPI 环境安装依赖，并把 `VLA/openpi/src` 放入 `PYTHONPATH`：

```bash
PYTHONPATH=/path/to/VLA/openpi/src:$PYTHONPATH python serve_pi05.py \
  --checkpoint-dir /path/to/openpi-checkpoint \
  --policy-config pi05_astral --host 0.0.0.0 --port 8001
```

LoRA checkpoint 应选 `pi05_astral_lora`。服务启动时预热一次，可能触发较长的
JAX 编译；日志显示 `ready` 后再连接机器人。服务启动时检查训练配置恰好使用
`base_0_rgb` 和 `left_wrist_0_rgb` 两个真实相机槽。检查 checkpoint 的归一化
统计和任务文本与训练数据一致。

## 机器人端

先复制并核对 `config.example.yaml` 的相机物理路径、GPU 地址、任务文本、
夹爪初始比例。USB 设备路径在换口后可能变化。**关闭现有 ROS 机器人驱动和
相机采集进程**，避免重复占用 UDP 端口或视频设备。

```bash
python infer_astral_single.py --config my_config.yaml --mode observe --duration 10 \
  --log-file /tmp/pi05_observe.jsonl
python infer_astral_single.py --config my_config.yaml --mode infer --duration 10 \
  --log-file /tmp/pi05_infer.jsonl
python infer_astral_single.py --config my_config.yaml --mode execute --duration 10 \
  --log-file /tmp/pi05_execute.jsonl
```

`observe` 只读取相机和关节，`infer` 还调用模型但不下发动作，`execute` 下发
模型动作。默认不会上电或归零；必须已由其他方式让机器人处于 WORK、POSITION、
enabled 状态。只有显式添加 `--ready` 才会调用 SDK `one_click_ready`，**该调用
会使能并将所有关节归零**，应在工作空间安全时使用。退出时只断开 SDK，
不会关闭机器人电源。三种模式都要求两路图像及关节反馈新鲜。

动作主循环目标为 30 Hz，模型请求在后台执行。机械臂关节目标每步变化限制为
`max_joint_vel_rad_s/control_hz`；日志同时记录原始动作和实际命令。每个新 chunk 执行到第 10 行时
开始请求下一份；新结果到达后从首行切换。请求变慢时继续执行当前 chunk；
chunk 用尽仍无新结果时停止下发并退出。当前实现刻意没有融合、插值或 ROS
队列，JSONL 逐行记录输入年龄、控制延迟和实际动作，便于定位卡顿。模型
RTT 与服务端耗时在终端日志中显示。单次运行默认 10 秒。

## 验证范围

本仓库开发环境没有相机 `/dev/video*` 和真机控制板，因此只能在本机执行
静态及假设备检查；真实图像、checkpoint 和机械臂动作需在部署机器上逐阶段
验证。首次真机运行请先核对夹爪方向、任务文本、图像画面与 8 维动作范围。
