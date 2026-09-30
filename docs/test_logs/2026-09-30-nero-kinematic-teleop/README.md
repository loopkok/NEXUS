# Nero 直接关节可视化与 IK 无解恢复测试 LOG

日期：2026-09-30。代码版本：`5652732cf168e4e0b934a3f1388a24904ff09d88`。
平台：部署主机 Ubuntu 22.04、ROS 2 Humble，主机桌面 MuJoCo 窗口。
所有运行组件使用 `nero_mujoco`，Quest 输入由合成源提供；没有启动 CAN、串口或真实相机驱动。

## 修改与原因

用户真实 Quest 日志显示：左臂无解时求解 p95 102.5 ms；原仿真位置伺服仍有约 111–113 ms 的命令/反馈相位滞后。
本次默认使用 kinematic：最终仲裁命令直接写入 qpos，清零速度，通过 mj_forward 更新几何。physics 模式仍可显式选择。
仿真腕部 EMA 默认由 0.8 改为 0；实机仍保持 0.8。腕输入与关节反馈订阅使用 best-effort depth 1。
IK 无解时，仅在腕输入及关节反馈仍新鲜的情况下发布当前实测关节的保持候选；回到可达区域可以自动继续。
求解结束后再次检查输入/反馈新鲜度，不用保持消息掩盖真实断流。
原 IK 核心、link7 法兰目标、零 TCP 偏移、Quest 左右坐标映射均未改动。
默认行为在适配器实现中改变，未改装配 profile 内容，profile SHA256 仍为 `bc7b5a34cd066f9bdffa06cce939ef681f10149314691f50b1fe3437ac78d3fc`。

## 回归测试

`colcon build --packages-select nexus_core nero_mujoco_sim --symlink-install` 成功。

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=122 ROS_LOCALHOST_ONLY=1 \
  python3 -m pytest src/nero_mujoco_sim/test src/nexus_core/test/test_contract.py -q
```

33 passed。覆盖直接目标在单步内等于反馈、physics 仍使用伺服、超时保持、急停、过期命令、归位队列隔离，以及 IK 无解保持/恢复、输入断流和求解中输入过期。
关闭 pytest 插件自动加载是为避免主机已有 anyio 插件与系统 pytest 的版本冲突，没有更改系统依赖。

## 带窗口性能

```bash
ROS_DOMAIN_ID=122 ROS_LOCALHOST_ONLY=1 python3 scripts/nexus_nero_sim_runtime_probe.py \
  --profile src/nexus_core/profiles/nero_dual_xhand_mujoco.json --viewer --duration 12
```

| 测量 | 结果 |
|---|---|
| 仿真更新 | 500.0 Hz，实时因子 1.000 |
| 窗口同步 | 25.0 Hz，p95 6.5 ms |
| 两臂命令到反馈相位滞后 | 11 / 12 ms |
| 双手命令到反馈相位滞后 | 12 / 12 ms |
| 命令到达年龄 p95 | 6.2–8.8 ms |
| 反馈 | 四组件约 100 Hz |

该相位测量包含 ROS 投递和反馈采样，并非真实 Quest 到屏幕的端到端延迟。
日志最初的命令超时发生在 DDS 发现/窗口创建期间；统计排除启动阶段。不要将其解读为稳定运行断流。

## Web 完整遥操链

通过 FastAPI TestClient 生命周期启动监控 ROS 节点，使用独立 ROS 域；按真实 Web API 执行启动（viewer=true）、ready、enable、home、teleop/start、stop。
合成 Quest 左右腕与 landmarks 均为每侧 72 Hz；初始 z 轴小幅 0.008 m，18–25 秒切换为 0.8 m 不可达运动，再回到 0.008 m。
连续采样 30 秒控制状态，验证输入持续时始终 TELEOP；无解期间出现保持告警，恢复后两臂重新达到 50 Hz、求解 p95 约 9 ms、failed=0。
不可达期间求解仍可能约 100 ms：本次没有改动原 IK 核心或声称不可达目标可以实时求解。
恢复阶段末 4 秒实测关节运动跨度：左臂 0.0343 rad、右臂 0.0337 rad，确认恢复后实际位置继续变化；四组件反馈维度 7/7/12/12 且均新鲜。
停止合成输入，1.5 秒后验证 PAUSED；完成后停止本次启动的仿真进程。

## 原始记录

- `nexus_kinematic_build.log`：构建。
- `nexus_kinematic_tests.log`：33 项回归。
- `nexus_kinematic_viewer.log`：窗口与命令/反馈性能。
- `nexus_kinematic_full_graph.log`：Web 完整链、无解、恢复和断流。
- `nexus_kinematic_source.log`：合成输入发布统计。末尾异常来自临时输入夹具在 SIGINT 后重复调用 rclpy.shutdown；发生在主动断流阶段，不是产品节点故障。

运动学直接显示不模拟重力、驱动器动力学或物体受力抓取。需要接触/动力学测试时显式选 physics 模式。
本次没有使用真实 Quest 或真机，真实佩戴体验需在用户设备上复测。
