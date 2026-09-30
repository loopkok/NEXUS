# Astral / Nero 双臂遥操 100 Hz 测试 LOG

日期：2026-09-30。代码版本：`cf0fcf6a2d0f4b3c2a7fe6811f07d2afec5a40a0`。
平台：部署主机 ROS 2 Humble，合成输入每侧 72 Hz。Astral dry_run 驱动、Nero MuJoCo；没有启动真机或真实相机。

## 修改

- Astral 左右 YAML、节点缺省频率设为 100 Hz。
- NEXUS Astral 适配器显式加载左右预设为参数字典，解决 YAML 节点名与 NEXUS 实例节点名不匹配导致全部预设未加载的问题。
- Astral 与 Nero 的装配入口显式设置 control_rate=100 Hz，可通过组件 teleop.control_rate 覆盖；Nero 直接运行的节点缺省也为 100 Hz。
- 原 IK 核心及 profile 内容/哈希未改。Astral 加载后的 solver_type 为预设 geometric。
- Nero 默认速度上限保持 3.25 rad/s，按实际成功更新间隔计算步长，并将延迟追赶限制到最多两个控制周期。100 Hz 标称每周期 0.0325 rad；显式 max_joint_step 配置保持其原语义。
- 最终命令仲裁此前已为 100 Hz，无需改变。

## 构建与单元测试

```bash
colcon build --packages-select nexus_core astral_arm_teleop nero_mujoco_sim --symlink-install
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=132 ROS_LOCALHOST_ONLY=1 \
  python3 -m pytest src/nexus_core/test src/nero_mujoco_sim/test -q
```

构建成功，45 passed。覆盖改名组件加载 Astral 全预设、两种 Nero profile 频率、原接口/IK/超时回归、100 Hz 默认速度限制及延迟追赶。

## 运行验证

通过 Web 后端启动两个装配，按 ready → enable → home → teleop/start → stop 流程运行。
Astral：astral_dual_gripper，dry_run=true；Nero：nero_dual_xhand_mujoco，viewer=true。
关闭设备输入和物理相机，独立 ROS 域，仅由合成输入提供每侧 72 Hz 腕位姿/landmarks。
通过实际节点的 ROS 参数服务查询左右臂 control_rate；Astral 额外查询 solver_type。
每套装配观察 12 秒，统计排除前两秒发现阶段，计数窗口 9 秒。

| 装配 | 左臂候选 | 右臂候选 | 左/右最终命令 | 实际 control_rate |
|---|---:|---:|---:|---:|
| Astral | 100.0 Hz | 99.2 Hz | 100.0 / 100.0 Hz | 100.0 / 100.0 |
| Nero MuJoCo | 100.0 Hz | 100.0 Hz | 100.0 / 100.0 Hz | 100.0 / 100.0 |

两套装配结束时均 TELEOP，四组件反馈新鲜；捕获的装配启动/运行日志无 ERROR 或 Traceback。测试完成后停止本次启动的进程。
合成输入夹具的 source 日志末尾保留了主动 SIGINT 时的 KeyboardInterrupt 和重复 shutdown 异常，均发生在频率采样结束后的停止阶段，未影响上述运行断言；不是装配控制节点的运行故障。

100 Hz 表示控制周期 10 ms，不代表 100 个不同 Quest 输入或每秒完成 100 个新 IK 解。真实 Quest 当前约 72 Hz。
真机 Nero 的原 0.8 EMA 系数未改，在 100 Hz 下低频平滑延迟约 40 ms；本次未测 CAN/电机端到端响应。

## 原始记录

- nexus_100hz_build.log / nexus_100hz_tests.log：构建与 45 项回归。
- nexus_100hz_runtime.log：Web 操作、参数实际值、控制状态、候选/最终命令频率与启动日志。
- *_source.log：合成输入统计。
- nexus_100hz_probe.py：纯订阅频率探针，运行时须使用本次测试同一 ROS 域，从工作空间根目录传入 profile 路径。
