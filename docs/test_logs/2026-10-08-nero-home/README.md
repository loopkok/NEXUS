# Nero 归位与 XHand QoS 修复验证 — 2026-10-08

## 现场信息与诊断

- 主机：`loopkok@192.168.0.231`，ROS 2 Humble，工作空间 `/home/loopkok/NEXUS`。
- 修改前版本：`7ac71641d7a1f0cef5b2f2dd28f5cdc5661f11f7`。
- 用户原始日志见 [user_home_failure.log](user_home_failure.log)，归位错误为 `home failed: right_arm: home timed ou`；用户补充“左右目测都到初始位置了，但是右臂到了位置后又往下动了一下”。
- 确认代码存在 5 秒管理器服务预算与 20 秒驱动归位预算不一致、同步归位阻塞反馈发布、失败后提前返回 IDLE 的问题。旧保持目标重新下发是现场回动的一种可能原因，尚不能据此确认硬件现象的唯一原因。
- 日志另外包含 XHand 状态订阅可靠性不兼容，以及两手 SDK `1501030: Control parameters exceed the allowable range`。已修复 QoS 与未初始化的命令包保留字段；后者与硬件报错的因果关系尚未真机确认。

## 安全隔离

测试目录 `/tmp/nexus_home_fix_20261008` 独立于主机正在运行的安装目录。归位测试在构造驱动前替换 `pyAgxArm` 工厂，返回 FakeArm，不打开 CAN。使用真实驱动、管理器与仲裁 ROS 节点，ROS 域 186，实例名带测试进程 PID。XHand 桥接使用域 185，重定向 QoS 使用域 187；原生手话题另行重映射。C++ 检查不启动串口节点。

本次没有对真机发送使能、归位、运动或急停指令，没有停止或重启用户的真机进程。

## 结果

| 检查 | 结果 | 记录 |
| --- | --- | --- |
| Nero 归位真实 ROS + FakeArm | 4 / 4 通过 | [home_ros.log](home_ros.log) |
| XHand 桥接与网页在线判定回归 | 5 / 5 通过 | [xhand_bridge_ros.log](xhand_bridge_ros.log) |
| XHand 重定向双侧 best effort 实际收包 | 1 / 1 通过 | [xhand_qos_ros.log](xhand_qos_ros.log) |
| C++ 反馈过滤和命令包初始化 | 2 / 2 通过 | [cpp_tests.log](cpp_tests.log) |
| Profile、数据布局、仲裁和 Astral/Nero 100 Hz 启动配置 | 19 / 19 通过 | [contract.log](contract.log) |
| 修复后 XHand 原生包独立编译 | 通过 | [isolated_build.log](isolated_build.log) |

归位场景覆盖：右臂用 8 秒完成而左臂先到位；全程仍发布反馈；先到位的一侧不触发普通命令超时；完成后拦截旧保持命令；不可达目标超时后不返回 IDLE；归位途中急停；SDK 陈旧缓存不能被误判为到位。并行测试时两臂最大反馈间隔约 301 ms，低于保留的 500 ms 时效门槛；这属于隔离测试调度数据，不代表真机反馈延迟或遥操性能。

XHand 合成反馈负载测试收到 283 帧反馈、208 个候选命令，最大反馈间隔 11.5 ms。未执行真实手运动或复现硬件参数错误。

## 复现命令

以下命令在主机更新代码并编译后运行，只执行上述隔离测试：

```bash
cd /home/loopkok/NEXUS
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m unittest discover -s src/nexus_core/test -p test_nero_home_ros.py -v
python3 -m unittest discover -s src/nexus_core/test -p test_xhand_bridge_ros.py -v
python3 -m unittest discover -s src/xhand_retargeting/test -p test_feedback_qos_ros.py -v
python3 -m unittest discover -s src/nexus_core/test -p test_contract.py -v
ctest --test-dir build/xhand_control_ros2 -R '^xhand_(feedback_validation|command_packet)$' --output-on-failure
```

归位测试源码：[test_nero_home_ros.py](../../../src/nexus_core/test/test_nero_home_ros.py)。QoS 测试源码：[test_feedback_qos_ros.py](../../../src/xhand_retargeting/test/test_feedback_qos_ros.py)。

## 用户复测

停止网页中旧的机器人运行会话，重新启动 Nero 真机配置，再按准备流程检查、使能、归位。终端应分别出现左右臂 `Nero home reached`，网页归位成功后才进入下一步。失败时保留故障状态与日志；不要在旧进程上反复点击归位。

实际真机到位、归位后的保持效果及手部 SDK 参数错误是否消失，均为本报告尚未验证的项目。
