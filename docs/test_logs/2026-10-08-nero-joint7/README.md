# Nero 第二次归位失败：第 7 关节残差诊断 — 2026-10-08

## 已确认的信息

用户在 `f3cdffe` 修复版重新启动后，归位等待完整 20 秒才失败。该现场记录与此前管理器 5 秒提前超时不同。本次前六关节已经很接近配置目标，第七关节反馈未进入原有 0.05 rad 到位范围。

| 侧别 | J7 目标 rad | J7 实测 rad | J7 误差 | J1–J6 最大误差 |
| --- | --- | --- | --- | --- |
| 左 | -0.163 | -0.0836710843 | 0.0793289157 rad / 4.5452° | 0.0001879186 rad |
| 右 | -0.163 | -0.0902858822 | 0.0727141178 rad / 4.1662° | 0.0000708777 rad |

原始输入见 [user_home_failure.log](user_home_failure.log)，计算结果见 [joint_error_analysis.json](joint_error_analysis.json)。目测整体到位与末端腕部仍有几度偏差可以同时存在；尚不能据此认定关节卡住、标定错误或控制器存在特定故障。

## 只读真机检查

- 用 `candump` 分别被动记录两臂的 `0x2A9` 第七关节位置、`0x257` 电机信息、`0x267` 驱动信息及 `0x2A1` 控制器状态，未发送运动或配置命令。见 [left_can.log](left_can.log)、[right_can.log](right_can.log)。这些记录在归位超时并停止电机之后取得，不能将此时的失能/抱闸状态当作之前失败的原因。
- 发送仅用于读取固件信息的 `0x4AF` 请求，两臂均返回 1.10。记录见 [firmware.json](firmware.json)。目前使用 `NeroFW.DEFAULT` 与厂商的 [固件选择表](https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/nero/firmware_reference.md) 一致。
- 比较原 `/home/loopkok/xnero_ws` 与 NEXUS 的 SDK，第七关节同样用 `0x170`、有符号毫度编码。未发现漏掉第七关节或弧度单位错误。
- 原 xnero 遥操节点等待初始位姿最多 15 秒，但退出等待后无条件打印 `Initial pose reached`，因此旧成功提示不能证明七关节全部通过到位判定。
- 没有触发真机使能、归位、遥操、急停、固件更新或标定，也未停止用户的机器人进程。只读查询没有更改机器人状态。

## 本次代码改进

1. 归位过程中每 2 秒记录实测/目标、逐关节误差（rad/deg）、未到位关节、控制器状态与七个驱动器的使能/错误/碰撞/堵转标志。SDK 状态仅从现有接收缓存读取。
2. 失败时先保存并打印停机前的完整诊断，再执行原有停止流程；后续可读取 `last_home_failure_before_stop`，区分故障前后状态。
3. 新增只读服务 `/nexus/<instance>/drivers/<component>/diagnostics`。不调用 `connect`、固件请求、使能、运动或标定接口。
4. 保留原目标、速度、20 秒预算和 0.05 rad 到位范围。没有将第七关节误差直接忽略，没有自动重发运动目标或切换到 JS 追赶目标。

## 隔离验证

`test_nero_*.py` 共 11 项通过，见 [isolated_tests.log](isolated_tests.log)：

- 6 项真实 ROS + FakeArm 归位/诊断检查：已有 4 项，以及七轴残差复现、诊断服务不发送硬件命令。
- 1 项实际 SDK 发包测试：使用内存传输，覆盖左右目标和 DEFAULT/V111 编码，验证 `move_j` 向传输层提交全部四帧，含 `0x170` 的正确负角度，未打开物理 CAN。
- 4 项已有 Nero IK 适配器回归。

七轴残差复现模拟前六轴到位、J7 仍差 0.0793 rad，即使合成控制器声称运动完成也必须失败，并保留停止前的使能状态。诊断读取不得改写硬件命令计数。

## 下一步现场采集

此更新增加定位证据，**尚未解决或验证真机第七轴的残差原因**。需要用户重新启动机器人会话加载诊断代码，在现场操作准备流程后提供 `Nero home progress` 和 `Nero home diagnostic before stop` 日志。随后才能判断是否是第七轴驱动异常、抱闸、指令执行或零点/传动标定问题。

运行会话中可随时执行以下只读命令（包括归位途中），不需要额外打开 CAN 设备：

```bash
cd /home/loopkok/NEXUS
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1
ros2 service call /nexus/nero/drivers/left_arm/diagnostics std_srvs/srv/Trigger '{}'
ros2 service call /nexus/nero/drivers/right_arm/diagnostics std_srvs/srv/Trigger '{}'
```

日志中的 XHand SDK `1501030` 在保留字段清零后仍出现，应作为另一个未解决的手部命令参数问题继续处理；本次未更改手部增益、模式或电流限制。
