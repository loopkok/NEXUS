# Nero 归位阈值配置化验证 · 2026-10-08

## 版本与边界

- 运行代码：`4824c61dace5b4761e15b170bb21b880c166785c`。
- 部署主机：`192.168.0.231`，`/home/loopkok/NEXUS`，ROS 2 Humble / Python 3.10。
- 只进行构建、静态检查与隔离 ROS 测试。ROS domain 186，临时实例名；创建驱动前替换 SDK factory，未打开 CAN，未向真机发送任何命令。
- 未重启主机现有 Web 或机器人进程，现有会话不热更新配置。

## 结果

| 检查 | 结果 |
| --- | --- |
| 本地 Python 3.10 配置测试 | 4 项通过 |
| 主机 nexus_core symlink 构建 | 通过，2.01 秒 |
| 主机配置测试 | 4 项通过，0.007 秒 |
| 主机接口回归 | 19 项通过，0.509 秒 |
| 主机真实 ROS 服务＋假 SDK 归位回归 | 15 项通过，27.385 秒 |

配置测试覆盖旧配置缺省、统一标量、七轴数组、左右侧独立配置、缺侧、非法长度、
非正数、字符串、布尔值、NaN/Inf、错误侧名。启动构建驱动动作时还会校验所用侧存在。

新增 ROS 用例从临时 profile 加载左臂前六轴 0.05、J7 0.10，右臂全部 0.03 rad：

1. 左 J7 固定残差 0.0793 rad，通过 0.10 rad 的到位和稳定判定；逐轴诊断记录阈值，未到位列表为空。
2. 同时固定左 J1、J7 残差 0.0793 rad，只把 J1 列为未到位并报告超时，确认放宽 J7 不会放宽其他关节。
3. 原有缺省 0.05 rad 的 J7 残差拒绝测试继续通过。

其余用例继续覆盖长归位的反馈连续性、旧命令抑制、急停、失联、模式等待及超时、
旧模式缓存、控制器故障、部分发送不重试、只读诊断和异常 SDK 缓存。
长归位测试双臂均约 20 Hz 反馈，最大接收间隔 52.3 / 52.0 ms，最大 SDK 轮询间隔 51.1 / 51.7 ms。

## 保留的警告与限制

- 构建提示已 source 当前工作空间后覆盖同一 `nexus_core` underlay；构建成功，没有代码编译失败。
- ROS 测试退出清理仍打印 6 条 `Destroyable` 警告，`RCLError` 为 0；这是已记录的清理问题，本次没有修改退出逻辑，不能称为零警告通过。
- 日志中其他归位 ERROR 是故障注入测试的预期输出。全部 15 项功能断言通过。
- 假 SDK 结果验证软件判定与生命周期，不证明实机在任何负载下的归位精度。

## 配置生效检查

内置真机 profile 的默认值保持 `home_tolerance_rad: 0.05`，两臂七轴均为 0.05 rad。
安装目录的 profile 已确认链接到源文件：

```text
/home/loopkok/NEXUS/install/nexus_core/share/nexus_core/profiles/nero_dual_xhand.json
→ /home/loopkok/NEXUS/src/nexus_core/profiles/nero_dual_xhand.json
```

配置哈希及实际加载路径见 `result.json`。使用自定义 `NEXUS_PROFILES_DIR` 时，应修改实际选中的文件。

## 重现命令

```bash
cd /home/loopkok/NEXUS
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select nexus_core --event-handlers console_direct+
export ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1
python3 -m unittest discover -s src/nexus_core/test -p test_nero_home_config.py -v
python3 -m unittest discover -s src/nexus_core/test -p test_contract.py -v
python3 src/nexus_core/test/test_nero_home_ros.py -v
```

完整输出保留在本目录的 `build.log`、`config_tests.log`、`contract_tests.log`、`home_ros.log`。
