# Nero IK 与真机流程审计 LOG

日期：2026-09-30。修复代码版本：`3eabb90d1dda66b962f22e8b03a8f024c4cc5c26`。
测试平台：部署主机，Ubuntu 22.04 / ROS 2 Humble。只运行离线求解、MuJoCo、合成输入和虚拟反馈。
Web 完整链在 `b41a634` 完成；随后 `3eabb90` 增加退出及异常防护，重跑 41 项回归与真机 profile 虚拟反馈测试。
没有打开 CAN、串口、真实相机或控制真机。本次没有接入用户真实 Quest，因此不能把某一次真实输入无解归为已确认的具体原因。

## 对比与问题定位

两份 profile 的 Nero IK、DH 参数、关节限位、归位关节、Quest 每侧坐标转换、link7 法兰目标与零 TCP 偏移一致；共用 `nexus_nero_teleop`。因此旧版 IK 阻塞和工作空间问题同样影响真机流程。

1. **错误的工作空间假设**：将机械臂的 0.58 m 肩部到腕部最大距离用作基座原点到法兰的球形半径。肩部在基座 z=0.138 m，法兰距腕部还有 0.0235 m，不能用该球代替实际可达性。
   192 个限位内随机关节的 FK 已知可达目标，原 IK 全部求解成功，最大 FK 矩阵残差约 2.7e-13；其中 129 个法兰位置超出旧球。旧裁剪会改写这些合法目标。
2. **真实距离或限位约束**：随机测试每侧 80 个组合目标（各轴位置 ±0.25 m、姿态旋转向量各轴 ±0.9 rad）。旧 IK 左/右分别失败 32/37；其中 21/29 超出肩腕最大距离。
   用精确三角约束进一步分类，另有每侧 5 个受肘关节限位限制；剩余左 6、右 3 为 `limits_or_search`，仅表示限位/搜索未找到解，不能据此断言绝对无解。
3. **全局搜索阻塞**：原单线程 ROS 定时回调会在局部搜索失败时扫描全圈，即使目标不能满足限位也会耗时约 100 ms，导致输入、反馈与候选命令发布受阻。
4. **归位接近部分关节边界**：左右臂 J5 距最近限位为 0.075621 rad（约 4.33°）。腕姿态变化可能需要其他冗余分支；没有擅自改变物理限位或归位姿态。
5. **真机附加平滑**：真机保持原 `pos_smoothing=rot_smoothing=0.8`，50 Hz 下低频 EMA 群延迟约 80 ms；仿真为 0。真实 CAN、固件、电机伺服还会增加响应延迟，本次不能给出其端到端测量值。

归位附近测试：每侧 64 个已知可达 FK 目标及各 48 个独立位置/姿态偏移全部成功。没有证据表明原 IK 核心在这些正常可达目标上系统性错误。

## 修复

- 保留原 `nero_quest_teleop/ik_solver.py` 文件，未修改原算法核心。
- 新增共用交互适配器，使用 profile 限位，按当前肘部几何恢复冗余角，避免重锚首次强制全局扫描。
- 未显式配置 workspace_radius 时取消错误球形裁剪，改为肩腕几何、肘限位检查；显式站点工作空间约束仍可保留。
- 局部解析失败后进行有 8 ms 预算、最多 16 次迭代的局部数值细化；只接受位置误差 ≤0.5 mm、姿态误差 ≤0.002 rad、限位内的解。
- 数值局部细化不能代替所有全局分支，因此保留原全局搜索，运行在独立 worker。最多一个在途和一个可覆盖的待解目标，不积压历史输入。
- ROS 控制回调持续接收输入和反馈并发布保持；全局搜索本身仍可能 100–150 ms，不能解读为算法耗时已完全消除。
- 超过目标偏差阈值（2 cm / 0.1 rad）、完成后超过 0.1 秒的结果，以及重锚前的结果不能用作新动作；实际断流仍停用遥操。worker 异常停用遥操。
- 无解日志输出 reason / wrist_distance_m / target_position_m / method / solve_ms。
- 真机 CAN 驱动命令订阅使用 best-effort depth 1，拒绝时间戳超过 command_timeout 的命令；测试时 CAN 被禁用，仅用 SDK mock 验证入口。
- 修复该节点在 ROS SIGINT 后重复 shutdown 的退出异常。

两份 profile 未改内容和哈希，Quest 转换与 TCP 未改；真机平滑默认保持原值。

## 测试结果

```bash
colcon build --packages-select nexus_core nero_mujoco_sim --symlink-install
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=124 ROS_LOCALHOST_ONLY=1 \
  python3 -m pytest src/nero_mujoco_sim/test src/nexus_core/test -q
```

41 passed。包含已知可达目标、旧球外目标、全位姿与限位、几何快速拒绝、队列覆盖、重锚隔离、worker 异常、断流、过期 CAN 命令以及原 MuJoCo/接口回归。

### 离线旧/新对比

256 个归位附近组合 FK 目标，新适配器全部成功。左侧 127 局部解析＋1 数值细化；右侧 117 局部解析＋6 数值细化＋5 全局回退。p95 约 5.2 / 9.8 ms；少数全局回退约 112 ms。
160 个大幅组合目标中，新适配器与旧 IK 成功集合一致，没有因限制局部计算预算而丢失可达分支。全局回退由 worker 执行，离线总 solve 耗时仍可约 110 ms。

### Web 完整仿真链

Web 启动带窗口装配，ready → enable → home → start。合成输入每侧 72 Hz，0.008 m 小幅移动 → 0.8 m 大幅移动 → 小幅恢复。
30 秒控制采样均 TELEOP，无解期间保持，恢复后两臂位置继续变化（末 4 秒关节跨度约 0.046 rad）；四组件反馈维度 7/7/12/12 且新鲜。
无解告警明确出现 reach_outer 和 limits_or_search；回到小幅范围后 failed=0、求解约 50 Hz。主动断流后 1.5 秒为 PAUSED。

### 真机 profile 的虚拟反馈测试

只启动两臂 `nexus_nero_teleop`，读取未经修改的 `nero_dual_xhand.json`；发布每侧 72 Hz 腕输入和 20 Hz 虚拟关节反馈，按候选命令更新虚拟关节。
正常 → 无解 → 恢复三阶段。正常与恢复阶段候选约 50 Hz，无解阶段约 48 Hz，且保持位置不动。故障后无需重新武装即可恢复可达运动。
该测试覆盖真实 profile 与共用遥操节点，不包含 CAN 链路、电机/机械响应或真实 Quest 延迟验收。

## 复现与原始记录

从 NEXUS 工作空间根目录、已 source ROS 与 install 环境运行：

```bash
python3 docs/test_logs/2026-09-30-nero-ik-audit/nexus_ik_audit.py
python3 docs/test_logs/2026-09-30-nero-ik-audit/nexus_ik_extended.py
ROS_DOMAIN_ID=125 ROS_LOCALHOST_ONLY=1 \
  python3 docs/test_logs/2026-09-30-nero-ik-audit/nexus_nero_hardware_profile_probe.py
```

当前目录所有 `.log` 均为原始记录；其中硬件 profile 测试没有创建物理驱动。测试夹具只终止自己启动的子进程组。

## 后续真实 Quest 判断方法

重新启动 Web Nero 仿真后查看新告警：

| reason | 含义 |
|---|---|
| reach_outer / reach_inner | 腕部目标超出精确肩腕几何距离范围 |
| elbow_limit | 距离要求的肘角不在 profile 限位内 |
| limits_or_search | 其余关节限位或分支搜索未找到候选，需结合目标进一步分析 |
| solver_exception | 计算异常，遥操停用 |

这些字段能定位用户真实轨迹中的无解；不能靠放宽真实关节限位或忽略姿态来假造成功。
