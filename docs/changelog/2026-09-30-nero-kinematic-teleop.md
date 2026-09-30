# 2026-09-30 Nero 遥操仿真直接更新关节

- 默认 simulation_mode=kinematic，直接设置四组件 qpos 并 mj_forward；保留 physics 可选项。
- 适配器装配和独立 launch 均可选择模式，Web 的现有 Nero 仿真入口默认使用直接关节模式。
- 仿真腕部默认取消 EMA 延迟，实机默认值不变；腕与反馈订阅改用 best-effort depth 1。
- IK 无解期间发布新鲜反馈的保持候选，回到可达范围自动跟随；真实断流仍触发暂停。
- 增加直接关节/物理模式差异、超时保持和 IK 恢复的回归测试。
- 主机构建成功，33 项测试通过，窗口 500 Hz、实时因子 1.00；命令/反馈相位 11–12 ms。
- 原 IK 核心、TCP、坐标映射及 profile 内容/哈希均未变。

详见 [测试 LOG](../test_logs/2026-09-30-nero-kinematic-teleop/README.md)。
