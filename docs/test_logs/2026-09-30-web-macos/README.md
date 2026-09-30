# macOS 风格 Web 验证

## 环境

- Windows 本地 TypeScript/Vite 生产构建。
- 主机独立预览端口 8092，ROS_DOMAIN_ID=175、ROS_LOCALHOST_ONLY=1。
- 预览目录 `/tmp/nexus_web_optimization`，数据目录 `/tmp/nexus_macos_preview_data`。
- Nero profile 使用 dry_run=true，with_inputs=false、with_cameras=false、with_recording=false、with_policy=false。
- 不打开 CAN、XHand 串口、Quest 接收器或相机；不操作真实硬件。

## 检查项目

- 编译通过；系统页加载，无布局或类型错误。
- 待机时显示 0/4 组件、等待反馈，没有虚构频率。
- 假驱动运行后显示 4/4 组件；双臂约 20Hz、双手约 50Hz，与后台诊断一致。
- 组件历史来自 `feedback/<component>`；切换 profile 后清空历史。
- 运行配置默认折叠；驱动控制和停止入口保留。
- 浏览器点击检查反馈，异步结果为 `检查 · 完成 / ready complete`。
- 浏览器检查系统、数采和训练页并保存截图；推理页检查模型参数、控制权和未启用提示。
- 390px 窄屏数采页没有横向溢出；截图为 `capture-mobile.png`。
- 各截图中的在线状态来自假驱动，不能作为真实 Nero 通信或运动验收结果。
- 浏览器错误日志为空，见 `browser-errors.json`。截图记录各轮检查；最终版还隐藏了未启用数采和推理时的无效操作，以及停止后的运行时长。

截图与进一步页面验证记录在本目录。
