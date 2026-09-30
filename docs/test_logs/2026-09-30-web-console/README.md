# Web 控制台回归测试

## 环境与范围

- 本地：Windows，TypeScript/Vite 生产构建。
- 部署测试：192.168.0.231，ROS 2 Humble，隔离测试目录 `/tmp/nexus_web_optimization`。
- 专用 Web 端口 8091，ROS_DOMAIN_ID=174，数据根目录 `/tmp/nexus_web_optimization_data`。
- 原 Web 8080 服务保持运行；不启动真机设备驱动。

## 测试命令

```bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
python3 -m pytest src/astral_web_monitor/test src/nero_mujoco_sim/test/test_model_builder.py -q
python3 scripts/nexus_web_console_acceptance.py \
  --url http://127.0.0.1:8091 \
  --include-simulation \
  --output /tmp/nexus_web_optimization_results
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD` 避免主机系统 pytest 与用户目录 anyio 插件版本不一致；不修改系统 Python 环境。

## 已验证项目

- 前端生产构建通过。
- 浏览器实际加载系统页，导航包含六个功能页面，没有独立 NEXUS 标签。
- 页面显示 profile、组件类型、自由度、反馈状态、统一驱动控制及模型配置。
- 自动化接口测试覆盖异步归位、重复操作、急停入口、暂停与恢复重锚、录制退出保护、旧硬件 API 隔离和报告记录。

完整运行结果和日志在本目录归档。

## 验证结果

- 单元测试：36 passed，见 `nexus_web_optimization_test.log`。
- Astral 双臂双夹爪假驱动：PASS。
- Nero 双臂双 XHand 假驱动：PASS。
- Nero 双臂双 XHand MuJoCo：PASS。
- 三套测试均覆盖反馈、使能、归位、重锚、暂停恢复、合成相机与录制保存、录制中退出保护、唯一命令发布者、急停、报告下载和退出。
- TypeScript 与 Vite 生产构建通过。
- 浏览器确认机器人卡片、模式按钮、功能开关及折叠高级参数；Nero 仿真配置显示可视化开关。
- 检查系统、数采、数据与训练、推理和诊断页面；768px 窄屏下配置与控制纵向排列，没有横向溢出。
- 主机编译 nexus_core、nero_mujoco_sim、astral_web_monitor 三个包成功。

运行事件和每套机器人报告已归档。本次使用合成输入，未启动真机驱动，未执行 GPU 实际训练。
