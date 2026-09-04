"""节点防护回归：domain 单例锁、段号原子占位、launch 逗号解析、低帧率告警。

背景：实测事故——web 杀监控后残留 /data_collect，再点启动变双开；
/data_collect/control 谁订阅谁开录，一批数据录成字节级重复的两份
（12 文件 = 6 段 ×2）；段号 scan-then-create 非原子，一个 start 在两个
节点分别写出 000000/000001；参考相机 3.3fps 无人察觉直到离线复盘。
"""
from __future__ import annotations

import os
import time

import pytest
import rclpy
from rclpy.parameter import Parameter

from astral_data_collect.data_collect_node import (
    STATE_IDLE,
    STATE_RECORDING,
    DataCollectNode,
    _low_fps_warning,
)
from astral_data_collect.schema import CollectSchema


@pytest.fixture()
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make(tmp_path, session="s", extra=()):
    return DataCollectNode(
        parameter_overrides=[
            Parameter("save_root", value=str(tmp_path)),
            Parameter("session", value=session),
            Parameter("cameras", value=["cam_a"]),
            *extra,
        ]
    )


def test_singleton_lock_rejects_second_node(tmp_path, ros_context):
    """控制面是全局单例：第二个节点（哪怕 session 不同）必须构造即拒绝。"""
    node_a = _make(tmp_path)
    try:
        with pytest.raises(RuntimeError, match="另一个 data_collect"):
            _make(tmp_path)
        with pytest.raises(RuntimeError, match="另一个 data_collect"):
            _make(tmp_path, session="other")
    finally:
        node_a.destroy_node()


def test_singleton_lock_released_on_destroy(tmp_path, ros_context):
    """节点销毁后锁释放，可再起（web 卡片"重启节点"路径）。"""
    node_a = _make(tmp_path)
    node_a.destroy_node()
    node_b = _make(tmp_path)  # 不抛异常即通过
    node_b.destroy_node()


def test_episode_index_atomic_claim(tmp_path, ros_context):
    """段号被占（残留目录/并发节点）时原子让位到下一号，不覆盖已有段。"""
    node = _make(tmp_path)
    try:
        session_dir = tmp_path / "s"
        (session_dir / "episode000000").mkdir(parents=True)  # 已被占
        (session_dir / "episode000002").mkdir()  # 空洞不占低位
        node._apply("start")
        assert node._episode_index == 3
        assert os.path.isdir(session_dir / "episode000003")
        # 被占目录里不能有新文件（未被覆盖写入）
        assert list((session_dir / "episode000000").iterdir()) == []
        node._apply("discard")  # 清理 000003
        assert not (session_dir / "episode000003").exists()
        assert (session_dir / "episode000000").exists()  # 占位目录不受影响
    finally:
        node.destroy_node()


def test_cameras_comma_string_parsed(tmp_path, ros_context):
    """launch 传入逗号字符串时必须拆成列表（曾被逐字符拆解）。"""
    node = DataCollectNode(
        parameter_overrides=[
            Parameter("save_root", value=str(tmp_path)),
            Parameter("session", value="s"),
            Parameter("cameras", value="video8,video0, video2"),
        ]
    )
    try:
        assert node._schema.cameras == ["video8", "video0", "video2"]
    finally:
        node.destroy_node()


def test_jpeg_quality_override_honored(tmp_path, ros_context):
    node = _make(tmp_path, extra=[Parameter("jpeg_quality", value=80)])
    try:
        assert node._schema.jpeg_quality == 80
    finally:
        node.destroy_node()


def test_low_fps_warning_logic():
    schema = CollectSchema(cameras=["d435i", "wrist_left"], dataset_fps=30)
    # 录制中参考相机 3.2fps << 15fps 地板 → 告警
    warn = _low_fps_warning(STATE_RECORDING, {"cam:d435i": 3.2}, schema)
    assert warn and "d435i" in warn and "3fps" in warn
    # 达标 → 无告警
    assert _low_fps_warning(STATE_RECORDING, {"cam:d435i": 29.0}, schema) is None
    # 非录制状态不告警
    assert _low_fps_warning(STATE_IDLE, {"cam:d435i": 0.0}, schema) is None
    # 相机数据完全没进来（订阅未匹配/相机离线）也告警
    warn = _low_fps_warning(STATE_RECORDING, {}, schema)
    assert warn and "0fps" in warn


def test_cmd_stream_uses_arrival_time(tmp_path, ros_context):
    """*_cmd 流必须用到达时刻：teleop 把 joint_commands 的 stamp 打成上游
    VR 输入戳（延迟面板用），IK 比 VR 快时同戳复用——实机 3 段数据 ~45%
    cmd 样本戳完全重复，validate F4 全灭。指令样本时刻语义 = 发出时刻。"""
    from types import SimpleNamespace

    node = _make(tmp_path)
    try:
        node._state = STATE_RECORDING
        cb = node._mk_joint_cb("left_arm_cmd", 7)
        dup_stamp = SimpleNamespace(sec=1788233881, nanosec=456269000)
        for _ in range(3):
            cb(SimpleNamespace(
                header=SimpleNamespace(stamp=dup_stamp), position=[0.0] * 7
            ))
            time.sleep(0.002)
        ts = [t for t, _ in node._stream_buf["left_arm_cmd"]]
        assert len(ts) == 3
        assert all(b > a for a, b in zip(ts, ts[1:]))  # 严格递增
        assert all(t > 1788233882.0 for t in ts)  # 是当前墙钟，非上游旧戳
    finally:
        node._state = STATE_IDLE
        node.destroy_node()


def test_state_stream_keeps_header_stamp(tmp_path, ros_context):
    """state 流不受影响：header.stamp 有效时仍用戳（驱动侧时钟更准）。"""
    from types import SimpleNamespace

    node = _make(tmp_path)
    try:
        node._state = STATE_RECORDING
        cb = node._mk_joint_cb("left_arm_state", 7)
        cb(SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1788233881, nanosec=500000000)),
            position=[0.0] * 7,
        ))
        ts = node._stream_buf["left_arm_state"][0][0]
        assert abs(ts - 1788233881.5) < 1e-6
    finally:
        node._state = STATE_IDLE
        node.destroy_node()


def test_low_fps_warning_published_while_recording(tmp_path, ros_context):
    """端到端：录制中参考相机无帧 → state JSON 带 low_fps_warning。"""
    from std_msgs.msg import String as _String

    node = _make(tmp_path)
    received = []
    probe = rclpy.create_node("probe")
    probe.create_subscription(
        _String, "/data_collect/state", lambda m: received.append(m.data), 10
    )
    try:
        node._apply("start")
        deadline = time.time() + 5.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(probe, timeout_sec=0.05)
            if received and "low_fps_warning" in received[-1] and "cam_a" in received[-1]:
                # JSON 值非 null 才算告警出现
                import json as _json

                if _json.loads(received[-1])["low_fps_warning"]:
                    break
        else:
            pytest.fail("录制 5s 内未出现 low_fps_warning")
    finally:
        node._apply("discard")
        probe.destroy_node()
        node.destroy_node()


def test_quiet_start_warning_when_completely_empty(tmp_path, ros_context):
    """源未就绪的空录：录制 >2s 数值/图像全 0 → state JSON 带 empty_warning；
    丢段开新段后告警复位（段级生命周期）。"""
    import json as _json
    from std_msgs.msg import String as _String

    node = _make(tmp_path)
    received = []
    probe = rclpy.create_node("probe")
    probe.create_subscription(
        _String, "/data_collect/state", lambda m: received.append(m.data), 10
    )
    try:
        node._apply("start")
        node._episode_start_wall = time.time() - 3.0  # 回拨墙钟，跳过 2s 等待
        node._publish_state()
        deadline = time.time() + 3.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(probe, timeout_sec=0.05)
            if received and "未收到任何样本" in received[-1]:
                break
        else:
            pytest.fail("空录 2s 未出现 empty_warning")
        assert _json.loads(received[-1])["empty_warning"]
        # 段生命周期：丢弃 → 开新段 → 告警清空
        node._apply("discard")
        node._apply("start")
        assert node._empty_warning is None
    finally:
        probe.destroy_node()
        node.destroy_node()


def test_quiet_start_warning_partial_and_healthy(tmp_path, ros_context):
    """部分空录只告警缺失侧；有数据流入时不误报（空录检查不能打正常段）。"""
    import numpy as np

    node = _make(tmp_path)
    try:
        # ① 只有数值、无图像 → 图像侧告警
        node._apply("start")
        node._robot_writer.write(
            "left_arm_state", np.zeros(7, dtype=np.float32), time.time()
        )
        node._episode_start_wall = time.time() - 3.0
        node._publish_state()
        assert "无图像帧" in (node._empty_warning or "")
        node._apply("discard")

        # ② 数值+图像都流入 → 不误报
        node._apply("start")
        node._robot_writer.write(
            "left_arm_state", np.zeros(7, dtype=np.float32), time.time()
        )
        node._camera_writer.write_image("cam_a", b"\xff\xd8\xff\xe0", time.time())
        node._episode_start_wall = time.time() - 3.0
        node._publish_state()
        assert node._empty_warning is None
        assert node._quiet_start_checked  # 检查确实执行过（非因未到 2s 跳过）
    finally:
        node.destroy_node()


def test_ignored_commands_counted_and_published(tmp_path, ros_context):
    """状态不合法被吞的指令（SAVING/状态机拒绝）要计数并进 state JSON——
    键盘/VR 无按钮 disabled 视觉，被吞的 start 必须留痕。"""
    import json as _json
    from std_msgs.msg import String as _String

    node = _make(tmp_path)
    received = []
    probe = rclpy.create_node("probe")
    probe.create_subscription(
        _String, "/data_collect/state", lambda m: received.append(m.data), 10
    )
    try:
        node._apply("start")        # RECORDING
        node._apply("start")        # RECORDING 中再 start → 忽略
        node._apply("pause")        # 合法 → PAUSED
        node._apply("resume")       # 合法 → RECORDING
        node._apply("pause")
        node._apply("pause")        # PAUSED 中 pause → 忽略
        node._apply("stop")         # 合法 → IDLE
        node._apply("stop")         # IDLE 中 stop → 忽略
        assert node._ignored_cmds == {"start": 1, "pause": 1, "stop": 1}
        node._publish_state()
        deadline = time.time() + 3.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(probe, timeout_sec=0.05)
            if received:
                last = _json.loads(received[-1])
                if last.get("ignored", {}).get("start") == 1:
                    break
        else:
            pytest.fail("state JSON 未携带 ignored 计数")
        last = _json.loads(received[-1])
        assert last["ignored"] == {"start": 1, "pause": 1, "stop": 1}
    finally:
        probe.destroy_node()
        node.destroy_node()
