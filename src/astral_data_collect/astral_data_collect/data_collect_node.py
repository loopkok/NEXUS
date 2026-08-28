"""astral_data_collect 采集节点：与遥操作并行录制 raw HDF5。

职责边界：只负责"忠实落盘"——所有订阅流原样写入 episode 目录
（robot_data.h5 / camera_data.h5 / meta.json），不做在线对齐。

控制面（键盘与话题完全等价，见 keyboard_controller）：
  订阅 /data_collect/control (std_msgs/String, RELIABLE)：
      start | stop | discard | next | pause | resume
  订阅 /data_collect/task (std_msgs/String, latched)：
      下一段 episode 的任务文本
  发布 /data_collect/state (std_msgs/String, latched JSON)：
      状态/段号/时长/各流计数与频率/task，1Hz + 状态跃迁时

线程模型：rclpy 回调只入队（deque + 各自原子 append）；独立写盘线程
每 20ms 批量 drain 到 HDF5。丢弃段 = 关闭后删除目录，不落盘即无垃圾。
"""

from __future__ import annotations

import json
import os
import shutil
import threading
import time
from collections import deque
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String

from astral_data_collect import __version__
from astral_data_collect.data_writer import (
    CAMERA_H5,
    META_JSON,
    ROBOT_H5,
    CameraDataWriter,
    StreamDataWriter,
)
from astral_data_collect.schema import (
    EE_GRIPPER,
    STREAM_TOPICS,
    CollectSchema,
)

STATE_IDLE = "IDLE"
STATE_RECORDING = "RECORDING"
STATE_PAUSED = "PAUSED"
STATE_SAVING = "SAVING"

_CMDS = ("start", "stop", "discard", "next", "pause", "resume")

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
)
_IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=4,
)
_CONTROL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
_LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# 遥操作事件话题 → 事件名（latched Bool，记录跃迁用于 armed 覆盖率统计）
_TELEOP_EVENTS = {
    "/teleop/armed": "teleop_armed",
    "/teleop/disarm": "teleop_disarmed",
    "/teleop/start": "teleop_start",
}

_STREAM_BUFFER_MAX = 20000   # 单流缓冲上限（1000Hz 手 ≈ 20s）
_IMAGE_BUFFER_MAX = 600      # 单相机缓冲上限（30fps ≈ 20s）


def _stamp_sec(header: Any, fallback: float) -> float:
    """header.stamp 有效（sec>0）用之，否则用 fallback（到达时刻）。"""
    try:
        sec = float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9
        if sec > 0.0:
            return sec
    except AttributeError:
        pass
    return fallback


def _to_bool(v: Any) -> bool:
    """宽松 bool 解析：launch 传入的是字符串 "true"/"false"，bool("false") 恒真。"""
    if isinstance(v, str):
        return v.strip().lower() in ("1", "true", "yes", "on")
    return bool(v)


class DataCollectNode(Node):
    def __init__(self, **node_kwargs: Any) -> None:
        super().__init__(
            "data_collect",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
            **node_kwargs,
        )
        p = lambda name, default: self._param(name, default)

        self._save_root = os.path.expanduser(str(p("save_root", "~/astral_data")))
        self._session = str(p("session", "default_task"))
        self._camera_prefix = str(p("camera_topic_prefix", "/quest3_video_streamer/collect"))

        arms_param = p("arms", ["left", "right"])
        if isinstance(arms_param, str):
            # launch 传入逗号字符串（"left,right"），yaml 传入字符串数组
            arms_param = [s.strip() for s in arms_param.split(",") if s.strip()]
        self._schema = CollectSchema(
            arms=[str(s) for s in arms_param],
            end_effector_left=str(p("end_effector_left", "gripper")),
            end_effector_right=str(p("end_effector_right", "gripper")),
            include_waist=_to_bool(p("include_waist", False)),
            include_head=_to_bool(p("include_head", False)),
            cameras=[str(c) for c in p("cameras", ["wrist_left", "wrist_right"])],
            dataset_fps=int(p("dataset_fps", 30)),
            action_source=str(p("action_source", "next_state")),
            hold_frames=int(p("hold_frames", 10)),
            max_gap_ms=float(p("max_gap_ms", 100.0)),
        )
        self._streams = self._schema.required_streams()

        self._state = STATE_IDLE
        self._state_lock = threading.Lock()
        self._episode_dir: str | None = None
        self._episode_index = -1
        self._episode_start_wall = 0.0
        self._pending_task = str(p("default_task", ""))
        self._events: list[dict[str, Any]] = []
        self._robot_writer: StreamDataWriter | None = None
        self._camera_writer: CameraDataWriter | None = None
        self._stream_counts: dict[str, int] = {}
        self._cam_counts: dict[str, int] = {}
        self._drop_counts: dict[str, int] = {}

        # 采集缓冲：回调 append，写盘线程 drain
        self._buf_lock = threading.Lock()
        self._stream_buf: dict[str, deque] = {
            name: deque(maxlen=_STREAM_BUFFER_MAX) for name in self._streams
        }
        self._cam_buf: dict[str, deque] = {
            cam: deque(maxlen=_IMAGE_BUFFER_MAX) for cam in self._schema.cameras
        }
        # 遥操作当前状态（latched 话题持续跟踪，段开始时记初始值）
        self._teleop_state: dict[str, bool] = {v: False for v in _TELEOP_EVENTS.values()}

        self._mk_subscriptions()
        self._state_pub = self.create_publisher(String, "/data_collect/state", _LATCHED_QOS)
        self.create_subscription(
            String, "/data_collect/control", self._on_control, _CONTROL_QOS
        )
        self.create_subscription(String, "/data_collect/task", self._on_task, _LATCHED_QOS)

        self._writer_stop = threading.Event()
        self._writer_thread = threading.Thread(
            target=self._writer_loop, name="data-collect-writer", daemon=True
        )
        self._writer_thread.start()
        self._status_timer = self.create_timer(1.0, self._publish_state)

        self.get_logger().info(
            f"data_collect ready: save_root={self._save_root} session={self._session} "
            f"schema={self._schema.to_json()}"
        )
        self._publish_state()

    # -- 参数与订阅 ------------------------------------------------------------

    def _param(self, name: str, default: Any) -> Any:
        try:
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
            val = self.get_parameter(name).value
            return default if val is None else val
        except Exception:
            return default

    def _mk_subscriptions(self) -> None:
        from sensor_msgs.msg import CompressedImage, JointState

        for name, dim in self._streams.items():
            topic = STREAM_TOPICS[name][0]
            if name.endswith("_gripper_ratio"):
                self.create_subscription(
                    Float64, topic,
                    self._mk_float_cb(name), _SENSOR_QOS,
                )
            else:
                self.create_subscription(
                    JointState, topic,
                    self._mk_joint_cb(name, dim), _SENSOR_QOS,
                )
        for cam in self._schema.cameras:
            topic = f"{self._camera_prefix}/{cam}"
            self.create_subscription(
                CompressedImage, topic, self._mk_cam_cb(cam), _IMAGE_QOS
            )
        for topic, event in _TELEOP_EVENTS.items():
            self.create_subscription(
                Bool, topic, self._mk_event_cb(event), _LATCHED_QOS
            )

    def _accepting(self) -> bool:
        """仅 RECORDING 才接收样本：IDLE 期数据不污染下一段，PAUSED 期真正暂停。"""
        return self._state == STATE_RECORDING

    def _mk_joint_cb(self, name: str, dim: int):
        def _cb(msg: Any) -> None:
            if not self._accepting():
                return
            now = time.time()
            pos = list(msg.position)
            if len(pos) > dim:
                pos = pos[:dim]
            elif len(pos) < dim:
                return  # 维度不足的消息视为无效（配置错误由 validate 暴露）
            ts = _stamp_sec(msg.header, now)
            with self._buf_lock:
                dq = self._stream_buf[name]
                if len(dq) == dq.maxlen:
                    self._drop_counts[name] = self._drop_counts.get(name, 0) + 1
                dq.append((ts, np.asarray(pos, dtype=np.float32)))
        return _cb

    def _mk_float_cb(self, name: str):
        def _cb(msg: Any) -> None:
            if not self._accepting():
                return
            ts = time.time()  # Float64 无 header
            with self._buf_lock:
                dq = self._stream_buf[name]
                if len(dq) == dq.maxlen:
                    self._drop_counts[name] = self._drop_counts.get(name, 0) + 1
                dq.append((ts, np.asarray([msg.data], dtype=np.float32)))
        return _cb

    def _mk_cam_cb(self, cam: str):
        def _cb(msg: Any) -> None:
            if not self._accepting():
                return
            now = time.time()
            ts = _stamp_sec(msg.header, now)
            with self._buf_lock:
                dq = self._cam_buf[cam]
                if len(dq) == dq.maxlen:
                    self._drop_counts[f"cam:{cam}"] = self._drop_counts.get(f"cam:{cam}", 0) + 1
                dq.append((ts, bytes(msg.data)))
        return _cb

    def _mk_event_cb(self, event: str):
        def _cb(msg: Any) -> None:
            value = bool(msg.data)
            prev = self._teleop_state.get(event)
            self._teleop_state[event] = value
            if prev is not None and prev == value:
                return  # 非跃迁（如 latched 重发）不记
            with self._state_lock:
                recording = self._state in (STATE_RECORDING, STATE_PAUSED)
            if recording:
                self._events.append(
                    {"t": time.time(), "name": event, "data": value}
                )
        return _cb

    # -- 控制面 ------------------------------------------------------------------

    def _on_task(self, msg: String) -> None:
        self._pending_task = msg.data
        self.get_logger().info(f"task for next episode <- {msg.data!r}")
        self._publish_state()

    def _on_control(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if cmd not in _CMDS:
            self.get_logger().warning(f"unknown control cmd {cmd!r} (expect {_CMDS})")
            return
        self.get_logger().info(f"control <- {cmd}")
        with self._state_lock:
            try:
                self._apply(cmd)
            except Exception as exc:  # 状态机错误不致命
                self.get_logger().error(f"cmd {cmd} failed: {exc}")
        self._publish_state()

    def _apply(self, cmd: str) -> None:
        st = self._state
        if cmd == "start":
            if st != STATE_IDLE:
                self.get_logger().warning(f"start ignored in state {st}")
                return
            self._begin_episode()
        elif cmd == "stop":
            if st not in (STATE_RECORDING, STATE_PAUSED):
                self.get_logger().warning(f"stop ignored in state {st}")
                return
            self._end_episode(save=True)
        elif cmd == "discard":
            if st not in (STATE_RECORDING, STATE_PAUSED):
                self.get_logger().warning(f"discard ignored in state {st}")
                return
            self._end_episode(save=False)
        elif cmd == "next":
            if st not in (STATE_RECORDING, STATE_PAUSED):
                self.get_logger().warning(f"next ignored in state {st}")
                return
            self._end_episode(save=True)
            self._begin_episode()
        elif cmd == "pause":
            if st == STATE_RECORDING:
                self._drain_buffers()  # 暂停前落盘已缓冲数据
                self._state = STATE_PAUSED
                self._events.append({"t": time.time(), "name": "pause", "data": True})
        elif cmd == "resume":
            if st == STATE_PAUSED:
                self._state = STATE_RECORDING
                self._events.append({"t": time.time(), "name": "pause", "data": False})

    # -- episode 生命周期 ---------------------------------------------------------

    @property
    def _session_dir(self) -> str:
        return os.path.join(self._save_root, self._session)

    def _next_episode_index(self) -> int:
        d = self._session_dir
        if not os.path.isdir(d):
            return 0
        best = -1
        for name in os.listdir(d):
            if name.startswith("episode") and name[len("episode"):].isdigit():
                best = max(best, int(name[len("episode"):]))
        return best + 1

    def _begin_episode(self) -> None:
        os.makedirs(self._session_dir, exist_ok=True)
        self._episode_index = self._next_episode_index()
        self._episode_dir = os.path.join(
            self._session_dir, f"episode{self._episode_index:06d}"
        )
        # 防御性清空（回调已被 _accepting 门控，此处防状态竞态残留）；
        # 丢弃计数按段归零（meta.json 的 dropped 是段级 provenance）
        with self._buf_lock:
            for dq in (*self._stream_buf.values(), *self._cam_buf.values()):
                dq.clear()
            self._drop_counts = {}
        self._robot_writer = StreamDataWriter(
            os.path.join(self._episode_dir, ROBOT_H5), self._streams
        ).open()
        self._camera_writer = CameraDataWriter(
            os.path.join(self._episode_dir, CAMERA_H5), self._schema.cameras
        ).open()
        self._events = [
            {"t": time.time(), "name": "episode_start", "data": True},
            # 段开始时的遥操作状态快照（armed 覆盖率统计的起点）
            *(
                {"t": time.time(), "name": k, "data": v}
                for k, v in self._teleop_state.items()
                if v
            ),
        ]
        self._episode_start_wall = time.time()
        self._state = STATE_RECORDING
        self.get_logger().info(
            f"REC start episode{self._episode_index:06d} task={self._pending_task!r} "
            f"-> {self._episode_dir}"
        )

    def _end_episode(self, *, save: bool) -> None:
        self._state = STATE_SAVING
        ep_dir = self._episode_dir
        ep_idx = self._episode_index
        duration = time.time() - self._episode_start_wall
        self._drain_buffers()
        stream_counts = self._robot_writer.close() if self._robot_writer else {}
        cam_counts = self._camera_writer.close() if self._camera_writer else {}
        self._robot_writer = None
        self._camera_writer = None

        if save and ep_dir is not None:
            self._events.append({"t": time.time(), "name": "episode_end", "data": True})
            meta = {
                "package": "astral_data_collect",
                "package_version": __version__,
                "format": "raw_hdf5_v1",
                "task": self._pending_task,
                "schema": self._schema.to_dict(),
                "episode_index": ep_idx,
                "start_time_wall": self._episode_start_wall,
                "end_time_wall": time.time(),
                "duration_s": duration,
                "events": self._events,
                "streams": {
                    name: {"dim": dim, "count": int(stream_counts.get(name, 0))}
                    for name, dim in self._streams.items()
                },
                "cameras": {
                    cam: {"count": int(cam_counts.get(cam, 0))}
                    for cam in self._schema.cameras
                },
                "dropped": dict(self._drop_counts),
            }
            with open(os.path.join(ep_dir, META_JSON), "w", encoding="utf-8") as f:
                json.dump(meta, f, ensure_ascii=False, indent=2)
            self.get_logger().info(
                f"REC saved episode{ep_idx:06d} ({duration:.1f}s, "
                f"streams={sum(stream_counts.values())} samples, "
                f"images={sum(cam_counts.values())}) -> {ep_dir}"
            )
        else:
            if ep_dir and os.path.isdir(ep_dir):
                shutil.rmtree(ep_dir, ignore_errors=True)
            self.get_logger().info(f"REC discarded episode{ep_idx:06d} (deleted {ep_dir})")

        self._episode_dir = None
        self._episode_index = -1
        self._state = STATE_IDLE

    # -- 写盘线程 ------------------------------------------------------------------

    def _writer_loop(self) -> None:
        while not self._writer_stop.is_set():
            with self._state_lock:
                recording = self._state == STATE_RECORDING
                if recording:
                    self._drain_buffers()
            time.sleep(0.02)

    def _drain_buffers(self) -> None:
        """把缓冲写进 HDF5。调用方须持 _state_lock 或处于写盘线程+RECORDING。"""
        if self._robot_writer is None:
            return
        with self._buf_lock:
            stream_items = {
                name: [dq.popleft() for _ in range(len(dq))]
                for name, dq in self._stream_buf.items()
                if dq
            }
            cam_items = {
                cam: [dq.popleft() for _ in range(len(dq))]
                for cam, dq in self._cam_buf.items()
                if dq
            }
        rw = self._robot_writer
        cw = self._camera_writer
        for name, items in stream_items.items():
            for ts, values in items:
                rw.write(name, values, ts)
        if cw is not None:
            for cam, items in cam_items.items():
                for ts, jpeg in items:
                    cw.write_image(cam, jpeg, ts)

    # -- 状态发布 ------------------------------------------------------------------

    def _publish_state(self) -> None:
        with self._state_lock:
            st = self._state
            ep_idx = self._episode_index
            elapsed = (
                time.time() - self._episode_start_wall
                if st in (STATE_RECORDING, STATE_PAUSED)
                else 0.0
            )
            stream_counts = (
                self._robot_writer.counts() if self._robot_writer else {}
            )
            cam_counts = (
                self._camera_writer.counts() if self._camera_writer else {}
            )
        prev = getattr(self, "_prev_counts", {})
        rates = {}
        for name, c in stream_counts.items():
            rates[name] = round(c - prev.get(name, 0), 1)
        for cam, c in cam_counts.items():
            rates[f"cam:{cam}"] = round(c - prev.get(f"cam:{cam}", 0), 1)
        self._prev_counts = {
            **{k: v for k, v in stream_counts.items()},
            **{f"cam:{k}": v for k, v in cam_counts.items()},
        }
        payload = {
            "state": st,
            "session": self._session,
            "episode_index": ep_idx,
            "elapsed_s": round(elapsed, 1),
            "task_next": self._pending_task,
            "samples_per_s": rates,
            "dropped": dict(self._drop_counts),
            "schema": self._schema.to_dict(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._state_pub.publish(msg)

    # -- 关闭 ------------------------------------------------------------------

    def destroy_node(self) -> bool:
        with self._state_lock:
            if self._state in (STATE_RECORDING, STATE_PAUSED):
                self.get_logger().warning("shutdown while recording — discarding episode")
                self._end_episode(save=False)
        self._writer_stop.set()
        self._writer_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DataCollectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
