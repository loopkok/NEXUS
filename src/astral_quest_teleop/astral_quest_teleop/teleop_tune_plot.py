#!/usr/bin/env python3
"""Live teleop tuner: overlay hand vs filter vs commanded EE (PID-style).

Subscribe to ``/teleop/{side}/tune/xyz`` published by
``astral_teleop_arm_node`` (also ``ee_vr`` / ``ee_filt`` / ``ee_cmd`` for
PlotJuggler). Positions are millimetres relative to the zero you set
(first sample, or press ``z`` while holding still).

How to read
-----------
* Hand (VR) and EE overlap on a 1–2 cm move → 跟手.
* EE wiggles while Hand is flat → 控制/IK 抖.
* Hand itself wiggles → Quest 跟踪噪声（降平滑会更跟、更抖）.
* EE smooth but lags Hand → ``pos_smoothing`` 太大.
* Hand amplitude > EE amplitude → ``motion_scale`` 已含在 VR 映射里；
  若仍偏小，是 IK/限速没跟上，不是 scale 没乘.

Keys: ``z`` re-zero, ``s`` save CSV of the visible buffer, ``q`` quit.
Bottom sliders: feel params (both solvers) + URDF LM weights.
Radio: hot-switch ``analytic_dh`` / ``urdf_numerical``.
"""

from __future__ import annotations

import csv
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64MultiArray

_HOT = (
    "pos_smoothing",
    "rot_smoothing",
    "motion_scale",
    "max_joint_vel",
    "ik_w_pos",
    "ik_w_ori",
    "ik_w_reg",
    "ik_max_iter",
    "ik_tol",
    "solver_type",
)
_DEFAULTS = {
    "pos_smoothing": 0.8,
    "rot_smoothing": 0.8,
    "motion_scale": 0.65,
    "max_joint_vel": 4.0,
    "ik_w_pos": 1.0,
    "ik_w_ori": 0.3,
    "ik_w_reg": 1e-4,
    "ik_max_iter": 20.0,
    "ik_tol": 1e-8,
    "solver_type": "analytic_dh",
}
_INT_PARAMS = {"ik_max_iter"}
_STR_PARAMS = {"solver_type"}


def _xcorr_lag_ms(ref: np.ndarray, sig: np.ndarray, dt: float) -> float | None:
    x = np.asarray(ref, dtype=float)
    y = np.asarray(sig, dtype=float)
    if x.size < 40 or dt <= 0:
        return None
    x = x - x.mean()
    y = y - y.mean()
    if x.std() < 1e-4 or y.std() < 1e-4:
        return None
    max_lag = max(1, int(0.25 / dt))
    c = np.correlate(y, x, mode="full")
    lags = np.arange(-len(x) + 1, len(x))
    mid = len(x) - 1
    lo = max(0, mid - max_lag)
    hi = min(len(c), mid + max_lag + 1)
    i = lo + int(np.argmax(c[lo:hi]))
    return float(lags[i] * dt * 1000.0)


class TeleopTunePlot(Node):
    def __init__(self) -> None:
        super().__init__("teleop_tune_plot")
        self.declare_parameter("arm_side", "right")
        self.declare_parameter("window_sec", 8.0)
        self.declare_parameter("metric_sec", 2.0)
        self.declare_parameter("save_dir", "/tmp")
        self.declare_parameter("apply_both_arms", True)
        self.declare_parameter("teleop_left", "astral_teleop_left")
        self.declare_parameter("teleop_right", "astral_teleop_right")

        self.side = str(self.get_parameter("arm_side").value).lower()
        if self.side not in ("left", "right"):
            raise ValueError("arm_side must be left|right")
        self.window_sec = float(self.get_parameter("window_sec").value)
        self.metric_sec = float(self.get_parameter("metric_sec").value)
        self.save_dir = Path(str(self.get_parameter("save_dir").value))
        self.sync_both = bool(self.get_parameter("apply_both_arms").value)
        self._left_name = str(self.get_parameter("teleop_left").value).strip("/")
        self._right_name = str(self.get_parameter("teleop_right").value).strip("/")
        self._set_cli = {}
        self._get_cli = {}
        self._pending: dict = {}
        self._sent: dict = {}
        self._param_status = "params: waiting"

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        prefix = f"/teleop/{self.side}/tune"
        self.create_subscription(
            Float64MultiArray, f"{prefix}/xyz", self._on_xyz, qos
        )

        self._lock = threading.Lock()
        n = max(300, int(self.window_sec * 200))
        self._t: deque[float] = deque(maxlen=n)
        self._vr: deque[np.ndarray] = deque(maxlen=n)
        self._filt: deque[np.ndarray] = deque(maxlen=n)
        self._cmd: deque[np.ndarray] = deque(maxlen=n)
        self._vr_q: deque[np.ndarray] = deque(maxlen=n)
        self._filt_q: deque[np.ndarray] = deque(maxlen=n)
        self._cmd_q: deque[np.ndarray] = deque(maxlen=n)

        self._zero: np.ndarray | None = None
        self._zero_R: Rotation | None = None
        self._rezero = False
        self._save_req = False
        self._last_metric_print = 0.0
        self._metric_text = f"waiting for {prefix}/xyz ..."

        self.get_logger().info(
            f"teleop tune plot: {self.side}  subscribing {prefix}/xyz  "
            "keys: z=zero  s=save csv  q=quit"
        )

    def _on_xyz(self, msg: Float64MultiArray) -> None:
        d = np.asarray(msg.data, dtype=float)
        if d.size < 9:
            return
        vr_p, fi_p, cmd_p = d[0:3], d[3:6], d[6:9]
        if d.size >= 21:
            vr_q, fi_q, cmd_q = d[9:13], d[13:17], d[17:21]
        else:
            vr_q = fi_q = cmd_q = np.array([0.0, 0.0, 0.0, 1.0])
        t = time.monotonic()
        with self._lock:
            if self._rezero or self._zero is None:
                self._zero = cmd_p.copy()
                nq = float(np.linalg.norm(cmd_q))
                if nq > 0.5:
                    self._zero_R = Rotation.from_quat(cmd_q)
                self._rezero = False
                self._t.clear()
                self._vr.clear()
                self._filt.clear()
                self._cmd.clear()
                self._vr_q.clear()
                self._filt_q.clear()
                self._cmd_q.clear()
            self._t.append(t)
            self._vr.append(np.asarray(vr_p, dtype=float).copy())
            self._filt.append(np.asarray(fi_p, dtype=float).copy())
            self._cmd.append(np.asarray(cmd_p, dtype=float).copy())
            self._vr_q.append(np.asarray(vr_q, dtype=float).copy())
            self._filt_q.append(np.asarray(fi_q, dtype=float).copy())
            self._cmd_q.append(np.asarray(cmd_q, dtype=float).copy())

    def snapshot(self):
        with self._lock:
            if len(self._t) < 5 or self._zero is None:
                return None
            t = np.fromiter(self._t, dtype=float, count=len(self._t))
            vr = np.vstack(self._vr)
            filt = np.vstack(self._filt)
            cmd = np.vstack(self._cmd)
            vr_q = np.vstack(self._vr_q)
            filt_q = np.vstack(self._filt_q)
            cmd_q = np.vstack(self._cmd_q)
            zero = self._zero.copy()
            zero_R = self._zero_R
            save = self._save_req
            self._save_req = False
        t0 = t[-1] - self.window_sec
        m = t >= t0
        return {
            "t": t[m] - t[m][-1],
            "vr": (vr[m] - zero) * 1e3,
            "filt": (filt[m] - zero) * 1e3,
            "cmd": (cmd[m] - zero) * 1e3,
            "vr_q": vr_q[m],
            "filt_q": filt_q[m],
            "cmd_q": cmd_q[m],
            "zero_R": zero_R,
            "save": save,
        }

    def metrics(self, snap: dict) -> str:
        t = snap["t"]
        vr, filt, cmd = snap["vr"], snap["filt"], snap["cmd"]
        dt = float(np.median(np.diff(t))) if t.size > 2 else 0.007
        t_abs = t - t[0]
        m2 = t_abs >= (t_abs[-1] - self.metric_sec)
        if m2.sum() < 10:
            m2 = np.ones(t.size, dtype=bool)
        e = vr[m2] - cmd[m2]
        e_n = np.linalg.norm(e, axis=1)
        rms = float(np.sqrt(np.mean(e_n**2)))
        p95 = float(np.percentile(e_n, 95))
        e_filt = np.linalg.norm(filt[m2] - cmd[m2], axis=1)
        ik_rms = float(np.sqrt(np.mean(e_filt**2)))
        vr_std = float(np.linalg.norm(vr[m2].std(axis=0)))
        cmd_std = float(np.linalg.norm(cmd[m2].std(axis=0)))
        hold = vr_std < 2.0
        jitter = cmd_std if hold else float("nan")

        axis = int(np.argmax(vr[m2].std(axis=0)))
        lag = _xcorr_lag_ms(vr[m2, axis], cmd[m2, axis], max(dt, 1e-4))

        ori = "n/a"
        if snap["zero_R"] is not None:
            try:
                r0 = snap["zero_R"]
                ev = Rotation.from_quat(snap["vr_q"][m2])
                ec = Rotation.from_quat(snap["cmd_q"][m2])
                ang = (r0.inv() * ev).inv() * (r0.inv() * ec)
                deg = np.degrees(ang.magnitude())
                ori = f"{float(np.sqrt(np.mean(deg**2))):.2f} deg"
            except Exception:  # noqa: BLE001
                ori = "n/a"

        lag_s = "n/a" if lag is None else f"{lag:.0f} ms"
        jit_s = f"{jitter:.2f} mm" if hold else f"n/a (hand std {vr_std:.2f} mm)"
        lines = [
            f"arm  {self.side}    window {self.window_sec:.0f}s    last {self.metric_sec:.1f}s",
            f"pos err RMS   {rms:6.2f} mm",
            f"pos err p95   {p95:6.2f} mm",
            f"IK err RMS    {ik_rms:6.2f} mm   (filt vs EE)",
            f"hold jitter   {jit_s}",
            f"lag (xcorr)   {lag_s}   (+ = EE behind hand)",
            f"ori err RMS   {ori}",
            "",
            self._param_status,
            "",
            "z re-zero   s save csv   q quit",
        ]
        text = "\n".join(lines)
        now = time.monotonic()
        if now - self._last_metric_print > 2.0:
            self._last_metric_print = now
            extra = f"  HOLD jitter={jitter:.2f}mm" if hold else ""
            self.get_logger().info(
                f"[Tune][{self.side}] pos_rms={rms:.2f}mm p95={p95:.2f}mm "
                f"ik_rms={ik_rms:.2f}mm lag={lag_s} ori={ori}{extra}"
            )
        self._metric_text = text
        return text

    def save_csv(self, snap: dict) -> None:
        self.save_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = self.save_dir / f"teleop_tune_{self.side}_{ts}.csv"
        with path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(
                [
                    "t_rel_s",
                    "vr_x_mm",
                    "vr_y_mm",
                    "vr_z_mm",
                    "filt_x_mm",
                    "filt_y_mm",
                    "filt_z_mm",
                    "cmd_x_mm",
                    "cmd_y_mm",
                    "cmd_z_mm",
                    "err_x_mm",
                    "err_y_mm",
                    "err_z_mm",
                    "err_norm_mm",
                ]
            )
            err = snap["vr"] - snap["cmd"]
            for i in range(snap["t"].size):
                w.writerow(
                    [
                        f"{snap['t'][i]:.4f}",
                        *snap["vr"][i],
                        *snap["filt"][i],
                        *snap["cmd"][i],
                        *err[i],
                        np.linalg.norm(err[i]),
                    ]
                )
        self.get_logger().info(f"saved {path}")

    def _target_nodes(self) -> list[str]:
        if self.sync_both:
            return [self._left_name, self._right_name]
        return [self._left_name if self.side == "left" else self._right_name]

    def _client(self, kind: str, node_name: str):
        cache = self._set_cli if kind == "set" else self._get_cli
        if node_name not in cache:
            srv = SetParameters if kind == "set" else GetParameters
            cache[node_name] = self.create_client(srv, f"/{node_name}/{kind}_parameters")
        return cache[node_name]

    def fetch_params(self, timeout: float = 1.5) -> dict:
        out = dict(_DEFAULTS)
        target = self._left_name if self.side == "left" else self._right_name
        cli = self._client("get", target)
        if not cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f"no {target}/get_parameters, using defaults")
            return out
        req = GetParameters.Request()
        req.names = list(_HOT)
        fut = cli.call_async(req)
        deadline = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not fut.done() or fut.result() is None:
            return out
        for name, val in zip(_HOT, fut.result().values):
            if val.type == ParameterType.PARAMETER_DOUBLE:
                out[name] = float(val.double_value)
            elif val.type == ParameterType.PARAMETER_INTEGER:
                out[name] = float(val.integer_value)
            elif val.type == ParameterType.PARAMETER_STRING:
                out[name] = str(val.string_value)
        self._sent = dict(out)
        st = out.get("solver_type", "?")
        self._param_status = f"solver={st}"
        return out

    def queue_param(self, name: str, value) -> None:
        self._pending[name] = value

    def _param_msg(self, name: str, value) -> Parameter:
        p = Parameter()
        p.name = name
        if name in _STR_PARAMS:
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=str(value)
            )
        elif name in _INT_PARAMS:
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_INTEGER,
                integer_value=int(round(float(value))),
            )
        else:
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)
            )
        return p

    def flush_params(self) -> None:
        pending = self._pending
        if not pending:
            return

        def _changed(k, v) -> bool:
            prev = self._sent.get(k)
            if isinstance(v, str) or k in _STR_PARAMS:
                return prev != v
            if prev is None:
                return True
            try:
                return abs(float(v) - float(prev)) > 1e-12
            except (TypeError, ValueError):
                return prev != v

        to_send = {k: v for k, v in pending.items() if _changed(k, v)}
        self._pending = {}
        if not to_send:
            return
        params = [self._param_msg(k, v) for k, v in to_send.items()]
        ok_nodes = []
        for target in self._target_nodes():
            cli = self._client("set", target)
            if not cli.service_is_ready():
                if not cli.wait_for_service(timeout_sec=0.05):
                    continue
            req = SetParameters.Request()
            req.parameters = params
            cli.call_async(req)
            ok_nodes.append(target)
        self._sent.update(to_send)
        if "solver_type" in to_send:
            with self._lock:
                self._rezero = True
        who = "+".join(ok_nodes) if ok_nodes else "none"
        bits = []
        for k, v in to_send.items():
            if isinstance(v, str):
                bits.append(f"{k}={v}")
            elif k == "ik_tol" or k == "ik_w_reg":
                bits.append(f"{k}={float(v):.1e}")
            else:
                bits.append(f"{k}={float(v):.2f}")
        self._param_status = f"hot → {who}  " + "  ".join(bits)
        now = time.monotonic()
        if now - getattr(self, "_last_param_log", 0.0) > 0.8:
            self._last_param_log = now
            self.get_logger().info(self._param_status)


def _run_plot(node: TeleopTunePlot) -> None:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.widgets import CheckButtons, RadioButtons, Slider
    except ImportError as exc:
        raise SystemExit("need matplotlib: pip install matplotlib") from exc

    init = node.fetch_params()
    plt.rcParams.update(
        {
            "font.size": 10,
            "axes.grid": True,
            "grid.alpha": 0.35,
        }
    )
    fig, axes = plt.subplots(
        4,
        1,
        sharex=True,
        figsize=(13, 10.2),
        gridspec_kw={"height_ratios": [1, 1, 1, 0.7]},
    )
    fig.canvas.manager.set_window_title(f"teleop tune · {node.side}")
    labels = ["X (mm)", "Y (mm)", "Z (mm)", "|pos err| (mm)"]
    lines = []
    for ax, lab in zip(axes, labels):
        ax.set_ylabel(lab)
        if lab.startswith("|"):
            (le,) = ax.plot([], [], color="C3", lw=1.2, label="|hand − EE|")
            lines.append((le,))
        else:
            (l0,) = ax.plot([], [], color="C0", lw=1.4, label="手 VR映射")
            (l1,) = ax.plot(
                [], [], color="C1", lw=1.1, ls="--", label="滤波目标"
            )
            (l2,) = ax.plot([], [], color="C2", lw=1.4, label="末端 EE")
            lines.append((l0, l1, l2))
    axes[0].legend(loc="upper right", ncol=3, fontsize=9)
    axes[-1].set_xlabel("time (s)  ·  0 = now")
    fig.suptitle(
        f"{node.side} arm  ·  DH/URDF + live sliders",
        fontsize=12,
    )
    stats = fig.text(
        0.99,
        0.68,
        node._metric_text,
        va="center",
        ha="right",
        family="monospace",
        fontsize=9,
        transform=fig.transFigure,
        bbox={"facecolor": "white", "edgecolor": "#ccc", "pad": 6},
    )
    fig.subplots_adjust(
        right=0.72, hspace=0.08, left=0.10, top=0.94, bottom=0.40
    )

    st0 = str(init.get("solver_type", "analytic_dh")).lower()
    ax_radio = fig.add_axes([0.08, 0.02, 0.22, 0.10])
    radio = RadioButtons(
        ax_radio,
        ("DH analytic", "URDF LM"),
        active=1 if "urdf" in st0 else 0,
    )

    def _on_solver(label: str) -> None:
        name = "urdf_numerical" if label.startswith("URDF") else "analytic_dh"
        node.queue_param("solver_type", name)
        node.get_logger().info(f"queue solver_type={name}")

    radio.on_clicked(_on_solver)

    feel_specs = [
        ("pos_smoothing", "pos_smooth", 0.0, 0.99, 0.01, "%.2f"),
        ("rot_smoothing", "rot_smooth", 0.0, 0.99, 0.01, "%.2f"),
        ("motion_scale", "motion_scale", 0.10, 1.50, 0.01, "%.2f"),
        ("max_joint_vel", "max_vel rad/s", 0.50, 12.0, 0.05, "%.2f"),
    ]
    urdf_specs = [
        ("ik_w_pos", "ik_w_pos", 0.05, 3.0, 0.05, "%.2f"),
        ("ik_w_ori", "ik_w_ori", 0.0, 2.0, 0.05, "%.2f"),
        ("ik_max_iter", "ik_max_iter", 5.0, 50.0, 1.0, "%.0f"),
    ]
    sliders = []

    def _add_slider(name, label, vmin, vmax, step, fmt, x, y, w=0.28):
        ax_s = fig.add_axes([x, y, w, 0.025])
        sl = Slider(
            ax_s,
            label,
            vmin,
            vmax,
            valinit=float(np.clip(float(init.get(name, vmin)), vmin, vmax)),
            valstep=step,
            valfmt=fmt,
        )
        sl.on_changed(lambda val, n=name: node.queue_param(n, val))
        sliders.append(sl)
        return sl

    for i, spec in enumerate(feel_specs):
        _add_slider(*spec, 0.16, 0.34 - i * 0.038)
    fig.text(0.08, 0.355, "feel (DH+URDF)", fontsize=9, color="#444")
    for i, spec in enumerate(urdf_specs):
        _add_slider(*spec, 0.58, 0.34 - i * 0.038, 0.26)
    fig.text(0.52, 0.355, "URDF LM only (DH ignores)", fontsize=9, color="#444")

    tol0 = float(init.get("ik_tol", 1e-8))
    tol0 = float(np.clip(np.log10(max(tol0, 1e-12)), -10.0, -4.0))
    ax_tol = fig.add_axes([0.58, 0.34 - 3 * 0.038, 0.26, 0.025])
    sl_tol = Slider(ax_tol, "ik_tol log10", -10.0, -4.0, valinit=tol0, valstep=0.5, valfmt="%.1f")
    sl_tol.on_changed(lambda v: node.queue_param("ik_tol", 10.0 ** float(v)))
    sliders.append(sl_tol)

    reg0 = float(init.get("ik_w_reg", 1e-4))
    reg0 = float(np.clip(np.log10(max(reg0, 1e-8)), -6.0, -2.0))
    ax_reg = fig.add_axes([0.58, 0.34 - 4 * 0.038, 0.26, 0.025])
    sl_reg = Slider(ax_reg, "ik_w_reg log10", -6.0, -2.0, valinit=reg0, valstep=0.5, valfmt="%.1f")
    sl_reg.on_changed(lambda v: node.queue_param("ik_w_reg", 10.0 ** float(v)))
    sliders.append(sl_reg)

    ax_chk = fig.add_axes([0.32, 0.02, 0.22, 0.08])
    ax_chk.set_frame_on(False)
    chk = CheckButtons(ax_chk, ["sync left+right"], [node.sync_both])

    def _on_sync(_label):
        node.sync_both = bool(chk.get_status()[0])
        node.get_logger().info(f"sync both arms = {node.sync_both}")

    chk.on_clicked(_on_sync)

    def on_key(event):
        if event.key == "z":
            with node._lock:
                node._rezero = True
            node.get_logger().info("re-zero on next sample")
        elif event.key == "s":
            with node._lock:
                node._save_req = True
        elif event.key in ("q", "escape"):
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_):
        node.flush_params()
        snap = node.snapshot()
        if snap is None:
            stats.set_text(node._metric_text)
            return []
        if snap["save"]:
            node.save_csv(snap)
        t = snap["t"]
        vr, filt, cmd = snap["vr"], snap["filt"], snap["cmd"]
        err = np.linalg.norm(vr - cmd, axis=1)
        for i in range(3):
            l0, l1, l2 = lines[i]
            l0.set_data(t, vr[:, i])
            l1.set_data(t, filt[:, i])
            l2.set_data(t, cmd[:, i])
            ymin = float(min(vr[:, i].min(), filt[:, i].min(), cmd[:, i].min()))
            ymax = float(max(vr[:, i].max(), filt[:, i].max(), cmd[:, i].max()))
            pad = max(2.0, 0.15 * (ymax - ymin + 1e-6))
            axes[i].set_xlim(t[0], 0.05)
            axes[i].set_ylim(ymin - pad, ymax + pad)
        lines[3][0].set_data(t, err)
        axes[3].set_xlim(t[0], 0.05)
        axes[3].set_ylim(0.0, max(2.0, float(err.max()) * 1.2))
        stats.set_text(node.metrics(snap))
        artists = [a for group in lines for a in group]
        artists.append(stats)
        return artists

    _anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    _keep = (sliders, chk, radio)
    plt.show()
    del _anim, _keep


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopTunePlot()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        _run_plot(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
