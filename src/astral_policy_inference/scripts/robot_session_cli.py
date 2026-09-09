#!/usr/bin/env python3
"""Non-ROS real-robot policy session — keyboard-driven CLI.

Wires the pure-Python :class:`PolicyRunner` to actual hardware
(``astral_robot_sdk`` UDP board + V4L2 / pyrealsense2 cameras) **without
ROS2**. ``RobotSession`` (``astral_policy_inference/session.py``) is the
programmable API; this script is a thin keyboard driver over it.

Verbs mirror the ROS keyboard::

  s            start POLICY (needs fresh obs)
  y [path:ep]  start PLAYBACK
  t <text>     set task / language instruction
  space        pause        n   resume
  h            HUMAN takeover (arm -> damping, drag by hand)
  g            release -> back to policy / playback
  x            stop -> IDLE (hold position)
  e            ESTOP (power off) + quit
  q            quit

Usage::

  python robot_session_cli.py --config config/robot_session.yaml
  # bring-up (no board, no model):
  python robot_session_cli.py --config config/robot_session.yaml --dry-run \
      --backend-type stub
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

sys.path.insert(0, "/home/robot/loopkok/sdk/astral_ws/src/astral_policy_inference")
sys.path.insert(0, "/home/robot/loopkok/sdk/astral_ws/src/astral_data_collect")

from astral_policy_inference.session import (
    build_robot_session,
    load_session_config,
)

_HELP = """\
keys:
  s           start POLICY
  y [path:ep] start PLAYBACK
  t <text>    set task / language instruction
  SPACE       pause      n   resume
  h           HUMAN takeover (damping, drag by hand)
  g           release -> back to policy / playback
  x           stop -> IDLE (hold position)
  e           ESTOP (power off) + quit
  q           quit
> """


def _read_line(prompt: str) -> str:
    sys.stdout.write(prompt)
    sys.stdout.flush()
    line = ""
    while True:
        ch = sys.stdin.read(1)
        if not ch:  # EOF
            return "q"
        if ch == "\n":
            return line
        if ch == "\x1b":  # ignore escape sequences (arrows)
            continue
        if ord(ch) == 32:  # space
            return "pause"
        line += ch


def _status_loop(sess, stop: threading.Event) -> None:
    """Compact status line on stderr every second (never pollutes stdin)."""
    while not stop.is_set():
        try:
            st = sess.stats()
            eng = st.get("engine") or {}
            err = st.get("session_error")
            loop = (st.get("latency_ms") or {}).get("loop") or {}
            line = (
                f"[session] state={st['state']}"
                f"{'/PAUSED' if st.get('paused') else ''}"
                f" prompt={st.get('prompt')!r:.40}"
                f" engine={ {k: eng.get(k) for k in ('plans', 'pops', 'last_plan_ms')} }"
                f" loop={loop.get('avg')}ms"
            )
            if err:
                line += f" ERROR={err}"
            print(line, file=sys.stderr, flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"[session] status error: {exc}", file=sys.stderr, flush=True)
        stop.wait(1.0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "config",
        "robot_session.yaml"))
    ap.add_argument("--dry-run", action="store_true",
                    help="不连控制板：默认零位 + 命令打日志")
    ap.add_argument("--record-dir", default="",
                    help="Session 记录目录（覆盖 yaml session.record_dir；空=不覆盖）")
    ap.add_argument("--backend-type", default="", choices=["", "openpi", "act", "stub"])
    ap.add_argument("--host", default="")
    ap.add_argument("--port", type=int, default=0)
    ap.add_argument("--checkpoint-dir", default="")
    ap.add_argument("--camera-image-size", type=int, default=0)
    ap.add_argument("--prompt", default="")
    ap.add_argument("--abs-action-min-scale", type=float, default=None)
    ap.add_argument("--engine-mode", default="")
    ap.add_argument("--verbose", action="store_true",
                    help="INFO 日志（dry-run 时打印 set_target_positions 等下发日志）")
    args = ap.parse_args()

    if args.verbose:
        import logging

        logging.basicConfig(level=logging.INFO)
        logging.getLogger("astral_policy_inference").setLevel(logging.INFO)
        logging.getLogger("astral_policy_inference.hw_io").setLevel(logging.INFO)

    cfg = load_session_config(args.config)
    if args.dry_run:
        cfg.robot["dry_run"] = True
    if args.backend_type:
        cfg.model["backend_type"] = args.backend_type
    if args.host:
        cfg.model["host"] = args.host
    if args.port:
        cfg.model["port"] = args.port
    if args.checkpoint_dir:
        cfg.model["checkpoint_dir"] = args.checkpoint_dir
    if args.camera_image_size:
        cfg.model["camera_image_size"] = args.camera_image_size
    if args.engine_mode:
        cfg.model["engine_mode"] = args.engine_mode
    if args.prompt:
        cfg.model["default_prompt"] = args.prompt
    if args.abs_action_min_scale is not None:
        cfg.model["abs_action_min_scale"] = args.abs_action_min_scale
    elif str(cfg.model.get("backend_type", "")).lower() == "stub":
        # stub 是开发冒烟后端（微小漂移动作），不适用绝对语义守卫
        cfg.model["abs_action_min_scale"] = 0.0

    sess = build_robot_session(
        cfg, record_dir=args.record_dir or None
    )
    print(f"config: {args.config}\n"
          f"backend={cfg.model.get('backend_type')} "
          f"image_size={cfg.model.get('camera_image_size')} "
          f"dry_run={cfg.robot.get('dry_run')} "
          f"recorder={'on' if sess.recorder is not None else 'off'}")
    print("starting session...", flush=True)
    sess.start()
    print(_HELP)

    status_stop = threading.Event()
    t_status = threading.Thread(
        target=_status_loop, args=(sess, status_stop), daemon=True
    )
    t_status.start()

    try:
        while True:
            line = _read_line("> ").strip()
            if not line:
                continue
            if line in ("q", "quit"):
                break
            if line == "e":
                print("ESTOP: powering off + teardown", flush=True)
                sess.estop()
                return 2
            if line == "s":
                sess.request("policy")
            elif line in ("y", "y "):
                sess.request("playback")
            elif line.startswith("y ") or line.startswith("y:"):
                rest = line[1:].strip().lstrip(":")
                sess.request("playback:" + rest)
            elif line == "pause":
                sess.request("pause")
            elif line == "n":
                sess.request("resume")
            elif line == "h":
                sess.request("takeover")
            elif line == "g":
                sess.request("release")
            elif line == "x":
                sess.request("stop")
            elif line.startswith("t "):
                text = line[2:].strip()
                sess.set_prompt(text)
                print(f"prompt -> {text!r}")
            else:
                print(_HELP)
            time.sleep(0.05)  # let the runner consume before next prompt
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        status_stop.set()
        sess.stop()
        print("session stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
