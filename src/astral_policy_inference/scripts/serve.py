#!/usr/bin/env python3
"""Unified policy server — serve any model family over one websocket protocol.

替代独立的 serve_act.py / serve_policy.py 入口。``--model`` 选择模型族，推理适配
分派到对应加载路径；协议层（metadata 首帧、msgpack、字符串错误、每连接 reset）完全共享。

模型族：
  --model act    （默认）lerobot 策略，进程内 ``InprocBackend`` 加载 checkpoint，
                 需 py3.12 lerobot + torch 环境（本机 conda lerobot env）；
  --model pi05   openpi pi0.5，经 ``create_trained_policy(get_config("pi05_astral"),
                 checkpoint_dir)`` 加载，需 openpi 环境（jax/torch + CUDA）+ **openpi
                 格式 checkpoint**（含 params/ 或 model.safetensors + assets/<asset_id>/
                 norm_stats.json；本仓库 astral_ckpt 是 ACT 格式，serve pi05 前需先有）。

机器人侧节点（py3.10/rclpy）用 ``backend_type=remote`` + ``host``/``port`` 连它——
线上动作恒为**绝对量**（(n, action_dim)，dataset 布局序）。

协议（与 openpi websocket_policy_server 一致，客户端用包内 RemoteBackend/client 即可）：
  * 连接后服务端先发 msgpack metadata；
  * 每请求 msgpack ``{observation/state, observation/camera/<slot>, prompt}``；
  * 每响应 msgpack ``{actions: (n, action_dim), server_timing?}``；
  * 字符串响应视为错误。序列化用包内 vendored ``protocol``（键 __ndarray__）。

用法：
  # ACT
  <py3.12 lerobot-env>/bin/python serve.py --model act \
      --checkpoint-dir <lerobot checkpoint> --port 8001
  # pi05（需 openpi env + openpi 格式 checkpoint）
  <openpi-env>/bin/python serve.py --model pi05 \
      --checkpoint-dir <openpi checkpoint> --port 8001
  # 节点：-p backend_type:=remote -p host:=<server> -p port:=8001 -p camera_image_size:=480
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys

# 脚本可直接运行：无论 cwd，先确保包可 import
sys.path.insert(0, "/home/robot/loopkok/sdk/astral_ws/src/astral_policy_inference")
sys.path.insert(0, "/home/robot/loopkok/sdk/astral_ws/src/astral_data_collect")

from astral_policy_inference import protocol as _proto
from astral_policy_inference.image_codec import normalize_request_images

_CONNECTIONS = 0  # 连接计数：观测断线/重连频率


# ------------------------------------------------------------ 推理适配

def _build_act_backend(args) -> object:
    from astral_policy_inference.backend import InprocBackend

    return InprocBackend(
        checkpoint_dir=args.checkpoint_dir,
        action_dim=args.action_dim,
        image_keys={label: label for label in args.slot_map.values()},
        device=args.device,
        default_prompt=args.default_prompt,
    )


def _infer_act(backend, slot_map: dict[str, str], request: dict) -> dict:
    from astral_policy_inference.backend import ObsBatch

    state = request.get("observation/state")
    if state is None:
        raise ValueError("request missing observation/state")
    images = {}
    for slot, label in slot_map.items():
        key = f"observation/camera/{slot}"
        if key in request:
            images[label] = request[key]
    obs = ObsBatch(
        state=state,
        images=images,
        prompt=request.get("prompt", ""),
    )
    actions = backend.infer(obs)
    response: dict = {"actions": actions}
    timing = getattr(backend, "last_timing", None)
    if timing:
        response["server_timing"] = timing  # prep/pre/infer/post/total ms
    return response


def _build_pi05_policy(args):
    try:
        from openpi.policies import policy_config
        from openpi.training import config as _config
    except ImportError as exc:  # pragma: no cover
        raise SystemExit(
            "--model pi05 requires the openpi environment "
            "(run in VLA/openpi uv venv with PYTHONPATH=openpi/src): "
            f"{exc}"
        ) from exc
    # 持久化 XLA 编译缓存（与训练侧同一目录，见 scripts/train.py）：换进程 / 换
    # checkpoint 重启 serve 时，首轮推理可从磁盘复用已编译图，不必每次重编 2-5 分钟。
    try:
        import pathlib as _pathlib

        import jax

        jax.config.update(
            "jax_compilation_cache_dir", str(_pathlib.Path("~/.cache/jax").expanduser())
        )
    except Exception:  # pragma: no cover - 缓存只是加速，失败不影响 serve
        pass
    train_config = _config.get_config(args.policy_config)
    policy = policy_config.create_trained_policy(
        train_config,
        args.checkpoint_dir,
        default_prompt=args.default_prompt or None,
    )
    return policy


def _infer_pi05(policy, request: dict) -> dict:
    resp = policy.infer(request)
    actions = resp.get("actions")
    if actions is None:
        raise ValueError("openpi policy returned no actions")
    response: dict = {"actions": actions}
    timing = resp.get("policy_timing")
    if timing:
        response["server_timing"] = timing
    return response


def _warmup_policy(policy, state_dim: int, camera_map: dict[str, str]) -> None:
    """进程内预热：pi05 首轮推理要 XLA 编译（2-5 分钟），启动时先做一次 dummy
    infer 把编译做掉，真机连上后第一个 chunk 不用干等。"""
    import time

    import numpy as np

    req: dict = {
        "observation/state": np.zeros(state_dim, dtype=np.float32),
        "prompt": "warmup",
    }
    for slot in camera_map:
        req[f"observation/camera/{slot}"] = np.zeros((224, 224, 3), dtype=np.uint8)
    t0 = time.monotonic()
    policy.infer(req)
    print(f"serve[pi05]: warmup done in {time.monotonic() - t0:.1f}s", flush=True)


# ------------------------------------------------------------ 协议层

async def _handle(websocket, infer_fn, reset_fn, packer, model: str) -> None:
    import websockets.exceptions

    global _CONNECTIONS
    _CONNECTIONS += 1
    print(f"serve[{model}]: connection #{_CONNECTIONS} opened", flush=True)

    # 新连接 = 新节点 POLICY 会话：重置策略内部状态（如 ACT 的 50 行缓存队列，
    # 否则上次会话未排完的缓存会被吐出 → 陈旧动作）。协议无 reset 消息，服务端每连接重置。
    reset_fn()

    try:
        await websocket.send(packer.pack({"model": model}))  # metadata
        while True:
            data = await websocket.recv()
            if isinstance(data, str):
                # 客户端对任何字符串响应抛错；我方 client 不会发字符串请求
                await websocket.send("unexpected string request")
                continue
            request = _proto.unpackb(data)
            try:
                # 节点 jpeg_transport 时 camera 槽位是 JPEG 字节 → 归一化为 RGB 数组，
                # 并移除 image_format 标志（ACT/pi05 两分支都消费 RGB，无需各自处理）。
                request = normalize_request_images(request)
                response = infer_fn(request)
            except Exception as exc:  # noqa: BLE001
                await websocket.send(f"infer failed: {exc}")
                continue
            await websocket.send(packer.pack(response))
    except websockets.exceptions.ConnectionClosed:
        pass  # 客户端正常断开，非错误


async def _serve(infer_fn, reset_fn, host: str, port: int, model: str) -> None:
    from websockets.asyncio.server import serve

    packer = _proto.Packer()
    async with serve(
        lambda ws: _handle(ws, infer_fn, reset_fn, packer, model),
        host,
        port,
        compression=None,
        max_size=None,
    ) as server:
        print(f"serve[{model}] ready: {model} on ws://{host}:{port}", flush=True)
        await server.serve_forever()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="act", choices=["act", "pi05"],
                        help="模型族：act(lerobot) | pi05(openpi)")
    parser.add_argument("--checkpoint-dir", required=True,
                        help="act=lerobot checkpoint；pi05=openpi 格式 checkpoint")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8001)
    parser.add_argument("--action-dim", type=int, default=8, help="act 用")
    parser.add_argument("--policy-config", default="pi05_astral", help="pi05 用：openpi 训练配置名")
    parser.add_argument("--device", default=None, help="act 用：torch device")
    parser.add_argument(
        "--warmup",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="pi05 用：加载模型后先做一次 dummy 推理预热 XLA 编译（默认开；--no-warmup 关）",
    )
    parser.add_argument("--default-prompt", default="")
    parser.add_argument(
        "--slot-map",
        # 值 = **模型 input_features 里的图像键**（观察传到 observation.images.<值>），
        # 不是 collect label。旧模型（video8/video0 训的）= video8/video0；新模型
        # （语义名 base/left_wrist 训的）= base/left_wrist。键=传输槽名，须与节点 camera_map 键一致。
        default='{"base_0_rgb": "video8", "left_wrist_0_rgb": "video0"}',
        help="JSON: model slot -> model feature image key（须与模型 input_features 一致）",
    )
    args = parser.parse_args()
    try:
        args.slot_map = {str(k): str(v) for k, v in json.loads(args.slot_map).items()}
    except json.JSONDecodeError as exc:
        parser.error(f"--slot-map must be JSON object: {exc}")

    print(f"serve[{args.model}]: loading checkpoint {args.checkpoint_dir} ...", flush=True)
    if args.model == "pi05":
        from openpi.training import config as _config

        train_config = _config.get_config(args.policy_config)
        policy = _build_pi05_policy(args)
        if args.warmup:
            _warmup_policy(
                policy, args.action_dim, getattr(train_config.data, "camera_map", {})
            )
        infer_fn = lambda req: _infer_pi05(policy, req)  # noqa: E731
        reset_fn = lambda: getattr(policy, "reset", lambda: None)()  # noqa: E731
    else:
        backend = _build_act_backend(args)
        backend.open()
        infer_fn = lambda req: _infer_act(backend, args.slot_map, req)  # noqa: E731
        reset_fn = backend.reset
    print(f"serve[{args.model}]: loaded; serving over websocket", flush=True)
    try:
        asyncio.run(_serve(infer_fn, reset_fn, args.host, args.port, args.model))
    except KeyboardInterrupt:
        print(f"serve[{args.model}] stopped")


if __name__ == "__main__":
    main()
