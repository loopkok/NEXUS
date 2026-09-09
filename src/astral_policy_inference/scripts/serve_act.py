#!/usr/bin/env python3
"""Serve a trained ACT checkpoint over the openpi websocket protocol.

Run this in a **py3.12** environment with lerobot + CUDA torch (e.g. the
``lerobot`` conda env). The robot-side ``policy_node`` (py3.10/rclpy) connects
with ``backend_type=openpi`` + ``host``/``port`` pointing at this process — no
node-side changes. This lets an in-process-only ACT policy be deployed the same
way as the remote openpi pi0.5 server.

Protocol (mirrors openpi ``websocket_policy_server`` exactly, so the existing
``OpenPiServerBackend`` client works unchanged):

* on connect the server sends packed metadata (msgpack_numpy),
* each request is msgpack_numpy ``{observation/state, observation/camera/<slot>, prompt}``,
* each response is msgpack_numpy ``{actions: (n, action_dim)}`` (absolute,
  dataset layout order — same thing the driver/MuJoCo consumers take),
* a string response is treated by the client as an error.

Example:
  PYTHONPATH=astral_ws/src/astral_policy_inference:astral_ws/src/astral_data_collect \
    /home/robot/miniconda3/envs/lerobot/bin/python serve_act.py \
      --checkpoint-dir astral_ckpt/pickup_act_480/checkpoints/080000/pretrained_model \
      --port 8001
  # 然后节点：-p backend_type:=openpi -p host:=127.0.0.1 -p port:=8001 \
  #        -p camera_image_size:=480（与模型 preprocessor 一致）
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys

# Node client (OpenPiServerBackend) packs with openpi_client's vendored msgpack.
_OPENPI_CLIENT_SRC = (
    "/home/robot/loopkok/sdk/VLA/openpi/packages/openpi-client/src"
)

_CONNECTIONS = 0  # 连接计数：观测断线/重连频率


def _build_backend(args) -> object:
    from astral_policy_inference.backend import LerobotActBackend

    return LerobotActBackend(
        checkpoint_dir=args.checkpoint_dir,
        action_dim=args.action_dim,
        image_keys={label: label for label in args.slot_map.values()},
        device=args.device,
        default_prompt=args.default_prompt,
    )


def _infer(backend, slot_map: dict[str, str], request: dict) -> dict:
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


def _load_msgpack_numpy():
    """Use openpi_client's *vendored* msgpack_numpy — wire format differs from
    the pip msgpack-numpy (keys: b"__ndarray__" vs b"nd"), and the node client
    (OpenPiServerBackend -> WebsocketClientPolicy) packs with the vendored one."""
    sys.path.insert(0, _OPENPI_CLIENT_SRC)
    from openpi_client import msgpack_numpy

    return msgpack_numpy


def _make_packer():
    return _load_msgpack_numpy().Packer()


async def _handle(websocket, backend, slot_map, packer) -> None:
    import websockets.exceptions

    global _CONNECTIONS
    _CONNECTIONS += 1
    print(f"serve_act: connection #{_CONNECTIONS} opened", flush=True)

    msgpack_numpy = _load_msgpack_numpy()

    # 新连接 = 新节点 POLICY 会话：清 ACT 内部 action 队列，否则上次会话未排完的
    # 50 行缓存会先被吐出来（陈旧动作，最多 ~1.7s @30fps）。openpi 协议无 reset 消息，
    # 故在服务端每个连接建立时重置。
    backend.reset()

    try:
        await websocket.send(packer.pack({}))  # metadata (openpi client waits for it)
        while True:
            data = await websocket.recv()
            if isinstance(data, str):
                # openpi client raises on any string response; not expected from it.
                await websocket.send("unexpected string request")
                continue
            request = msgpack_numpy.unpackb(data)
            try:
                response = _infer(backend, slot_map, request)
            except Exception as exc:  # noqa: BLE001
                await websocket.send(f"infer failed: {exc}")
                continue
            await websocket.send(packer.pack(response))
    except websockets.exceptions.ConnectionClosed:
        pass  # 客户端正常断开（benchmark/e2e 结束），非错误


async def _serve(backend, slot_map, host: str, port: int) -> None:
    from websockets.asyncio.server import serve

    packer = _make_packer()
    async with serve(
        lambda ws: _handle(ws, backend, slot_map, packer),
        host,
        port,
        compression=None,
        max_size=None,
    ) as server:
        print(f"serve_act ready: ACT on ws://{host}:{port}", flush=True)
        await server.serve_forever()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint-dir", required=True,
                        help="lerobot ACT checkpoint dir (contains config.json)")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8001)
    parser.add_argument("--action-dim", type=int, default=8)
    parser.add_argument("--device", default=None, help="torch device (default: checkpoint default)")
    parser.add_argument("--default-prompt", default="")
    parser.add_argument(
        "--slot-map",
        default='{"base_0_rgb": "video8", "left_wrist_0_rgb": "video0"}',
        help="JSON: model slot -> collect camera label (must match node camera_map)",
    )
    args = parser.parse_args()
    try:
        args.slot_map = {str(k): str(v) for k, v in json.loads(args.slot_map).items()}
    except json.JSONDecodeError as exc:
        parser.error(f"--slot-map must be JSON object: {exc}")

    print(f"loading checkpoint {args.checkpoint_dir} ...", flush=True)
    backend = _build_backend(args)
    backend.open()
    print("checkpoint loaded; serving ACT over websocket", flush=True)
    try:
        asyncio.run(_serve(backend, args.slot_map, args.host, args.port))
    except KeyboardInterrupt:
        print("serve_act stopped")


if __name__ == "__main__":
    sys.path.insert(0, "/home/robot/loopkok/sdk/astral_ws/src/astral_policy_inference")
    sys.path.insert(0, "/home/robot/loopkok/sdk/astral_ws/src/astral_data_collect")
    main()
