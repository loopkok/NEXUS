#!/usr/bin/env python3
"""Standalone ACT policy inference — no ROS, portable across lerobot environments.

参考 VLA/lerobot 的官方策略推理路径（`lerobot/scripts/lerobot_eval.py` +
`lerobot/policies/utils.py`）：

    policy  = get_policy_class(config["type"]).from_pretrained(checkpoint_dir)
    pre, post = make_pre_post_processors(policy.config, pretrained_path=checkpoint_dir)
    obs    = prepare_observation_for_inference({observation.state, observation.images.*}, device)
    chunk  = post(policy.predict_action_chunk(pre(obs)))   # 完整 n_action_steps 行（绝对）
    # 或逐行（与官方 eval 一致）：step = post(policy.select_action(pre(obs)))

**只依赖 lerobot**（`get_policy_class` / `make_pre_post_processors` /
`prepare_observation_for_inference`），无需 ROS、无需 astral_policy_inference ——
在任何 `import lerobot` 可用的环境（如 `PYTHONPATH=VLA/lerobot/src` 或装了 lerobot
的 conda 环境）都能跑。动作输出为**绝对量**（arm rad + 夹爪比值 [0,1]），与训练数据
next-state 同序，与后端契约一致。

关键实现点（本工程踩过的坑，已内置）：
  * 用 `get_policy_class(config.type).from_pretrained` 加载 —— `PreTrainedPolicy` 在
    lerobot 0.6.2 是抽象基类，不能直接 `from_pretrained`；
  * 无时序融合（`temporal_ensemble_coeff` 为空）时用 `predict_action_chunk` 一次返回
    完整 chunk；否则用 `select_action`（保留指数加权）；
  * 观测/动作维度、相机 label、图像尺寸全部从 checkpoint 的 `config.json`
    （input_features/output_features）自动推导，可 CLI 覆盖。

用法：
  # 1) 冒烟（合成观测，打印 50 行动作块）
  python act_inference.py --checkpoint-dir <ckpt> --scratch
  # 2) 单次推理（显式 state + 图像文件）
  python act_inference.py --checkpoint-dir <ckpt> \
      --state "[-0.35,0.19,-0.004,-1.89,-0.2,-0.001,0.014,0.5]" \
      --image video8=img8.jpg --image video0=img0.jpg
  # 3) 真实数据批量推理 + 对比录制动作 MAE（无需 ROS）
  python act_inference.py --checkpoint-dir <ckpt> --dataset-dir <v3数据集> --episode 0
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np


class ActPolicy:
    """最小 ACT 推理包装：load → 组装观测 → 推完整 chunk / 单步。"""

    def __init__(
        self,
        checkpoint_dir: str,
        *,
        action_dim: int | None = None,
        image_keys: list[str] | None = None,
        image_size: int | None = None,
        device: str | None = None,
        default_prompt: str = "",
    ) -> None:
        self.checkpoint_dir = os.path.abspath(os.path.expanduser(checkpoint_dir))
        self.action_dim = int(action_dim) if action_dim is not None else None
        self.image_keys = list(image_keys) if image_keys is not None else None
        self.image_size = int(image_size) if image_size is not None else None
        self.device = device
        self.default_prompt = default_prompt
        self._policy = None
        self._pre = None
        self._post = None
        self.n_action_steps: int | None = None
        self.temporal_ensemble_coeff: float | None = None
        self.last_timing: dict[str, float] = {}

    # ------------------------------------------------------------- load

    def load(self) -> "ActPolicy":
        if self._policy is not None:
            return self
        from lerobot.policies import get_policy_class, make_pre_post_processors

        with open(os.path.join(self.checkpoint_dir, "config.json"), encoding="utf-8") as f:
            cfg = json.load(f)
        policy_type = str(cfg.get("type", "act"))
        policy = get_policy_class(policy_type).from_pretrained(self.checkpoint_dir)
        if self.device is not None:
            policy.to(self.device)
        # 强制 preprocessor 与策略同 device：checkpoint 保存的 preprocessor 可能绑 cuda，
        # 若用户 --device cpu 而 pre 仍绑 cuda，观测会被来回搬运导致冲突
        # （参考 lerobot_eval.py 的 preprocessor_overrides 做法）。
        effective = (
            self.device
            or getattr(policy.config, "device", None)
            or str(next(policy.parameters()).device)
        )
        pre, post = make_pre_post_processors(
            policy.config,
            pretrained_path=self.checkpoint_dir,
            preprocessor_overrides={"device_processor": {"device": str(effective)}},
        )
        # 自动推导：image keys / action_dim / image_size / n_action_steps / 时序融合
        in_feat = cfg.get("input_features", {})
        out_feat = cfg.get("output_features", {})
        if self.image_keys is None:
            self.image_keys = [
                k.split("observation.images.", 1)[1]
                for k in in_feat
                if k.startswith("observation.images.")
            ]
        if self.action_dim is None:
            act_shape = out_feat.get("action", {}).get("shape", [])
            self.action_dim = int(act_shape[0]) if act_shape else 8
        if self.image_size is None:
            for k, v in in_feat.items():
                if k.startswith("observation.images.") and v.get("shape"):
                    self.image_size = int(v["shape"][-1])
                    break
        self.n_action_steps = int(cfg.get("n_action_steps", 50))
        self.temporal_ensemble_coeff = cfg.get("temporal_ensemble_coeff")
        self._policy = policy
        self._pre = pre
        self._post = post
        print(
            f"[act_inference] loaded {policy_type} {self.checkpoint_dir}\n"
            f"  action_dim={self.action_dim} image_keys={self.image_keys} "
            f"image_size={self.image_size} n_action_steps={self.n_action_steps} "
            f"temporal_ensemble={self.temporal_ensemble_coeff}"
        )
        return self

    def close(self) -> None:
        self._policy = self._pre = self._post = None

    def reset(self) -> None:
        if self._policy is not None:
            try:
                self._policy.reset()
            except Exception:  # noqa: BLE001
                pass

    # ----------------------------------------------------------- inference

    def _prepare(self, state, images, prompt: str):
        import torch

        from lerobot.policies.utils import prepare_observation_for_inference

        raw = {"observation.state": np.asarray(state, dtype=np.float32).reshape(-1)}
        if isinstance(images, dict):
            for label in self.image_keys or []:
                img = images.get(label)
                if img is not None:
                    raw[f"observation.images.{label}"] = np.asarray(img, dtype=np.uint8)
        obs = prepare_observation_for_inference(
            raw,
            next(self._policy.parameters()).device,
            task=prompt or self.default_prompt,
            robot_type=None,
        )
        return self._pre(obs)

    def predict_chunk(self, state, images, prompt: str = "") -> np.ndarray:
        """返回完整动作块 (n_action_steps, action_dim)，绝对量。

        无时序融合时等价于一次模型前向的全部输出（引擎分块/预取的本源）。
        有 `temporal_ensemble_coeff` 时本方法绕过指数加权（保留 `select_action` 语义），
        此时仅用于观测模型原始输出。
        """
        import torch

        if self.temporal_ensemble_coeff is not None:
            print(
                "[act_inference] warn: temporal_ensemble_coeff is set; "
                "predict_chunk bypasses it. Prefer select_action() for real control."
            )
        obs = self._prepare(state, images, prompt)
        t0 = time.perf_counter()
        with torch.inference_mode():
            action = self._policy.predict_action_chunk(obs)
        action = self._post(action)
        self.last_timing["predict_chunk_ms"] = (time.perf_counter() - t0) * 1000.0
        arr = np.asarray(action.detach().cpu().squeeze(0), dtype=np.float64)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        return arr

    def select_action(self, state, images, prompt: str = "") -> np.ndarray:
        """返回单步动作 (1, action_dim)，绝对量（与官方 eval 一致的逐行路径）。

        首次调用触发一次前向并填充内部队列，后续调用逐行弹出（缓存），队列耗尽才重推理。
        每次独立观测前应调用 `reset()` 清队列，否则拿到上一 chunk 缓存行。
        """
        import torch

        obs = self._prepare(state, images, prompt)
        t0 = time.perf_counter()
        with torch.inference_mode():
            action = self._policy.select_action(obs)
        action = self._post(action)
        self.last_timing["select_action_ms"] = (time.perf_counter() - t0) * 1000.0
        arr = np.asarray(action.detach().cpu().squeeze(0), dtype=np.float64)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        return arr


# ---------------------------------------------------------------- image IO


def load_image(path: str) -> np.ndarray:
    """加载图像为 HxWx3 uint8 RGB。"""
    path = os.path.abspath(os.path.expanduser(path))
    try:
        import cv2

        img = cv2.imread(path, cv2.IMREAD_COLOR)
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    except ImportError:
        from PIL import Image

        return np.asarray(Image.open(path).convert("RGB"), dtype=np.uint8)


def decode_video_frame(video_path: str, frame_idx: int, fps: int = 30) -> np.ndarray:
    """按需解码 mp4 指定帧为 HxWx3 uint8（seek 到关键帧再向前，O(1) 内存）。"""
    import av

    with av.open(video_path) as c:
        stream = c.streams.video[0]
        tb = stream.time_base
        target_pts = int(round((frame_idx / fps) / float(tb)))
        c.seek(max(0, target_pts - 1), stream=stream, backward=True)
        for frame in c.decode(stream):
            if frame.pts is None:
                continue
            if round(frame.pts * float(tb) * fps) >= frame_idx:
                return frame.to_ndarray(format="rgb24")
    raise RuntimeError(f"could not decode frame {frame_idx}")


# ---------------------------------------------------------------- CLI 模式


def run_scratch(policy: ActPolicy) -> int:
    rng = np.random.default_rng(0)
    size = policy.image_size or 480
    dim = policy.action_dim or 8
    state = np.array(
        [-0.35, 0.19, -0.004, -1.89, -0.20, -0.001, 0.014, 0.5][:dim], dtype=np.float64
    )
    images = {
        k: rng.integers(0, 256, (size, size, 3), dtype=np.uint8)
        for k in (policy.image_keys or [])
    }
    chunk = policy.predict_chunk(state, images, prompt="scratch")
    print(f"[scratch] chunk shape={chunk.shape} finite={np.isfinite(chunk).all()}")
    print(f"[scratch] row0: {np.round(chunk[0], 4)}")
    print(f"[scratch] predict_chunk took {policy.last_timing.get('predict_chunk_ms', 0):.1f}ms")
    return 0 if chunk.shape[1] == dim and np.isfinite(chunk).all() else 1


def run_single(policy: ActPolicy, args) -> int:
    state = json.loads(args.state) if args.state.startswith("[") else np.load(
        os.path.abspath(os.path.expanduser(args.state))
    )
    images = {}
    for spec in args.image or []:
        label, path = spec.split("=", 1)
        images[label] = load_image(path)
    missing = [k for k in (policy.image_keys or []) if k not in images]
    if missing:
        print(f"[single] warn: missing images {missing} — policy may ignore them")
    chunk = policy.predict_chunk(state, images, prompt=args.prompt or "")
    print(f"[single] chunk shape={chunk.shape}")
    print(f"[single] row0: {np.round(chunk[0], 4)}")
    if args.output:
        np.save(os.path.abspath(os.path.expanduser(args.output)), chunk)
        print(f"[single] saved to {args.output}")
    return 0


def run_dataset(policy: ActPolicy, args) -> int:
    """真实数据批量推理 + 逐帧 reset 后对比录制动作 MAE（无需 ROS）。"""
    import pyarrow.parquet as pq

    ds = args.dataset_dir
    info = json.load(open(os.path.join(ds, "meta", "info.json")))
    fps = int(info.get("fps", 30))
    eps = pq.read_table(
        os.path.join(ds, "meta", "episodes", "chunk-000", "file-000.parquet")
    ).to_pydict()
    if args.episode >= len(eps["episode_index"]):
        raise ValueError(f"episode {args.episode} >= {len(eps['episode_index'])}")
    row_from, length = int(eps["dataset_from_index"][args.episode]), int(eps["length"][args.episode])
    data = pq.read_table(
        os.path.join(ds, "data", "chunk-000", "file-000.parquet"),
        columns=["observation.state", "action"],
    ).slice(row_from, length)
    states = np.asarray(data.column("observation.state").to_pylist(), dtype=np.float64)
    actions = np.asarray(data.column("action").to_pylist(), dtype=np.float64)

    video = {}
    for k in policy.image_keys or []:
        vch = int(eps[f"videos/observation.images.{k}/chunk_index"][args.episode])
        vfile = int(eps[f"videos/observation.images.{k}/file_index"][args.episode])
        off = int(round(float(eps[f"videos/observation.images.{k}/from_timestamp"][args.episode]) * fps))
        p = os.path.join(ds, "videos", f"observation.images.{k}", f"chunk-{vch:03d}", f"file-{vfile:03d}.mp4")
        if not os.path.isfile(p):
            raise FileNotFoundError(f"{k}: {p}")
        video[k] = (p, off)

    idxs = list(range(0, length, args.stride))[: args.max_frames]
    preds, targets = [], []
    for t in idxs:
        policy.reset()  # 关键：清队列，保证每次独立推理
        images = {k: decode_video_frame(p, off + t, fps) for k, (p, off) in video.items()}
        chunk = policy.predict_chunk(states[t], images)
        preds.append(chunk[0])
        targets.append(actions[t])
    preds, targets = np.asarray(preds), np.asarray(targets)
    mae = np.abs(preds - targets).mean(axis=0)
    overall = float(np.abs(preds - targets).mean())
    print(f"[dataset] episode {args.episode} frames={len(idxs)}")
    print(f"[dataset] per-dim MAE (predicted[0] vs recorded action):")
    for i, m in enumerate(mae):
        print(f"    d{i}: {m:.4f}")
    print(f"[dataset] overall MAE = {overall:.4f} rad")
    return 0 if overall < 0.15 else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--checkpoint-dir", required=True, help="lerobot ACT checkpoint dir")
    ap.add_argument("--device", default=None, help="torch device（默认按 checkpoint 自动）")
    ap.add_argument("--action-dim", type=int, default=None, help="覆盖；默认从 config 推导")
    ap.add_argument("--image-keys", default=None, help="逗号分隔 camera label（覆盖推导）")
    ap.add_argument("--image-size", type=int, default=None, help="覆盖；默认从 config 推导")
    ap.add_argument("--default-prompt", default="", help="缺省语言指令")
    ap.add_argument("--scratch", action="store_true", help="合成观测冒烟（默认）")
    ap.add_argument("--state", default=None, help="JSON 数组字符串，或 .npy 路径（单次推理）")
    ap.add_argument("--image", action="append", default=None, help="label=path，可重复")
    ap.add_argument("--prompt", default="", help="单次推理的语言指令")
    ap.add_argument("--output", default=None, help="单次推理保存动作块到 .npy")
    ap.add_argument("--dataset-dir", default=None, help="LeRobot v3 数据集目录（批量+MAE）")
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--max-frames", type=int, default=60)
    ap.add_argument("--stride", type=int, default=4)
    args = ap.parse_args()

    if args.image_keys:
        image_keys = [k.strip() for k in args.image_keys.split(",") if k.strip()]
    else:
        image_keys = None

    policy = ActPolicy(
        args.checkpoint_dir,
        action_dim=args.action_dim,
        image_keys=image_keys,
        image_size=args.image_size,
        device=args.device,
        default_prompt=args.default_prompt,
    ).load()

    try:
        if args.dataset_dir:
            return run_dataset(policy, args)
        if args.state is not None:
            return run_single(policy, args)
        return run_scratch(policy)
    finally:
        policy.close()


if __name__ == "__main__":
    sys.exit(main())