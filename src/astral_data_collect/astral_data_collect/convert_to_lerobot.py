"""aligned_data.h5 → LeRobot v2.1 数据集（OpenPI pi0.5 直接可读）。

不依赖 lerobot 包，纯 pyarrow + av 手写 v2.1 布局；格式真值逐字段对齐
VLA/lerobot @0cf86487（OpenPI pyproject 钉死的提交）：

  meta/info.json            create_empty_dataset_info() 同构
  meta/tasks.jsonl          {"task_index", "task"}
  meta/episodes.jsonl       {"episode_index", "tasks", "length"}
  meta/episodes_stats.jsonl {"episode_index", "stats": {key: {min,max,mean,std,count}}}
  meta/stats.json           全部 episode 的加权聚合
  data/chunk-000/episode_{i:06d}.parquet
  videos/chunk-000/{video_key}/episode_{i:06d}.mp4

parquet 视频列 = VideoFrame struct{path: string, timestamp: float32}
（path 为视频相对路径，timestamp = frame_index/fps；与 VideoFrame docstring 示例一致。
v2.1 读取侧实际用 timestamp 列 + video_path 模板，struct 列为兼容性保留）。

用法：
  python3 convert_to_lerobot.py --session ~/astral_data/pick_place \
      --output ~/astral_data/lerobot/pick_place [--robot-type astral_dual_arm]
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from fractions import Fraction
from typing import Any, Iterable

import h5py
import numpy as np

from astral_data_collect.align_data import ALIGNED_H5
from astral_data_collect.schema import CollectSchema

CODEBASE_VERSION = "v2.1"
CHUNK_SIZE = 1000  # 与上游 DEFAULT_CHUNK_SIZE 一致
DATA_PATH_TEMPLATE = "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet"
VIDEO_PATH_TEMPLATE = "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4"

_IMAGE_STAT_MAX_SAMPLES = 100  # 与上游 estimate_num_samples 上限一致


# ---------------------------------------------------------------- 视频编码


def encode_video(
    frames: Iterable[np.ndarray],
    out_path: str,
    *,
    fps: int,
    width: int,
    height: int,
    codec: str = "libsvtav1",
    crf: int = 30,
    gop: int = 2,
) -> str:
    """把 uint8 RGB 帧序列写成 mp4；返回实际使用的 codec。

    默认 libsvtav1（上游 v2.1 默认），不可用时回退 h264。
    """
    import av

    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    def _open(codec_name: str):
        container = av.open(out_path, "w")
        try:
            stream = container.add_stream(
                codec_name, rate=fps, options={"crf": str(crf), "g": str(gop)}
            )
        except Exception:
            container.close()  # 回退路径：不泄漏容器/残留文件句柄
            raise
        stream.pix_fmt = "yuv420p"
        stream.width = width - (width % 2)  # yuv420p 要求偶数
        stream.height = height - (height % 2)
        return container, stream

    try:
        container, stream = _open(codec)
    except Exception:
        codec = "h264"
        container, stream = _open(codec)

    time_base = Fraction(1, fps)
    try:
        for i, img in enumerate(frames):
            frame = av.VideoFrame.from_ndarray(img, format="rgb24")
            frame.pts = i
            frame.time_base = time_base
            for packet in stream.encode(frame):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)
    finally:
        container.close()
    if not os.path.exists(out_path):
        raise OSError(f"video encode failed: {out_path}")
    return codec


def get_video_info(video_path: str) -> dict[str, Any]:
    """与上游 get_video_info 同构（写进 features[key].info）。"""
    import av

    info: dict[str, Any] = {}
    with av.open(video_path, "r") as f:
        stream = f.streams.video[0]
        info["video.height"] = int(stream.height)
        info["video.width"] = int(stream.width)
        info["video.codec"] = str(stream.codec.canonical_name or stream.codec.name)
        info["video.pix_fmt"] = str(stream.pix_fmt)
        info["video.is_depth_map"] = False
        info["video.fps"] = int(stream.base_rate)
        pix = str(stream.pix_fmt)
        info["video.channels"] = (
            1 if ("gray" in pix or "mono" in pix) else (4 if "a" in pix[-2:] else 3)
        )
        info["has_audio"] = len(f.streams.audio) > 0
    return info


# ---------------------------------------------------------------- 统计


def _feature_stats(arr: np.ndarray, axis, keepdims=True) -> dict[str, np.ndarray]:
    return {
        "min": np.min(arr, axis=axis, keepdims=keepdims),
        "max": np.max(arr, axis=axis, keepdims=keepdims),
        "mean": np.mean(arr, axis=axis, keepdims=keepdims),
        "std": np.std(arr, axis=axis, keepdims=keepdims),
        "count": np.array([arr.shape[0] if arr.ndim else 1]),
    }


def _to_jsonable(v: Any) -> Any:
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, (np.floating, np.integer)):
        return v.item()
    return v


def compute_episode_stats(
    state: np.ndarray,
    action: np.ndarray,
    image_samples: dict[str, np.ndarray],  # cam → (N,H,W,C) uint8
) -> dict[str, dict]:
    """与上游 compute_episode_stats 同构：数值全量，图像抽样 /255。"""
    stats: dict[str, dict] = {
        "observation.state": _feature_stats(state, axis=0),
        "action": _feature_stats(action, axis=0),
    }
    for key, imgs in image_samples.items():
        if len(imgs) == 0:
            continue
        arr = imgs.astype(np.float32) / 255.0  # (N,H,W,C)
        arr = np.transpose(arr, (0, 3, 1, 2))  # (N,C,H,W)
        s = _feature_stats(arr, axis=(0, 2, 3), keepdims=True)
        stats[key] = {
            k: (v if k == "count" else np.squeeze(v, axis=0))  # (C,1,1)
            for k, v in s.items()
        }
    return stats


def aggregate_stats(all_stats: list[dict[str, dict]]) -> dict[str, dict]:
    """上游 aggregate_stats 同构：加权 mean、并行方差、min/max/count。"""
    if not all_stats:
        return {}
    keys = all_stats[0].keys()
    out: dict[str, dict] = {}
    for key in keys:
        stats_list = [s[key] for s in all_stats if key in s]
        if not stats_list:
            continue
        counts = np.concatenate(
            [np.asarray(s["count"], dtype=np.float64).reshape(-1) for s in stats_list]
        )
        total = counts.sum()
        means = np.stack([np.asarray(s["mean"], dtype=np.float64) for s in stats_list])
        stds = np.stack([np.asarray(s["std"], dtype=np.float64) for s in stats_list])
        w = counts.reshape((-1,) + (1,) * (means.ndim - 1))
        mean = (means * w).sum(axis=0) / total
        var = (((stds ** 2 + means ** 2) * w).sum(axis=0) / total) - mean ** 2
        out[key] = {
            "min": np.min(
                np.stack([np.asarray(s["min"], dtype=np.float64) for s in stats_list]),
                axis=0,
            ),
            "max": np.max(
                np.stack([np.asarray(s["max"], dtype=np.float64) for s in stats_list]),
                axis=0,
            ),
            "mean": mean,
            "std": np.sqrt(np.clip(var, 0.0, None)),
            "count": np.array([int(total)]),
        }
    return out


# ---------------------------------------------------------------- parquet


def write_episode_parquet(
    path: str,
    *,
    state: np.ndarray,
    action: np.ndarray,
    fps: int,
    episode_index: int,
    index_start: int,
    task_index: int,
    video_keys: list[str],
    video_paths: dict[str, str],
) -> None:
    import pyarrow as pa
    import pyarrow.parquet as pq

    n = state.shape[0]
    columns: dict[str, Any] = {
        "observation.state": pa.array(
            [state[i].tolist() for i in range(n)], type=pa.list_(pa.float32())
        ),
        "action": pa.array(
            [action[i].tolist() for i in range(n)], type=pa.list_(pa.float32())
        ),
    }
    for key in video_keys:
        rel = video_paths[key]
        columns[key] = pa.array(
            [
                {"path": rel, "timestamp": np.float32(i / fps)}
                for i in range(n)
            ],
            type=pa.struct({"path": pa.string(), "timestamp": pa.float32()}),
        )
    columns["timestamp"] = pa.array(
        [np.float32(i / fps) for i in range(n)], type=pa.float32()
    )
    columns["frame_index"] = pa.array(np.arange(n, dtype=np.int64))
    columns["episode_index"] = pa.array(np.full(n, episode_index, dtype=np.int64))
    columns["index"] = pa.array(
        np.arange(index_start, index_start + n, dtype=np.int64)
    )
    columns["task_index"] = pa.array(np.full(n, task_index, dtype=np.int64))

    os.makedirs(os.path.dirname(path), exist_ok=True)
    pq.write_table(pa.table(columns), path)


# ---------------------------------------------------------------- 主流程


def _decode_jpeg(jpeg: np.ndarray) -> np.ndarray | None:
    import cv2

    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return None
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def convert_session(
    session_dir: str,
    output_dir: str,
    *,
    robot_type: str = "astral_dual_arm",
    codec: str = "libsvtav1",
    log=print,
) -> dict[str, Any]:
    """把 session 下所有已对齐 episode 导出为一个 LeRobot v2.1 数据集。"""
    import cv2  # noqa: F401  (JPEG 解码依赖)

    session_dir = os.path.abspath(os.path.expanduser(session_dir))
    output_dir = os.path.abspath(os.path.expanduser(output_dir))
    episode_dirs = sorted(
        os.path.join(session_dir, d)
        for d in os.listdir(session_dir)
        if d.startswith("episode")
        and os.path.isdir(os.path.join(session_dir, d))
        and os.path.exists(os.path.join(session_dir, d, ALIGNED_H5))
    )
    if not episode_dirs:
        raise RuntimeError(f"no aligned episodes under {session_dir} (run align_data first)")

    os.makedirs(os.path.join(output_dir, "meta"), exist_ok=True)

    # -- 第一遍：读 schema / task / 尺寸，建 tasks 表 -----------------------------
    schemas: list[CollectSchema] = []
    tasks: list[str] = []
    metas: list[dict] = []
    shapes: dict[str, tuple[int, int]] = {}
    for ep in episode_dirs:
        with open(os.path.join(ep, "meta.json"), encoding="utf-8") as f:
            meta = json.load(f)
        metas.append(meta)
        schemas.append(CollectSchema.from_dict(meta["schema"]))
        task = str(meta.get("task", ""))
        if task not in tasks:
            tasks.append(task)
    schema = schemas[0]
    for s in schemas[1:]:
        if s.to_dict()["state_blocks"] != schema.to_dict()["state_blocks"]:
            raise RuntimeError("episode 间 schema 不一致，无法合并导出（分 session 重录）")
    fps = int(schema.dataset_fps)
    state_dim = schema.state_dim
    state_names = schema.state_names()
    video_keys = [f"observation.images.{cam}" for cam in schema.cameras]

    # -- 第二遍：逐 episode 写 parquet + mp4 + meta -------------------------------
    tasks_f = open(os.path.join(output_dir, "meta", "tasks.jsonl"), "w", encoding="utf-8")
    episodes_f = open(os.path.join(output_dir, "meta", "episodes.jsonl"), "w", encoding="utf-8")
    ep_stats_f = open(os.path.join(output_dir, "meta", "episodes_stats.jsonl"), "w", encoding="utf-8")
    for ti, task in enumerate(tasks):
        tasks_f.write(json.dumps({"task_index": ti, "task": task}, ensure_ascii=False) + "\n")
    tasks_f.close()

    all_stats: list[dict[str, dict]] = []
    total_frames = 0
    video_info: dict[str, dict] = {}
    used_codec: str | None = None
    ep_index = 0  # LeRobot 要求 0..N-1 连续：跳过坏段时用独立计数器重编号

    for ep_dir, meta in zip(episode_dirs, metas):
        chunk = ep_index // CHUNK_SIZE
        task = str(meta.get("task", ""))
        task_index = tasks.index(task)

        with h5py.File(os.path.join(ep_dir, ALIGNED_H5), "r") as fa:
            state = fa["observation/state"][:]
            action = fa["action"][:]
            if not (np.all(np.isfinite(state)) and np.all(np.isfinite(action))):
                log(
                    f"  SKIP {ep_dir}: state/action 含 NaN/Inf"
                    "（先跑 validate_data --apply 隔离）"
                )
                continue
            n = state.shape[0]

            # -- 视频：JPEG → RGB 帧 → mp4 ------------------------------------------
            rel_paths: dict[str, str] = {}
            img_samples: dict[str, np.ndarray] = {}
            for cam, vkey in zip(schema.cameras, video_keys):
                rel = VIDEO_PATH_TEMPLATE.format(
                    episode_chunk=chunk, video_key=vkey, episode_index=ep_index
                )
                rel_paths[vkey] = rel
                mp4_path = os.path.join(output_dir, rel)

                jpegs = fa[f"{cam}/images"]
                first = _decode_jpeg(jpegs[0])
                if first is None:
                    raise RuntimeError(f"{ep_dir}: {cam} frame 0 undecodable")
                h, w = first.shape[:2]
                shapes[vkey] = (h, w)

                def _frames(cam=cam, n=n, jpegs=jpegs):
                    last = None
                    for i in range(n):
                        img = _decode_jpeg(jpegs[i])
                        if img is None:
                            img = last if last is not None else np.zeros(
                                (h, w, 3), dtype=np.uint8
                            )
                        last = img
                        yield img

                used = encode_video(
                    _frames(), mp4_path, fps=fps, width=w, height=h, codec=codec
                )
                used_codec = used

                # 图像统计抽样（均匀 ≤100 帧，剔除解码失败帧）
                step = max(1, n // _IMAGE_STAT_MAX_SAMPLES)
                sampled = [
                    img
                    for i in range(0, n, step)
                    if (img := _decode_jpeg(jpegs[i])) is not None
                ]
                img_samples[vkey] = (
                    np.stack(sampled) if sampled else np.zeros((0, h, w, 3), np.uint8)
                )

            if ep_index == 0:
                for vkey in video_keys:
                    video_info[vkey] = get_video_info(
                        os.path.join(output_dir, rel_paths[vkey])
                    )

            # -- parquet --------------------------------------------------------------
            pq_rel = DATA_PATH_TEMPLATE.format(
                episode_chunk=chunk, episode_index=ep_index
            )
            write_episode_parquet(
                os.path.join(output_dir, pq_rel),
                state=state,
                action=action,
                fps=fps,
                episode_index=ep_index,
                index_start=total_frames,
                task_index=task_index,
                video_keys=video_keys,
                video_paths=rel_paths,
            )

            ep_stats = compute_episode_stats(state, action, img_samples)
            all_stats.append(ep_stats)
            episodes_f.write(
                json.dumps(
                    {"episode_index": ep_index, "tasks": [task], "length": int(n)},
                    ensure_ascii=False,
                )
                + "\n"
            )
            ep_stats_f.write(
                json.dumps(
                    {
                        "episode_index": ep_index,
                        "stats": {
                            k: {kk: _to_jsonable(vv) for kk, vv in v.items()}
                            for k, v in ep_stats.items()
                        },
                    },
                    ensure_ascii=False,
                )
                + "\n"
            )
            total_frames += n
            log(f"  episode {ep_index:06d}: {n} frames task={task!r}")
            ep_index += 1

    episodes_f.close()
    ep_stats_f.close()

    # -- stats.json（全局聚合） ------------------------------------------------------
    global_stats = aggregate_stats(all_stats)
    with open(os.path.join(output_dir, "meta", "stats.json"), "w", encoding="utf-8") as f:
        json.dump(
            {k: {kk: _to_jsonable(vv) for kk, vv in v.items()} for k, v in global_stats.items()},
            f, ensure_ascii=False, indent=2,
        )

    # -- info.json ---------------------------------------------------------------------
    features: dict[str, Any] = {
        "observation.state": {
            "dtype": "float32", "shape": [state_dim], "names": state_names,
        },
        "action": {
            "dtype": "float32", "shape": [state_dim], "names": state_names,
        },
    }
    for vkey in video_keys:
        h, w = shapes[vkey]
        features[vkey] = {
            "dtype": "video",
            "shape": [h, w, 3],
            "names": ["height", "width", "channels"],
            "info": video_info.get(vkey, {}),
        }
    features.update({
        "timestamp": {"dtype": "float32", "shape": [1], "names": None},
        "frame_index": {"dtype": "int64", "shape": [1], "names": None},
        "episode_index": {"dtype": "int64", "shape": [1], "names": None},
        "index": {"dtype": "int64", "shape": [1], "names": None},
        "task_index": {"dtype": "int64", "shape": [1], "names": None},
    })

    n_ep = ep_index  # 实际写入的段数（NaN 段已跳过）
    if n_ep == 0:
        raise RuntimeError("没有可导出的健康 episode（全部含 NaN/Inf）")
    info = {
        "codebase_version": CODEBASE_VERSION,
        "robot_type": robot_type,
        "total_episodes": n_ep,
        "total_frames": total_frames,
        "total_tasks": len(tasks),
        "total_videos": n_ep * len(video_keys),
        "total_chunks": (n_ep + CHUNK_SIZE - 1) // CHUNK_SIZE if n_ep else 0,
        "chunks_size": CHUNK_SIZE,
        "fps": fps,
        "splits": {"train": f"0:{n_ep}"},
        "data_path": DATA_PATH_TEMPLATE,
        "video_path": VIDEO_PATH_TEMPLATE,
        "features": features,
    }
    with open(os.path.join(output_dir, "meta", "info.json"), "w", encoding="utf-8") as f:
        json.dump(info, f, ensure_ascii=False, indent=2)

    summary = {
        "output": output_dir,
        "episodes": n_ep,
        "frames": total_frames,
        "fps": fps,
        "state_dim": state_dim,
        "video_keys": video_keys,
        "codec": used_codec,
        "tasks": tasks,
    }
    log(
        f"LeRobot v2.1 dataset -> {output_dir}\n"
        f"  {n_ep} episodes / {total_frames} frames @ {fps}Hz, "
        f"state_dim={state_dim}, cams={len(video_keys)}, codec={used_codec}"
    )
    return summary


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--session", required=True, help="session 目录（含已对齐 episode*）")
    ap.add_argument("--output", required=True, help="LeRobot 数据集输出目录")
    ap.add_argument("--robot-type", default="astral_dual_arm")
    ap.add_argument("--codec", default="libsvtav1", help="libsvtav1（默认）| h264，不可用自动回退")
    args = ap.parse_args(argv)
    convert_session(
        args.session, args.output, robot_type=args.robot_type, codec=args.codec
    )


if __name__ == "__main__":
    main(sys.argv[1:])
