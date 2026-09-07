"""convert_to_act.py — 产出 ACT 可训数据集（官方 lerobot v3 布局），ACT-first 专属入口。

与 convert_to_lerobot_v3（通用 v2.1→v3 升版）不同，本脚本把官方 lerobot ACT 训练器
（`lerobot-train --policy.type=act`）的需求做成硬保证 + 内置两级自检：

两种输入（二选一）：
  --session <raw 会话目录>   完整链路: 对齐 → v2.1(临时) → v3 → 自检
  --v21-root <v2.1 目录>     直接对已有 v2.1 升版（不重编码，复用 vla_process_openpi.sh
                             产物）→ v3 → 自检 —— 与 openpi 共享同一份 v2.1 中间层

自检：
  - 结构级（强制，脚本内、不依赖 lerobot）: stats.json 图像/state/action 统计齐全、
    各相机同 shape、tasks 非空、parquet 可读、视频可解码 —— 不过即退出；
  - 深度级（--check-python <现代 lerobot python>）: 用指定解释器真装载 LeRobotDataset
    + 构建 ACT 预处理管线 + 逐帧解码 —— 金标准"无缝衔接官方 ACT"。

用法:
  python3 -m astral_data_collect.convert_to_act \
      --session ~/astral_data/default_task \
      --output ~/astral_data_act \
      [--image-size 224] [--check-python <conda lerobot 的 python>] \
      [--keep-v21 <dir>] [--overwrite] [--force-align]

  # 或从已有 v2.1 直接转（openpi 与 ACT 共享 v2.1）：
  python3 -m astral_data_collect.convert_to_act \
      --v21-root ~/astral_data_lerobot \
      --output ~/astral_data_act
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import shutil
import subprocess
import tempfile
from typing import Any, Callable

import av
import pyarrow.parquet as pq

from astral_data_collect.align_data import align_session
from astral_data_collect.convert_to_lerobot import convert_session
from astral_data_collect.convert_to_lerobot_v3 import convert_dataset_v21_to_v30


def _info(out_root: str) -> dict:
    with open(os.path.join(out_root, "meta", "info.json"), encoding="utf-8") as f:
        return json.load(f)


def _camera_keys(info: dict) -> list[str]:
    return sorted(k for k in info.get("features", {}) if k.startswith("observation.images."))


def structural_check(out_root: str, log: Callable = print) -> list[str]:
    """结构级自检（不依赖 lerobot 包）：返回问题列表，空 = 通过。

    覆盖 ACT 的硬需求：VISUAL/STATE/ACTION 的 MEAN_STD 归一化统计齐全、
    全部相机同 shape（ACT 只支持同 shape）、tasks 非空、数据可读可解码。
    """
    issues: list[str] = []
    meta = os.path.join(out_root, "meta")

    stats_path = os.path.join(meta, "stats.json")
    if not os.path.exists(stats_path):
        return [f"缺少 {stats_path}"]
    with open(stats_path, encoding="utf-8") as f:
        stats = json.load(f)

    info = _info(out_root)
    cams = _camera_keys(info)
    if not cams:
        issues.append("info.json features 中无 observation.images.*（ACT 需要至少一路图像）")
    required = ["observation.state", "action", *cams]
    for k in required:
        if k not in stats:
            issues.append(f"stats.json 缺 {k}（ACT MEAN_STD 归一化必需）")
            continue
        s = stats[k]
        if "mean" not in s or "std" not in s:
            issues.append(f"stats.json {k} 缺 mean/std")

    tasks_path = os.path.join(meta, "tasks.parquet")
    if not os.path.exists(tasks_path):
        issues.append("缺 meta/tasks.parquet")
    else:
        try:
            if pq.read_table(tasks_path).num_rows < 1:
                issues.append("meta/tasks.parquet 为空（ACT 训练需要任务文本）")
        except Exception as exc:  # noqa: BLE001
            issues.append(f"tasks.parquet 不可读: {exc}")

    data_files = sorted(glob.glob(os.path.join(out_root, "data", "**", "*.parquet"), recursive=True))
    if not data_files:
        issues.append("data/ 下无 parquet")
    else:
        total_rows = 0
        for p in data_files:
            try:
                total_rows += pq.read_table(p).num_rows
            except Exception as exc:  # noqa: BLE001
                issues.append(f"parquet 不可读 {p}: {exc}")
        if total_rows == 0:
            issues.append("parquet 全空（0 帧）")

    shapes: dict[str, tuple[int, int]] = {}
    for cam in cams:
        vids = sorted(glob.glob(os.path.join(out_root, "videos", cam, "**", "*.mp4"), recursive=True))
        if not vids:
            issues.append(f"{cam} 无 mp4")
            continue
        try:
            with av.open(vids[0]) as cont:
                sv = cont.streams.video[0]
                shapes[cam] = (sv.codec_context.width, sv.codec_context.height)
                next(cont.decode(sv), None)  # 抽首帧，触发实际解码
        except Exception as exc:  # noqa: BLE001
            issues.append(f"{cam} 视频不可解码 {vids[0]}: {exc}")
    if len(shapes) > 1:
        ref = next(iter(shapes.values()))
        for cam, shp in shapes.items():
            if shp != ref:
                issues.append(f"相机尺寸不一致 {cam}={shp} != 参考 {ref}（ACT 要求全部同 shape）")
    return issues


def deep_check_source() -> str:
    """深度自检脚本源码：在 --check-python（现代 lerobot 环境）里执行。

    金标准：LeRobotDataset 真装载 + ACT 预处理管线（含 MEAN_STD 归一化绑定
    dataset stats）可构建 + 逐帧图像特征解码正确。
    """
    return r'''
import sys
root = sys.argv[1]
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.processor_act import make_default_pre_post_processors

ds = LeRobotDataset(root)
assert ds.num_episodes > 0, "数据集空（0 episodes）"
assert ds.num_frames > 0, "数据集空（0 frames）"
stats = ds.meta.stats
pre, post = make_default_pre_post_processors(ACTConfig(), stats)  # 会校验 stats 齐全
sample = ds[0]
imgs = {k: v for k, v in sample.items() if k.startswith("observation.images")}
assert imgs, "无图像特征"
for k, v in imgs.items():
    assert v.ndim == 3 and v.shape[0] == 3, f"{k} shape={tuple(v.shape)} 异常"
print(f"OK episodes={ds.num_episodes} frames={ds.num_frames} cams={sorted(imgs)}")
'''


def run_pipeline(
    output_dir: str,
    *,
    session_dir: str | None = None,
    v21_root: str | None = None,
    image_size: int | None = 224,
    check_python: str | None = None,
    keep_v21: str | None = None,
    overwrite: bool = False,
    force_align: bool = False,
    log: Callable = print,
) -> dict[str, Any]:
    """→ ACT 数据集（官方 v3 布局），含两级自检。

    两种输入互斥：`session_dir`（raw，完整链路）或 `v21_root`（已有 v2.1，
    跳过对齐与重编码，openpi/ACT 共享同一份 v2.1 中间层）。
    """
    if (session_dir is None) == (v21_root is None):
        raise ValueError("必须且只能给 --session 或 --v21-root 之一")
    output_dir = os.path.abspath(os.path.expanduser(output_dir))
    if os.path.exists(output_dir) and not overwrite:
        raise FileExistsError(f"输出目录已存在（--overwrite 允许覆盖）: {output_dir}")

    if session_dir is not None:
        session_dir = os.path.abspath(os.path.expanduser(session_dir))
        if not os.path.isdir(session_dir):
            raise FileNotFoundError(f"session 目录不存在: {session_dir}")
        log(f"[1/4] 对齐 raw session: {session_dir}")
        align_session(session_dir, force=force_align, log=log)
        v21_dir = keep_v21 or tempfile.mkdtemp(prefix="act_v21_")
        log(f"[2/4] 转 v2.1 中间产物（image_size={image_size}）-> {v21_dir}")
        convert_session(session_dir, v21_dir, image_size=image_size, log=log)
    else:
        v21_dir = os.path.abspath(os.path.expanduser(v21_root))
        if not os.path.isdir(v21_dir):
            raise FileNotFoundError(f"v2.1 目录不存在: {v21_dir}")
        log(f"[2/4] 复用已有 v2.1（不重编码）-> {v21_dir}")

    try:
        log(f"[3/4] 升版 v3（= ACT 输入布局）-> {output_dir}")
        convert_dataset_v21_to_v30(v21_dir, output_dir, overwrite=True, log=log)

        log("[4/4] 结构级自检...")
        issues = structural_check(output_dir, log=log)
        if issues:
            for i in issues:
                log(f"  [FAIL] {i}")
            raise RuntimeError(
                f"结构级自检未通过（{len(issues)} 项）——ACT 无法无缝衔接"
            )
        log("  通过: stats 齐全 / 相机同 shape / tasks 非空 / parquet+视频可读可解码")

        summary: dict[str, Any] = {
            "output": output_dir,
            "image_size": image_size,
            "source": session_dir if session_dir else v21_dir,
        }
        if check_python:
            log(f"[4b] 深度自检（{check_python}）...")
            proc = subprocess.run(
                [check_python, "-c", deep_check_source(), output_dir],
                capture_output=True, text=True,
            )
            if proc.returncode != 0:
                raise RuntimeError(
                    "深度自检未通过（现代 lerobot 无法装载）:\n"
                    f"{proc.stdout}\n{proc.stderr}"
                )
            summary["deep_check"] = proc.stdout.strip()
            log(f"  {proc.stdout.strip()}")
        return summary
    finally:
        if (
            session_dir is not None
            and not keep_v21
            and v21_dir
            and os.path.isdir(v21_dir)
        ):
            shutil.rmtree(v21_dir, ignore_errors=True)


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--session", default=None, help="raw 会话目录（含 episode*）；与 --v21-root 二选一")
    ap.add_argument("--v21-root", default=None,
                    help="已有 v2.1 数据集目录（跳过对齐/重编码，openpi 与 ACT 共享）；与 --session 二选一")
    ap.add_argument("--output", required=True, help="ACT 可训数据集输出目录（官方 v3 布局）")
    ap.add_argument("--image-size", type=int, default=224,
                    help="letterbox 边长（默认 224；0=原分辨率；仅 --session 模式生效）")
    ap.add_argument("--check-python", default=None,
                    help="现代 lerobot 的 python（conda lerobot 环境）；给则跑深度自检")
    ap.add_argument("--keep-v21", default=None, help="保留 v2.1 中间产物到该目录（仅 --session 模式）")
    ap.add_argument("--overwrite", action="store_true",
                    help="输出目录已存在时允许覆盖")
    ap.add_argument("--force-align", action="store_true",
                    help="强制重对齐已有 aligned_data.h5 的段（仅 --session 模式）")
    args = ap.parse_args(argv)

    image_size = args.image_size if args.image_size else None
    run_pipeline(
        args.output,
        session_dir=args.session,
        v21_root=args.v21_root,
        image_size=image_size,
        check_python=args.check_python,
        keep_v21=args.keep_v21,
        overwrite=args.overwrite,
        force_align=args.force_align,
    )
    print("\nACT 数据集就绪。官方 lerobot 训练命令:")
    print(f"  HF_LEROBOT_HOME=~/lerobot_home lerobot-train \\")
    print(f"      --dataset.root={os.path.abspath(os.path.expanduser(args.output))} \\")
    print(f"      --dataset.repo_id=astral/{os.path.basename(args.session or args.v21_root)} \\")
    print(f"      --policy.type=act --steps=100000 --batch_size=64 --job_name=astral_act")


if __name__ == "__main__":
    main()
