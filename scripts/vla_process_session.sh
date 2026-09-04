#!/usr/bin/env bash
# vla_process_session.sh — 采集后处理一条龙：对齐 → 校验 → 转 LeRobot v2.1
#   （可选 → 升版 v3.0，供现代 lerobot ACT 训练）
#
# 用法:
#   ./vla_process_session.sh <session_dir> <output_dir> [--force-align] [--no-quarantine] [--image-size N] [--act-output DIR] [--overwrite-v3]
#
# 参数:
#   session_dir   采集 session 目录（含 episode*/ 子目录），如 ~/astral_data/pick_cube
#   output_dir    LeRobot 数据集输出目录，如 /home/robot/loopkok/sdk/astral_data_lerobot
#   --force-align       重新对齐已有 aligned_data.h5 的 episode（默认跳过）
#   --apply-quarantine  兼容旧写法（与默认一致：校验失败段隔离到 quarantine/）
#   --no-quarantine     显式放行：校验失败段只出报告不隔离，仍会进转换——
#                       坏段将进训练集，仅用于人工确认过报告的场景
#   --image-size N      视频 letterbox 边长（默认 224；0 = 保留原分辨率）
#   --act-output DIR    顺带把 v2.1 升版为 v3.0 写入 DIR（现代 lerobot ACT 直接
#                       可读；源 output_dir 只读不改）。--overwrite-v3 用于输出
#                       目录已存在时允许重建。
#
# 环境: 用 VLA/openpi 的 uv venv（pyarrow/cv2/av/h5py 全齐），PYTHONPATH 指向本包源码。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
PKG="$WS_ROOT/src/astral_data_collect"
PY="/home/robot/loopkok/sdk/VLA/openpi/.venv/bin/python"

SESSION="${1:?usage: vla_process_session.sh <session_dir> <output_dir> [--force-align] [--apply-quarantine] [--image-size N] [--act-output DIR]}"
OUTPUT="${2:?missing output_dir}"
shift 2
FORCE_ALIGN="" APPLY="--apply" IMAGE_SIZE=224 ACT_OUTPUT="" OVERWRITE_V3=""
while [ $# -gt 0 ]; do
    case "$1" in
        --force-align) FORCE_ALIGN="--force";;
        --apply-quarantine) APPLY="--apply";;
        --no-quarantine) APPLY="";;
        --image-size) IMAGE_SIZE="$2"; shift;;
        --act-output) ACT_OUTPUT="$2"; shift;;
        --overwrite-v3) OVERWRITE_V3="--overwrite";;
        *) echo "unknown arg: $1" >&2; exit 2;;
    esac
    shift
done

[ -x "$PY" ] || { echo "缺少 openpi venv：$PY（先 cd VLA/openpi && uv sync）" >&2; exit 1; }
export PYTHONPATH="$PKG"

echo "=== [1/3] 时间对齐 align_data: $SESSION"
"$PY" -m astral_data_collect.align_data --session "$SESSION" $FORCE_ALIGN

echo "=== [2/3] 清洗校验 validate_data ${APPLY:+（默认隔离 fail 段到 quarantine/，移动不删除）}"
# 门禁：默认 --apply（fail 段移入 quarantine/ 后可逆恢复）；隔离后仍有 fail
# （移动异常）即中止转换——防止坏段进训练数据。--no-quarantine 显式放行。
if "$PY" -m astral_data_collect.validate_data --session "$SESSION" $APPLY; then
    :
else
    if [ -n "$APPLY" ]; then
        echo "!! validate 隔离后仍有 fail 段（隔离失败）——中止转换，人工排查后重跑" >&2
        exit 1
    fi
    echo "!! validate 未通过（--no-quarantine 显式放行）——fail 段将进转换产物，请人工检查报告" >&2
fi

echo "=== [3/3] 转 LeRobot v2.1（letterbox ${IMAGE_SIZE}，0=原分辨率）"
"$PY" -m astral_data_collect.convert_to_lerobot \
    --session "$SESSION" --output "$OUTPUT" --image-size "$IMAGE_SIZE"

if [ -n "$ACT_OUTPUT" ]; then
    echo "=== [4/4] 升版 v2.1 → v3.0（现代 lerobot ACT 直接可读）"
    "$PY" -m astral_data_collect.convert_to_lerobot_v3 \
        --v21-root "$OUTPUT" --output "$ACT_OUTPUT" $OVERWRITE_V3
    echo "=== 完成: v2.1=$OUTPUT ; ACT v3.0=$ACT_OUTPUT"
else
    echo "=== 完成: $OUTPUT"
fi
echo "提示: 数据有更新后，到 VLA/openpi 重算 norm stats 再训练："
echo "  uv run scripts/compute_norm_stats.py --config-name pi05_astral_lora"
