#!/usr/bin/env bash
# vla_process_session.sh — 采集后处理一条龙：对齐 → 校验 → 转 LeRobot v2.1
#
# 用法:
#   ./vla_process_session.sh <session_dir> <output_dir> [--force-align] [--apply-quarantine] [--image-size N]
#
# 参数:
#   session_dir   采集 session 目录（含 episode*/ 子目录），如 ~/astral_data/pick_cube
#   output_dir    LeRobot 数据集输出目录，如 /home/robot/loopkok/sdk/astral_data_lerobot
#   --force-align       重新对齐已有 aligned_data.h5 的 episode（默认跳过）
#   --apply-quarantine  校验失败的 episode 直接隔离（默认只出报告不隔离）
#   --image-size N      视频 letterbox 边长（默认 224；0 = 保留原分辨率）
#
# 环境: 用 VLA/openpi 的 uv venv（pyarrow/cv2/av/h5py 全齐），PYTHONPATH 指向本包源码。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
PKG="$WS_ROOT/src/astral_data_collect"
PY="/home/robot/loopkok/sdk/VLA/openpi/.venv/bin/python"

SESSION="${1:?usage: vla_process_session.sh <session_dir> <output_dir> [--force-align] [--apply-quarantine] [--image-size N]}"
OUTPUT="${2:?missing output_dir}"
shift 2
FORCE_ALIGN="" APPLY="" IMAGE_SIZE=224
while [ $# -gt 0 ]; do
    case "$1" in
        --force-align) FORCE_ALIGN="--force";;
        --apply-quarantine) APPLY="--apply";;
        --image-size) IMAGE_SIZE="$2"; shift;;
        *) echo "unknown arg: $1" >&2; exit 2;;
    esac
    shift
done

[ -x "$PY" ] || { echo "缺少 openpi venv：$PY（先 cd VLA/openpi && uv sync）" >&2; exit 1; }
export PYTHONPATH="$PKG"

echo "=== [1/3] 时间对齐 align_data: $SESSION"
"$PY" -m astral_data_collect.align_data --session "$SESSION" $FORCE_ALIGN

echo "=== [2/3] 清洗校验 validate_data ${APPLY:+(apply 模式)}"
# validate 失败不阻断转换（转换器自身会跳过 NaN 段），但退出码透传报告
"$PY" -m astral_data_collect.validate_data --session "$SESSION" $APPLY || {
    echo "!! validate 未通过；未加 --apply-quarantine 时仅告警，继续转换" >&2
}

echo "=== [3/3] 转 LeRobot v2.1（letterbox ${IMAGE_SIZE}，0=原分辨率）"
"$PY" -m astral_data_collect.convert_to_lerobot \
    --session "$SESSION" --output "$OUTPUT" --image-size "$IMAGE_SIZE"

echo "=== 完成: $OUTPUT"
echo "提示: 数据有更新后，到 VLA/openpi 重算 norm stats 再训练："
echo "  uv run scripts/compute_norm_stats.py --config-name pi05_astral_lora"
