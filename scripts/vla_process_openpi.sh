#!/usr/bin/env bash
# vla_process_openpi.sh — 从 raw 采集目录产出 openpi(pi0.5) 数据集（LeRobot v2.1）
#
# openpi 与 ACT 从【同一个 raw 采集目录】各取所需，输出独立文件夹：
#   openpi: 本脚本 → <output_dir>（v2.1，openpi 直接消费）
#   ACT:    vla_process_act.sh → 另一个目录（v3 + 自检）
#
# 用法:
#   ./vla_process_openpi.sh <session_dir> <output_dir> [--force-align] [--no-quarantine] [--image-size N]
#
# 参数:
#   session_dir   采集 raw 会话目录（含 episode*/ 子目录），openpi/ACT 共用同一份
#   output_dir    openpi 数据集输出目录（LeRobot v2.1），建议命名 <session>_openpi
#   --force-align       重新对齐已有 aligned_data.h5 的 episode（默认跳过）
#   --no-quarantine     校验失败段不隔离仍进转换（默认隔离到 session/quarantine/，移动不删除）
#   --image-size N      视频 letterbox 边长（默认 224；0 = 保留原分辨率）
#
# 环境: 用 VLA/openpi 的 uv venv（pyarrow/cv2/av/h5py 全齐），PYTHONPATH 指向本包源码。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
PKG="$WS_ROOT/src/astral_data_collect"
PY="/home/robot/loopkok/sdk/VLA/openpi/.venv/bin/python"

SESSION="${1:?usage: vla_process_openpi.sh <session_dir> <output_dir> [--force-align] [--no-quarantine] [--image-size N]}"
OUTPUT="${2:?missing output_dir}"
shift 2
FORCE_ALIGN="" APPLY="--apply" IMAGE_SIZE=224
while [ $# -gt 0 ]; do
    case "$1" in
        --force-align) FORCE_ALIGN="--force";;
        --no-quarantine) APPLY="";;
        --image-size) IMAGE_SIZE="$2"; shift;;
        *) echo "unknown arg: $1" >&2; exit 2;;
    esac
    shift
done

[ -x "$PY" ] || { echo "缺少 openpi venv：$PY（先 cd VLA/openpi && uv sync）" >&2; exit 1; }
export PYTHONPATH="$PKG"

echo "=== openpi 链路: raw=$SESSION → openpi(v2.1)=$OUTPUT ==="

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

echo "=== [3/3] 转 LeRobot v2.1（openpi 直接消费；letterbox ${IMAGE_SIZE}，0=原分辨率）"
"$PY" -m astral_data_collect.convert_to_lerobot \
    --session "$SESSION" --output "$OUTPUT" --image-size "$IMAGE_SIZE"

echo "=== 完成: openpi(v2.1)=$OUTPUT"
echo "提示: 数据有更新后，到 VLA/openpi 重算 norm stats 再训练："
echo "  uv run scripts/compute_norm_stats.py --config-name pi05_astral_lora"
