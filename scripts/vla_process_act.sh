#!/usr/bin/env bash
# vla_process_act.sh — 从 raw 采集目录产出 ACT 可训数据集
#                     （官方 lerobot v3 布局 + convert_to_act 两级自检）
#
# openpi 与 ACT 从【同一个 raw 采集目录】各取所需，输出独立文件夹：
#   ACT:    本脚本 → <output_dir>（v3 + 内置自检，lerobot-train --policy.type=act 直接吃）
#   openpi: vla_process_openpi.sh → 另一个目录（v2.1）
#
# 用法:
#   ./vla_process_act.sh <session_dir> <output_dir> [--image-size N] [--check-python PY] \
#                        [--keep-v21 DIR] [--overwrite] [--force-align] [--no-quarantine]
#
# 参数:
#   session_dir   采集 raw 会话目录（含 episode*/ 子目录），openpi/ACT 共用同一份
#   output_dir    ACT 数据集输出目录（官方 v3 布局），建议命名 <session>_act
#   --image-size N      letterbox 边长（可选 224 默认 / 480 / 720 / 0=原生；
#                          原生要求各相机原生同尺寸，否则报错并提示改用 224/480/720）
#   --check-python PY   现代 lerobot 的 python（conda lerobot 环境）；给则跑深度自检（金标准）
#   --keep-v21 DIR      保留 v2.1 中间产物到 DIR（那份正是 openpi 要的；不保留则临时目录用完删）
#   --overwrite         输出目录已存在时允许覆盖
#   --force-align       重新对齐已有 aligned_data.h5 的 episode（默认跳过）
#   --no-quarantine     校验失败段不隔离仍进转换（默认隔离到 session/quarantine/，移动不删除）
#
# 环境: VLA/openpi 的 uv venv（pyarrow/cv2/av/h5py）+ 可选 conda lerobot（--check-python）。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
PKG="$WS_ROOT/src/astral_data_collect"
PY="/home/robot/loopkok/sdk/VLA/openpi/.venv/bin/python"

SESSION="${1:?usage: vla_process_act.sh <session_dir> <output_dir> [--image-size N] [--check-python PY] [--keep-v21 DIR] [--overwrite] [--force-align] [--no-quarantine]}"
OUTPUT="${2:?missing output_dir}"
shift 2
FORCE_ALIGN="" APPLY="--apply" IMAGE_SIZE=224 CHECK_PY="" KEEP_V21="" OVERWRITE=""
while [ $# -gt 0 ]; do
    case "$1" in
        --force-align) FORCE_ALIGN="--force";;
        --no-quarantine) APPLY="";;
        --image-size) IMAGE_SIZE="$2"; shift;;
        --check-python) CHECK_PY="$2"; shift;;
        --keep-v21) KEEP_V21="$2"; shift;;
        --overwrite) OVERWRITE="--overwrite";;
        *) echo "unknown arg: $1" >&2; exit 2;;
    esac
    shift
done

[ -x "$PY" ] || { echo "缺少 openpi venv：$PY（先 cd VLA/openpi && uv sync）" >&2; exit 1; }
export PYTHONPATH="$PKG"

echo "=== ACT 链路: raw=$SESSION → ACT(v3)=$OUTPUT ==="

echo "=== [1/3] 时间对齐 align_data: $SESSION"
"$PY" -m astral_data_collect.align_data --session "$SESSION" $FORCE_ALIGN

echo "=== [2/3] 清洗校验 validate_data ${APPLY:+（默认隔离 fail 段到 quarantine/，移动不删除）}"
# 门禁与 openpi 链路一致：默认隔离 fail 段，隔离后仍有 fail 即中止。
if "$PY" -m astral_data_collect.validate_data --session "$SESSION" $APPLY; then
    :
else
    if [ -n "$APPLY" ]; then
        echo "!! validate 隔离后仍有 fail 段（隔离失败）——中止转换，人工排查后重跑" >&2
        exit 1
    fi
    echo "!! validate 未通过（--no-quarantine 显式放行）——fail 段将进 ACT 数据集，请人工检查报告" >&2
fi

echo "=== [3/3] convert_to_act（升版 v3 + 两级自检；letterbox ${IMAGE_SIZE}，0=原分辨率）"
"$PY" -m astral_data_collect.convert_to_act \
    --session "$SESSION" --output "$OUTPUT" --image-size "$IMAGE_SIZE" \
    $OVERWRITE ${CHECK_PY:+--check-python "$CHECK_PY"} \
    ${KEEP_V21:+--keep-v21 "$KEEP_V21"} $FORCE_ALIGN

echo "=== 完成: ACT(v3)=$OUTPUT"
