#!/usr/bin/env bash
# Run astral_policy_inference unit tests (pure modules; no rclpy required but
# needs numpy/cv2/h5py + astral_data_collect on PYTHONPATH).
set -euo pipefail
# ROS setup.bash touches AMENT_* env vars that trip 'set -u'; keep -e only here.
set +u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
# WS == astral_ws
source /opt/ros/humble/setup.bash
PYTHONPATH="$WS/src/astral_data_collect:$WS/src/astral_policy_inference:$PYTHONPATH"
export PYTHONPATH

fail=0
for t in "$WS"/src/astral_policy_inference/tests/test_*.py; do
  echo "== $(basename "$t")"
  /usr/bin/python3 "$t" "$@" || fail=1
done
exit "$fail"
