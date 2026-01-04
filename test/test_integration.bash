#!/bin/bash
# SPDX-FileCopyrightText: 2025 Kazuha Mogi <mogi2fruits.kazu@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

# ============================================================
# Integration Test: Bash style
# ============================================================

ng() {
  echo "ERROR: ${1}行目が間違っています"
  echo "---------------------------------------------------"
  echo "CONTEXT: テストが失敗しました．直前のログを確認してください．"
  echo "---------------------------------------------------"
  res=1
}

cleanup() {
  jobs -p | xargs -r kill
}
trap cleanup EXIT

res=0

echo ">>> Setup ROS 2 environment (Silent)..."
source /opt/ros/humble/setup.bash
source install/setup.bash

echo ">>> Starting Test Execution..."
echo "=========================================================="

set -xv

ros2 launch sensor_fusion_pkg generic_kf_system.launch.py >/dev/null 2>&1 &
PID_LAUNCH=$!
sleep 5

ros2 topic pub -r 10 /sensor_1/data std_msgs/msg/Float64 "{data: 10.0}" >/dev/null 2>&1 &
PID_PUB=$!
sleep 3

out=$(timeout 5s ros2 topic echo --once /kf_state --field data)

set +x
echo "---------------------------------------------------"
echo "[DEBUG] Captured Output: '$out'"
echo "---------------------------------------------------"
set -x

if [ -z "$out" ]; then
  echo "[FAIL] Output is empty. Topic might not be published."
  ng "$LINENO"
else
  # 9.x または 10.x が含まれているか確認
  echo "$out" | grep -E "9\.|10\." >/dev/null
  if [ $? -ne 0 ]; then
    echo "[FAIL] Value did not converge to ~10.0"
    ng "$LINENO"
  else
    echo "[SUCCESS] Value converged: $out"
  fi
fi

# エラーがあれば終了コード1
exit $res
