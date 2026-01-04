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

MODE=${1:-"manual"} # 引数なし

echo ">>> Test Mode: $MODE"

if [ "$MODE" == 'sim' ]; then
  ros2 launch sensor_fusion_pkg generic_kf_system.launch.py use_sim_sensors:=true >/dev/null 2>&1 &
  PID_LAUNCH=$!
  sleep 5

  EXPECTED_REGEX="24\.|25\.|26\."

# 1. Launchを裏で起動 (センサノードを無効化する!)

if [ "$MODE" == "manual" ]; then
  ros2 launch sensor_fusion_pkg generic_kf_system.launch.py use_sim_sensors:=false >/dev/null 2>&1 &
  PID_LAUNCH=$!
  sleep 5

  # 2. テスト入力: sensor_1 と sensor_2 の両方に 10.0 を投入
  # 設定ファイルが2入力(dim_z=2)を期待しているため，片方だけだと同期待ちで止まるリスクがある
  ros2 topic pub -r 10 /sensor_1/data std_msgs/msg/Float64 "{data: 10.0}" >/dev/null 2>&1 &
  ros2 topic pub -r 10 /sensor_2/data std_msgs/msg/Float64 "{data: 10.0}" >/dev/null 2>&1 &
  
  EXPECTED_REGEX="9\.|10\.|11\."

else
  echo "Unknown mode. Use 'sim' or 'manual'."
  exit 1
fi

sleep 3 # 収束待ち

# 3. 出力確認
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
  # 予想範囲に収束しているか
  echo "$out" | grep -E "$EXPECTED_REGEX" >/dev/null
  if [ $? -eq 0 ]; then
    echo "[SUCCESS] Value converged as expected ($MODE mode)."
  else
    echo "[FAIL] Value did not much expectation for $MODE mode."
    exit 1
  fi
fi

exit $res
