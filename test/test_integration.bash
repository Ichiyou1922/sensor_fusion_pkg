#!/bin/bash -xv
# SPDX-FileCopyrightText: 2025 Kazuha Mogi <mogi2fruits.kazu@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

# ============================================================
# Integration Test for Sensor Fusion Package
# ------------------------------------------------------------
# 目的: GitHub Actionsのログ上で「入力」と「判定」を可視化する
# ============================================================

ng() {
  echo "ERROR: ${1}行目が間違っています"
  kill_process
  exit 1
}

kill_process() {
  if [ -n "$PID" ]; then
    kill $PID
  fi
}

res=0

# ------------------------------------------------------------
# 1. 環境セットアップ & ノード起動
# ------------------------------------------------------------
source /opt/ros/humble/setup.bash
source install/setup.bash

echo ">>> Launching generic_kf_system.launch.py in background..."
ros2 launch sensor_fusion_pkg generic_kf_system.launch.py >/dev/null 2>&1 &
PID=$!
sleep 8

# ------------------------------------------------------------
# 2. トピックの存在確認 (Sanity Check)
# ------------------------------------------------------------
# 期待されるトピックリストに /kf_state があるか
out=$(ros2 topic list)
echo "$out" | grep -q "/kf_state" || ng "$LINENO"
echo "$out" | grep -q "/sensor_1/data" || ng "$LINENO"

[ "$res" = 0 ] && echo "Topic Existence OK"

# ------------------------------------------------------------
# 3. データ入出力テスト (I/O Verification)
# ------------------------------------------------------------
# シナリオ: センサ1に '10.0' を入力すると，推定値も '10.0' 付近になるはず (初期状態0から遷移)

# Step 3.1: 入力 (Publish)
echo ">>> Injecting Input: 10.0 to sensor_1"
ros2 topic pub --once /sensor_1/data std_msgs/msg/Float64 "{data: 10.0}" >/dev/null
sleep 2

# Step 3.2: 出力確認 (Subscribe & Check)
echo ">>> Checking Output from /kf_state (First element of state)"
# 一回だけ取得し，データ構造を確認
output_data=$(timeout 5s ros2 topic echo --once /kf_state --field data)

# 出力が空でないか確認
[ -n "$output_data" ] || ng "$LINENO"

# 注: ROSの出力は厳密な数値一致が難しいため (浮動小数点誤差)，
# grepで「それらしい値が含まれているか」を確認するアプローチをとる
# 配列出力 'array('d', [10.0, ...])' や yaml形式の中に '10' が含まれるか
echo "$output_data" | grep -E "10\.|9\." >/dev/null || ng "$LINENO"

[ "$res" = 0 ] && echo "I/O Verification OK (Input 10.0 -> Output converged)"

# ------------------------------------------------------------
# 4. 終了処理
# ------------------------------------------------------------
kill_process
echo "ALL INTEGRATION TESTS PASSED"
exit 0
