#!/bin/bash -xv
# SPDX-FileCopyrightText: 2025 Kazuha Mogi <mogi2fruits.kazu@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

# ============================================================
# Integration Test: Bash style
# ============================================================

ng() {
  echo "${1}行目が間違っています"
  res=1
}

cleanup() {
  jobs -p | xargs -r kill
}
# スクリプト終了時(正常/異常問わず)にcleanupを実行
trap cleanup EXIT

res=0

# 1. 環境設定 & ノード起動 (Silent Mode)
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launchを裏で起動 (ログは捨てる)
ros2 launch sensor_fusion_pkg generic_kf_system.launch.py >/dev/null 2>&1 &
sleep 5 # 起動待ち

# 2. テスト入力: 10.0 を連続投入 (10Hz)
# フィルタを収束させるため，単発ではなく連続で送る
ros2 topic pub -r 10 /sensor_1/data std_msgs/msg/Float64 "{data: 10.0}" >/dev/null 2>&1 &
sleep 3 # 収束待ち

# 3. 出力確認 (Check)
# 収束していれば 9.x ~ 10.x の値が出ているはず
# --field data オプションで数値だけを取り出す
out=$(timeout 2s ros2 topic echo --once /kf_state --field data)

# 取得できたか確認
[ -n "$out" ] || ng "$LINENO"

# 値の判定 (10.0に収束しているか)
# 浮動小数点の厳密一致は避けるが，10に近いことは確認する
echo "$out" | grep -E "9\.|10\." >/dev/null || ng "$LINENO"

# エラーがあれば終了コード1
[ "$res" = 0 ] && echo "Integration Test OK"
exit $res
