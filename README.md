![test](https://github.com/Ichiyou1922/sensor_fusion_pkg/actions/workflows/ros2_test.yml/badge.svg)

# sensor_fusion_pkg

ROS 2 (Humble) 向けのセンサフュージョンパッケージです．
1次元の温度センサ融合から始まり，任意次元のシステムや任意のセンサ数に拡張可能な，汎用N次元カルマンフィルタノードを提供します．

## パッケージの特徴

- **1D専用フィルタ**: 単純な1次元モデルの融合例を提供
- **N次元カルマンフィルタ**: 任意次元の状態空間モデルに対応
- **複数センサの同時処理**: 任意の数のセンサ入力をサポート
- **堅牢な設計**: 観測行列や雑音行列の次元整合性を厳密にチェックし，異常時は例外を送出
- **品質保証**:
  - `pytest` による十分なテストカバレッジ
  - `flake8`, `pep257` 準拠のコード
  - 著作権表記 (`copyright`) の整備
- **容易な実行**: `launch` ファイルを使用し，パラメータファイル (`yaml`) を切り替えるだけで実行可能

## 含まれるノード

### `noisy_sensor`
ランダムノイズを付加した模擬センサデータを配信します．
- **パラメータ**
  - `variance`: ノイズの分散
  - `sensor_id`: 出力トピックのID番号（例: `1` なら `sensor_1/data`）

### `fusion_node`
固定の1次元モデル（温度推定など）を使用した最小構成のデモ用ノードです．
- 状態: 1次元
- センサ: 2つの入力を融合

### `generic_kf_node`
設定ファイルにより挙動を変更できる汎用カルマンフィルタノードです．
- **入力パラメータ**
  - `dim_x` (int): 状態ベクトルの次元
  - `dim_z` (int): 観測ベクトルの次元
  - `sensor_topics` (list[str]): 各観測に対応するトピック名のリスト
  - `F` (list[float]): 状態遷移行列 (dim_x × dim_x)
  - `Q` (list[float]): プロセス雑音共分散行列 (dim_x × dim_x)
  - `H` (list[float]): 観測行列 (dim_z × dim_x)
  - `R` (list[float]): 観測雑音共分散行列 (dim_z × dim_z)
  - `x0` (list[float]): 初期状態ベクトル
  - `P0` (list[float]): 初期誤差共分散行列
- **機能**
  - 初期化時に行列サイズの整合性をチェック（不一致の場合は `ValueError`）
  - 全状態推定値を `Float64MultiArray` として `/kf_state` に配信

## トピック

### Subscribed Topics

- `sensor_1/data` (std_msgs/msg/Float64)
  - `fusion_node` が購読するセンサ1のデータ
- `sensor_2/data` (std_msgs/msg/Float64)
  - `fusion_node` が購読するセンサ2のデータ
- *Any topic specified in parameters* (std_msgs/msg/Float64)
  - `generic_kf_node` はパラメータ `sensor_topics` で指定されたリストのトピックをすべて購読します

### Published Topics

- `fused_estimate` (std_msgs/msg/Float64)
  - `fusion_node` が出力する融合推定値（状態の第1要素）
- `/kf_state` (std_msgs/msg/Float64MultiArray)
  - `generic_kf_node` が出力する状態ベクトル全体．トピック名はパラメータ `output_topic` で変更可能
- `sensor_{id}/data` (std_msgs/msg/Float64)
  - `noisy_sensor` が出力するノイズ付きデータ

## 実行方法

### 1D 温度融合デモ
最もシンプルな構成での実行例です．

```bash
ros2 launch sensor_fusion_pkg fusion_system.launch.py
```

### 汎用KFの実行
`sensor_fusion_pkg/config/` 内の `yaml` ファイルで設定を記述し，launch 引数として渡します．

#### 設定例 1: 最小構成 (2センサ・1次元状態)
```yaml
generic_kf_node:
  ros__parameters:
    dim_x: 1
    dim_z: 2
    sensor_topics:
      - "/sensor_1/data"
      - "/sensor_2/data"

    # 定常モデル: x_k = x_{k-1}
    F: [1.0]
    H: [[1.0],
        [1.0]]
    Q: [0.01]
    R: [[1.0, 0.0],
        [0.0, 5.0]]

    x0: [0.0]
    P0: [[1.0]]
```
実行コマンド:
```bash
ros2 launch sensor_fusion_pkg generic_kf_system.launch.py
```

#### 設定例 2: 4次元状態 (位置・速度 $x, \dot{x}, y, \dot{y}$)
すでに用意されている `generic_kf_4d2sens.yaml` を使用する例です．

```bash
ros2 launch sensor_fusion_pkg generic_kf_system.launch.py \
  param_file:=install/sensor_fusion_pkg/share/sensor_fusion_pkg/config/generic_kf_4d2sens.yaml
```

トピックの確認:
```bash
ros2 topic echo /kf_state
```

## よくあるエラーと対処

1. **行列サイズの不一致 (`ValueError`)**
   - パラメータ `dim_z` の値と `H` の行数が一致していない
   - `sensor_topics` リストの長さが `dim_z` と一致していない
   - `x0` や `P0` の要素数が `dim_x` と整合していない

2. **トピックの型エラー**
   - 現在，入力は全て `std_msgs/msg/Float64` を想定しています．

## 内部データ構造

KalmanFilter クラスは以下の行列・ベクトルを保持します．

1. **状態推定ベクトル**: $x \in \mathbb{R}^{n}$
2. **誤差共分散行列**: $P \in \mathbb{R}^{n \times n}$
3. **状態遷移行列**: $F \in \mathbb{R}^{n \times n}$
4. **プロセス雑音共分散**: $Q \in \mathbb{R}^{n \times n}$
5. **観測行列**: $H \in \mathbb{R}^{m \times n}$
6. **観測雑音共分散**: $R \in \mathbb{R}^{m \times m}$
7. **制御入力行列 (Optional)**: $B$

## テスト環境

- **OS**: Ubuntu 24.04.3 LTS
- **ROS Distro**: ROS 2 Humble
- **Python**: 3.10+

テスト実行コマンド:
```bash
colcon test --packages-select sensor_fusion_pkg
colcon test-result --all
```

## ライセンス

本パッケージは **BSD-3-Clause License** の下で公開されています．
© 2025 Kazuha Mogi
