![test](https://github.com/Ichiyou1922/sensor_fusion_pkg/actions/workflows/ros2_test.yml/badge.svg)

# sensor_fusion_pkg

- ROS2(Humble)向けのセンサフュージョンパッケージ．
- 1次元温度センサ融合から始まり，任意次元，任意センサ数に拡張可能な汎用N次元カルマンフィルタノードを提供します．

## パッケージの内容

- 1D専用フィルタ
- N次元カルマンフィルタ
- 複数センサの同時処理に対応
- 観測行列，雑音行列のshape checkにより例外を投げます．
- pytestによる強固な単体テスト．
- `flake8`, `pep257`, `copyright`．
- `launch`ファイルを使用した容易な実行．

## 含まれるノード

#### noisy_sensor.py

- ランダムノイズを含むセンサ
- パラメータ
  - `variance`: ノイズの分散
  - `sensor_id`: 出力トピック番号

#### fusion_node.py

- 状態1次元
- センサ2本
- 固定の観測モデルによる温度推定
- 最小構成

#### generic_kf_node.py

- 汎用N次元カルマンフィルタノード
- 入力パラメータ
  - `dim_x`: int型/状態ベクトル次元
  - `dim_z`: int型/観測ベクトル次元
  - `sensor_topics`: list[str]型/各観測に対応するトピック名
  - `F`: list[list[float]]型/状態遷移行列(dim_x ×dim_x)
  - `Q`: list[list[float]]型/プロセス雑音共分散
  - `H`: list[list[float]]型/観測行列(dim_z×dim_x)
  - `R`: list[list[float]]型/観測雑音共分散(dim_z×dim_z)
  - `x0`: list[float]型/初期状態
  - `P0`: list[list[float]]型/初期共分散
- チェック機能
  - 行列サイズが一致しない場合`ValueError`
  - センサ数 $\neq$ dim_zの場合も`ValueError`
  - 全データはFloat64MultiArrayとして`/kf_state`にpublish

## 実行方法

- 1D温度融合

```bash
ros2 launch sensor_fusion_pkg fusion_system.launch.py
```

- 汎用KFの起動
`sensor_fusion_pkg/config/`内の`yaml`ファイルに設定値を与える．
- 最小構成例: 2センサ1次元状態を推定するKF

```yaml
generic_kf_node:
  ros__parameters:
    dim_x: 1
    dim_z: 2
    sensor_topics:
      - "/sensor_1/data"
      - "/sensor_2/data"

    # x_k = x_{k-1} とする（定常温度モデル）
    F: [1.0]
    H: [[1.0],
        [1.0]]
    Q: [0.01]
    R: [[1.0, 0.0],
        [0.0, 5.0]]

    x0: [0.0]
    P0: [[1.0]]

```

- 以下でlaunch

```bash
ros2 launch sensor_fusion_pkg generic_kf_system.launch.py
```

- 4次現状態: 例えば $[x, \dot x, y, \dot y]$ など
- 以下は既に用意された`generic_kf_4d2sens.yaml`

```yaml
generic_kf_node:
  ros__parameters:
    # 4 次元状態: [x, vx, y, vy]^T
    dim_x: 4
    # 観測は2次元: [x_meas, y_meas]^T
    dim_z: 2

    # センサトピック
    sensor_topics:
      - /sensor_1/data
      - /sensor_2/data

    # 状態遷移行列 F (4x4) をフラットなリストで
    F: [1.0, 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 1.0,
        0.0, 0.0, 0.0, 1.0]

    # プロセス雑音共分散 Q (4x4)
    Q: [0.01, 0.0,  0.0,  0.0,
        0.0,  0.1,  0.0,  0.0,
        0.0,  0.0,  0.01, 0.0,
        0.0,  0.0,  0.0,  0.1]

    # 観測行列 H (2x4)
    H: [1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0]

    # 観測雑音共分散 R (2x2)
    R: [1.0, 0.0,
        0.0, 5.0]

    # 初期状態 x0 (4,)
    x0: [0.0, 0.0, 0.0, 0.0]

    # 初期共分散 P0 (4x4)
    P0: [100.0, 0.0,   0.0,   0.0,
         0.0,   10.0,  0.0,   0.0,
         0.0,   0.0,   100.0, 0.0,
         0.0,   0.0,   0.0,   10.0]

```

- 使用するyamlファイルを選択(今回`generic_kf_4d2sens.yaml`)

```bash
$ ros2 launch sensor_fusion_pkg generic_kf_system.launch.py \
  param_file:=install/sensor_fusion_pkg/share/sensor_fusion_pkg/config/generic_kf_4d2sens.yaml
```

- `echo`などで確認

```bash
ros2 topic echo /kf_state
```

- publishされるトピック
  - /kf_state: Float64MultiArray型/事後状態推定値
  - /sensor_i/data: Float64型/観測された生データ

## よくあるエラー

1. 行列サイズの不一致

- `dim_z`に与えた数と`H`の行数が一致していない
- センサtopicの数が`dim_z`と一致していないなど

1. `Float64MultiArray`に複数の型がpublishされている

- 現在`generic_kf_node`に統一されているため改善されているはずです

## KalmanFilterの内部で保持するデータ類

1. 状態推定ベクトル: $x\in \mathbb{R}^{n}$
2. 共分散行列: $P\in \mathbb{R}^{n\times n}$
3. 状態遷移行列: $F\in \mathbb{R}^{n\times n}$
4. プロセス雑音共分散: $Q\in \mathbb{R}^{n\times n}$
5. 観測行列: $H\in \mathbb{R}^{m\times n}$
6. 観測雑音共分散: $R\in \mathbb{R}^{m\times m}$
7. 制御入力(option): $B$

## テスト環境

- Ubuntu-24.04.3-LTS
- ROS2-Humble
- Python: 3.10
- テスト実行コマンド例

```bash
colcon test --packages-select sensor_fusion_pkg
colcon test-result --all
```

## 権利について

- このソフトウェアパッケージは，3条項BSDライセンスの下，再頒布及び使用が許可されます．
- © 2025 Kazuha Mogi
