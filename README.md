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
- flake8, pep257, copyright．
- launchファイルを使用した容易な実行．

## 含まれるノード
#### noisy_sensor.py
- ランダムノイズを含むセンサ
- パラメータ
  - variance: ノイズの分散
  - sensor_id: 出力トピック番号
#### fusion_node.py
- 状態1次元
- センサ2本
- 固定の観測モデルによる温度推定
- 最小構成
#### generic_kf_node.py
- 汎用N次元カルマンフィルタノード
- 入力パラメータ
  - dim_x: int型/状態ベクトル次元
  - dim_z: int型/観測ベクトル次元
  - sensor_topics: list[str]型/各観測に対応するトピック名
  - F: list[list[float]]型/状態遷移行列(dim_x ×dim_x)
  - Q: list[list[float]]型/プロセス雑音共分散
  - H: list[list[float]]型/観測行列(dim_z×dim_x)
  - R: list[list[float]]型/観測雑音共分散(dim_z×dim_z)
  - x0: list[float]型/初期状態
  - P0: list[list[float]]型/初期共分散
- チェック機能
  - 行列サイズが一致しない場合ValueError
  - センサ数 $\neq$ dim_zの場合もValueError
  - 全データはFloat64MultiArrayとして`/kf_state`にpublish

## 実行方法
- 1D温度融合
```bash
$ ros2 launch sensor_fusion_pkg fusion_system.launch.py
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
$ ros2 launch sensor_fusion_pkg generic_kf_system.launch.py
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
- 使用するyamlファイルを選択(今回generic_kf_4d2sens.yaml)
```bash
$ ros2 launch sensor_fusion_pkg generic_kf_system.launch.py \
  param_file:=install/sensor_fusion_pkg/share/sensor_fusion_pkg/config/generic_kf_4d2sens.yaml
```
- echoなどで確認
```bash
$ ros2 topic echo /kf_state
```
- publishされるトピック
  - /kf_state: Float64MultiArray型/事後状態推定値
  - /sensor_i/data: Float64型/観測された生データ

## よくあるエラー
1. 行列サイズの不一致
- dim_zに与えた数とHの行列が一致していない
- センサtopicの数がdim_zと一致していないなど
2. Float64MultiArrayに複数の型がpublishされている
- 現在generic_kf_nodeに統一されているため改善されているはずです
## テストについて
- 本パッケージは以下の性質をpytestにより検証している．
  - predictで共分散が増加すること．
  - updateで共分散が減少すること．
  - 2センサのノイズ反転で推定バイアスが反転すること．
  - shape mismatchを正しく検出すること．
  - Nステップで推定値が理論通りに収束すること．
- テストコード: `test_kalman`
- CIでは以下も確認済み．
  - flake8
  - pep257
  - copyright

## KalmanFilterの内部で保持するデータ類
1. 状態推定ベクトル: $x\in \mathbb{R}^{n}$
2. 共分散行列: $P\in \mathbb{R}^{n\times n}$
3. 状態遷移行列: $F\in \mathbb{R}^{n\times n}$
4. プロセス雑音共分散: $Q\in \mathbb{R}^{n\times n}$
5. 観測行列: $H\in \mathbb{R}^{m\times n}$
6. 観測雑音共分散: $R\in \mathbb{R}^{n\times m}$
7. 制御入力(option): $B$

## 理論
1. 状態方程式
```math
x_{k}=Fx_{k-1}+Bu_{k}+w_{k}
```
- 状態ベクトル: $x_{k}\in \mathbb{R^{n}}$
    - 推定したい量を全て並べたもの．
- 状態遷移行列: $F\in \mathbb{R^{n\times n}}$
    - 1ステップ前の状態が，今の状態にどう線形変換されるかを記述する行列．
- 制御入力 $u_{k}$ と制御行列 $\mathbf{B}$
    - 何かしらの司令
    - 今回制御入力を使わないため，どちらも0．
- プロセス雑音 $w_{k}$
    - 平均0のガウス雑音と仮定する．
```math
w_{k}\sim \mathcal{N}(0,\mathcal{Q})
```
2. 観測方程式
```math
z_{k}=Hx_{k}+v_{k}
```
- 観測ベクトル: $z_{k}\in \mathbb{R^{m}}$
    - センサが返す値．
    - 状態の次元とは異なる可能性もある．例えば状態は3次元でも，センサがそのうち1つの次元についてしかわからない場合など．
- 観測行列: $H\in \mathbb{R^{m\times n}}$
    - 状態ベクトルから観測値を取り出す．
- 観測雑音: $v_{k}$
    - センサのノイズ->ガウス雑音
```math
v_{k}\sim \mathcal{N}(0, \mathcal{R})
```
3. 予測
- 目的は時刻kにおける状態分布 $p(x_{k}|z_{1:k})$ を計算すること．
3.1. 予測ステップ
- 1ステップ前の事後分布 $x_{k-1}|z_{1:k-1}\sim \mathcal{N}(\mu_{k-1},\mathcal{P_{k-1}})$ が既知として，コレを状態方程式に通せば，時刻kの事前分布が得られる．
```math
x_{k}=Fx_{k-1}+Bu_{k}+w_{k}
```

- ガウス分布に線形変換をかけてもガウス分布のままだから，
```math
x_{k}|z_{1:k-1}\sim \mathcal{\mu_{k}^{-},\mathcal{P}_{k}^{-}}
```

- 平均(予測値)は
```math
\mu_{k}^{-}=F\mu_{k-1}+Bu_{k}
```

- 共分散(予測の不確かさ)は
```math
\mathcal{P}_{k}^{-}=F\mathcal{P}_{k-1}F^{T}+\mathcal{Q}
```

- 右辺第1項目
```math
F\mathcal{P}_{k-1}F^{T}
```

では以前の推定の不確かさがモデルに従って広がる．
- $\mathcal{Q}$ が大きければフィルタはモデルを信用しない．逆なら信用する．
- 今回 $u_{k}, B = 0$ だから，予測ステップは
```math
\mu_{k}^{-}=F\mu_{k-1}
```
```math
\mathcal{P}_{k}^{-}=F\mathcal{P}_{k-1}F^{T}+\mathcal{Q}
```

4. 更新
- 観測予測
```math
\hat z_{k}=\mathcal{H}\mu_{k}^{-}
```

  - 観測値が来る前にフィルタが予測するもの．
- イノベーション
```math
y_{k}=z_{k}-\hat z_{k}
```

  - 予測と現実のズレ
- イノベーション共分散
```math
\mathcal{S}_{k}=\mathcal{H}\mathcal{P}_{k}^{-}\mathcal{H}^{T}+\mathcal{R}
```

  - この値が大きいほど観測の信頼度が低い．
- カルマンゲイン
```math
\mathcal{K}_{k}=\mathcal{P}_{k}^{-}\mathcal{H}^{T}\mathcal{S}_{k}^{-1}
```

  - 予測の不確かさ $P_{k}^{-1}$ が大きい-> $K_{k}$ が大きくなる．
  - 観測の不確かさ $R$ が大きい-> $S_{k}$ が大きい-> $K_{k}$ が小さくなる．
- 最終的な事後平均
```math
\mu_{k}=\mu_{k}^{-}+\mathcal{K}_{k}y_{k}
```

  - 予測に「ズレ×どれだけ観測を信じるか」を足す．
- 最終的な事後共分散
```math
\mathcal{P}_{k}=(\mathcal{I}-\mathcal{K}_{k}\mathcal{H}\mathcal{P}_{k}^{-})
```

  - 更新後の不確かさを表す．

- 予測ステップ: 時間発展によって事前分布を計算する．
- 更新ステップ: 観測に基づいて事後分布を計算する．
- この2つの操作を繰り返す．

## 必要なソフトウェア
- Ubuntu-24.04.3-LTS
- ROS2-Humble
- Python-Package
  - numpy
  - pytest
  - flake8
  - pep257

## テスト環境
- Ubuntu-24.04.3-LTS
- ROS2-Humble
- Python: 3.10
- テスト実行コマンド例
```bash
$ colcon test --packages-select sensor_fusion_pkg
$ colcon test-result --all
```

## 権利について
- このソフトウェアパッケージは，3条項BSDライセンスの下，再頒布及び使用が許可されます．
- © 2025 Kazuha Mogi
