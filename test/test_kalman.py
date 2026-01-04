# SPDX-FileCopyrightText: 2025 Kazuha Mogi <mogi2fruits.kazu@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause
import numpy as np
import pytest

from sensor_fusion_pkg.filters.kalman_filters import KalmanFilter

def print_matrix(name, mat):
    print(f"\n[ {name}]")
    print(mat)

# predict(), update()のテスト
def test_kalman_1d_no_noise():
    print("\n" + "="*60)
    print(" TEST CASE: 1D Ideal Model (No Noise)")
    print("="*60)

    # 1次元の理想的なモデル
    x0 = np.array([0.0])        # 初期状態 x0 = 0
    P0 = np.array([[1.0]])      # 初期共分散 P0 = 1
    F = np.array([[1.0]])      # 定常モデル x_k = x_{k-1}
    Q = np.array([[0.0]])      # プロセスノイズなし
    H = np.array([[1.0]])      # 状態をそのまま観測
    R = np.array([[0.0]])      # 観測ノイズなし

    print_matrix("Initial State (x0)", x0)
    print_matrix("Initial Covariance (P0)", P0)
    print_matrix("Process Noise (Q)", Q)
    print_matrix("Measurement Noise (R)", R)

    kf = KalmanFilter(x0, P0, F, Q, H, R)
    z = np.array([10.0])        # 観測値 z = 10

    print(f"\n>>>  Input Measurement (z): {z}")

    # 予測＋更新
    kf.predict()
    print("\n--- After Predict ---")
    print(f"Pred State: {kf.x}")
    print(f"Pred Cov  :\n{kf.P}")

    kf.update(z)
    print("\n--- After Update ---")
    print(f"Post State: {kf.x}")
    print(f"Post Cov  :\n{kf.P}")

    expected_x = np.array([10.0])
    expected_P = np.array([[0.0]])

    print(f"\n>> Expectation: x={expected_x}, P={expected_P}")
    
    # 理論的に x は 10，P は 0 になっていなければならない
    assert np.allclose(kf.x, np.array([10.0]))
    assert np.allclose(kf.P, np.array([[0.0]]))

    print("\n[ RESULT ] OK")


def _valid_params_1d():
    # 1次元の最小構成（正常系）
    x0 = np.array([0.0])
    P0 = np.array([[1.0]])
    F = np.array([[1.0]])
    Q = np.array([[0.1]])
    H = np.array([[1.0]])
    R = np.array([[1.0]])
    return x0, P0, F, Q, H, R


def test_init_raises_on_x0_shape_mismatch():
    print("\n" + "="*60)
    print(" TEST CASE: Init Raises on x0 Shape Mismatch")
    print("="*60)

    x0, P0, F, Q, H, R = _valid_params_1d()
    # x0 を(2,) にしてわざと壊す
    bad_x0 = np.array([0.0, 1.0])

    print(f"Input x0 shape: {bad_x0.shape} (Invalid)")
    print(f"Input P0 shape: {P0.shape}     (Valid 1x1)")
    print("Expected: ValueError due to shape mismatch")

    with pytest.raises(ValueError):
        KalmanFilter(bad_x0, P0, F, Q, H, R)
    print("\n[ RESULT ] Raised ValueError as expected")


def test_init_raises_on_P0_shape_mismatch():
    print("\n" + "="*60)
    print(" TEST CASE: Init Raises on P0 Shape Mismatch")
    print("="*60)

    x0, P0, F, Q, H, R = _valid_params_1d()
    # P0 を(2,2) にしてわざと壊す
    bad_P0 = np.eye(2)

    print(f"Input x0 shape: {x0.shape}     (Valid 1D)")
    print(f"Input P0 shape: {bad_P0.shape} (Invalid 2x2)")
    print("Expected: ValueError due to shape mismatch")

    with pytest.raises(ValueError):
        KalmanFilter(x0, bad_P0, F, Q, H, R)
    print("\n[ RESULT ] Raised ValueError as expected")


def test_init_raises_on_H_R_shape_mismatch():
    print("\n" + "="*60)
    print(" TEST CASE: Init Raises on H/R Shape Mismatch")
    print("="*60)

    x0, P0, F, Q, H, R = _valid_params_1d()
    # H の行数とR のサイズを矛盾させる
    bad_H = np.array([[1.0],
                      [1.0]])      # 観測2次元だと主張
    
    print(f"Input H shape: {bad_H.shape} (2 rows -> implies 2D measurement)")
    print(f"Input R shape: {R.shape}     (1x1 -> implies 1D measurement)")
    print("Expected: ValueError due to H/R dimension mismatch")

    # なのに R は 1x1 のまま
    with pytest.raises(ValueError):
        KalmanFilter(x0, P0, F, Q, bad_H, R)
    print("\n[ RESULT ] Raised ValueError as expected")


# 以下2つのセンサを用いたテスト
def test_two_sensors_more_precise_sensor_dominates():
    print("\n" + "="*60)
    print(" TEST CASE: Two Sensors (Precise Sensor Dominates)")
    print("="*60)

    # 1次元状態，2本センサ
    x0 = np.array([0.0])
    P0 = np.array([[1e6]])         # 事前はほぼ無情報
    F = np.array([[1.0]])
    Q = np.array([[0.0]])
    H = np.array([[1.0],
                  [1.0]])

    # センサ1が高精度，センサ2はノイズ大
    R = np.array([[1.0, 0.0],
                  [0.0, 9.0]])

    print_matrix("Initial State (x0)", x0)
    print_matrix("Initial Covariance (P0)", P0)
    print_matrix("Measurement Noise (R)", R)

    kf = KalmanFilter(x0, P0, F, Q, H, R)

    z = np.array([10.0, 30.0])
    print(f"\n>>> Input Measurement (z): {z}")

    kf.predict()
    kf.update(z)

    print("\n--- After Update ---")
    print(f"Post State: {kf.x}")
    print(f"Post Cov  :\n{kf.P}")

    x_hat = float(kf.x[0])
    print(f"\nResult x_hat: {x_hat}")

    # 10 と 30 の間にはあるはず
    assert 10.0 < x_hat < 30.0
    print("CHECK: 10.0 < x_hat < 30.0 -> OK")

    # センサ1（10）の方に強く寄っていること
    assert abs(x_hat - 10.0) < abs(x_hat - 30.0)
    print("CHECK: Closer to 10.0 than 30.0 -> OK")

    # 平均 20 よりも 10 側に寄っていること（重み付き平均の性質）
    assert x_hat < 20.0
    print("CHECK: x_hat < 20.0 -> OK")

    print("\n[ RESULT ] OK")


def test_two_sensors_swapping_noise_swaps_bias_direction():
    print("\n" + "="*60)
    print(" TEST CASE: Two Sensors (Noise Swapped)")
    print("="*60)

    x0 = np.array([0.0])
    P0 = np.array([[1e6]])
    F = np.array([[1.0]])
    Q = np.array([[0.0]])
    H = np.array([[1.0],
                  [1.0]])

    # 今度はセンサ2が高精度
    R = np.array([[9.0, 0.0],
                  [0.0, 1.0]])

    print_matrix("Initial State (x0)", x0)
    print_matrix("Initial Covariance (P0)", P0)
    print_matrix("Measurement Noise (R)", R)

    kf = KalmanFilter(x0, P0, F, Q, H, R)

    z = np.array([10.0, 30.0])
    print(f"\n>>> Input Measurement (z): {z}")

    kf.predict()
    kf.update(z)

    print("\n--- After Update ---")
    print(f"Post State: {kf.x}")
    print(f"Post Cov  :\n{kf.P}")

    x_hat = float(kf.x[0])
    print(f"\nResult x_hat: {x_hat}")

    # 10 と 30 の間
    assert 10.0 < x_hat < 30.0
    print("CHECK: 10.0 < x_hat < 30.0 -> OK")

    # 今度は 30 の方に強く寄っていること
    assert abs(x_hat - 30.0) < abs(x_hat - 10.0)
    print("CHECK: Closer to 30.0 than 10.0 -> OK")

    # 平均 20 よりも 30 側に寄っていること
    assert x_hat > 20.0
    print("CHECK: x_hat > 20.0 -> OK")

    print("\n[ RESULT ] OK")


# Q>0で予測においてPが増えるかどうかのテスト
def test_predict_increases_covariance_when_process_noise_positive():
    print("\n" + "="*60)
    print(" TEST CASE: Predict Increases Covariance (Q > 0)")
    print("="*60)

    # 1次元モデル
    x0 = np.array([0.0])
    P0 = np.array([[1.0]])
    F = np.array([[1.0]])    # 単位遷移
    Q = np.array([[0.5]])    # プロセスノイズあり
    H = np.array([[1.0]])
    R = np.array([[1.0]])

    kf = KalmanFilter(x0, P0, F, Q, H, R)

    P_before = kf.P.copy()
    print(f"P_before: {P_before[0,0]}")

    kf.predict()
    P_after = kf.P.copy()
    print(f"P_after : {P_after[0,0]}")

    # Q > 0 なので P は必ず増えているはず
    assert P_after[0, 0] > P_before[0, 0]
    print(f"CHECK: {P_after[0,0]} > {P_before[0,0]} (Q={Q[0,0]}) -> OK")

    print("\n[ RESULT ] OK")


# Q>0で観測後にPが減るかどうか
def test_update_reduces_covariance_relative_to_prediction():
    print("\n" + "="*60)
    print(" TEST CASE: Update Reduces Covariance")
    print("="*60)

    x0 = np.array([0.0])
    P0 = np.array([[2.0]])    # 少し大きめの不確かさ
    F = np.array([[1.0]])
    Q = np.array([[0.0]])    # プロセスノイズなし
    H = np.array([[1.0]])
    R = np.array([[1.0]])    # 観測ノイズあり

    kf = KalmanFilter(x0, P0, F, Q, H, R)

    # 予測
    kf.predict()
    P_pred = kf.P.copy()
    print(f"P_pred: {P_pred[0,0]}")

    # 適当な観測値
    z = np.array([1.0])

    # 更新
    kf.update(z)
    P_post = kf.P.copy()
    print(f"P_post: {P_post[0,0]}")

    # 観測を取り込んだあとは，不確かさは減っていなければならない
    assert P_post[0, 0] < P_pred[0, 0]
    print(f"CHECK: {P_post[0,0]} < {P_pred[0,0]} -> OK")

    print("\n[ RESULT ] OK")


# 収束テスト: ステップを繰り返すごとに推定値は収束し，共分散は小さくなるはず
def test_kalman_1d_converges_to_constant_measurement():
    print("\n" + "="*60)
    print(" TEST CASE: 1D Convergence Test")
    print("="*60)

    # 真の状態は常に 10 とする
    true_value = 10.0
    print(f"True Value: {true_value}")

    # 初期状態は外した値にしておく
    x0 = np.array([0.0])
    P0 = np.array([[100.0]])   # かなり不確か
    print(f"Initial State x0: {x0}")
    print(f"Initial Cov P0  : {P0}")

    F = np.array([[1.0]])      # 定常モデル
    Q = np.array([[0.1]])      # 少しだけプロセスノイズ
    H = np.array([[1.0]])      # 状態をそのまま観測
    R = np.array([[1.0]])      # 観測ノイズあり

    kf = KalmanFilter(x0, P0, F, Q, H, R)

    z = np.array([true_value])

    print("Running 50 iterations...")
    # 複数ステップ回す
    for _ in range(50):
        kf.predict()
        kf.update(z)

    x_hat = float(kf.x[0])
    P_hat = float(kf.P[0, 0])

    print(f"Final Estimated x: {x_hat}")
    print(f"Final Covariance P: {P_hat}")

    # 推定値は真値の近くまで収束しているはず
    assert abs(x_hat - true_value) < 1e-2
    print("CHECK: Converged to true_value (err < 1e-2) -> OK")

    # 共分散も十分小さくなっているはず
    assert P_hat < 1.0
    print("CHECK: P_hat < 1.0 -> OK")

    print("\n[ RESULT ] OK")
