import numpy as np
from sensor_fusion_pkg.kalman_filters import KalmanFilter


# predict(), update()のテスト
def test_kalman_1d_no_noise_2():
    # 1次元の理想的なモデル
    x0 = np.array([0.0])        # 初期状態 x0 = 0
    P0 = np.array([[1.0]])      # 初期共分散 P0 = 1
    F = np.array([[1.0]])      # 定常モデル x_k = x_{k-1}
    Q = np.array([[0.0]])      # プロセスノイズなし
    H = np.array([[1.0]])      # 状態をそのまま観測
    R = np.array([[0.0]])      # 観測ノイズなし

    kf = KalmanFilter(x0, P0, F, Q, H, R)

    z = np.array([10.0])        # 観測値 z = 10

    # 予測＋更新
    kf.predict()
    kf.update(z)

    # 理論的に x は 10，P は 0 になっていなければならない
    assert np.allclose(kf.x, np.array([10.0]))
    assert np.allclose(kf.P, np.array([[0.0]]))
