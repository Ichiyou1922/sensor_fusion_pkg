import numpy as np
from typing import Optional

class KalmanFilter:
    def __init__(self, x0, P0, F, Q, H, R):
        dtype = np.float64
        x0 = np.asarray(x0, dtype=dtype)
        P0 = np.asarray(P0, dtype=dtype)
        F = np.asarray(F, dtype=dtype)
        Q = np.asarray(Q, dtype=dtype)
        H = np.asarray(H, dtype=dtype)
        R = np.asarray(R, dtype=dtype)

        n = x0.shape[0]
        m = H.shape[0]

