import numpy as np

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

        if x0.shape != (n,):
            raise ValueError(f"x0 must have shape ({n},), got {x0.shape}")
        if P0.shape != (n, n):
            raise ValueError(f"P0 must have shape ({n}, {n}), got {P0.shape}")
        if F.shape != (n, n):
            raise ValueError(f"F must have shape ({n}, {n}), got {F.shape}")
        if Q.shape != (n, n):
            raise ValueError(f"Q must have shape ({n}, {n}), got {Q.shape}")
        if H.shape != (m, n):
            raise ValueError(f"H must have shape ({m}, {n}), got {H.shape}")
        if R.shape != (m, m):
            raise ValueError(f"R must have shape ({m}, {m}), got {R.shape}")

        self.x = x0
        self.P = P0
        self.F = F
        self.Q = Q
        self.H = H
        self.R = R
        self.dim_x = n
        self.dim_z = m

    def predict(self):
        raise NotImplementedError

    def update(self, z):
        raise NotImplementedError
