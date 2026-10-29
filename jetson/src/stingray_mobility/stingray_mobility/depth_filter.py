"""Derinlik icin Kalman filtresi.

Durum vektoru x = [z, w]
  z : derinlik (m, asagi pozitif)
  w : dikey hiz (m/s)

Model: sabit hiz, ivme beyaz gurultu olarak modellenir.
  z_k = z_{k-1} + w_{k-1} * dt
  w_k = w_{k-1}
Olcum: sadece derinlik sensoru (z).

Analog basinc sensoru ~10 Hz ve gurultulu oldugu icin PID'in turev
terimi ham derinlikten hesaplaninca titresiyordu. Filtre hem
yumusatilmis derinligi hem de dikey hizi veriyor; kontrolcu turev
terimini dogrudan w'den aliyor.
"""

import numpy as np


class DepthKalmanFilter:

    def __init__(self, accel_std=0.5, meas_std=0.02):
        # accel_std: modellenmemis dikey ivme (m/s^2)
        # meas_std : sensor olcum gurultusu (m)
        self.q = accel_std ** 2
        self.R = np.array([[meas_std ** 2]])
        self.H = np.array([[1.0, 0.0]])
        self.x = np.zeros(2)
        self.P = np.diag([1.0, 1.0])
        self.initialized = False

    def reset(self, z0=0.0):
        self.x = np.array([z0, 0.0])
        self.P = np.diag([meas_var_floor(self.R), 0.25])
        self.initialized = True

    def predict(self, dt):
        if not self.initialized or dt <= 0.0:
            return
        F = np.array([[1.0, dt],
                      [0.0, 1.0]])
        # Ayrik beyaz gurultu ivme modeli
        G = np.array([[0.5 * dt * dt], [dt]])
        Q = G @ G.T * self.q
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z_meas):
        if not self.initialized:
            self.reset(z_meas)
            return
        y = z_meas - (self.H @ self.x)[0]          # inovasyon
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T / S[0, 0]
        self.x = self.x + K[:, 0] * y
        # Joseph formu: sayisal olarak P'yi simetrik/pozitif tutar
        I_KH = np.eye(2) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

    @property
    def depth(self):
        return float(self.x[0])

    @property
    def rate(self):
        return float(self.x[1])


def meas_var_floor(R):
    return float(R[0, 0])
