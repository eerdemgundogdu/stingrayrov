#!/usr/bin/env python3
"""Su alti araci (ROV) icin Genisletilmis Kalman Filtresi (EKF) konum kestirimi.

Su altinda GNSS ve tekerlek odometrisi bulunmadigi icin konum tahmini:
  - Hidrodinamik itki + kuadratik su direnci (Fossen) hiz modeli,
  - IMU Z-eksen jiroskop acisal hizi (bias kestirimli),
  - Hidrostatik basinc sensorunden gelen derinlik (z),
  - Sualti kamerasiyla tespit edilen bilinen gorev nesnelerinin (gecis kapisi,
    samandira, boru hatti referans noktasi) mesafe/yonelim (range/bearing)
    guncellemeleri
birlestirilerek yapilir.

Durum vektoru x = [px, py, pz, v_surge, theta, gyro_bias]  (6 durum)
  px, py    : yatay havuz konumu (m)
  pz        : derinlik (m, asagi pozitif)
  v_surge   : govde ekseninde ileri hiz (m/s)
  theta     : sapma / yaw acisi (rad, [-pi, pi])
  gyro_bias : IMU Z-jiroskop kaymasi (rad/s)
"""

import math
from typing import Tuple

import numpy as np


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class UnderwaterEKF:

    def __init__(self,
                 mass_kg: float = 12.0,
                 max_thrust_n: float = 35.0,
                 quad_drag: float = 18.0,
                 lin_drag: float = 4.0,
                 sigma_accel: float = 0.15,
                 sigma_depth_vel: float = 0.08,
                 sigma_gyro: float = 0.015,
                 sigma_bias: float = 0.0005,
                 sigma_depth: float = 0.02,
                 sigma_heading: float = math.radians(2.0),
                 sigma_range: float = 0.08,
                 sigma_bearing: float = math.radians(3.0)):
        self.mass = mass_kg
        self.max_thrust = max_thrust_n
        self.quad_drag = quad_drag
        self.lin_drag = lin_drag

        self.x = np.zeros(6, dtype=np.float64)
        self.P = np.diag([
            0.02 ** 2,
            0.02 ** 2,
            0.02 ** 2,
            0.05 ** 2,
            math.radians(2.0) ** 2,
            math.radians(1.0) ** 2,
        ])

        self.sigma_accel = sigma_accel
        self.sigma_depth_vel = sigma_depth_vel
        self.sigma_gyro = sigma_gyro
        self.sigma_bias = sigma_bias

        self.R_depth = np.array([[sigma_depth ** 2]])
        self.R_heading = np.array([[sigma_heading ** 2]])
        self.R_landmark = np.diag([sigma_range ** 2, sigma_bearing ** 2])

        # 2 serbestlik dereceli chi-kare %99 esigi (su alti yansima/kabarcik aykiri olcum kapisi)
        self.landmark_gate = 9.21
        self.rejected_landmarks = 0

    def set_pose(self, px: float, py: float, pz: float, theta: float) -> None:
        self.x[0:3] = [px, py, pz]
        self.x[4] = wrap_angle(theta)

    @property
    def pose(self) -> Tuple[float, float, float, float]:
        """(px, py, pz, theta_rad) dondurur."""
        return float(self.x[0]), float(self.x[1]), float(self.x[2]), float(self.x[4])

    @property
    def velocity(self) -> float:
        return float(self.x[3])

    @property
    def gyro_bias(self) -> float:
        return float(self.x[5])

    def steady_state_velocity(self, u_surge: float) -> float:
        """Verilen surge komutunda [-1, 1] hidrodinamik denge hizini hesaplar."""
        u = max(-1.0, min(1.0, float(u_surge)))
        f_net = abs(u) * self.max_thrust
        if f_net <= 0.0:
            return 0.0
        # quad_drag * v^2 + lin_drag * v - f_net = 0
        disc = self.lin_drag ** 2 + 4.0 * self.quad_drag * f_net
        v_mag = (-self.lin_drag + math.sqrt(disc)) / (2.0 * self.quad_drag)
        return math.copysign(v_mag, u)

    def predict(self, u_surge: float, gyro_z: float, dt: float) -> None:
        """Hidrodinamik itki-direnc modeli ve jiroskop ile durum tahmini yapar."""
        if dt <= 0.0:
            return

        px, py, pz, v, th, b = self.x
        u = max(-1.0, min(1.0, float(u_surge)))

        thrust = u * self.max_thrust
        drag = self.quad_drag * abs(v) * v + self.lin_drag * v
        accel = (thrust - drag) / self.mass
        dv_dv = -(2.0 * self.quad_drag * abs(v) + self.lin_drag) / self.mass

        w = gyro_z - b
        c, s = math.cos(th), math.sin(th)

        v_next = v + accel * dt
        self.x = np.array([
            px + v * c * dt,
            py + v * s * dt,
            pz,
            v_next,
            wrap_angle(th + w * dt),
            b,
        ])

        F = np.eye(6)
        F[0, 3] = c * dt
        F[0, 4] = -v * s * dt
        F[1, 3] = s * dt
        F[1, 4] = v * c * dt
        F[3, 3] = 1.0 + dv_dv * dt
        F[4, 5] = -dt

        Q = np.zeros((6, 6))
        pos_var = (0.5 * self.sigma_accel * dt * dt) ** 2
        Q[0, 0] = pos_var + (0.02 * dt) ** 2
        Q[1, 1] = pos_var + (0.02 * dt) ** 2
        Q[2, 2] = (self.sigma_depth_vel * dt) ** 2
        Q[3, 3] = (self.sigma_accel * dt) ** 2
        Q[4, 4] = (self.sigma_gyro * dt) ** 2
        Q[5, 5] = (self.sigma_bias ** 2) * dt

        self.P = F @ self.P @ F.T + Q

    def _joseph_update(self, y: np.ndarray, H: np.ndarray, R: np.ndarray) -> np.ndarray:
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[4] = wrap_angle(self.x[4])
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        return S

    def update_depth(self, depth_m: float) -> None:
        """Hidrostatik basinc sensorunden gelen derinlik olcumu ile gunceller."""
        H = np.zeros((1, 6))
        H[0, 2] = 1.0
        y = np.array([depth_m - self.x[2]])
        self._joseph_update(y, H, self.R_depth)

    def update_heading(self, heading_rad: float) -> None:
        """Boru hatti ekseni veya pusula referansi ile yonelim ve gyro bias gunceller."""
        H = np.zeros((1, 6))
        H[0, 4] = 1.0
        y = np.array([wrap_angle(heading_rad - self.x[4])])
        self._joseph_update(y, H, self.R_heading)

    def update_landmark(self, lx: float, ly: float,
                        range_m: float, bearing_rad: float) -> bool:
        """Bilinen su alti nesnesi (gecis kapisi, samandira) mesafe/aci guncellemesi.

        Mahalanobis kapisi ile yansima/kabarcik kaynakli aykiri olcumleri reddeder.
        """
        dx = lx - self.x[0]
        dy = ly - self.x[1]
        q = dx * dx + dy * dy
        dist = math.sqrt(q)
        if dist < 1e-4:
            return False

        pred_bearing = wrap_angle(math.atan2(dy, dx) - self.x[4])
        y = np.array([range_m - dist, wrap_angle(bearing_rad - pred_bearing)])

        H = np.zeros((2, 6))
        H[0, 0] = -dx / dist
        H[0, 1] = -dy / dist
        H[1, 0] = dy / q
        H[1, 1] = -dx / q
        H[1, 4] = -1.0

        S = H @ self.P @ H.T + self.R_landmark
        d2 = float(y.T @ np.linalg.inv(S) @ y)
        if d2 > self.landmark_gate:
            self.rejected_landmarks += 1
            return False

        self._joseph_update(y, H, self.R_landmark)
        return True
