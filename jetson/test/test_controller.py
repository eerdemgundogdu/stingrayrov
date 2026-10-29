"""PID denetleyici (anti-windup + Kalman hiz turevi) ve 6-itici (45° X-config + 2 dikey)
guc dagitim matrisi icin donanimsiz test.

Calistirma (repo kokunden):
    python3 jetson/test/test_controller.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src',
                                'stingray_mobility'))
from stingray_mobility import config  # noqa: E402
from stingray_mobility.utils import angle_diff, clamp  # noqa: E402


class StandalonePID:
    """controller_node.PID ile birebir ayni kontrol kanunu (rclpy bagimsiz)."""

    def __init__(self, kp, ki, kd, max_output, max_integral=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.max_integral = max_integral or (max_output * 0.5)
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_derivative = 0.0

    def compute(self, error, dt, error_rate=None):
        p_term = self.kp * error
        self._integral = clamp(
            self._integral + error * dt, -self.max_integral, self.max_integral)
        i_term = self.ki * self._integral
        if error_rate is not None:
            self._prev_derivative = error_rate
        else:
            raw_d = (error - self._prev_error) / dt
            self._prev_derivative = 0.3 * self._prev_derivative + 0.7 * raw_d
        d_term = self.kd * self._prev_derivative
        self._prev_error = error
        return clamp(p_term + i_term + d_term, -self.max_output, self.max_output)


def mix_thrusters(surge_cmd, sway_cmd, heave_cmd, yaw_cmd):
    cmd_vec = [surge_cmd, sway_cmd, heave_cmd, yaw_cmd]
    pwm_out = []
    for row in config.THRUSTER_MATRIX:
        thrust = sum(row[i] * cmd_vec[i] for i in range(4))
        pwm = int(clamp(config.PWM_NEUTRAL + int(thrust),
                        config.PWM_MIN, config.PWM_MAX))
        pwm_out.append(pwm)
    return pwm_out


def test_thruster_matrix_orthogonality():
    M = np.array(config.THRUSTER_MATRIX, dtype=float)
    assert M.shape == (6, 4)

    # Sadece surge verildiginde dikey motorlar (M4, M5) 1500 us kalmali
    pwm_surge = mix_thrusters(200.0, 0.0, 0.0, 0.0)
    assert pwm_surge[4] == 1500 and pwm_surge[5] == 1500
    assert pwm_surge[0] == pwm_surge[1] > 1500
    assert pwm_surge[2] == pwm_surge[3] < 1500

    # Sadece heave (dalis) verildiginde yatay motorlar (M0..M3) 1500 us kalmali
    pwm_heave = mix_thrusters(0.0, 0.0, 250.0, 0.0)
    assert pwm_heave[:4] == [1500, 1500, 1500, 1500]
    assert pwm_heave[4] == 1750 and pwm_heave[5] == 1750

    # Sutunlar (Surge, Sway, Heave, Yaw) birbirine dik (ortogonal) olmali
    gram = M.T @ M
    off_diag = gram - np.diag(np.diag(gram))
    assert float(np.max(np.abs(off_diag))) < 1e-9


def test_pid_anti_windup_and_kalman_damping():
    pid = StandalonePID(
        config.DEPTH_KP, config.DEPTH_KI, config.DEPTH_KD,
        config.DEPTH_MAX_OUTPUT)

    # Uzun sureli buyuk hata altinda integral terimi sinirlanmali (anti-windup)
    for _ in range(500):
        out = pid.compute(error=2.0, dt=0.02, error_rate=0.0)
    assert out == config.DEPTH_MAX_OUTPUT
    assert abs(pid._integral) <= pid.max_integral + 1e-9

    # Hedefe yaklasirken pozitif dikey hiz (error_rate < 0) cikisi frenlemeli
    pid._integral = 0.0
    out_static = pid.compute(error=0.1, dt=0.02, error_rate=0.0)
    pid._integral = 0.0
    out_braking = pid.compute(error=0.1, dt=0.02, error_rate=-0.5)
    assert out_braking < out_static


def test_angle_diff_wrapping():
    assert abs(angle_diff(10.0, 350.0) - 20.0) < 1e-9
    assert abs(angle_diff(350.0, 10.0) - (-20.0)) < 1e-9
    assert abs(angle_diff(180.0, 0.0)) == 180.0


if __name__ == '__main__':
    test_thruster_matrix_orthogonality()
    test_pid_anti_windup_and_kalman_damping()
    test_angle_diff_wrapping()
    print('OK')
