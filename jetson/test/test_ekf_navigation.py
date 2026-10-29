"""6 durumlu su alti EKF navigasyon filtresi icin donanimsiz test.

Calistirma (repo kokunden):
    python3 jetson/test/test_ekf_navigation.py
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src',
                                'stingray_mobility', 'stingray_mobility'))
from ekf_navigation import UnderwaterEKF, wrap_angle  # noqa: E402


def simulate_mission(seed=7):
    rng = np.random.default_rng(seed)
    dt = 0.02                      # 50 Hz IMU / EKF tahmini
    t = np.arange(0.0, 60.0, dt)
    n = len(t)

    # Kapi / samandira isaretcileri (havuz koordinatlari, metre)
    landmarks = [
        (3.5, 0.0),
        (3.5, 3.5),
        (0.0, 3.5),
        (0.0, 0.0),
    ]

    true_x = np.zeros(n)
    true_y = np.zeros(n)
    true_z = np.zeros(n)
    true_v = np.zeros(n)
    true_th = np.zeros(n)

    ekf = UnderwaterEKF()
    true_bias = math.radians(0.8)  # 0.8 deg/s jiroskop kaymasi

    dr_x = np.zeros(n)
    dr_y = np.zeros(n)
    dr_th = 0.0

    est_x = np.zeros(n)
    est_y = np.zeros(n)
    est_z = np.zeros(n)
    est_v = np.zeros(n)
    est_th = np.zeros(n)
    est_bias = np.zeros(n)

    vx = 0.0
    for k in range(1, n):
        tk = t[k]
        # 4 kenarli kare parkur: her 15 saniyede 90 derece donus
        leg = int(tk // 15.0) % 4
        target_th = wrap_angle(leg * (math.pi / 2.0))
        th_err = wrap_angle(target_th - true_th[k - 1])
        true_omega = float(np.clip(2.0 * th_err, -0.6, 0.6))

        u_surge = 0.35 if abs(th_err) < 0.25 else 0.10
        thrust = u_surge * ekf.max_thrust
        drag = ekf.quad_drag * abs(vx) * vx + ekf.lin_drag * vx
        acc = (thrust - drag) / ekf.mass
        vx = max(0.0, vx + acc * dt)

        true_th[k] = wrap_angle(true_th[k - 1] + true_omega * dt)
        true_x[k] = true_x[k - 1] + vx * math.cos(true_th[k]) * dt
        true_y[k] = true_y[k - 1] + vx * math.sin(true_th[k]) * dt
        true_z[k] = min(1.0, tk * 0.2)
        true_v[k] = vx

        # Gurultulu + kaymali jiroskop olcumu
        gyro_z = float(true_omega + true_bias + rng.normal(0.0, 0.008))

        # Dead-reckoning (jiroskop kaymasini telafi etmeyen saf entegrasyon)
        v_ss = ekf.steady_state_velocity(u_surge)
        dr_th = wrap_angle(dr_th + gyro_z * dt)
        dr_x[k] = dr_x[k - 1] + v_ss * math.cos(dr_th) * dt
        dr_y[k] = dr_y[k - 1] + v_ss * math.sin(dr_th) * dt

        # EKF adimi
        ekf.predict(u_surge=u_surge, gyro_z=gyro_z, dt=dt)

        # 10 Hz derinlik + pusula/yönelim guncellemesi
        if k % 5 == 0:
            z_meas = float(true_z[k] + rng.normal(0.0, 0.02))
            th_meas = wrap_angle(float(true_th[k] + rng.normal(0.0, math.radians(1.2))))
            ekf.update_depth(z_meas)
            ekf.update_heading(th_meas)

        # 5 Hz gorsel kapi/samandira menzil-kerteriz guncellemesi
        if k % 10 == 0:
            lx, ly = landmarks[leg]
            dx = lx - true_x[k]
            dy = ly - true_y[k]
            rng_true = math.hypot(dx, dy)
            if 0.35 < rng_true < 6.0:
                bear_true = wrap_angle(math.atan2(dy, dx) - true_th[k])
                rng_m = max(0.15, float(rng_true + rng.normal(0.0, 0.05)))
                bear_m = wrap_angle(float(bear_true + rng.normal(0.0, math.radians(1.8))))
                ekf.update_landmark(lx, ly, rng_m, bear_m)

        px, py, pz, th = ekf.pose
        est_x[k] = px
        est_y[k] = py
        est_z[k] = pz
        est_v[k] = ekf.velocity
        est_th[k] = th
        est_bias[k] = ekf.gyro_bias

    return {
        't': t,
        'true_x': true_x, 'true_y': true_y, 'true_z': true_z,
        'true_v': true_v, 'true_th': true_th, 'true_bias': true_bias,
        'dr_x': dr_x, 'dr_y': dr_y,
        'est_x': est_x, 'est_y': est_y, 'est_z': est_z,
        'est_v': est_v, 'est_th': est_th, 'est_bias': est_bias,
        'landmarks': landmarks,
    }


def test_ekf_trajectory_and_bias():
    sim = simulate_mission()
    dr_err = np.hypot(sim['dr_x'] - sim['true_x'], sim['dr_y'] - sim['true_y'])
    ekf_err = np.hypot(sim['est_x'] - sim['true_x'], sim['est_y'] - sim['true_y'])

    dr_rmse = float(np.sqrt(np.mean(dr_err ** 2)))
    ekf_rmse = float(np.sqrt(np.mean(ekf_err ** 2)))
    final_bias_deg = math.degrees(sim['est_bias'][-1])
    true_bias_deg = math.degrees(sim['true_bias'])

    print(f'yatay konum RMSE  Dead-Reckoning: {dr_rmse:.3f} m   EKF: {ekf_rmse:.3f} m')
    print(f'jiroskop bias     gercek: {true_bias_deg:.2f} deg/s   '
          f'EKF: {final_bias_deg:.2f} deg/s')

    assert ekf_rmse < 0.12
    assert dr_rmse > 0.60
    assert ekf_rmse < 0.20 * dr_rmse
    assert abs(final_bias_deg - true_bias_deg) < 0.25


def test_mahalanobis_outlier_rejection():
    ekf = UnderwaterEKF()
    # Normal gorsel olcum kabul edilmeli
    ok = ekf.update_landmark(2.0, 0.0, 2.0, 0.0)
    assert ok is True

    # Su alti yansimasindan kaynaklanan bozuk olcum (10 m sapma) reddedilmeli
    px_before = ekf.pose[0]
    rejected = ekf.update_landmark(2.0, 0.0, 12.0, 1.2)
    assert rejected is False
    assert abs(ekf.pose[0] - px_before) < 1e-9
    assert ekf.rejected_landmarks == 1


if __name__ == '__main__':
    test_ekf_trajectory_and_bias()
    test_mahalanobis_outlier_rejection()
    print('OK')
