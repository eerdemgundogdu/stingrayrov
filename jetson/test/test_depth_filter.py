"""Derinlik filtresi icin donanimsiz test.

Calistirma (repo kokunden):
    python3 jetson/test/test_depth_filter.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src',
                                'stingray_mobility', 'stingray_mobility'))
from depth_filter import DepthKalmanFilter  # noqa: E402


def simulate(seed=1):
    rng = np.random.default_rng(seed)
    dt = 0.02                      # 50 Hz telemetri
    sensor_period = 5              # sensor 10 Hz guncelleniyor
    t = np.arange(0, 20, dt)

    # 0 -> 1 m'ye 0.2 m/s ile dalis, sonra sabit
    true_z = np.clip(0.2 * (t - 2.0), 0.0, 1.0)
    true_w = np.where((t > 2.0) & (true_z < 1.0), 0.2, 0.0)

    kf = DepthKalmanFilter(accel_std=0.5, meas_std=0.02)
    est_z, est_w, raw = [], [], []
    last = 0.0
    for k, z in enumerate(true_z):
        if k % sensor_period == 0:
            last = z + rng.normal(0, 0.02)
            kf.predict(dt)
            kf.update(last)
        else:
            kf.predict(dt)
        raw.append(last)
        est_z.append(kf.depth)
        est_w.append(kf.rate)
    return t, true_z, true_w, np.array(raw), np.array(est_z), np.array(est_w)


def test_filter():
    t, true_z, true_w, raw, est_z, est_w = simulate()
    settle = t > 1.0

    raw_rmse = np.sqrt(np.mean((raw[settle] - true_z[settle]) ** 2))
    kf_rmse = np.sqrt(np.mean((est_z[settle] - true_z[settle]) ** 2))
    raw_rate = np.gradient(raw, t)
    raw_rate_rmse = np.sqrt(np.mean((raw_rate[settle] - true_w[settle]) ** 2))
    kf_rate_rmse = np.sqrt(np.mean((est_w[settle] - true_w[settle]) ** 2))

    print(f'derinlik RMSE   ham: {raw_rmse:.4f} m   KF: {kf_rmse:.4f} m')
    print(f'dikey hiz RMSE  ham turev: {raw_rate_rmse:.3f} m/s   '
          f'KF: {kf_rate_rmse:.3f} m/s')

    assert kf_rmse < raw_rmse
    assert kf_rate_rmse < 0.5 * raw_rate_rmse
    # sabit derinlikte hiz kestirimi sifira yakinsamali
    assert abs(np.mean(est_w[t > 12.0])) < 0.02


if __name__ == '__main__':
    test_filter()
    print('OK')
