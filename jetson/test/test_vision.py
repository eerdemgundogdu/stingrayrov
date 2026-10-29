"""Su alti renk telafisi, boru hatti takibi ve kapi kestirimi icin donanimsiz test.

Calistirma (repo kokunden):
    python3 jetson/test/test_vision.py
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src',
                                'stingray_mobility', 'stingray_mobility'))
from vision_pipeline import (  # noqa: E402
    compensate_underwater_color,
    detect_gate_landmark,
    detect_pipeline,
)


def make_synthetic_pipeline_image(h=120, w=160, offset_px=24.0, angle_deg=16.0):
    # Mavi-yesil su alti arka plani (RGB)
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:, :, 0] = 15
    img[:, :, 1] = 75
    img[:, :, 2] = 95

    yy, xx = np.mgrid[0:h, 0:w]
    cx = w / 2.0 + offset_px
    cy = h / 2.0
    ang = math.radians(angle_deg)
    # Dikey eksene gore egik sari-turuncu boru hatti seridi
    perp = (xx - cx) * math.cos(ang) - (yy - cy) * math.sin(ang)
    mask = np.abs(perp) <= 7.0
    # Su altinda kirmizi kanali zayiflamis sari/turuncu boru
    img[mask, 0] = 165
    img[mask, 1] = 155
    img[mask, 2] = 25
    return img


def make_synthetic_gate_image(h=120, w=160, known_width_m=1.20,
                              hfov_deg=70.0, true_range_m=2.20,
                              center_shift_px=14):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:, :, 0] = 12
    img[:, :, 1] = 65
    img[:, :, 2] = 85

    focal_px = (w * 0.5) / math.tan(math.radians(hfov_deg * 0.5))
    span_px = int(round((known_width_m * focal_px) / true_range_m))
    cx = w // 2 + center_shift_px
    left_x = cx - span_px // 2
    right_x = cx + span_px // 2

    # Iki dikey kirmizi/turuncu kapi dikmesi
    img[25:95, max(0, left_x - 3):min(w, left_x + 3), 0] = 190
    img[25:95, max(0, left_x - 3):min(w, left_x + 3), 1] = 80
    img[25:95, max(0, left_x - 3):min(w, left_x + 3), 2] = 20

    img[25:95, max(0, right_x - 3):min(w, right_x + 3), 0] = 190
    img[25:95, max(0, right_x - 3):min(w, right_x + 3), 1] = 80
    img[25:95, max(0, right_x - 3):min(w, right_x + 3), 2] = 20
    return img


def test_color_compensation():
    img = make_synthetic_pipeline_image()
    comp = compensate_underwater_color(img, alpha=0.85)
    assert comp.shape == img.shape
    assert comp.dtype == np.uint8
    # Kirmizi kanal ortalamasi telafi sonrasi artmali
    assert float(comp[:, :, 0].mean()) > float(img[:, :, 0].mean())


def test_pipeline_detection():
    img = make_synthetic_pipeline_image(offset_px=24.0, angle_deg=16.0)
    res = detect_pipeline(img, min_pixels=80)
    assert res.detected is True
    assert 0.15 < res.lateral_error < 0.45
    assert abs(res.heading_error_deg - 16.0) < 5.0
    assert res.pixel_count > 500


def test_gate_landmark_range_and_bearing():
    true_range = 2.20
    hfov = 70.0
    shift_px = 14
    w = 160
    focal_px = (w * 0.5) / math.tan(math.radians(hfov * 0.5))
    img = make_synthetic_gate_image(
        known_width_m=1.20, hfov_deg=hfov,
        true_range_m=true_range, center_shift_px=shift_px)
    det = detect_gate_landmark(
        img, known_width_m=1.20, hfov_deg=hfov, min_pixels=50)
    assert det is not None and det.detected is True
    assert abs(det.range_m - true_range) < 0.20
    expected_bearing = math.atan2(-shift_px, focal_px)
    assert abs(det.bearing_rad - expected_bearing) < math.radians(3.0)


if __name__ == '__main__':
    test_color_compensation()
    test_pipeline_detection()
    test_gate_landmark_range_and_bearing()
    print('OK')
