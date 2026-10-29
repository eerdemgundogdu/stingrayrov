#!/usr/bin/env python3
"""Su alti goruntu isleme: renk telafisi, boru hatti takibi ve kapi/samandira kestirimi.

Su altinda kirmizi dalga boyu (~620-700 nm) ilk birkac metrede soguruldugu icin
goruntuler mavi-yesil agirliklidir. Bu modul:
  1) Gray-World tabanli kirmizi kanal kazanc telafisi uygular,
  2) HSV renk uzayinda su alti boru hatti / gecis kapisi maskesini cikarir,
  3) Morfolojik kutu filtresi ile kabarcik/parcacik gurultusunu temizler,
  4) Görüntü momentlerinden (m00, m10, m01, mu20, mu02, mu11) boru hattinin
     yatay sapmasini ([-1, +1]) ve ana eksen acisini (derece) hesaplar,
  5) Gecis kapisi / samandira gorunur piksel genisliginden pinhole kamera
     modeliyle mesafe (m) ve yonelim acisi (rad) cikarir.

Modul yalnizca NumPy gerektirir; ROS 2 veya kamera donanimi olmadan birim
testlerinde (jetson/test/test_vision.py) dogrudan calisir.
"""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class PipelineDetection:
    detected: bool
    lateral_error: float       # [-1.0, +1.0], +1 = hat sagda
    heading_error_deg: float   # [-90, +90], + = saat yonu sapma
    pixel_count: int
    centroid_xy: tuple


@dataclass(frozen=True)
class GateLandmarkDetection:
    detected: bool
    range_m: float
    bearing_rad: float
    bbox_width_px: int
    centroid_xy: tuple


def compensate_underwater_color(rgb: np.ndarray, alpha: float = 0.85) -> np.ndarray:
    """Su altinda zayiflayan kirmizi kanali yesil kanal ortalamasina gore dengeler.

    giris/cikis: uint8 RGB goruntu (H, W, 3).
    """
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError('RGB goruntu (H, W, 3) bekleniyor')
    img = rgb.astype(np.float32)
    r, g, b = img[:, :, 0], img[:, :, 1], img[:, :, 2]
    mean_r = float(np.mean(r)) + 1e-3
    mean_g = float(np.mean(g))
    # Kirmizi kanal kazancini (1 - R/255) ile agirliklandirarak parlak piksellerin
    # doymaya (clipping) gitmesini onle (Ancuti et al. su alti renk telafisi).
    gain = min(2.5, max(1.0, mean_g / mean_r))
    r_comp = r + alpha * (gain - 1.0) * r * (1.0 - r / 255.0)
    out = np.stack([np.clip(r_comp, 0, 255), g, b], axis=-1)
    return out.astype(np.uint8)


def rgb_to_hsv_np(rgb: np.ndarray) -> np.ndarray:
    """NumPy tabanli hizli RGB -> HSV donusumu.

    Cikis: H [0..180], S [0..255], V [0..255] (OpenCV kurallariyla uyumlu uint8).
    """
    f = rgb.astype(np.float32) / 255.0
    r, g, b = f[:, :, 0], f[:, :, 1], f[:, :, 2]
    cmax = np.maximum(np.maximum(r, g), b)
    cmin = np.minimum(np.minimum(r, g), b)
    delta = cmax - cmin

    h = np.zeros_like(cmax)
    nz = delta > 1e-6
    rmask = nz & (cmax == r)
    gmask = nz & (cmax == g) & ~rmask
    bmask = nz & ~rmask & ~gmask

    h[rmask] = (60.0 * ((g[rmask] - b[rmask]) / delta[rmask])) % 360.0
    h[gmask] = 60.0 * (((b[gmask] - r[gmask]) / delta[gmask]) + 2.0)
    h[bmask] = 60.0 * (((r[bmask] - g[bmask]) / delta[bmask]) + 4.0)

    s = np.where(cmax > 1e-6, delta / np.maximum(cmax, 1e-6), 0.0)
    v = cmax

    return np.stack([
        np.clip(h * 0.5, 0, 180).astype(np.uint8),
        np.clip(s * 255.0, 0, 255).astype(np.uint8),
        np.clip(v * 255.0, 0, 255).astype(np.uint8),
    ], axis=-1)


def clean_mask_3x3(mask: np.ndarray) -> np.ndarray:
    """3x3 komsuluk yogunlugu ile tekil gurultu piksellerini temizler (morfolojik acma benzeri)."""
    m = mask.astype(np.uint8)
    padded = np.pad(m, 1, mode='constant')
    acc = np.zeros_like(m, dtype=np.uint8)
    for dy in range(3):
        for dx in range(3):
            acc += padded[dy:dy + m.shape[0], dx:dx + m.shape[1]]
    return acc >= 5


def detect_pipeline(rgb: np.ndarray,
                    hsv_low: tuple = (15, 90, 80),
                    hsv_high: tuple = (42, 255, 255),
                    min_pixels: int = 80) -> PipelineDetection:
    """Su alti boru hattini (varsayilan: sari/turuncu boru) tespit eder ve
    moment analiziyle yatay hata + yonelim acisini dondurur.
    """
    comp = compensate_underwater_color(rgb)
    hsv = rgb_to_hsv_np(comp)
    raw_mask = (
        (hsv[:, :, 0] >= hsv_low[0]) & (hsv[:, :, 0] <= hsv_high[0]) &
        (hsv[:, :, 1] >= hsv_low[1]) & (hsv[:, :, 1] <= hsv_high[1]) &
        (hsv[:, :, 2] >= hsv_low[2]) & (hsv[:, :, 2] <= hsv_high[2])
    )
    mask = clean_mask_3x3(raw_mask)
    ys, xs = np.nonzero(mask)
    n = int(xs.size)
    h, w = rgb.shape[:2]

    if n < min_pixels:
        return PipelineDetection(False, 0.0, 0.0, n, (w * 0.5, h * 0.5))

    cx = float(np.mean(xs))
    cy = float(np.mean(ys))
    lateral_error = (cx - (w * 0.5)) / (w * 0.5)
    lateral_error = max(-1.0, min(1.0, lateral_error))

    # Merkezi ikinci derece momentler: boru hattinin ana eksen acisi
    dx = xs.astype(np.float64) - cx
    dy = ys.astype(np.float64) - cy
    mu20 = float(np.mean(dx * dx))
    mu02 = float(np.mean(dy * dy))
    mu11 = float(np.mean(dx * dy))

    # Dikey eksene (ileri yon, -y) gore egim acisi: dx / dy
    theta_rad = 0.5 * math.atan2(2.0 * mu11, mu02 - mu20)
    heading_error_deg = math.degrees(theta_rad)

    return PipelineDetection(
        detected=True,
        lateral_error=lateral_error,
        heading_error_deg=heading_error_deg,
        pixel_count=n,
        centroid_xy=(cx, cy),
    )


def detect_gate_landmark(rgb: np.ndarray,
                         known_width_m: float = 1.20,
                         hfov_deg: float = 70.0,
                         hsv_low: tuple = (5, 110, 80),
                         hsv_high: tuple = (25, 255, 255),
                         min_pixels: int = 60) -> Optional[GateLandmarkDetection]:
    """Su alti gecis kapisi veya samandirasinin gorunur genisliginden
    mesafe (range) ve yonelim acisi (bearing) kestirir.
    """
    comp = compensate_underwater_color(rgb)
    hsv = rgb_to_hsv_np(comp)
    raw_mask = (
        (hsv[:, :, 0] >= hsv_low[0]) & (hsv[:, :, 0] <= hsv_high[0]) &
        (hsv[:, :, 1] >= hsv_low[1]) & (hsv[:, :, 1] <= hsv_high[1]) &
        (hsv[:, :, 2] >= hsv_low[2]) & (hsv[:, :, 2] <= hsv_high[2])
    )
    mask = clean_mask_3x3(raw_mask)
    ys, xs = np.nonzero(mask)
    if xs.size < min_pixels:
        return GateLandmarkDetection(False, 0.0, 0.0, 0, (0.0, 0.0))

    h, w = rgb.shape[:2]
    # Aykiri piksellerden etkilenmemek icin %2 - %98 yuzdelik dilim genisligi
    x_min = float(np.percentile(xs, 2))
    x_max = float(np.percentile(xs, 98))
    bbox_w = max(1, int(round(x_max - x_min)))

    cx = float(np.mean(xs))
    cy = float(np.mean(ys))

    focal_px = (w * 0.5) / math.tan(math.radians(hfov_deg * 0.5))
    range_m = (known_width_m * focal_px) / float(bbox_w)
    # Kamera ekseninde sol pozitif (CCW, EKF koordinat sistemiyle uyumlu)
    bearing_rad = math.atan2((w * 0.5) - cx, focal_px)

    return GateLandmarkDetection(
        detected=True,
        range_m=range_m,
        bearing_rad=bearing_rad,
        bbox_width_px=bbox_w,
        centroid_xy=(cx, cy),
    )
