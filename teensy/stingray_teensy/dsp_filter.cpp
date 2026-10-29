#include "dsp_filter.h"

#include <math.h>

namespace {
constexpr float kPi = 3.14159265358979323846f;
}  // namespace

void BiquadFilter::configure_lowpass(float fs_hz, float fc_hz, float q) {
    float w0 = 2.0f * kPi * (fc_hz / fs_hz);
    float cos_w0 = cosf(w0);
    float sin_w0 = sinf(w0);
    float alpha = sin_w0 / (2.0f * q);
    float inv_a0 = 1.0f / (1.0f + alpha);

    b0 = ((1.0f - cos_w0) * 0.5f) * inv_a0;
    b1 = (1.0f - cos_w0) * inv_a0;
    b2 = b0;
    a1 = (-2.0f * cos_w0) * inv_a0;
    a2 = (1.0f - alpha) * inv_a0;
    reset();
}

void BiquadFilter::configure_notch(float fs_hz, float f_notch_hz, float q) {
    float w0 = 2.0f * kPi * (f_notch_hz / fs_hz);
    float cos_w0 = cosf(w0);
    float sin_w0 = sinf(w0);
    float alpha = sin_w0 / (2.0f * q);
    float inv_a0 = 1.0f / (1.0f + alpha);

    b0 = 1.0f * inv_a0;
    b1 = (-2.0f * cos_w0) * inv_a0;
    b2 = 1.0f * inv_a0;
    a1 = (-2.0f * cos_w0) * inv_a0;
    a2 = (1.0f - alpha) * inv_a0;
    reset();
}

float Median5Filter::step(float sample) {
    win[idx] = sample;
    idx = static_cast<uint8_t>((idx + 1) % 5);
    if (count < 5) {
        ++count;
    }

    float tmp[5];
    for (uint8_t i = 0; i < count; ++i) {
        tmp[i] = win[i];
    }
    // Kucuk dizi icin deterministik yerlestirmeli siralama (insertion sort)
    for (uint8_t i = 1; i < count; ++i) {
        float key = tmp[i];
        int j = static_cast<int>(i) - 1;
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            --j;
        }
        tmp[j + 1] = key;
    }

    float med = tmp[count / 2];
    if (count == 1) {
        ema = med;
    } else {
        ema = alpha * med + (1.0f - alpha) * ema;
    }
    return ema;
}
