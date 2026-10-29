#ifndef STINGRAY_DSP_FILTER_H
#define STINGRAY_DSP_FILTER_H

#include <stddef.h>
#include <stdint.h>

// Direct-Form II Transposed 2. derece IIR Biquad filtre
// (1 kHz IMU jiroskop/ivmeolcer kanallarinda 6-itici govde rezonansini bastirmak icin
//  Cauer/Notch ve Butterworth alcak geciren filtre kaskadi)
struct BiquadFilter {
    float b0, b1, b2;
    float a1, a2;
    float z1, z2;

    void reset() {
        z1 = 0.0f;
        z2 = 0.0f;
    }

    // 2. derece Butterworth alcak geciren katsayilarini hesapla
    void configure_lowpass(float fs_hz, float fc_hz, float q = 0.70710678f);

    // 2. derece dar-bant centik (notch) filtre katsayilarini hesapla (itici vibrasyon bastirma)
    void configure_notch(float fs_hz, float f_notch_hz, float q = 4.0f);

    float step(float x) {
        float y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
};

// 5-orneklik kayan medyan + EMA filtre (SEN0257 derinlik ve 6S batarya ADC dikensi gurultu bastirma)
struct Median5Filter {
    float win[5];
    uint8_t idx;
    uint8_t count;
    float ema;
    float alpha;

    void init(float ema_alpha = 0.35f) {
        for (int i = 0; i < 5; ++i) win[i] = 0.0f;
        idx = 0;
        count = 0;
        ema = 0.0f;
        alpha = ema_alpha;
    }

    float step(float sample);
};

#endif  // STINGRAY_DSP_FILTER_H
