#ifndef FRAME_CODEC_H
#define FRAME_CODEC_H

// ============================================================================
//  Stingray seri protokolu: cerceve kodlama / cozme ve itici rampa sinirlayici
//
//  Bu modul Arduino'ya bagimli degildir; ayni kod hem Teensy'de hem de
//  host bilgisayarda birim testleriyle (teensy/test/) calisir.
//
//  Komut cercevesi (Jetson -> Teensy), 15 bayt:
//    [0]    0xAA
//    [1]    0x55
//    [2..13] 6 x uint16 PWM (big-endian, mikrosaniye)
//    [14]   CRC-8 (poly 0x07, init 0x00) — [2..13] uzerinden
//
//  Sensor cercevesi (Teensy -> Jetson), 16 bayt:
//    [0]    0xBB
//    [1]    0x66
//    [2..3] yaw   uint16  (derece x100)
//    [4..5] pitch int16   (derece x100)
//    [6..7] roll  int16   (derece x100)
//    [8..9] depth uint16  (mm)
//    [10]   leak  uint8
//    [11..12] battery uint16 (V x100, ornek 2310 = 23.10 V)
//    [13]   ayrilmis (0)
//    [14]   CRC-8 — [2..13] uzerinden
//    [15]   ayrilmis (0)
// ============================================================================

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FRAME_CMD_HEADER_0      0xAA
#define FRAME_CMD_HEADER_1      0x55
#define FRAME_SENSOR_HEADER_0   0xBB
#define FRAME_SENSOR_HEADER_1   0x66

#define FRAME_CMD_LEN           15
#define FRAME_SENSOR_LEN        16
#define FRAME_PAYLOAD_LEN       12
#define FRAME_MOTOR_COUNT       6

#define FRAME_PWM_MIN           1100
#define FRAME_PWM_MAX           1900
#define FRAME_PWM_NEUTRAL       1500

// CRC-8/SMBUS: poly 0x07, init 0x00, refin/refout yok, xorout 0x00.
// Kontrol degeri: crc8("123456789") == 0xF4
uint8_t frame_crc8(const uint8_t *data, size_t len);

// --- Komut cercevesi ---------------------------------------------------------

// Kayan pencereli, tahsis yapmayan cozucu. Bayt bayt beslenir; bozuk
// cercevede sadece 1 bayt atilir, boylece bozuk cercevenin icinde baslayan
// gecerli bir cerceve kaybolmaz.
typedef struct {
    uint8_t  buf[FRAME_CMD_LEN];
    uint8_t  len;
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t bytes_dropped;
} CmdFrameParser;

typedef enum {
    FRAME_NONE = 0,     // henuz tam cerceve yok
    FRAME_OK   = 1,     // gecerli cerceve; pwm_out dolduruldu
} FrameResult;

void        cmd_parser_init(CmdFrameParser *p);
FrameResult cmd_parser_feed(CmdFrameParser *p, uint8_t byte,
                            uint16_t pwm_out[FRAME_MOTOR_COUNT]);

// Test ve simulasyon icin: Jetson tarafinin urettigi cerceveyi uretir.
void cmd_frame_encode(const uint16_t pwm[FRAME_MOTOR_COUNT],
                      uint8_t out[FRAME_CMD_LEN]);

// PWM degerini [lo, hi] araligina sinirlar.
uint16_t frame_clamp_pwm(uint16_t pwm, uint16_t lo, uint16_t hi);

// Notr (1500 us) cevresinde olu bolge (deadband) uygular ve ani geri-ileri
// gecislerinde 6S Li-Po hattinda rejeneratif gerilim sicramalarini onlemek icin
// degisim hizini (slew-rate, us/adim) sinirlar.
uint16_t frame_slew_pwm(uint16_t current_pwm, uint16_t target_pwm,
                        uint16_t max_step_us, uint16_t deadband_us);

// --- Sensor cercevesi --------------------------------------------------------

typedef struct {
    uint16_t yaw_cdeg;
    int16_t  pitch_cdeg;
    int16_t  roll_cdeg;
    uint16_t depth_mm;
    uint8_t  leak;
    uint16_t battery_cv;   // V x100
} SensorFrame;

void sensor_frame_encode(const SensorFrame *s, uint8_t out[FRAME_SENSOR_LEN]);

// out icin gecerli bir cerceve ise 1, degilse 0 doner.
int  sensor_frame_decode(const uint8_t in[FRAME_SENSOR_LEN], SensorFrame *out);

#ifdef __cplusplus
}
#endif

#endif // FRAME_CODEC_H
