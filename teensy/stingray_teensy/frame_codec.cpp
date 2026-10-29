#include "frame_codec.h"
#include <string.h>

uint8_t frame_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t frame_clamp_pwm(uint16_t pwm, uint16_t lo, uint16_t hi)
{
    if (pwm < lo) return lo;
    if (pwm > hi) return hi;
    return pwm;
}

uint16_t frame_slew_pwm(uint16_t current_pwm, uint16_t target_pwm,
                        uint16_t max_step_us, uint16_t deadband_us)
{
    uint16_t tgt = frame_clamp_pwm(target_pwm, FRAME_PWM_MIN, FRAME_PWM_MAX);
    int32_t diff_neutral = (int32_t)tgt - (int32_t)FRAME_PWM_NEUTRAL;
    if (diff_neutral > -(int32_t)deadband_us && diff_neutral < (int32_t)deadband_us) {
        tgt = FRAME_PWM_NEUTRAL;
    }
    if (max_step_us == 0) {
        return tgt;
    }
    int32_t delta = (int32_t)tgt - (int32_t)current_pwm;
    if (delta > (int32_t)max_step_us) {
        return (uint16_t)(current_pwm + max_step_us);
    }
    if (delta < -(int32_t)max_step_us) {
        return (uint16_t)(current_pwm - max_step_us);
    }
    return tgt;
}

static void put_u16_be(uint8_t *dst, uint16_t v)
{
    dst[0] = (uint8_t)(v >> 8);
    dst[1] = (uint8_t)(v & 0xFF);
}

static uint16_t get_u16_be(const uint8_t *src)
{
    return (uint16_t)(((uint16_t)src[0] << 8) | src[1]);
}

// --- Komut cercevesi ---------------------------------------------------------

void cmd_parser_init(CmdFrameParser *p)
{
    memset(p, 0, sizeof(*p));
}

static void drop_front(CmdFrameParser *p, uint8_t n)
{
    if (n >= p->len) {
        p->bytes_dropped += p->len;
        p->len = 0;
        return;
    }
    memmove(p->buf, p->buf + n, (size_t)(p->len - n));
    p->len = (uint8_t)(p->len - n);
    p->bytes_dropped += n;
}

// Tampondaki baytlar her zaman gecerli bir cerceve oneki olmali:
// [0] == 0xAA ve (varsa) [1] == 0x55. Degilse bastan bayt atilir.
static void resync(CmdFrameParser *p)
{
    while (p->len > 0) {
        if (p->buf[0] != FRAME_CMD_HEADER_0) {
            drop_front(p, 1);
            continue;
        }
        if (p->len >= 2 && p->buf[1] != FRAME_CMD_HEADER_1) {
            drop_front(p, 1);
            continue;
        }
        break;
    }
}

FrameResult cmd_parser_feed(CmdFrameParser *p, uint8_t byte,
                            uint16_t pwm_out[FRAME_MOTOR_COUNT])
{
    p->buf[p->len++] = byte;
    resync(p);

    while (p->len == FRAME_CMD_LEN) {
        const uint8_t crc = frame_crc8(&p->buf[2], FRAME_PAYLOAD_LEN);
        if (crc == p->buf[FRAME_CMD_LEN - 1]) {
            for (uint8_t i = 0; i < FRAME_MOTOR_COUNT; i++) {
                pwm_out[i] = get_u16_be(&p->buf[2 + i * 2]);
            }
            p->len = 0;
            p->frames_ok++;
            return FRAME_OK;
        }
        // Bozuk cerceve: sadece baslik baytini at, kalanin icinde
        // yeni bir baslik olabilir.
        p->crc_errors++;
        drop_front(p, 1);
        resync(p);
    }
    return FRAME_NONE;
}

void cmd_frame_encode(const uint16_t pwm[FRAME_MOTOR_COUNT],
                      uint8_t out[FRAME_CMD_LEN])
{
    out[0] = FRAME_CMD_HEADER_0;
    out[1] = FRAME_CMD_HEADER_1;
    for (uint8_t i = 0; i < FRAME_MOTOR_COUNT; i++) {
        put_u16_be(&out[2 + i * 2], pwm[i]);
    }
    out[FRAME_CMD_LEN - 1] = frame_crc8(&out[2], FRAME_PAYLOAD_LEN);
}

// --- Sensor cercevesi --------------------------------------------------------

void sensor_frame_encode(const SensorFrame *s, uint8_t out[FRAME_SENSOR_LEN])
{
    out[0] = FRAME_SENSOR_HEADER_0;
    out[1] = FRAME_SENSOR_HEADER_1;
    put_u16_be(&out[2], s->yaw_cdeg);
    put_u16_be(&out[4], (uint16_t)s->pitch_cdeg);
    put_u16_be(&out[6], (uint16_t)s->roll_cdeg);
    put_u16_be(&out[8], s->depth_mm);
    out[10] = s->leak;
    put_u16_be(&out[11], s->battery_cv);
    out[13] = 0x00;
    out[14] = frame_crc8(&out[2], FRAME_PAYLOAD_LEN);
    out[15] = 0x00;
}

int sensor_frame_decode(const uint8_t in[FRAME_SENSOR_LEN], SensorFrame *out)
{
    if (in[0] != FRAME_SENSOR_HEADER_0 || in[1] != FRAME_SENSOR_HEADER_1) return 0;
    if (frame_crc8(&in[2], FRAME_PAYLOAD_LEN) != in[14]) return 0;
    out->yaw_cdeg   = get_u16_be(&in[2]);
    out->pitch_cdeg = (int16_t)get_u16_be(&in[4]);
    out->roll_cdeg  = (int16_t)get_u16_be(&in[6]);
    out->depth_mm   = get_u16_be(&in[8]);
    out->leak       = in[10];
    out->battery_cv = get_u16_be(&in[11]);
    return 1;
}
