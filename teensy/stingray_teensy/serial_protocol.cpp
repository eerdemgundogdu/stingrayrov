#include "serial_protocol.h"
#include "frame_codec.h"
#include "config.h"
#include <string.h>

// Cerceve kodlama/cozme frame_codec.cpp'de (Arduino'dan bagimsiz, host'ta
// test ediliyor). Bu dosya sadece USB seri port ile kodlayici arasindaki
// yapistiricidir.

static CmdFrameParser parser;
static CMD_Packet     cmd_packet;

uint8_t protocol_crc8(const uint8_t *data, uint16_t len)
{
    return frame_crc8(data, len);
}

void protocol_init(void)
{
    Serial.begin(SERIAL_BAUD);

    cmd_parser_init(&parser);
    memset(&cmd_packet, 0, sizeof(cmd_packet));
    for (uint8_t i = 0; i < FRAME_MOTOR_COUNT; i++)
        cmd_packet.motor_pwm[i] = PWM_NEUTRAL;
}

void protocol_process_rx(void)
{
    // Tek seferde islenecek bayt sayisi sinirli: comm gorevi baska
    // isleri (sensor gonderimi) geciktirmesin.
    int budget = RX_BUF_SIZE;
    uint16_t pwm[FRAME_MOTOR_COUNT];

    while (budget-- > 0 && Serial.available() > 0) {
        const uint8_t b = (uint8_t)Serial.read();
        if (cmd_parser_feed(&parser, b, pwm) == FRAME_OK) {
            for (uint8_t i = 0; i < FRAME_MOTOR_COUNT; i++)
                cmd_packet.motor_pwm[i] = frame_clamp_pwm(pwm[i], PWM_MIN, PWM_MAX);
            cmd_packet.valid = 1;
            cmd_packet.last_rx_tick = millis();
        }
    }
}

CMD_Packet* protocol_get_cmd(void)
{
    return &cmd_packet;
}

void protocol_send_sensor(uint16_t yaw, int16_t pitch, int16_t roll,
                          uint16_t depth_mm, uint8_t leak,
                          uint16_t bat_cv)
{
    SensorFrame s;
    s.yaw_cdeg   = yaw;
    s.pitch_cdeg = pitch;
    s.roll_cdeg  = roll;
    s.depth_mm   = depth_mm;
    s.leak       = leak;
    s.battery_cv = bat_cv;

    uint8_t pkt[FRAME_SENSOR_LEN];
    sensor_frame_encode(&s, pkt);
    Serial.write(pkt, FRAME_SENSOR_LEN);
}

uint32_t protocol_crc_errors(void)
{
    return parser.crc_errors;
}
