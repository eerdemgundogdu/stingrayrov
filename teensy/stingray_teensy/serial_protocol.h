#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>

#define CMD_HEADER_0        0xAA
#define CMD_HEADER_1        0x55
#define SENSOR_HEADER_0     0xBB
#define SENSOR_HEADER_1     0x66

#define CMD_PACKET_LEN      15      
#define SENSOR_PACKET_LEN   16      

#define RX_BUF_SIZE         128

typedef struct {
    uint16_t motor_pwm[6];
    uint8_t  valid;
    uint32_t last_rx_tick;
} CMD_Packet;

void protocol_init(void);
void protocol_process_rx(void);
CMD_Packet* protocol_get_cmd(void);
void protocol_send_sensor(uint16_t yaw, int16_t pitch, int16_t roll,
                          uint16_t depth_mm, uint8_t leak,
                          uint16_t bat_cv);   // batarya V x100
uint8_t protocol_crc8(const uint8_t *data, uint16_t len);
uint32_t protocol_crc_errors(void);   // tanilama: reddedilen bozuk cerceve sayisi

#endif 
