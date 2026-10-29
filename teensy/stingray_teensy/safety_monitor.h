#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <Arduino.h>

typedef struct {
    uint16_t leak_adc;
    uint16_t battery_mv;
    uint8_t  leak_alarm;
    uint8_t  battery_low;
    uint8_t  comm_lost;
    uint8_t  kill_switch_open;
} Safety_Status;

void safety_init(void);
void safety_update(Safety_Status *status, uint32_t current_tick,
                   uint32_t last_cmd_tick);
uint8_t safety_is_emergency(Safety_Status *status);
void safety_set_actuators(const Safety_Status *status, bool cmd_valid);

#endif
