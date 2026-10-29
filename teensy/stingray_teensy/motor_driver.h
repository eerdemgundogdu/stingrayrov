#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

void motor_init(void);
void motor_set_pwm(uint8_t index, uint16_t pulse_us);
void motor_set_all(uint16_t *pulse_us);
void motor_stop_all(void);
void motor_arm_esc(void);

#endif 
