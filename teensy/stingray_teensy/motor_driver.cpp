#include "motor_driver.h"
#include "config.h"
#include "frame_codec.h"
#include <Servo.h>

static Servo motor_servo[MOTOR_COUNT];
static uint16_t current_pwm[MOTOR_COUNT];
static const uint8_t motor_pins[MOTOR_COUNT] = {
    MOTOR0_PIN, MOTOR1_PIN, MOTOR2_PIN,
    MOTOR3_PIN, MOTOR4_PIN, MOTOR5_PIN
};

void motor_init(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        current_pwm[i] = PWM_NEUTRAL;
        motor_servo[i].attach(motor_pins[i], PWM_MIN, PWM_MAX);
        motor_servo[i].writeMicroseconds(PWM_NEUTRAL);
    }
}

void motor_set_pwm(uint8_t index, uint16_t pulse_us)
{
    if (index >= MOTOR_COUNT) return;

    uint16_t next = frame_slew_pwm(current_pwm[index], pulse_us,
                                   PWM_SLEW_US_MS, PWM_DEADBAND_US);
    current_pwm[index] = next;
    motor_servo[index].writeMicroseconds(next);
}

void motor_set_all(uint16_t *pulse_us)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        motor_set_pwm(i, pulse_us[i]);
    }
}

void motor_stop_all(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        current_pwm[i] = PWM_NEUTRAL;
        motor_servo[i].writeMicroseconds(PWM_NEUTRAL);
    }
}

void motor_arm_esc(void)
{
    motor_stop_all();
    delay(ESC_ARM_MS);
}
