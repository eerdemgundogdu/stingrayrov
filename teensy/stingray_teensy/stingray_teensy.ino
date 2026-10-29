#include "config.h"
#include "serial_protocol.h"
#include "motor_driver.h"
#include "imu_sensor.h"
#include "depth_sensor.h"
#include "safety_monitor.h"
#include <Watchdog_t4.h>

WDT_T4<WDT1> wdt;

static IMU_Data imu;
static Depth_Data depth;
static Safety_Status safety;

static uint32_t loop_tick = 0;
static uint32_t sensor_send_tick = 0;
static uint32_t depth_read_tick = 0;
static uint32_t led_tick = 0;

static bool init_ok = false;

void wdt_callback()
{
#ifdef ESC_EN_PIN
    digitalWrite(ESC_EN_PIN, LOW);
#endif
    motor_stop_all();
}

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    protocol_init();

    delay(500);

    motor_init();

    safety_init();

    if (!imu_init()) {
        while (true) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }

    if (!depth_init(&depth)) {
        while (true) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(200);
        }
    }

    imu_calibrate(&imu, IMU_CALIB_SAMPLES);

    // DFRobot analog sensör kalibrasyon gerektirmez,
    // atmosferik ofset Jetson tarafında (sensor_node) hesaplanır.

    motor_arm_esc();

    WDT_timings_t wdt_config;
    wdt_config.trigger = 2.0f;
    wdt_config.timeout = 3.0f;
    wdt_config.callback = wdt_callback;
    wdt.begin(wdt_config);

    loop_tick = millis();
    sensor_send_tick = millis();
    depth_read_tick = millis();
    led_tick = millis();

    init_ok = true;
    digitalWrite(LED_PIN, LOW);
}

void loop()
{
    uint32_t now = millis();

    if ((now - loop_tick) < 1) return;
    float dt = (now - loop_tick) / 1000.0f;
    loop_tick = now;

    imu_read(&imu, dt);

    if ((now - depth_read_tick) >= DEPTH_READ_MS) {
        depth_read(&depth);
        depth_read_tick = now;
    }

    protocol_process_rx();
    CMD_Packet *cmd = protocol_get_cmd();

    safety_update(&safety, now, cmd->last_rx_tick);
    safety_set_actuators(&safety, cmd->valid != 0);

    if (safety_is_emergency(&safety)) {
        motor_stop_all();
    } else if (cmd->valid) {
        motor_set_all(cmd->motor_pwm);
    } else {
        motor_stop_all();
    }

    if ((now - sensor_send_tick) >= SENSOR_SEND_MS) {
        uint16_t yaw_tx   = (uint16_t)(imu.yaw * 100.0f);
        int16_t pitch_tx  = (int16_t)(imu.pitch * 100.0f);
        int16_t roll_tx   = (int16_t)(imu.roll * 100.0f);
        uint16_t depth_tx = (uint16_t)(depth.depth * 1000.0f);
        uint8_t leak_tx   = safety.leak_alarm;
        uint16_t bat_tx   = safety.battery_mv / 10;

        protocol_send_sensor(yaw_tx, pitch_tx, roll_tx,
                             depth_tx, leak_tx, bat_tx);
        sensor_send_tick = now;
    }

    if ((now - led_tick) >= 500) {
        if (safety_is_emergency(&safety)) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            led_tick = now - 400;
        } else if (!cmd->valid) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        } else {
            digitalWrite(LED_PIN, HIGH);
        }
#ifdef STATUS_LED_PIN
        digitalWrite(STATUS_LED_PIN, digitalRead(LED_PIN));
#endif
        led_tick = now;
    }

    wdt.feed();
}
