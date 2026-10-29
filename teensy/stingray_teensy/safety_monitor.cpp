#include "safety_monitor.h"
#include "config.h"

void safety_init(void)
{
    analogReadResolution(ADC_RESOLUTION_BITS);

    pinMode(LEAK_SENSOR_PIN, INPUT);
    pinMode(BATTERY_SENSOR_PIN, INPUT);
    pinMode(KILL_SENSE_PIN, KILL_SENSE_PIN_MODE);

#ifdef ESC_EN_PIN
    pinMode(ESC_EN_PIN, OUTPUT);
    digitalWrite(ESC_EN_PIN, LOW);  // Açılış ve kalibrasyon tamamlanana kadar kontaktör kapalı
#endif

#ifdef BUZZER_PIN
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
#endif

#ifdef STATUS_LED_PIN
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
#endif
}

void safety_update(Safety_Status *status, uint32_t current_tick,
                   uint32_t last_cmd_tick)
{
    status->leak_adc = (uint16_t)analogRead(LEAK_SENSOR_PIN);

    uint16_t bat_adc = (uint16_t)analogRead(BATTERY_SENSOR_PIN);
    status->battery_mv = (uint16_t)((uint32_t)bat_adc * 3300u *
                         (uint32_t)BATT_DIV_NUM / ((uint32_t)BATT_DIV_DEN * (ADC_MAX_VALUE + 1u)));

    status->leak_alarm = (status->leak_adc > LEAK_ADC_THRESHOLD) ? 1 : 0;
    status->battery_low = (status->battery_mv < BATTERY_LOW_MV) ? 1 : 0;

    if ((current_tick - last_cmd_tick) > COMM_TIMEOUT_MS)
        status->comm_lost = 1;
    else
        status->comm_lost = 0;

#if KILL_SWITCH_ENABLED
    status->kill_switch_open = (digitalRead(KILL_SENSE_PIN) == LOW) ? 1 : 0;
#else
    status->kill_switch_open = 0;
#endif
}

uint8_t safety_is_emergency(Safety_Status *status)
{
    if (status->leak_alarm) return 1;
    if (status->comm_lost)  return 1;
    if (status->kill_switch_open) return 1;
    if (status->battery_mv < BATTERY_CRITICAL_MV) return 1;
    return 0;
}

void safety_set_actuators(const Safety_Status *status, bool cmd_valid)
{
    const bool emergency = (safety_is_emergency((Safety_Status *)status) != 0);

#ifdef ESC_EN_PIN
    // Kontaktör yalnızca acil durum yokken ve geçerli Jetson komutu varken çekili kalır.
    // Harici manyetik kill-switch ayrıca donanımda bobin enerjisini fiziksel olarak keser.
    digitalWrite(ESC_EN_PIN, (!emergency && cmd_valid) ? HIGH : LOW);
#else
    (void)cmd_valid;
#endif

#ifdef BUZZER_PIN
    // Su sızıntısı veya kritik bataryada sesli alarm
    const bool critical_alarm = (status->leak_alarm != 0) ||
                                (status->battery_mv < BATTERY_CRITICAL_MV);
    digitalWrite(BUZZER_PIN, critical_alarm ? HIGH : LOW);
#endif
}
