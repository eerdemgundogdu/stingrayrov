// ============================================================================
//  STINGRAY - Teensy 4.1 FreeRTOS firmware
//
//  stingray_teensy/ altindaki suruculeri (IMU, derinlik, motor, protokol,
//  guvenlik) aynen kullanir; sadece zamanlama superloop yerine FreeRTOS
//  gorevleriyle yapilir.
//
//  Gorev           Oncelik  Periyot   Is
//  -------------   -------  --------  ----------------------------------------
//  control_task    5        1 ms      guvenlik kararina gore motor PWM uygula
//  imu_task        4        1 ms      MPU6050 oku + complementary filter
//  comm_task       3        1 ms      seri RX ayristir, 20 ms'de bir sensor TX
//  sensor_task     2        10 ms     batarya/sizinti (10 ms), derinlik (100 ms)
//  monitor_task    1        100 ms    LED durumu, gorev saglik kontrolu, WDT
//
//  Paylasilan veriler mutex ile korunur; her gorev kendi kopyasi uzerinde
//  calisir. ADC'yi iki gorev kullandigi icin ayrica adc_mutex var.
//
//  Gorev seviyesinde watchdog: her gorev her turunda kendi bitini set eder.
//  monitor_task tum bitler set ise donanim watchdog'unu besler; bir gorev
//  takilirsa (ornegin I2C kilitlenmesi) WDT tetiklenir, motorlar durur
//  ve kart yeniden baslar.
// ============================================================================

#include <Arduino.h>
#include <Watchdog_t4.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

#include "config.h"
#include "serial_protocol.h"
#include "motor_driver.h"
#include "imu_sensor.h"
#include "depth_sensor.h"
#include "safety_monitor.h"

// --- Paylasilan durum ---

static IMU_Data      g_imu;
static Depth_Data    g_depth;
static Safety_Status g_safety;
static CMD_Packet    g_cmd;           // protokolden alinan son komutun kopyasi

static SemaphoreHandle_t imu_mutex;
static SemaphoreHandle_t depth_mutex;
static SemaphoreHandle_t safety_mutex;
static SemaphoreHandle_t cmd_mutex;
static SemaphoreHandle_t adc_mutex;

static EventGroupHandle_t alive_group;

#define ALIVE_CONTROL  (1u << 0)
#define ALIVE_IMU      (1u << 1)
#define ALIVE_COMM     (1u << 2)
#define ALIVE_SENSOR   (1u << 3)
#define ALIVE_ALL      (ALIVE_CONTROL | ALIVE_IMU | ALIVE_COMM | ALIVE_SENSOR)

static volatile bool g_emergency = true;   // ilk guvenlik kontrolune kadar dur

WDT_T4<WDT1> wdt;

static void wdt_callback()
{
#ifdef ESC_EN_PIN
    digitalWrite(ESC_EN_PIN, LOW);
#endif
    motor_stop_all();
}

static void fatal_blink(uint32_t period_ms)
{
    while (true) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(period_ms);
    }
}

// --- Gorevler ---

static void control_task(void *)
{
    TickType_t last_wake = xTaskGetTickCount();
    CMD_Packet cmd;

    for (;;) {
        xSemaphoreTake(cmd_mutex, portMAX_DELAY);
        cmd = g_cmd;
        xSemaphoreGive(cmd_mutex);

        if (g_emergency || !cmd.valid)
            motor_stop_all();
        else
            motor_set_all(cmd.motor_pwm);

        xEventGroupSetBits(alive_group, ALIVE_CONTROL);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

static void imu_task(void *)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_us = micros();
    IMU_Data imu;

    xSemaphoreTake(imu_mutex, portMAX_DELAY);
    imu = g_imu;                       // kalibrasyon ofsetlerini al
    xSemaphoreGive(imu_mutex);

    for (;;) {
        uint32_t now_us = micros();
        float dt = (now_us - last_us) * 1e-6f;
        last_us = now_us;

        imu_read(&imu, dt);

        xSemaphoreTake(imu_mutex, portMAX_DELAY);
        g_imu = imu;
        xSemaphoreGive(imu_mutex);

        xEventGroupSetBits(alive_group, ALIVE_IMU);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

static void comm_task(void *)
{
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_tx = last_wake;

    for (;;) {
        protocol_process_rx();
        CMD_Packet *rx = protocol_get_cmd();

        xSemaphoreTake(cmd_mutex, portMAX_DELAY);
        g_cmd = *rx;
        xSemaphoreGive(cmd_mutex);

        if ((xTaskGetTickCount() - last_tx) >= pdMS_TO_TICKS(SENSOR_SEND_MS)) {
            last_tx = xTaskGetTickCount();

            IMU_Data imu;
            Depth_Data depth;
            Safety_Status safety;

            xSemaphoreTake(imu_mutex, portMAX_DELAY);
            imu = g_imu;
            xSemaphoreGive(imu_mutex);
            xSemaphoreTake(depth_mutex, portMAX_DELAY);
            depth = g_depth;
            xSemaphoreGive(depth_mutex);
            xSemaphoreTake(safety_mutex, portMAX_DELAY);
            safety = g_safety;
            xSemaphoreGive(safety_mutex);

            protocol_send_sensor((uint16_t)(imu.yaw * 100.0f),
                                 (int16_t)(imu.pitch * 100.0f),
                                 (int16_t)(imu.roll * 100.0f),
                                 (uint16_t)(depth.depth * 1000.0f),
                                 safety.leak_alarm,
                                 safety.battery_mv / 10);
        }

        xEventGroupSetBits(alive_group, ALIVE_COMM);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

static void sensor_task(void *)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t cycle = 0;
    Safety_Status safety = {};
    Depth_Data depth;

    xSemaphoreTake(depth_mutex, portMAX_DELAY);
    depth = g_depth;
    xSemaphoreGive(depth_mutex);

    for (;;) {
        uint32_t last_rx_tick;
        bool cmd_valid;
        xSemaphoreTake(cmd_mutex, portMAX_DELAY);
        last_rx_tick = g_cmd.last_rx_tick;
        cmd_valid = (g_cmd.valid != 0);
        xSemaphoreGive(cmd_mutex);

        xSemaphoreTake(adc_mutex, portMAX_DELAY);
        safety_update(&safety, millis(), last_rx_tick);
        if (cycle % (DEPTH_READ_MS / 10) == 0)
            depth_read(&depth);
        xSemaphoreGive(adc_mutex);

        safety_set_actuators(&safety, cmd_valid);

        xSemaphoreTake(safety_mutex, portMAX_DELAY);
        g_safety = safety;
        xSemaphoreGive(safety_mutex);
        xSemaphoreTake(depth_mutex, portMAX_DELAY);
        g_depth = depth;
        xSemaphoreGive(depth_mutex);

        g_emergency = safety_is_emergency(&safety);

        cycle++;
        xEventGroupSetBits(alive_group, ALIVE_SENSOR);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

static void monitor_task(void *)
{
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // Tum gorevler son 100 ms'de en az bir tur attiysa WDT'yi besle.
        EventBits_t bits = xEventGroupClearBits(alive_group, ALIVE_ALL);
        if ((bits & ALIVE_ALL) == ALIVE_ALL)
            wdt.feed();

        bool cmd_valid;
        xSemaphoreTake(cmd_mutex, portMAX_DELAY);
        cmd_valid = g_cmd.valid;
        xSemaphoreGive(cmd_mutex);

        // Acil durum: hizli yanip son, komut yok: yavas yanip son,
        // normal: sabit yan
        static uint8_t div = 0;
        div++;
        if (g_emergency)
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        else if (!cmd_valid) {
            if (div % 5 == 0) digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        } else
            digitalWrite(LED_PIN, HIGH);

#ifdef STATUS_LED_PIN
        digitalWrite(STATUS_LED_PIN, digitalRead(LED_PIN));
#endif

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
    }
}

// --- Baslatma ---

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    protocol_init();
    delay(500);

    motor_init();
    safety_init();

    if (!imu_init())
        fatal_blink(100);
    if (!depth_init(&g_depth))
        fatal_blink(200);

    imu_calibrate(&g_imu, IMU_CALIB_SAMPLES);
    motor_arm_esc();

    memset(&g_cmd, 0, sizeof(g_cmd));
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        g_cmd.motor_pwm[i] = PWM_NEUTRAL;

    imu_mutex    = xSemaphoreCreateMutex();
    depth_mutex  = xSemaphoreCreateMutex();
    safety_mutex = xSemaphoreCreateMutex();
    cmd_mutex    = xSemaphoreCreateMutex();
    adc_mutex    = xSemaphoreCreateMutex();
    alive_group  = xEventGroupCreate();

    if (!imu_mutex || !depth_mutex || !safety_mutex || !cmd_mutex ||
        !adc_mutex || !alive_group)
        fatal_blink(50);

    // Yigin boyutlari word cinsinden
    xTaskCreate(control_task, "control", 1024, nullptr, 5, nullptr);
    xTaskCreate(imu_task,     "imu",     1024, nullptr, 4, nullptr);
    xTaskCreate(comm_task,    "comm",    1536, nullptr, 3, nullptr);
    xTaskCreate(sensor_task,  "sensor",  1024, nullptr, 2, nullptr);
    xTaskCreate(monitor_task, "monitor", 512,  nullptr, 1, nullptr);

    WDT_timings_t wdt_config;
    wdt_config.trigger  = 2.0f;
    wdt_config.timeout  = 3.0f;
    wdt_config.callback = wdt_callback;
    wdt.begin(wdt_config);

    digitalWrite(LED_PIN, LOW);
    vTaskStartScheduler();
}

void loop()
{
    // FreeRTOS calistiginda buraya gelinmez
}
