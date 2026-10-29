#ifndef CONFIG_H
#define CONFIG_H

/* ---- Motor PWM Pinleri ---- */
#define MOTOR0_PIN      2       // Yatay sol-ön (FlexPWM4.2)
#define MOTOR1_PIN      3       // Yatay sağ-ön (FlexPWM4.2)
#define MOTOR2_PIN      4       // Yatay sol-arka (FlexPWM2.0)
#define MOTOR3_PIN      5       // Yatay sağ-arka (FlexPWM2.1)
#define MOTOR4_PIN      6       // Dikey ön (FlexPWM2.2)
#define MOTOR5_PIN      7       // Dikey arka (FlexPWM1.3)

#define MOTOR_COUNT     6

#define PWM_MIN         1100
#define PWM_MAX         1900
#define PWM_NEUTRAL     1500
#define PWM_DEADBAND_US 12      // Nötr çevresinde ölü bölge (±12 µs)
#define PWM_SLEW_US_MS  20      // 1 kHz döngüde maksimum adım (20 µs/ms -> 0..tam güç 20 ms)
#define ESC_ARM_MS      3000

/* ---- Taşıyıcı Kart (hardware/ klasörü) ----
 * 0 = kartsız mevcut elle kablolama (varsayılan)
 * 1 = Rev A taşıyıcı kart (manyetik kill-switch döngüsü, ESC kontaktörü, buzzer, durum LED)
 * PlatformIO'da `pio run -e freertos_carrier` veya `-e superloop_carrier` bunu 1 ile derler.
 */
#ifndef CARRIER_BOARD_REV
#define CARRIER_BOARD_REV       0
#endif

#if CARRIER_BOARD_REV >= 1
// Kartta manyetik kill-switch döngüsü: +24V -> J11 (NC manyetik anahtar) -> KILL_RET -> J12 (kontaktör).
// Mıknatıs çekilince kontaktör yazılımdan bağımsız düşer. Pin 36, 47k/10k bölücü + 3.3V zener ile
// döngüyü okur: normal = HIGH, çekili = LOW. Harici bölücü olduğu için INPUT_PULLUP kullanılmaz.
#define KILL_SWITCH_ENABLED     1
#define KILL_SENSE_PIN          36
#define KILL_SENSE_PIN_MODE     INPUT
// Q2 (BCP56-16) -> ESC güç kontaktörü bobini. HIGH = itici gücü açık.
// Teensy resetteyken 10k pull-down bobini kapalı tutar.
#define ESC_EN_PIN              38
#define BUZZER_PIN              37
#define STATUS_LED_PIN          11
// Kart üzerinde 68k / 10k (%1) batarya gerilim bölücü: (68 + 10) / 10 = 78 / 10
#define BATT_DIV_NUM            78
#define BATT_DIV_DEN            10
#else
#define KILL_SWITCH_ENABLED     0
#define KILL_SENSE_PIN          36
#define KILL_SENSE_PIN_MODE     INPUT_PULLUP
#define BATT_VOLTAGE_MULTIPLIER 8
#define BATT_DIV_NUM            (BATT_VOLTAGE_MULTIPLIER * 10)
#define BATT_DIV_DEN            10
#endif

/* IMU (MPU6050) -> Wire (Bus 0): Pin 18 (SDA0), Pin 19 (SCL0) */
#define I2C_CLOCK_HZ    400000

#define MPU6050_ADDR            0x68
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG_REG      0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B

#define GYRO_SENSITIVITY        (2000.0f / 32768.0f)
#define ACCEL_SENSITIVITY       (2.0f / 32768.0f)
#define COMP_FILTER_ALPHA       0.98f
#define IMU_CALIB_SAMPLES       500

/* Derinlik (DFRobot SEN0257 Analog Basınç Sensörü) */
#define DEPTH_SENSOR_PIN        A2      // Pin 16
#define DEPTH_VOLTAGE_DIVIDER   0.5f    // 5V sensörü 2.5V'a düşüren 10k-10k gerilim bölücü
#define DEPTH_MAX_MPA           1.6f    // Sensör maksimum kapasitesi (1.6 MPa)
#define DEPTH_ADC_VREF          3.3f    // Teensy analog referans voltajı

#define FLUID_DENSITY           1000.0f
#define GRAVITY                 9.80665f

/* ---- Analog Sensör Pinleri ---- */
#define LEAK_SENSOR_PIN         A0  // Pin 14
#define BATTERY_SENSOR_PIN      A1  // Pin 15
#define ADC_RESOLUTION_BITS     12
#define ADC_MAX_VALUE           4095

#define LEAK_ADC_THRESHOLD      2000
#define BATTERY_LOW_MV          21000   // 6S Li-Po 3.5 V/hücre uyarı
#define BATTERY_CRITICAL_MV     19800   // 6S Li-Po 3.3 V/hücre kritik
#define BATTERY_SHUTOFF_MV      18000   // 6S Li-Po 3.0 V/hücre kesme
#define COMM_TIMEOUT_MS         500

#define SERIAL_BAUD             115200

#define LOOP_INTERVAL_US        1000
#define SENSOR_SEND_MS          20
#define DEPTH_READ_MS           100
#define LED_PIN                 13      // Teensy 4.1 onboard LED

#endif
