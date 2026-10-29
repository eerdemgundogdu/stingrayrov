#include "depth_sensor.h"
#include "config.h"
#include "dsp_filter.h"
#include <Arduino.h>

static Median5Filter s_depth_filter;

bool depth_init(Depth_Data *data)
{
    pinMode(DEPTH_SENSOR_PIN, INPUT);
    // ADC çözünürlüğünü config.h'daki değere göre ayarla
    analogReadResolution(ADC_RESOLUTION_BITS);

    s_depth_filter.init(0.35f);

    data->pressure = 0.0f;
    data->depth = 0.0f;
    data->atm_pressure = 1013.25f;  // Başlangıç değeri, Jetson bunu kendi ofsetleyecek

    return true;
}

void depth_read(Depth_Data *data)
{
    // 1. ADC okuma
    int raw_adc = analogRead(DEPTH_SENSOR_PIN);

    // 2. ADC değerini voltaja çevirme (Teensy pinindeki voltaj)
    float pin_voltage = (float)raw_adc * DEPTH_ADC_VREF / (float)ADC_MAX_VALUE;

    // 3. 5-örneklik medyan + EMA filtre (ESC anahtarlama dikenlerini bastırır)
    float filtered_pin_v = s_depth_filter.step(pin_voltage);

    // 4. Gerçek sensör voltajını hesaplama (Gerilim bölücü hesabı)
    float sensor_voltage = filtered_pin_v / DEPTH_VOLTAGE_DIVIDER;

    // 5. Voltajı basınca çevirme (DFRobot formülü: 0.5V = 0 MPa, 4.5V = DEPTH_MAX_MPA)
    // P_MPa = (V - 0.5) / 4.0 * MAX_MPA
    float pressure_mpa = (sensor_voltage - 0.5f) / 4.0f * DEPTH_MAX_MPA;
    if (pressure_mpa < 0.0f) pressure_mpa = 0.0f;

    // 6. MPa'yı mbar (hPa)'ya çevirme (Jetson ve ROS2 uyumluluğu için)
    // 1 MPa = 10 bar = 10000 mbar
    data->pressure = pressure_mpa * 10000.0f;

    // 7. Derinlik hesabı
    // P_Pa = P_MPa * 1,000,000
    float pressure_pa = pressure_mpa * 1000000.0f;
    data->depth = pressure_pa / (FLUID_DENSITY * GRAVITY);
}
