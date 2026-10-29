#ifndef DEPTH_SENSOR_H
#define DEPTH_SENSOR_H

#include <Arduino.h>

/* ============================================================
 *  Derinlik Sensörü (DFRobot Analog Su Basınç) - Teensy 4.1
 *  Analog giriş: Pin A2 (Gerilim bölücü ile)
 *
 *  0-1.6 MPa basınç aralığı, 0.5V-4.5V çıkış
 *  Atmosferik ofset Jetson sensor_node tarafından hesaplanır
 * ============================================================ */

typedef struct {
    float pressure;       // mbar (veya hPa)
    float depth;          // metre
    float atm_pressure;   // mbar
} Depth_Data;

bool depth_init(Depth_Data *data);
void depth_read(Depth_Data *data);

#endif
