// PlatformIO giris noktasi: superloop firmware'i.
//
// PlatformIO .ino dosyalarini yalnizca src_dir kokunde (teensy/) arar;
// stingray_teensy/stingray_teensy.ino alt klasorde oldugu icin dogrudan
// derlenmez ve setup()/loop() tanimsiz kalir. Sketch, Arduino IDE ile de
// acilabilsin diye yerinde birakildi ve burada derlemeye dahil ediliyor.

#include <Arduino.h>
#include <Watchdog_t4.h>   // kutuphane bagimlilik tarayicisi (LDF) gorsun diye
#include "../stingray_teensy/stingray_teensy.ino"
