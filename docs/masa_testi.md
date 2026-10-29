# Masa Testi

Aracı suya indirmeden önce sistemi masada (kuru ortamda) test etmek için adımlar.

## 1. Donanım Bağlantıları
Bağlantı pinleri:
- **IMU (MPU6050):** SDA (Pin 18), SCL (Pin 19) - 3.3V besleme.
- **Derinlik Sensörü:** Sinyal (Pin 16 / A2) - 5V ile çalışır, araya 10k-10k gerilim bölücü (voltage divider) takılmalı.
- **Sızıntı Sensörü:** Sinyal (Pin 14 / A0)
- **Batarya Voltajı:** Sinyal (Pin 15 / A1)
- **ESC'ler (PWM):** M0(Pin 2), M1(Pin 3), M2(Pin 4), M3(Pin 5), M4(Pin 6), M5(Pin 7). Tüm ESC GND kablolarını Teensy GND'ye bağlanmalı.

## 2. Teensy Kurulumu
1. `teensy/stingray_teensy/stingray_teensy.ino` dosyasını Arduino IDE ile açın.
2. Kart olarak "Teensy 4.1" seçip yükleyin.
3. Üzerindeki turuncu LED yarım saniyede bir yanıp sönüyorsa hazır demektir. (Çok hızlı yanıp sönüyorsa sensör kablolarını kontrol et).

## 3. Jetson (ROS2) Çalıştırma
Jetson üzerinde:

```bash
# 1. Paketi derle
colcon build
source install/setup.bash

# 2. Tüm sistemi (haberleşme, kontrol, görev vb.) başlat
ros2 launch stingray_mobility mobility_launch.py
```

## 4. Motor Testi
Sistem açıkken pervanelerin dönüp dönmediğini test etmek için yeni bir terminalde şu komutu çalıştırın:
```bash
ros2 topic pub --once /cmd/surge std_msgs/msg/Float64 "{data: 0.3}"
```
Durdurmak için `0.3` yerine `0.0` gönderin.

## 5. Otonom Görev (Sensör ve IMU) Testi
Masada test yaparken derinlik sensörü 0 metre okuyacağı için görevin takılmaması adına dalış hedefi sıfırlanır. Yeni bir terminalde:

```bash
# 1. Dalış hedefini 0 yap (masada takılı kalmaması için)
ros2 param set /mission_node dive_depth 0.0

# 2. Görevi başlat
ros2 service call /mission/start std_srvs/srv/Trigger
```

Görev başladığında motorlar döner. IMU elle çevrilerek 90 derecelik dönüşlerde motor tepkileri gözlemlenebilir. Görevi durdurmak için:
```bash
ros2 service call /mission/stop std_srvs/srv/Trigger
```
