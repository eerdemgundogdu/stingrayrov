# Sensör verileri - Teensy'den gelen tüm sensör bilgileri
float64 yaw              # Yönelim (derece, 0-360)
float64 pitch            # Eğim (derece)
float64 roll             # Yalpa (derece)
float64 depth            # Derinlik (metre)
float64 depth_rate       # Dikey hiz (m/s, asagi pozitif) - Kalman filtresi
bool leak_detected       # Sızıntı algılandı mı
float64 battery_voltage  # Batarya voltajı (Volt)
builtin_interfaces/Time stamp  # Zaman damgası
