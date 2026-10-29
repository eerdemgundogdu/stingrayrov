import math

SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.01

LOOP_HZ = 50
LOOP_DT = 1.0 / LOOP_HZ

PWM_MIN = 1100
PWM_MAX = 1900
PWM_NEUTRAL = 1500
PWM_DEADZONE = 25

# 4 yatay (X-config, 45°) + 2 dikey
#
#        Ön (Bow)
#    M0 \      / M1
#        \    /
#         \  /
#         /  \
#        /    \
#    M2 /      \ M3
#        Arka (Stern)
#
#    M4: Ön dikey    M5: Arka dikey
#
S = math.sin(math.radians(45))

THRUSTER_MATRIX = [
    [+S,  -S,   0.0,  +1.0],   # M0: sol-ön yatay
    [+S,  +S,   0.0,  -1.0],   # M1: sağ-ön yatay
    [-S,  +S,   0.0,  +1.0],   # M2: sol-arka yatay
    [-S,  -S,   0.0,  -1.0],   # M3: sağ-arka yatay
    [0.0,  0.0, +1.0,  0.0],   # M4: ön dikey
    [0.0,  0.0, +1.0,  0.0],   # M5: arka dikey
]

DEPTH_KP = 400.0
DEPTH_KI = 30.0
DEPTH_KD = 150.0
DEPTH_MAX_OUTPUT = 350.0

YAW_KP = 3.5
YAW_KI = 0.1
YAW_KD = 1.2
YAW_MAX_OUTPUT = 300.0

DIVE_DEPTH = 1.0           # Hedef dalış derinliği (metre)
DEPTH_TOLERANCE = 0.08     # Derinlik toleransı (metre)
HEADING_TOLERANCE = 3.0    # Yönelim toleransı (derece)
LEG_DURATION = 16.0        # Her bacak süresi (saniye) - Kural: min 15 sn
FORWARD_POWER = 0.35       # İleri hareket güç oranı (0-1)
CIRCLE_DURATION = 16.0     # 360° dönüş süresi (saniye) - Kural: min 15 sn

COMM_TIMEOUT = 0.5         # Haberleşme timeout (saniye)
LEAK_THRESHOLD = 500       # Sızıntı ADC eşik değeri
BATTERY_LOW = 21.0         # Düşük batarya uyarısı (Volt)
BATTERY_CRITICAL = 19.8    # Kritik batarya (Volt)
MAX_DEPTH = 3.0            # Maksimum derinlik limiti (metre)
