# Kablolama — Stingray ROV taşıyıcı kart Rev A

Sızdırmaz elektronik tüp içindeki her konnektörün pin pin nereye gittiği. Pin 1, kart üzerinde kare pedli olan pindir. JST XH konnektörler kilitlidir, ters takılamaz.

> **Rev A henüz üretilmedi ve test edilmedi.** İlk enerji vermeden önce [DESIGN.md](DESIGN.md) içindeki devreye alma adımlarını uygulayın.

## Güç ve güvenlik

### J1 — 6S Li-Po batarya girişi (vidalı klemens, 5.08 mm)

| Pin | Net | Bağlantı |
|----|-----|----------|
| 1 | `VBAT_IN` | 6S Li-Po + (ana sigortadan **sonra**, ESC kontaktöründen **önce**) |
| 2 | `GND` | 6S Li-Po − (ortak şase barası) |

Kartın kendi sigortası (`F1`, 1 A) yalnızca lojik devreyi ve kontaktör bobinini korur. 6 adet ESC'nin yüksek akımlı güç hatları karttan geçmez; ana kontaktör üzerinden doğrudan güç dağıtım barasına bağlanır.

### J11 — Su geçirmez manyetik kill-switch (JST XH 2p)

| Pin | Net | Bağlantı |
|----|-----|----------|
| 1 | `+24V` | Manyetik anahtarın **NC** (normalde kapalı) kontağı, uç 1 |
| 2 | `KILL_RET` | NC kontak, uç 2 |

Mıknatıs takılıyken kontak kapalıdır. Mıknatıs çekildiğinde veya kablo koptuğunda döngü açılır ve itici kontaktörü yazılımdan bağımsız olarak düşer (fail-safe).

### J12 — Ana itici gücü kontaktörü bobini (JST XH 2p)

| Pin | Net | Bağlantı |
|----|-----|----------|
| 1 | `KILL_RET` | Bobin + (kill-switch'ten gelen 6S gerilimi) |
| 2 | `COIL_N` | Bobin − (kartta `Q2` ile `GND`'ye anahtarlanır) |

Bobin 24 V DC sınıfında ve **en fazla 0.4 A** olmalıdır. Serbest geçiş diyodu (`D10`) kart üzerindedir.

## İtici ESC çıkışları

### J5 … J10 — 6× Fırçasız İtici ESC PWM (JST XH 3p)

| Konnektör | Kanal | Konum | Teensy Pini |
|-----------|-------|-------|-------------|
| `J5` | `M0` | Yatay Sol-Ön (45° vektörel) | Pin 2 (`FlexPWM4.2`) |
| `J6` | `M1` | Yatay Sağ-Ön (45° vektörel) | Pin 3 (`FlexPWM4.2`) |
| `J7` | `M2` | Yatay Sol-Arka (45° vektörel) | Pin 4 (`FlexPWM2.0`) |
| `J8` | `M3` | Yatay Sağ-Arka (45° vektörel) | Pin 5 (`FlexPWM2.1`) |
| `J9` | `M4` | Dikey Ön (Dalış/Yükseliş) | Pin 6 (`FlexPWM2.2`) |
| `J10` | `M5` | Dikey Arka (Dalış/Yükseliş) | Pin 7 (`FlexPWM1.3`) |

Her `J5..J10` konnektörünün pin dizilimi:

| Pin | Net | ESC kablosu |
|----|-----|-------------|
| 1 | `GND` | Siyah (GND) |
| 2 | Boş (`NC`) | Kırmızı (ESC BEC 5 V — **kartta karşılığı yok, birbirine bağlanmaz**) |
| 3 | `ESC_PWM0..5` | Beyaz / Sarı (PWM sinyal, `1100–1900 µs`, `1500 µs` nötr) |

## Sensörler

### J2 — Hidrostatik basınç / derinlik sensörü (JST XH 3p)

| Pin | Net | Sensör (`DFRobot SEN0257`) |
|----|-----|----------------------------|
| 1 | `+5V` | Kırmızı (VCC, 5 V) |
| 2 | `GND` | Siyah (GND) |
| 3 | `DEPTH_RAW` | Sarı / Yeşil (`0.5–4.5 V` analog çıkış → `10k/10k` bölücü → Teensy `Pin 16 / A2`) |

Gerilim bölücü (`R21/R22`), `100 nF` filtre (`C11`) ve `BAT54S` aşırı gerilim kıskacı (`D6`) kart üzerindedir; kablo demetine harici direnç takılmaz.

### J3 — MPU6050 6-eksen IMU (JST XH 4p)

| Pin | Net | MPU6050 modülü |
|----|-----|----------------|
| 1 | `+3V3` | VCC (`AP2112K-3.3` LDO çıkışı) |
| 2 | `GND` | GND |
| 3 | `IMU_SCL` | SCL (Teensy `Pin 19 / SCL0`, `4.7 kΩ` pull-up + `33 Ω` seri) |
| 4 | `IMU_SDA` | SDA (Teensy `Pin 18 / SDA0`, `4.7 kΩ` pull-up + `33 Ω` seri) |

### J4 — Tüp içi su sızıntı probu (JST XH 2p)

| Pin | Net | Bağlantı |
|----|-----|----------|
| 1 | `LEAK_P` | Elektrot 1 (`+3V3` hattından `10 kΩ` akım sınırlamalı) |
| 2 | `LEAK_NODE` | Elektrot 2 (`47 kΩ` pull-down + `100 nF` filtre → Teensy `Pin 14 / A0`) |

## Teensy 4.1 pin eşlemesi

| Teensy pini | Net | Görev |
|-------------|-----|-------|
| `VIN` | `TEENSY_VIN` | Kart `+5V` hattı (`D4` Schottky üzerinden) |
| `2 … 7` | `T_PWM0` … `T_PWM5` | 6× ESC PWM çıkışı (`M0..M5`) |
| `11` | `T_LED` | Kart üstü mavi durum LED'i (`D9`) |
| `14 (A0)` | `T_LEAK_ADC` | Su sızıntı sensörü analog girişi |
| `15 (A1)` | `T_BATT_ADC` | 6S batarya gerilim ölçümü (`68 kΩ / 10 kΩ` bölücü) |
| `16 (A2)` | `T_DEPTH_ADC` | Basınç/derinlik sensörü analog girişi (`10 kΩ / 10 kΩ` bölücü) |
| `18 / 19` | `T_SDA0` / `T_SCL0` | `MPU6050` IMU (`I2C0`, 400 kHz) |
| `36` | `T_KILL_SENSE` | Manyetik kill-switch döngü algılama (normal `HIGH`, çekili `LOW`) |
| `37` | `T_BUZZER` | Tüp içi akustik buzzer (`Q1`) |
| `38` | `T_ESC_EN` | ESC kontaktör bobini kontrolü (`Q2`) — `CARRIER_BOARD_REV=1` |

## Kart üstü göstergeler ve test noktaları

| Eleman | Net / Renk | Anlamı |
|--------|------------|--------|
| `D5` | Yeşil LED | Kart `+5V` regülatörü aktif |
| `D9` | Mavi LED | Firmware durum LED'i (`Pin 11`) |
| `D11` | Kırmızı LED | ESC güç kontaktörü enerjili (iticilere güç gidiyor) |
| `BZ1` | 5 V Buzzer | Arming / sızıntı / kritik batarya sesli ikazı (`Pin 37`) |
| `TP1` | `+24V` | 6S Li-Po − ~0.35 V (`D1` düşümü) |
| `TP2` | `+5V` | `4.85 … 5.08 V` |
| `TP3` | `+3V3` | `3.23 … 3.37 V` |
| `TP4` | `GND` | `0 V` referans |
