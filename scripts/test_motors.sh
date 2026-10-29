#!/bin/bash

set -e

echo "========================================="
echo " STINGRAY - Motor Kuru Test"
echo "========================================="
echo ""
echo " Önce serial_bridge node'unun çalıştığından emin olun:"
echo "   ros2 run stingray_mobility serial_bridge"
echo ""
echo " Motor Numaraları:"
echo "   M0: Sol-ön yatay    M1: Sağ-ön yatay"
echo "   M2: Sol-arka yatay  M3: Sağ-arka yatay"
echo "   M4: Ön dikey        M5: Arka dikey"
echo ""
echo " Güvenlik: Kuru test için max %10 güç kullanın!"
echo "========================================="
echo ""

echo "Motorlar nötre alınıyor..."
ros2 topic pub --once /cmd/motors stingray_msgs/msg/MotorCommand \
    "{pwm_values: [1500, 1500, 1500, 1500, 1500, 1500]}"

sleep 1

echo ""
read -p "Motor numarası (0-5) girin [q=çık, s=durdur]: " motor_id

while [ "$motor_id" != "q" ]; do
    if [ "$motor_id" = "s" ]; then
        echo "Tüm motorlar durduruluyor..."
        ros2 topic pub --once /cmd/motors stingray_msgs/msg/MotorCommand \
            "{pwm_values: [1500, 1500, 1500, 1500, 1500, 1500]}"
        echo ""
        read -p "Motor numarası (0-5) girin [q=çık, s=durdur]: " motor_id
        continue
    fi

    read -p "Güç yüzdesi (-100 ile +100, kuru test için 10): " power

    pwm=$(python3 -c "print(int(1500 + 400 * ($power / 100.0)))")

    pwm_array="[1500, 1500, 1500, 1500, 1500, 1500]"
    case $motor_id in
        0) pwm_array="[$pwm, 1500, 1500, 1500, 1500, 1500]" ;;
        1) pwm_array="[1500, $pwm, 1500, 1500, 1500, 1500]" ;;
        2) pwm_array="[1500, 1500, $pwm, 1500, 1500, 1500]" ;;
        3) pwm_array="[1500, 1500, 1500, $pwm, 1500, 1500]" ;;
        4) pwm_array="[1500, 1500, 1500, 1500, $pwm, 1500]" ;;
        5) pwm_array="[1500, 1500, 1500, 1500, 1500, $pwm]" ;;
        *) echo "Hata: Motor 0-5 arası olmalı!"; continue ;;
    esac

    echo "M$motor_id çalıştırılıyor... PWM: $pwm (Güç: %$power)"
    ros2 topic pub --once /cmd/motors stingray_msgs/msg/MotorCommand \
        "{pwm_values: $pwm_array}"

    echo ""
    read -p "Motor numarası (0-5) girin [q=çık, s=durdur]: " motor_id
done

echo "Motorlar kapatılıyor..."
ros2 topic pub --once /cmd/motors stingray_msgs/msg/MotorCommand \
    "{pwm_values: [1500, 1500, 1500, 1500, 1500, 1500]}"

echo "Test tamamlandı."
