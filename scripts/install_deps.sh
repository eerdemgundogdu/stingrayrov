#!/bin/bash

set -e

echo "========================================="
echo " STINGRAY - Bağımlılık Kurulumu"
echo "========================================="

if [ -z "$ROS_DISTRO" ]; then
    echo "[HATA] ROS2 bulunamadı! Önce ROS2 Humble kurun."
    echo "  https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

if [ "$ROS_DISTRO" != "humble" ]; then
    echo "[UYARI] ROS2 $ROS_DISTRO tespit edildi. Humble bekleniyor."
fi

echo "[1/4] Python bağımlılıkları kuruluyor..."
pip3 install pyserial

echo "[2/4] ROS2 bağımlılıkları kuruluyor..."
sudo apt-get update
sudo apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-std-msgs \
    ros-humble-std-srvs

echo "[3/4] Teensy udev kuralı ekleniyor..."
UDEV_RULE='SUBSYSTEM=="usb", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", MODE="0666", GROUP="dialout"'
UDEV_FILE="/etc/udev/rules.d/49-teensy.rules"

if [ ! -f "$UDEV_FILE" ]; then
    echo "$UDEV_RULE" | sudo tee "$UDEV_FILE"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "  Teensy udev kuralı eklendi."
else
    echo "  Teensy udev kuralı zaten mevcut."
fi

if ! groups | grep -q dialout; then
    sudo usermod -aG dialout $USER
    echo "  Kullanıcı dialout grubuna eklendi (yeniden giriş gerekli)."
fi

echo "[4/4] ROS2 workspace build ediliyor..."
cd "$(dirname "$0")/../jetson"
source /opt/ros/humble/setup.bash

colcon build --packages-select stingray_msgs
source install/setup.bash

colcon build --packages-select stingray_mobility
source install/setup.bash

echo ""
echo "========================================="
echo " Kurulum tamamlandı!"
echo "========================================="
echo ""
echo " Kullanım:"
echo "   cd jetson/"
echo "   source install/setup.bash"
echo "   ros2 launch stingray_mobility mobility_launch.py"
echo ""
echo " Görevi başlatmak:"
echo '   ros2 service call /mission/start std_srvs/srv/Trigger'
echo ""
