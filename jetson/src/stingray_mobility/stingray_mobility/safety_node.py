import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Bool, Float64  # type: ignore
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy  # type: ignore
import time

from stingray_msgs.msg import SensorData, SafetyStatus, MotorCommand  # type: ignore
from . import config

class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')

        self.declare_parameter('leak_threshold', config.LEAK_THRESHOLD)
        self.declare_parameter('battery_low', config.BATTERY_LOW)
        self.declare_parameter('battery_critical', config.BATTERY_CRITICAL)
        self.declare_parameter('max_depth', config.MAX_DEPTH)
        self.declare_parameter('comm_timeout', config.COMM_TIMEOUT)

        self._leak_threshold = self.get_parameter('leak_threshold').value
        self._battery_low = self.get_parameter('battery_low').value
        self._battery_critical = self.get_parameter('battery_critical').value
        self._max_depth = self.get_parameter('max_depth').value
        self._comm_timeout = self.get_parameter('comm_timeout').value

        self.emergency_active = False
        self.emergency_reason = ""
        self._last_sensor_time = time.monotonic()
        self._last_warn_time = 0.0
        self._sensor_ready = False

        self._depth = 0.0
        self._yaw = 0.0
        self._battery_voltage = 25.2
        self._leak = False

        self.create_subscription(
            SensorData, '/sensors/imu',
            self._sensor_callback, 10)
        ready_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Bool, '/sensors/ready',
            self._sensor_ready_callback, ready_qos)

        self.emergency_pub = self.create_publisher(
            Bool, '/safety/emergency', 10)
        self.status_pub = self.create_publisher(
            SafetyStatus, '/safety/status', 10)
        self.motor_pub = self.create_publisher(
            MotorCommand, '/cmd/motors', 10)

        self.depth_pub = self.create_publisher(
            Float64, '/cmd/depth', 10)
        self.surge_pub = self.create_publisher(
            Float64, '/cmd/surge', 10)

        self.create_timer(config.LOOP_DT, self._safety_check)

        self.create_timer(0.5, self._publish_status)

        self.get_logger().info('Safety node başlatıldı')

    def _sensor_callback(self, msg: SensorData):
        self._depth = msg.depth
        self._yaw = msg.yaw
        self._battery_voltage = msg.battery_voltage
        self._leak = msg.leak_detected
        self._last_sensor_time = time.monotonic()

    def _sensor_ready_callback(self, msg: Bool):
        self._sensor_ready = msg.data
        if self._sensor_ready:
            self._last_sensor_time = time.monotonic()

    def _safety_check(self):

        if self.emergency_active:
            self._execute_surface()
            return

        if not self._sensor_ready:
            return

        if self._leak:
            self._trigger_emergency("SIZINTI ALGILANDI")
            return

        elapsed = time.monotonic() - self._last_sensor_time
        if elapsed > self._comm_timeout:
            self._trigger_emergency(
                f"HABERLESME KAYBI ({elapsed:.1f}s)")
            return

        if self._battery_voltage < self._battery_critical:
            self._trigger_emergency(
                f"BATARYA KRITIK: {self._battery_voltage:.1f}V")
            return

        if self._depth > self._max_depth:
            self._trigger_emergency(
                f"DERINLIK SINIRI: {self._depth:.2f}m")
            return

        now = time.monotonic()
        if self._battery_voltage < self._battery_low:
            if now - self._last_warn_time > 10.0:
                self.get_logger().warn(
                    f'[UYARI] Düşük batarya: '
                    f'{self._battery_voltage:.1f}V')
                self._last_warn_time = now

    def _trigger_emergency(self, reason: str):
        self.emergency_active = True
        self.emergency_reason = reason
        self.get_logger().error(f'[ACIL] {reason}')

        motor_msg = MotorCommand()
        motor_msg.pwm_values = [config.PWM_NEUTRAL] * 6
        self.motor_pub.publish(motor_msg)

        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_pub.publish(emergency_msg)

    def _execute_surface(self):
        # Araç acil durumda donanım seviyesinde (Teensy) motorları kapatıp 
        # pozitif yüzerlikle yukarı çıkacak şekilde tasarlandığı için
        # sadece nötr PWM gönderiyoruz.
        motor_msg = MotorCommand()
        motor_msg.pwm_values = [config.PWM_NEUTRAL] * 6
        self.motor_pub.publish(motor_msg)

    def _publish_status(self):
        elapsed = time.monotonic() - self._last_sensor_time
        comm_lost = self._sensor_ready and (elapsed > self._comm_timeout)

        msg = SafetyStatus()
        msg.emergency_active = self.emergency_active
        msg.leak_detected = self._leak
        msg.comm_lost = comm_lost
        msg.battery_low = (self._battery_voltage < self._battery_low)
        msg.battery_critical = (
            self._battery_voltage < self._battery_critical)
        msg.depth_exceeded = (self._depth > self._max_depth)
        msg.battery_voltage = self._battery_voltage
        msg.reason = self.emergency_reason if self.emergency_active else ""

        self.status_pub.publish(msg)

        emergency_msg = Bool()
        emergency_msg.data = self.emergency_active
        self.emergency_pub.publish(emergency_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
