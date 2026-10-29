import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
import serial
import threading
import time

from stingray_msgs.msg import SensorData, MotorCommand  # type: ignore
from . import config
from .protocol import SensorFrameParser, encode_motor_command

class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('serial_port', config.SERIAL_PORT)
        self.declare_parameter('serial_baud', config.SERIAL_BAUD)
        self.declare_parameter('serial_timeout', config.SERIAL_TIMEOUT)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('serial_baud').value
        timeout = self.get_parameter('serial_timeout').value

        self.sensor_pub = self.create_publisher(
            SensorData, '/sensors/raw', 10)

        self.motor_sub = self.create_subscription(
            MotorCommand, '/cmd/motors',
            self._motor_cmd_callback, 10)

        self._ser = None
        self._lock = threading.Lock()
        self._parser = SensorFrameParser()
        self._running = True
        self._connected = False
        self._last_sensor_time = time.monotonic()

        try:
            self._ser = serial.Serial(port, baud, timeout=timeout)
            self._connected = True
            self.get_logger().info(
                f'Teensy bağlantısı kuruldu: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial port açılamadı: {e}')
            return

        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.create_timer(1.0, self._connection_check)

    def _motor_cmd_callback(self, msg: MotorCommand):
        if not self._connected or self._ser is None:
            return

        try:
            packet = encode_motor_command(msg.pwm_values,
                                          config.PWM_MIN, config.PWM_MAX)
        except ValueError as e:
            self.get_logger().error(f'Gecersiz motor komutu: {e}')
            return

        with self._lock:
            try:
                self._ser.write(packet)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial yazma hatası: {e}')
                self._connected = False

    def _rx_loop(self):
        while self._running and self._ser is not None:
            try:
                chunk = self._ser.read(64)
                if chunk:
                    self._publish(self._parser.feed(chunk))
            except serial.SerialException:
                time.sleep(0.05)

    def _publish(self, readings):
        for r in readings:
            msg = SensorData()
            msg.yaw = r.yaw
            msg.pitch = r.pitch
            msg.roll = r.roll
            msg.depth = r.depth
            msg.leak_detected = r.leak_detected
            msg.battery_voltage = r.battery_voltage
            msg.stamp = self.get_clock().now().to_msg()
            self.sensor_pub.publish(msg)
            self._last_sensor_time = time.monotonic()

    def _connection_check(self):
        elapsed = time.monotonic() - self._last_sensor_time
        if elapsed > config.COMM_TIMEOUT * 2:
            if self._connected:
                self.get_logger().warn(
                    f'Teensy bağlantı kaybı! ({elapsed:.1f}s)')
            self._connected = False
        else:
            if not self._connected:
                self.get_logger().info('Teensy bağlantısı yeniden kuruldu')
            self._connected = True

    def is_connected(self) -> bool:
        elapsed = time.monotonic() - self._last_sensor_time
        return elapsed < config.COMM_TIMEOUT

    def stop_all_motors(self):
        msg = MotorCommand()
        msg.pwm_values = [config.PWM_NEUTRAL] * 6
        self._motor_cmd_callback(msg)

    def destroy_node(self):
        self._running = False
        if self._ser is not None:
            self.stop_all_motors()
            time.sleep(0.1)
            try:
                self._ser.close()
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
