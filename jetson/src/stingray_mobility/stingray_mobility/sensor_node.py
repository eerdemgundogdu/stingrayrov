import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Float64, Bool  # type: ignore
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy  # type: ignore

from stingray_msgs.msg import SensorData  # type: ignore
from . import config
from .depth_filter import DepthKalmanFilter

class SensorNode(Node):

    def __init__(self):
        super().__init__('sensor_node')

        self.declare_parameter('depth_offset_samples', 20)
        self.declare_parameter('depth_offset', 0.0)
        self.declare_parameter('depth_kf_accel_std', 0.5)
        self.declare_parameter('depth_kf_meas_std', 0.02)

        self.raw_sub = self.create_subscription(
            SensorData, '/sensors/raw',
            self._raw_callback, 10)

        self.imu_pub = self.create_publisher(
            SensorData, '/sensors/imu', 10)
        self.depth_pub = self.create_publisher(
            Float64, '/sensors/depth', 10)
        self.battery_pub = self.create_publisher(
            Float64, '/sensors/battery', 10)
        ready_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.ready_pub = self.create_publisher(
            Bool, '/sensors/ready', ready_qos)

        self._depth_offset = self.get_parameter('depth_offset').value
        self._calib_samples = self.get_parameter(
            'depth_offset_samples').value
        self._calib_readings = []
        self._calibrated = (self._depth_offset != 0.0)

        if self._calibrated:
            self.get_logger().info(
                f'Derinlik ofseti (param): {self._depth_offset:.3f}m')
            self._publish_ready(True)
        else:
            self.get_logger().info(
                f'Derinlik kalibrasyonu başlatılıyor '
                f'({self._calib_samples} örnek)...')
            self._publish_ready(False)

        self.kf = DepthKalmanFilter(
            accel_std=self.get_parameter('depth_kf_accel_std').value,
            meas_std=self.get_parameter('depth_kf_meas_std').value)
        self._last_raw_depth = None
        self._last_time = None
        self._last_meas_time = 0.0

        self.rate_pub = self.create_publisher(
            Float64, '/sensors/depth_rate', 10)

        self.latest = None

    def _raw_callback(self, msg: SensorData):

        if not self._calibrated:
            self._calib_readings.append(msg.depth)
            if len(self._calib_readings) >= self._calib_samples:
                self._depth_offset = (
                    sum(self._calib_readings) /
                    len(self._calib_readings))
                self._calibrated = True
                self.get_logger().info(
                    f'Derinlik ofseti: {self._depth_offset:.3f}m')
                self._publish_ready(True)
            return

        adjusted_depth = msg.depth - self._depth_offset
        if adjusted_depth < 0.0:
            adjusted_depth = 0.0

        # Kalman filtresi: her pakette tahmin, sadece yeni sensor
        # okumasinda guncelleme (Teensy derinligi 10 Hz okuyor,
        # telemetri 50 Hz geliyor; ayni degeri tekrar tekrar olcum
        # saymak filtreyi gereginden fazla emin yapar).
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_time is not None:
            self.kf.predict(now - self._last_time)
        self._last_time = now
        if (adjusted_depth != self._last_raw_depth or
                now - self._last_meas_time > 0.15):
            self.kf.update(adjusted_depth)
            self._last_raw_depth = adjusted_depth
            self._last_meas_time = now
        filtered_depth = max(0.0, self.kf.depth)

        imu_msg = SensorData()
        imu_msg.yaw = msg.yaw
        imu_msg.pitch = msg.pitch
        imu_msg.roll = msg.roll
        imu_msg.depth = filtered_depth
        imu_msg.depth_rate = self.kf.rate
        imu_msg.leak_detected = msg.leak_detected
        imu_msg.battery_voltage = msg.battery_voltage
        imu_msg.stamp = msg.stamp
        self.imu_pub.publish(imu_msg)

        depth_msg = Float64()
        depth_msg.data = filtered_depth
        self.depth_pub.publish(depth_msg)

        rate_msg = Float64()
        rate_msg.data = self.kf.rate
        self.rate_pub.publish(rate_msg)

        bat_msg = Float64()
        bat_msg.data = msg.battery_voltage
        self.battery_pub.publish(bat_msg)

        self.latest = imu_msg

    def _publish_ready(self, ready: bool):
        msg = Bool()
        msg.data = ready
        self.ready_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
