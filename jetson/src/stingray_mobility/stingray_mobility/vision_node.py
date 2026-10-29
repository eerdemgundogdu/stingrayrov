import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Float64, Bool  # type: ignore
from sensor_msgs.msg import Image  # type: ignore
import numpy as np

from .vision_pipeline import detect_pipeline


class VisionNode(Node):
    """Su alti kamera goruntulerinden boru hatti / iz takibi yapan ROS 2 dugumu.

    Goruntu isleme cekirdegi (vision_pipeline.py) ROS'tan bagimsizdir ve
    jetson/test/test_vision.py ile donanimsiz test edilir.
    """

    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('min_pixels', 80)
        self.declare_parameter('lateral_gain_deg', 25.0)
        self.declare_parameter('heading_gain', 0.6)

        self._min_pixels = self.get_parameter('min_pixels').value
        self._lat_gain = self.get_parameter('lateral_gain_deg').value
        self._head_gain = self.get_parameter('heading_gain').value

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, 10)

        self.lateral_pub = self.create_publisher(
            Float64, '/vision/lateral_error', 10)
        self.heading_err_pub = self.create_publisher(
            Float64, '/vision/heading_error', 10)
        self.detected_pub = self.create_publisher(
            Bool, '/vision/pipeline_detected', 10)

        self.get_logger().info('Vision node başlatıldı (boru hattı ve kapı tespiti)')

    def _image_callback(self, msg: Image):
        if msg.height == 0 or msg.width == 0:
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * msg.width * 3
        if frame.size < expected:
            return
        rgb = frame[:expected].reshape((msg.height, msg.width, 3))
        if msg.encoding.lower() == 'bgr8':
            rgb = rgb[:, :, ::-1]

        res = detect_pipeline(rgb, min_pixels=self._min_pixels)

        det_msg = Bool()
        det_msg.data = res.detected
        self.detected_pub.publish(det_msg)

        if res.detected:
            lat_msg = Float64()
            lat_msg.data = res.lateral_error
            self.lateral_pub.publish(lat_msg)

            head_msg = Float64()
            head_msg.data = (self._lat_gain * res.lateral_error +
                             self._head_gain * res.heading_error_deg)
            self.heading_err_pub.publish(head_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
