import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Float64, String, Bool  # type: ignore
from std_srvs.srv import Trigger  # type: ignore
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy  # type: ignore
import time

from stingray_msgs.msg import SensorData  # type: ignore
from . import config
from .utils import angle_diff, heading_reached, depth_reached

# FSM durumları
BEKLE = 0
DALIS = 1
ILERI_1 = 2
DONUS_1 = 3
ILERI_2 = 4
DAIRE_360 = 5
DONUS_2 = 6
ILERI_3 = 7
DONUS_3 = 8
ILERI_4 = 9
YUKSEL = 10
TAMAMLANDI = 11

STATE_NAMES = {
    BEKLE: "BEKLE", DALIS: "DALIS",
    ILERI_1: "ILERI_1", DONUS_1: "DONUS_1",
    ILERI_2: "ILERI_2", DAIRE_360: "DAIRE_360",
    DONUS_2: "DONUS_2", ILERI_3: "ILERI_3",
    DONUS_3: "DONUS_3", ILERI_4: "ILERI_4",
    YUKSEL: "YUKSEL", TAMAMLANDI: "TAMAMLANDI",
}

class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('dive_depth', config.DIVE_DEPTH)
        self.declare_parameter('leg_duration', config.LEG_DURATION)
        self.declare_parameter('forward_power', config.FORWARD_POWER)
        self.declare_parameter('depth_tolerance', config.DEPTH_TOLERANCE)
        self.declare_parameter('heading_tolerance', config.HEADING_TOLERANCE)

        self._dive_depth = self.get_parameter('dive_depth').value
        self._leg_duration = self.get_parameter('leg_duration').value
        self._forward_power = self.get_parameter('forward_power').value

        self.depth_pub = self.create_publisher(
            Float64, '/cmd/depth', 10)
        self.heading_pub = self.create_publisher(
            Float64, '/cmd/heading', 10)
        self.surge_pub = self.create_publisher(
            Float64, '/cmd/surge', 10)
        self.sway_pub = self.create_publisher(
            Float64, '/cmd/sway', 10)

        self.state_pub = self.create_publisher(
            String, '/mission/state', 10)

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
        self.create_subscription(
            Bool, '/safety/emergency',
            self._emergency_callback, 10)

        self.create_service(
            Trigger, '/mission/start', self._start_service)
        self.create_service(
            Trigger, '/mission/stop', self._stop_service)

        self.state = BEKLE
        self.initial_heading = 0.0
        self._state_start = 0.0
        self._circle_start_heading = 0.0
        self._circle_accumulated = 0.0
        self._circle_prev_heading = 0.0
        self._started = False
        self._emergency = False

        self._current_yaw = 0.0
        self._current_depth = 0.0
        self._sensor_ready = False

        self._last_log_time = 0.0

        self.create_timer(config.LOOP_DT, self._update)

        self.get_logger().info('Mission node hazır. '
                               'Başlatmak için: '
                               'ros2 service call /mission/start '
                               'std_srvs/srv/Trigger')

    def _sensor_callback(self, msg: SensorData):
        self._current_yaw = msg.yaw
        self._current_depth = msg.depth

    def _sensor_ready_callback(self, msg: Bool):
        self._sensor_ready = msg.data

    def _emergency_callback(self, msg: Bool):
        if msg.data and not self._emergency:
            self.get_logger().error('[ACIL] Güvenlik alarmı - görev durduruluyor')
            self._emergency = True
            self._send_surge(0.0)
            self._send_sway(0.0)

    def _start_service(self, request, response):
        if self._started:
            response.success = False
            response.message = 'Görev zaten çalışıyor'
            return response

        if not self._sensor_ready:
            response.success = False
            response.message = 'Sensör kalibrasyonu tamamlanmadan görev başlatılamaz'
            return response

        self.initial_heading = self._current_yaw
        self._started = True
        self._emergency = False
        self._enter(DALIS)

        self.get_logger().info(
            f'[GOREV] Başlangıç yönelimi: {self.initial_heading:.1f}°')
        response.success = True
        response.message = f'Görev başlatıldı. Yönelim: {self.initial_heading:.1f}°'
        return response

    def _stop_service(self, request, response):
        self._started = False
        self.state = TAMAMLANDI
        self._send_surge(0.0)
        self._send_sway(0.0)

        self.get_logger().info('[GOREV] Görev durduruldu')
        response.success = True
        response.message = 'Görev durduruldu'
        return response

    def _update(self):
        if not self._started or self._emergency:
            return

        state_msg = String()
        state_msg.data = STATE_NAMES.get(self.state, '?')
        self.state_pub.publish(state_msg)

        now = time.monotonic()
        if now - self._last_log_time > 1.0:
            self.get_logger().info(
                f'[{STATE_NAMES.get(self.state, "?")}] '
                f'D:{self._current_depth:.2f}m '
                f'Y:{self._current_yaw:.1f}°')
            self._last_log_time = now

        if self.state == DALIS:
            self._handle_dalis()
        elif self.state in (ILERI_1, ILERI_2, ILERI_3, ILERI_4):
            self._handle_ileri()
        elif self.state in (DONUS_1, DONUS_2, DONUS_3):
            self._handle_donus()
        elif self.state == DAIRE_360:
            self._handle_daire()
        elif self.state == YUKSEL:
            self._handle_yuksel()
        elif self.state == TAMAMLANDI:
            self._send_surge(0.0)
            self._send_sway(0.0)
            self._started = False

    def _enter(self, new_state):
        old_name = STATE_NAMES.get(self.state, '?')
        new_name = STATE_NAMES.get(new_state, '?')
        self.get_logger().info(f'[FSM] {old_name} -> {new_name}')

        self.state = new_state
        self._state_start = time.monotonic()

        if new_state == DALIS:
            self._send_depth(self._dive_depth)
            self._send_heading(self.initial_heading)
            self._send_surge(0.0)

        elif new_state == ILERI_1:
            heading = self.initial_heading
            self._send_heading(heading)
            self._send_depth(self._dive_depth)
            self._send_surge(self._forward_power)

        elif new_state == DONUS_1:
            heading = (self.initial_heading + 90.0) % 360.0
            self._send_heading(heading)
            self._send_depth(self._dive_depth)
            self._send_surge(0.0)

        elif new_state == ILERI_2:
            heading = (self.initial_heading + 90.0) % 360.0
            self._send_heading(heading)
            self._send_depth(self._dive_depth)
            self._send_surge(self._forward_power)

        elif new_state == DAIRE_360:
            self._circle_start_heading = self._current_yaw
            self._circle_accumulated = 0.0
            self._circle_prev_heading = self._current_yaw
            self._send_depth(self._dive_depth)
            self._send_surge(self._forward_power)

        elif new_state == DONUS_2:
            heading = (self.initial_heading + 180.0) % 360.0
            self._send_heading(heading)
            self._send_depth(self._dive_depth)
            self._send_surge(0.0)

        elif new_state == ILERI_3:
            heading = (self.initial_heading + 180.0) % 360.0
            self._send_heading(heading)
            self._send_depth(self._dive_depth)
            self._send_surge(self._forward_power)

        elif new_state == DONUS_3:
            heading = (self.initial_heading + 270.0) % 360.0
            self._send_heading(heading)
            self._send_depth(self._dive_depth)
            self._send_surge(0.0)

        elif new_state == ILERI_4:
            heading = (self.initial_heading + 270.0) % 360.0
            self._send_heading(heading)
            self._send_depth(self._dive_depth)
            self._send_surge(self._forward_power)

        elif new_state == YUKSEL:
            self._send_surge(0.0)
            self._send_depth(0.0)

    def _handle_dalis(self):
        self._send_depth(self._dive_depth)
        if depth_reached(self._dive_depth, self._current_depth):
            elapsed = time.monotonic() - self._state_start
            if elapsed > 2.0:  # 2 sn kararlılık bekleme
                self._enter(ILERI_1)

    def _handle_ileri(self):
        elapsed = time.monotonic() - self._state_start
        if elapsed >= self._leg_duration:
            transitions = {
                ILERI_1: DONUS_1,
                ILERI_2: DAIRE_360,
                ILERI_3: DONUS_3,
                ILERI_4: YUKSEL,
            }
            next_state = transitions.get(self.state)
            if next_state is not None:
                self._enter(next_state)

    def _handle_donus(self):
        if heading_reached(self._get_target_heading(), self._current_yaw):
            elapsed = time.monotonic() - self._state_start
            if elapsed > 1.0:  # 1 sn kararlılık bekleme
                transitions = {
                    DONUS_1: ILERI_2,
                    DONUS_2: ILERI_3,
                    DONUS_3: ILERI_4,
                }
                next_state = transitions.get(self.state)
                if next_state is not None:
                    self._enter(next_state)

    def _handle_daire(self):
        current = self._current_yaw
        delta = angle_diff(current, self._circle_prev_heading)
        self._circle_accumulated += delta
        self._circle_prev_heading = current

        elapsed = time.monotonic() - self._state_start

        # Dönüş hızını zamana yayıyoruz (16 saniyede tam tur)
        turn_rate = 360.0 / config.CIRCLE_DURATION
        target_heading = (self._circle_start_heading + (elapsed * turn_rate)) % 360.0
        self._send_heading(target_heading)

        # Dönüş tamamlanma şartı: hem fiziksel olarak 350+ derece dönmüş olacak, hem de kurallardaki 15 saniye (bizde 16.0) süresini geçirmiş olacak
        if self._circle_accumulated >= 350.0 and elapsed >= config.CIRCLE_DURATION:
            self.get_logger().info(
                f'[GOREV] 360° dönüş tamamlandı: '
                f'{self._circle_accumulated:.1f}° ({elapsed:.1f} sn)')
            self._enter(DONUS_2)

    def _handle_yuksel(self):
        self._send_depth(0.0)
        if self._current_depth < 0.1:
            elapsed = time.monotonic() - self._state_start
            if elapsed > 2.0:
                self._enter(TAMAMLANDI)
                self.get_logger().info('[GOREV] PARKUR TAMAMLANDI!')

    def _get_target_heading(self) -> float:
        offsets = {
            DONUS_1: 90.0,
            DONUS_2: 180.0,
            DONUS_3: 270.0,
        }
        offset = offsets.get(self.state, 0.0)
        return (self.initial_heading + offset) % 360.0

    def _send_depth(self, depth: float):
        msg = Float64()
        msg.data = depth
        self.depth_pub.publish(msg)

    def _send_heading(self, heading: float):
        msg = Float64()
        msg.data = heading % 360.0
        self.heading_pub.publish(msg)

    def _send_surge(self, power: float):
        msg = Float64()
        msg.data = max(-1.0, min(1.0, power))
        self.surge_pub.publish(msg)

    def _send_sway(self, power: float):
        msg = Float64()
        msg.data = max(-1.0, min(1.0, power))
        self.sway_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
