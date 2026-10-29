import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Float64, Bool  # type: ignore
import time

from stingray_msgs.msg import SensorData, MotorCommand  # type: ignore
from . import config
from .utils import angle_diff

class PID:

    def __init__(self, kp, ki, kd, max_output, max_integral=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.max_integral = max_integral or (max_output * 0.5)

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
        self._prev_derivative = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
        self._prev_derivative = 0.0

    def compute(self, error, dt=None, error_rate=None):
        # error_rate verilirse turev terimi sayisal turev yerine onu
        # kullanir (ornegin Kalman filtresinden gelen dikey hiz).
        now = time.monotonic()

        if dt is None:
            if self._prev_time is None:
                self._prev_time = now
                self._prev_error = error
                return 0.0
            dt = now - self._prev_time

        if dt <= 0:
            return 0.0

        self._prev_time = now

        p_term = self.kp * error

        self._integral += error * dt
        if self._integral > self.max_integral:
            self._integral = self.max_integral
        elif self._integral < -self.max_integral:
            self._integral = -self.max_integral
        i_term = self.ki * self._integral

        if error_rate is not None:
            self._prev_derivative = error_rate
        else:
            raw_d = (error - self._prev_error) / dt
            self._prev_derivative = (0.3 * self._prev_derivative +
                                     0.7 * raw_d)
        d_term = self.kd * self._prev_derivative

        self._prev_error = error

        output = p_term + i_term + d_term
        if output > self.max_output:
            output = self.max_output
        elif output < -self.max_output:
            output = -self.max_output

        return output

class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('depth_kp', config.DEPTH_KP)
        self.declare_parameter('depth_ki', config.DEPTH_KI)
        self.declare_parameter('depth_kd', config.DEPTH_KD)
        self.declare_parameter('depth_max_output', config.DEPTH_MAX_OUTPUT)
        self.declare_parameter('yaw_kp', config.YAW_KP)
        self.declare_parameter('yaw_ki', config.YAW_KI)
        self.declare_parameter('yaw_kd', config.YAW_KD)
        self.declare_parameter('yaw_max_output', config.YAW_MAX_OUTPUT)

        self.depth_pid = PID(
            self.get_parameter('depth_kp').value,
            self.get_parameter('depth_ki').value,
            self.get_parameter('depth_kd').value,
            self.get_parameter('depth_max_output').value
        )
        self.yaw_pid = PID(
            self.get_parameter('yaw_kp').value,
            self.get_parameter('yaw_ki').value,
            self.get_parameter('yaw_kd').value,
            self.get_parameter('yaw_max_output').value
        )

        self.target_depth = 0.0
        self.target_heading = 0.0
        self.surge_power = 0.0
        self.sway_power = 0.0
        self.emergency = False

        self._current_depth = 0.0
        self._current_depth_rate = 0.0
        self._current_heading = 0.0

        self.create_subscription(
            SensorData, '/sensors/imu',
            self._sensor_callback, 10)
        self.create_subscription(
            Float64, '/cmd/depth',
            self._depth_cmd_callback, 10)
        self.create_subscription(
            Float64, '/cmd/heading',
            self._heading_cmd_callback, 10)
        self.create_subscription(
            Float64, '/cmd/surge',
            self._surge_cmd_callback, 10)
        self.create_subscription(
            Float64, '/cmd/sway',
            self._sway_cmd_callback, 10)
        self.create_subscription(
            Bool, '/safety/emergency',
            self._emergency_callback, 10)

        self.motor_pub = self.create_publisher(
            MotorCommand, '/cmd/motors', 10)

        self.create_timer(config.LOOP_DT, self._control_loop)

        self.get_logger().info('Controller node başlatıldı (50 Hz)')

    def _sensor_callback(self, msg: SensorData):
        self._current_depth = msg.depth
        self._current_depth_rate = msg.depth_rate
        self._current_heading = msg.yaw

    def _depth_cmd_callback(self, msg: Float64):
        self.target_depth = msg.data

    def _heading_cmd_callback(self, msg: Float64):
        self.target_heading = msg.data % 360.0

    def _surge_cmd_callback(self, msg: Float64):
        self.surge_power = max(-1.0, min(1.0, msg.data))

    def _sway_cmd_callback(self, msg: Float64):
        self.sway_power = max(-1.0, min(1.0, msg.data))

    def _emergency_callback(self, msg: Bool):
        self.emergency = msg.data
        if self.emergency:
            self.surge_power = 0.0
            self.sway_power = 0.0

    def reset_controllers(self):
        self.depth_pid.reset()
        self.yaw_pid.reset()
        self.surge_power = 0.0
        self.sway_power = 0.0

    def _control_loop(self):

        if self.emergency:
            msg = MotorCommand()
            msg.pwm_values = [config.PWM_NEUTRAL] * 6
            self.motor_pub.publish(msg)
            return

        dt = config.LOOP_DT

        depth_err = self.target_depth - self._current_depth
        # Hedef sabitken d(hata)/dt = -dikey hiz
        heave_cmd = self.depth_pid.compute(
            depth_err, dt, error_rate=-self._current_depth_rate)

        head_err = angle_diff(self.target_heading, self._current_heading)
        yaw_cmd = self.yaw_pid.compute(head_err, dt)

        surge_cmd = self.surge_power * config.DEPTH_MAX_OUTPUT
        sway_cmd = self.sway_power * config.DEPTH_MAX_OUTPUT

        cmd_vec = [surge_cmd, sway_cmd, heave_cmd, yaw_cmd]

        pwm_out = []
        for row in config.THRUSTER_MATRIX:
            thrust = 0.0
            for i in range(4):
                thrust += row[i] * cmd_vec[i]
            pwm = config.PWM_NEUTRAL + int(thrust)
            if pwm > config.PWM_MAX:
                pwm = config.PWM_MAX
            elif pwm < config.PWM_MIN:
                pwm = config.PWM_MIN
            pwm_out.append(pwm)

        msg = MotorCommand()
        msg.pwm_values = pwm_out
        self.motor_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
