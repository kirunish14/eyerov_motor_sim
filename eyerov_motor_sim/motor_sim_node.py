"""ROS 2 node: /cmd_vel -> target RPM -> simulated motor -> RPM telemetry.

Behaves like a tiny piece of motor-control firmware:
  * a subscriber latches the latest command (event driven)
  * a fixed-rate timer runs the control/simulation loop (time driven)
  * a watchdog zeroes the targets if commands stop arriving (failsafe)
"""
import math
import random

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32

from .mixer import twist_to_target_rpm
from .motor_model import MotorModel

MOTOR_NAMES = ('left', 'right', 'vertical')
MAX_STEP_S = 0.1  # clamp dt so a scheduling stall can't cause a huge jump


class MotorSimNode(Node):
    def __init__(self):
        super().__init__('motor_sim_node')

        # ---- parameters (use decimals in YAML so they parse as doubles) ----
        self.declare_parameter('max_rpm', 3000.0)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('track_width', 0.3)
        self.declare_parameter('tau', 0.4)
        self.declare_parameter('max_accel_rpm_s', 6000.0)
        self.declare_parameter('deadband_rpm', 20.0)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('noise_std_rpm', 0.0)

        self._max_rpm = self._param('max_rpm')
        self._max_speed = self._param('max_speed')
        self._track_width = self._param('track_width')
        self._cmd_timeout = self._param('cmd_timeout_s')
        self._noise_std = self._param('noise_std_rpm')
        rate = self._param('publish_rate_hz')
        self._validate(rate)

        # ---- simulated motors and state ----
        self._motors = {
            name: MotorModel(
                tau=self._param('tau'),
                max_rpm=self._max_rpm,
                max_accel_rpm_s=self._param('max_accel_rpm_s'),
                deadband_rpm=self._param('deadband_rpm'))
            for name in MOTOR_NAMES
        }
        self._zero_targets = {name: 0.0 for name in MOTOR_NAMES}
        self._targets = dict(self._zero_targets)
        self._last_cmd_time = None
        self._watchdog_tripped = False
        self._last_step_time = self.get_clock().now()
        self._rng = random.Random()

        # ---- ROS interfaces ----
        self._sub = self.create_subscription(
            Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self._rpm_pubs = {
            name: self.create_publisher(Float32, f'motor_{name}/rpm', 10)
            for name in MOTOR_NAMES}
        self._target_pubs = {
            name: self.create_publisher(Float32, f'motor_{name}/target_rpm', 10)
            for name in MOTOR_NAMES}
        self._timer = self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f'motor_sim_node up: {rate:.0f} Hz, max_rpm={self._max_rpm:.0f}, '
            f'tau={self._param("tau")}s, watchdog={self._cmd_timeout}s')

    # ------------------------------------------------------------------
    def _param(self, name):
        return float(self.get_parameter(name).value)

    def _validate(self, rate):
        if self._max_rpm <= 0.0 or self._max_speed <= 0.0 or rate <= 0.0:
            raise ValueError(
                'max_rpm, max_speed and publish_rate_hz must be > 0')
        if self._track_width < 0.0 or self._noise_std < 0.0:
            raise ValueError('track_width and noise_std_rpm must be >= 0')

    # ------------------------------------------------------------------
    def _on_cmd_vel(self, msg):
        values = (msg.linear.x, msg.linear.z, msg.angular.z)
        if not all(math.isfinite(v) for v in values):
            self.get_logger().warn('Ignoring cmd_vel with NaN/inf values')
            return

        self._targets = twist_to_target_rpm(
            linear_x=msg.linear.x,
            linear_z=msg.linear.z,
            angular_z=msg.angular.z,
            track_width=self._track_width,
            max_speed=self._max_speed,
            max_rpm=self._max_rpm)
        self._last_cmd_time = self.get_clock().now()

        if self._watchdog_tripped:
            self.get_logger().info('cmd_vel resumed, watchdog cleared')
            self._watchdog_tripped = False

    def _active_targets(self, now):
        """Latest targets, or zeros if the command stream has gone stale."""
        if self._last_cmd_time is None:
            return self._zero_targets
        age = (now - self._last_cmd_time).nanoseconds * 1e-9
        if self._cmd_timeout > 0.0 and age > self._cmd_timeout:
            if not self._watchdog_tripped:
                self.get_logger().warn(
                    f'No cmd_vel for {self._cmd_timeout}s: '
                    'watchdog tripped, targets set to 0')
                self._watchdog_tripped = True
            return self._zero_targets
        return self._targets

    def _on_timer(self):
        now = self.get_clock().now()
        dt = (now - self._last_step_time).nanoseconds * 1e-9
        self._last_step_time = now
        if dt <= 0.0:
            return
        dt = min(dt, MAX_STEP_S)

        targets = self._active_targets(now)
        for name in MOTOR_NAMES:
            rpm = self._motors[name].step(targets[name], dt)
            if self._noise_std > 0.0:  # noise only on the reported value
                rpm += self._rng.gauss(0.0, self._noise_std)
            self._publish(self._rpm_pubs[name], rpm)
            self._publish(self._target_pubs[name], targets[name])

    @staticmethod
    def _publish(publisher, value):
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
