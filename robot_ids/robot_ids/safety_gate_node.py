"""Safety gate: the only node allowed to drive the robot.

Teleop or Nav2 publish on ``cmd_vel_in``; the gate forwards those commands
to ``cmd_vel`` while the alarm is clear. When the monitor latches
``/ids/attack_detected``, the gate drops all input and publishes zero
velocity at a fixed rate so the robot stops.

Restart interlock: clearing the alarm does not restart the robot by itself.
Teleop and Nav2 keep re-sending their last command while blocked, so the
gate waits for a stop command (all velocities zero, e.g. ``s`` in teleop)
before forwarding motion again. Resetting a safety function should never
start a machine on its own; an operator has to ask for motion again.

Note: the gate cannot block an attacker that publishes straight to
``cmd_vel``. Its zero commands compete with the injected ones, which slows
the attack but does not prevent it. Real prevention needs access control
on the topic (SROS2), which is a later stage of this project.
"""

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from robot_ids.compat import default_cmd_vel_stamped


def is_stop(twist):
    return all(v == 0.0 for v in (twist.linear.x, twist.linear.y, twist.linear.z,
                                  twist.angular.x, twist.angular.y, twist.angular.z))


class SafetyGateNode(Node):

    def __init__(self, **kwargs):
        super().__init__('safety_gate', **kwargs)
        self.declare_parameter('cmd_vel_stamped', default_cmd_vel_stamped())
        self.declare_parameter('require_stop_after_clear', True)
        self.declare_parameter('stop_rate_hz', 20.0)
        self.stamped = self.get_parameter('cmd_vel_stamped').value
        msg_type = TwistStamped if self.stamped else Twist

        self.pub = self.create_publisher(msg_type, 'cmd_vel', 10)
        self.create_subscription(msg_type, 'cmd_vel_in', self.on_cmd, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/ids/attack_detected', self.on_alarm, latched)
        # Wall-clock timer, so stop commands go out at the real rate even
        # when /clock ticks slowly.
        self.create_timer(1.0 / self.get_parameter('stop_rate_hz').value, self.on_timer,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.blocked = False
        self.awaiting_stop = False

    def on_cmd(self, msg):
        if self.blocked:
            return
        if self.awaiting_stop:
            if not is_stop(msg.twist if self.stamped else msg):
                return
            self.awaiting_stop = False
            self.get_logger().info('Stop command received: forwarding commands again')
        self.pub.publish(msg)

    def on_alarm(self, msg):
        if msg.data != self.blocked:
            self.blocked = msg.data
            if self.blocked:
                self.get_logger().warn('Alarm raised: robot stopped, input blocked')
            elif self.get_parameter('require_stop_after_clear').value:
                self.awaiting_stop = True
                self.get_logger().info('Alarm cleared: send a stop command (teleop: s) '
                                       'before driving again')
            else:
                self.get_logger().info('Alarm cleared: forwarding commands again')
            self.publish_stop()

    def on_timer(self):
        if self.blocked:
            self.publish_stop()

    def publish_stop(self):
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
        else:
            msg = Twist()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
