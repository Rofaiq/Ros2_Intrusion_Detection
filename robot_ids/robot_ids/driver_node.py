"""Scripted driver: wanders around the world so experiments need no human.

Drives forward and turns away from obstacles seen in the LiDAR scan,
publishing on ``cmd_vel_in`` (through the safety gate) like teleop would.
Speed changes are ramped, as a person or a planner would, so the driver
never looks like a second command source to the monitor.

When the alarm is raised it stops. After the alarm is cleared it sends
stop commands for a moment, which re-arms the safety gate's restart
interlock, and then drives on.
"""

import math
import time

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from robot_ids.compat import default_cmd_vel_stamped


def ramp(current, target, step):
    return current + max(-step, min(step, target - current))


def front_clearance(ranges, angle_min, angle_increment, half_angle):
    """Smallest valid range within +-half_angle of straight ahead."""
    best = math.inf
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0:
            continue
        a = math.atan2(math.sin(angle_min + i * angle_increment),
                       math.cos(angle_min + i * angle_increment))
        if abs(a) <= half_angle:
            best = min(best, r)
    return best


def freer_side(ranges, angle_min, angle_increment):
    """+1 to turn left (counter-clockwise), -1 to turn right."""
    left, right = [], []
    for i, r in enumerate(ranges):
        r = r if math.isfinite(r) else 3.5
        a = math.atan2(math.sin(angle_min + i * angle_increment),
                       math.cos(angle_min + i * angle_increment))
        if 0.0 < a < math.pi / 2:
            left.append(r)
        elif -math.pi / 2 < a < 0.0:
            right.append(r)
    mean = lambda xs: sum(xs) / len(xs) if xs else 0.0  # noqa: E731
    return 1.0 if mean(left) >= mean(right) else -1.0


class DriverNode(Node):

    def __init__(self, **kwargs):
        super().__init__('driver', **kwargs)
        self.declare_parameter('cmd_vel_stamped', default_cmd_vel_stamped())
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('turn_speed', 1.0)
        self.declare_parameter('obstacle_distance', 0.45)
        self.declare_parameter('front_half_angle_deg', 30.0)
        self.declare_parameter('max_linear_step', 0.02)
        self.declare_parameter('max_angular_step', 0.2)
        self.declare_parameter('rearm_hold_s', 1.0)
        self.declare_parameter('rate_hz', 10.0)
        p = lambda name: self.get_parameter(name).value  # noqa: E731

        self.stamped = p('cmd_vel_stamped')
        self.pub = self.create_publisher(TwistStamped if self.stamped else Twist,
                                         'cmd_vel_in', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/ids/attack_detected', self.on_alarm, latched)
        self.create_timer(1.0 / p('rate_hz'), self.on_timer,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.scan = None
        self.alarm = False
        self.hold_until = time.monotonic() + p('rearm_hold_s')
        self.turn_dir = 0.0  # 0 while driving straight
        self.v = 0.0
        self.w = 0.0

    def on_scan(self, msg):
        self.scan = msg

    def on_alarm(self, msg):
        if msg.data and not self.alarm:
            self.get_logger().info('Alarm: stopping')
        if not msg.data and self.alarm:
            self.get_logger().info('Alarm cleared: re-arming the gate, then driving on')
            self.hold_until = time.monotonic() + self.get_parameter('rearm_hold_s').value
        self.alarm = msg.data

    def target(self):
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        if self.scan is None:
            return 0.0, 0.0
        s = self.scan
        clearance = front_clearance(s.ranges, s.angle_min, s.angle_increment,
                                    math.radians(p('front_half_angle_deg')))
        obstacle = p('obstacle_distance')
        if self.turn_dir == 0.0 and clearance < obstacle:
            self.turn_dir = freer_side(s.ranges, s.angle_min, s.angle_increment)
        elif self.turn_dir != 0.0 and clearance > obstacle + 0.15:
            self.turn_dir = 0.0
        if self.turn_dir != 0.0:
            return 0.0, self.turn_dir * p('turn_speed')
        return p('forward_speed'), 0.0

    def on_timer(self):
        if self.alarm or time.monotonic() < self.hold_until:
            self.v = self.w = 0.0
        else:
            v, w = self.target()
            self.v = ramp(self.v, v, self.get_parameter('max_linear_step').value)
            self.w = ramp(self.w, w, self.get_parameter('max_angular_step').value)
        msg = TwistStamped() if self.stamped else Twist()
        twist = msg.twist if self.stamped else msg
        if self.stamped:
            msg.header.stamp = self.get_clock().now().to_msg()
        twist.linear.x = self.v
        twist.angular.z = self.w
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
