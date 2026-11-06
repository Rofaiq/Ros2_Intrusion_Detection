"""Intrusion monitor: watches robot topics and raises a latched alarm.

Subscribes to the velocity command, odometry and LiDAR topics, feeds them
to the detectors in ``detectors.py`` and also inspects the ROS graph for
unexpected publishers. On the first alert it latches ``/ids/attack_detected``
to true (the safety gate then stops the robot) until ``/ids/reset`` is
called.
"""

import json

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from robot_ids.compat import default_cmd_vel_stamped
from robot_ids.detectors import (CmdOdomConsistencyDetector, OdomReplayDetector,
                                 RateDetector, ScanSanityDetector, check_publishers)


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class MonitorNode(Node):

    def __init__(self, **kwargs):
        super().__init__('ids_monitor', **kwargs)

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_stamped', default_cmd_vel_stamped())
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('allowed_cmd_vel_publishers', ['/safety_gate'])
        self.declare_parameter('max_publishers_per_topic', 1)
        self.declare_parameter('graph_check_period_s', 0.5)
        self.declare_parameter('cmd_vel_max_hz', 60.0)
        self.declare_parameter('odom_max_hz', 60.0)
        self.declare_parameter('odom_min_hz', 5.0)
        self.declare_parameter('scan_max_hz', 8.0)
        self.declare_parameter('scan_min_hz', 2.0)
        self.declare_parameter('consistency_threshold', 0.15)
        self.declare_parameter('robot_max_speed', 0.5)
        self.declare_parameter('robot_max_angular_speed', 3.0)
        self.declare_parameter('graph_checks', True)
        self.declare_parameter('startup_grace_s', 5.0)
        self.declare_parameter('event_min_interval_s', 1.0)
        self.declare_parameter('enabled_detectors',
                               ['rate', 'cmd_odom_consistency', 'scan_sanity',
                                'odom_replay', 'publisher_allowlist', 'publisher_count'])

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.cmd_topic = p('cmd_vel_topic')
        self.odom_topic = p('odom_topic')
        self.scan_topic = p('scan_topic')
        self.allowed_cmd_pubs = list(p('allowed_cmd_vel_publishers'))
        self.max_pubs = p('max_publishers_per_topic')
        self.enabled = set(p('enabled_detectors'))
        if not p('graph_checks'):
            # Lets you test the data-based detectors on their own: with graph
            # checks on, every injection attack is caught by its publisher.
            self.enabled -= {'publisher_allowlist', 'publisher_count'}
        self.startup_grace = p('startup_grace_s')
        self.event_interval = p('event_min_interval_s')
        self.reset_detectors()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.alarm_pub = self.create_publisher(Bool, '/ids/attack_detected', latched)
        self.event_pub = self.create_publisher(String, '/ids/events', 10)
        self.create_service(Trigger, '/ids/reset', self.on_reset)

        cmd_type = TwistStamped if p('cmd_vel_stamped') else Twist
        self.create_subscription(cmd_type, self.cmd_topic, self.on_cmd, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(p('graph_check_period_s'), self.on_timer)

        self.attack_detected = False
        self.publish_alarm()
        self.get_logger().info(f'Monitoring {self.cmd_topic}, {self.odom_topic}, '
                               f'{self.scan_topic}; detectors: {sorted(self.enabled)}')

    def reset_detectors(self):
        """(Re)create all detectors and restart the startup grace period."""
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        max_speed = p('robot_max_speed')
        self.rates = {
            self.cmd_topic: RateDetector(self.cmd_topic, p('cmd_vel_max_hz')),
            self.odom_topic: RateDetector(self.odom_topic, p('odom_max_hz'), p('odom_min_hz')),
            self.scan_topic: RateDetector(self.scan_topic, p('scan_max_hz'), p('scan_min_hz')),
        }
        self.consistency = CmdOdomConsistencyDetector(
            self.cmd_topic, threshold=p('consistency_threshold'))
        self.scan_sanity = ScanSanityDetector(
            self.scan_topic, max_speed=max_speed,
            max_angular_speed=p('robot_max_angular_speed'))
        # Odometry may legitimately be a bit faster than the speed limit.
        self.odom_replay = OdomReplayDetector(self.odom_topic, max_speed=2.0 * max_speed)
        self.start_time = None
        self.last_time = None
        self.last_event = {}

    def now(self):
        """Current (simulation) time in seconds, or None if not usable yet.

        With use_sim_time the clock reads 0 until the first /clock message
        arrives; timestamps taken then would make every rate look like 0 Hz.
        A clock that jumps backwards means the simulation was reset, so the
        detectors start over.
        """
        t = self.get_clock().now().nanoseconds * 1e-9
        if t <= 0.0:
            return None
        if self.last_time is not None and t < self.last_time - 1.0:
            self.get_logger().info('Clock jumped backwards (simulation reset?): '
                                   'restarting detectors')
            self.reset_detectors()
        if self.start_time is None:
            self.start_time = t
        self.last_time = t
        return t

    def in_grace_period(self, t):
        return self.start_time is None or t - self.start_time < self.startup_grace

    def on_cmd(self, msg):
        twist = msg.twist if isinstance(msg, TwistStamped) else msg
        t = self.now()
        if t is None:
            return
        self.rates[self.cmd_topic].add(t)
        self.consistency.add_cmd(t, twist.linear.x, twist.angular.z)

    def on_odom(self, msg):
        t = self.now()
        if t is None:
            return
        self.rates[self.odom_topic].add(t)
        tw = msg.twist.twist
        self.report(self.consistency.add_odom(t, tw.linear.x, tw.angular.z))
        pos = msg.pose.pose.position
        self.report(self.odom_replay.update(stamp_to_sec(msg.header.stamp), pos.x, pos.y))

    def on_scan(self, msg):
        t = self.now()
        if t is None:
            return
        self.rates[self.scan_topic].add(t)
        self.report(self.scan_sanity.update(stamp_to_sec(msg.header.stamp), msg.ranges,
                                            msg.range_min, msg.range_max,
                                            msg.angle_increment))

    def on_timer(self):
        t = self.now()
        if t is None:
            return
        for detector in self.rates.values():
            self.report(detector.check(t))
        for topic in (self.cmd_topic, self.odom_topic, self.scan_topic):
            names = [self.qualified(i.node_namespace, i.node_name)
                     for i in self.get_publishers_info_by_topic(topic)]
            allowed = self.allowed_cmd_pubs if topic == self.cmd_topic else None
            self.report(check_publishers(topic, names, allowed, self.max_pubs))

    @staticmethod
    def qualified(namespace, name):
        return namespace.rstrip('/') + '/' + name

    def report(self, alerts):
        t = self.last_time
        if t is None or self.in_grace_period(t):
            # Topics are still starting up; ignore what the detectors say.
            return
        for alert in alerts:
            if alert.detector not in self.enabled:
                continue
            # Publish each kind of event at most once per interval, otherwise
            # a detector checked on every odometry message floods the topic.
            key = (alert.detector, alert.topic)
            if t - self.last_event.get(key, -1e9) >= self.event_interval:
                self.last_event[key] = t
                event = {'t': round(t, 3), 'detector': alert.detector,
                         'topic': alert.topic, 'message': alert.message,
                         'score': round(alert.score, 3)}
                self.event_pub.publish(String(data=json.dumps(event)))
            if not self.attack_detected:
                self.get_logger().warn(
                    f'ATTACK DETECTED by {alert.detector} on {alert.topic}: {alert.message}')
                self.attack_detected = True
                self.publish_alarm()

    def publish_alarm(self):
        self.alarm_pub.publish(Bool(data=self.attack_detected))

    def on_reset(self, request, response):
        self.reset_detectors()
        self.attack_detected = False
        self.publish_alarm()
        response.success = True
        response.message = 'Alarm cleared'
        self.get_logger().info('Alarm cleared by /ids/reset')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
