"""Integration test: run the monitor node against fake robot topics.

No simulator is needed. A fake robot publishes odometry and LiDAR scans and
a node named ``safety_gate`` sends matching velocity commands. The monitor
must stay quiet through startup and normal driving, then raise the alarm
when a second publisher appears on /cmd_vel.
"""

import time

import pytest

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import Twist, TwistStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

from robot_ids.compat import default_cmd_vel_stamped  # noqa: E402
from robot_ids.monitor_node import MonitorNode  # noqa: E402

SPEED = 0.1


class FakeRobot:
    """Publishes odometry, scans and commands the way the simulation would."""

    def __init__(self, node, driver):
        self.node = node
        self.stamped = default_cmd_vel_stamped()
        cmd_type = TwistStamped if self.stamped else Twist
        self.cmd_pub = driver.create_publisher(cmd_type, '/cmd_vel', 10)
        self.odom_pub = node.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.t0 = time.monotonic()
        node.create_timer(1.0 / 30.0, self.publish_odom)
        node.create_timer(1.0 / 5.0, self.publish_scan)
        driver.create_timer(1.0 / 10.0, self.publish_cmd)

    def publish_cmd(self):
        msg = TwistStamped() if self.stamped else Twist()
        twist = msg.twist if self.stamped else msg
        twist.linear.x = SPEED
        self.cmd_pub.publish(msg)

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = SPEED * (time.monotonic() - self.t0)
        msg.twist.twist.linear.x = SPEED
        self.odom_pub.publish(msg)

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_scan'
        msg.angle_increment = 2 * 3.14159 / 360
        msg.range_min = 0.12
        msg.range_max = 3.5
        msg.ranges = [2.0] * 360
        self.scan_pub.publish(msg)


def spin_for(executor, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def test_quiet_during_normal_driving_then_detects_spoofed_publisher(ros):
    monitor = MonitorNode(parameter_overrides=[
        Parameter('startup_grace_s', value=1.0)])
    robot_node = rclpy.create_node('fake_robot')
    driver = rclpy.create_node('safety_gate')
    attacker = rclpy.create_node('attacker')
    FakeRobot(robot_node, driver)

    alarms = []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    robot_node.create_subscription(Bool, '/ids/attack_detected',
                                   lambda m: alarms.append(m.data), latched)

    executor = SingleThreadedExecutor()
    for node in (monitor, robot_node, driver, attacker):
        executor.add_node(node)

    spin_for(executor, 4.0)
    assert not monitor.attack_detected, 'false alarm during normal driving'

    cmd_type = TwistStamped if default_cmd_vel_stamped() else Twist
    attacker.create_publisher(cmd_type, '/cmd_vel', 10)
    spin_for(executor, 2.0)
    assert monitor.attack_detected, 'second /cmd_vel publisher was not detected'
    assert alarms and alarms[-1] is True

    for node in (monitor, robot_node, driver, attacker):
        node.destroy_node()


def test_consistency_detector_catches_spoofed_commands_without_graph_checks(ros):
    monitor = MonitorNode(parameter_overrides=[
        Parameter('startup_grace_s', value=1.0),
        Parameter('graph_checks', value=False)])
    robot_node = rclpy.create_node('fake_robot')
    driver = rclpy.create_node('safety_gate')
    attacker = rclpy.create_node('attacker')
    FakeRobot(robot_node, driver)

    executor = SingleThreadedExecutor()
    for node in (monitor, robot_node, driver, attacker):
        executor.add_node(node)

    spin_for(executor, 3.0)
    assert not monitor.attack_detected, 'false alarm during normal driving'

    # The attacker commands a spin that the (fake) robot never performs.
    stamped = default_cmd_vel_stamped()
    pub = attacker.create_publisher(TwistStamped if stamped else Twist, '/cmd_vel', 10)

    def spoof():
        msg = TwistStamped() if stamped else Twist()
        (msg.twist if stamped else msg).angular.z = 1.5
        pub.publish(msg)

    attacker.create_timer(1.0 / 20.0, spoof)
    spin_for(executor, 3.0)
    assert monitor.attack_detected, 'spoofed commands were not detected'

    for node in (monitor, robot_node, driver, attacker):
        node.destroy_node()
