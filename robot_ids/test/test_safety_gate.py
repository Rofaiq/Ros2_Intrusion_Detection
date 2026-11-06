"""Safety gate: blocks on alarm and needs a stop command before driving again."""

import time

import pytest

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import Twist, TwistStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

from robot_ids.compat import default_cmd_vel_stamped  # noqa: E402
from robot_ids.safety_gate_node import SafetyGateNode  # noqa: E402


def test_alarm_blocks_and_clear_requires_stop_command():
    rclpy.init()
    try:
        gate = SafetyGateNode()
        helper = rclpy.create_node('test_helper')
        stamped = default_cmd_vel_stamped()
        msg_type = TwistStamped if stamped else Twist
        cmd_in = helper.create_publisher(msg_type, 'cmd_vel_in', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        alarm = helper.create_publisher(Bool, '/ids/attack_detected', latched)
        out = []
        helper.create_subscription(
            msg_type, 'cmd_vel',
            lambda m: out.append((m.twist if stamped else m).angular.z), 100)

        executor = SingleThreadedExecutor()
        executor.add_node(gate)
        executor.add_node(helper)

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=0.01)

        def send(angular):
            msg = msg_type()
            (msg.twist if stamped else msg).angular.z = angular
            cmd_in.publish(msg)
            spin(0.1)

        spin(0.5)  # discovery
        send(0.6)
        assert out and out[-1] == 0.6, 'command not forwarded while clear'

        alarm.publish(Bool(data=True))
        spin(0.5)
        out.clear()
        send(0.6)
        spin(0.3)
        assert out and all(v == 0.0 for v in out), 'gate forwarded motion during alarm'

        alarm.publish(Bool(data=False))
        spin(0.3)
        out.clear()
        send(0.6)
        assert out == [], 'robot restarted on its own after the alarm was cleared'

        send(0.0)
        send(0.6)
        assert out == [0.0, 0.6], 'gate did not re-arm after a stop command'

        gate.destroy_node()
        helper.destroy_node()
    finally:
        rclpy.try_shutdown()
