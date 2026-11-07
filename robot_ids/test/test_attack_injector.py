"""The attack injector must reach its real send rate under simulation time.

Gazebo Classic publishes /clock at only 10 Hz. A sim-time timer would cap
every attack at that rate, so this test drives a slow /clock and checks
that the flood still arrives at well over 10 Hz.
"""

import time

import pytest

rclpy = pytest.importorskip('rclpy')

from builtin_interfaces.msg import Time  # noqa: E402
from geometry_msgs.msg import Twist, TwistStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402

from robot_ids.attack_injector_node import AttackInjectorNode  # noqa: E402
from robot_ids.compat import default_cmd_vel_stamped  # noqa: E402


def test_flood_is_not_limited_by_a_slow_sim_clock():
    rclpy.init()
    try:
        injector = AttackInjectorNode(parameter_overrides=[
            Parameter('use_sim_time', value=True),
            Parameter('attack', value='flood'),
            Parameter('start_delay_s', value=0.0),
            Parameter('duration_s', value=0.0),
            Parameter('flood_hz', value=200.0)])
        helper = rclpy.create_node('test_helper')
        clock_pub = helper.create_publisher(Clock, '/clock', 10)
        received = []
        msg_type = TwistStamped if default_cmd_vel_stamped() else Twist
        helper.create_subscription(msg_type, '/cmd_vel', received.append, 1000)

        sim_t = [100.0]

        def tick():
            sim_t[0] += 0.1
            sec = int(sim_t[0])
            clock_pub.publish(Clock(clock=Time(sec=sec, nanosec=int((sim_t[0] - sec) * 1e9))))

        helper.create_timer(0.1, tick)  # 10 Hz /clock, like Gazebo Classic

        executor = SingleThreadedExecutor()
        executor.add_node(injector)
        executor.add_node(helper)
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.001)
        start = len(received)
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.001)
        rate = (len(received) - start) / 2.0
        assert rate > 50.0, f'flood only reached {rate:.0f} Hz'
        injector.destroy_node()
        helper.destroy_node()
    finally:
        rclpy.try_shutdown()


def test_attacker_reports_truth_and_finishes():
    """The attacker tags its ground truth and finishes when the attack ends,
    so its process can exit instead of lingering into later trials."""
    import json
    from std_msgs.msg import String
    rclpy.init()
    try:
        injector = AttackInjectorNode(parameter_overrides=[
            Parameter('use_sim_time', value=True),
            Parameter('attack', value='cmd_vel_spoof'),
            Parameter('start_delay_s', value=0.5),
            Parameter('duration_s', value=1.0),
            Parameter('run_id', value='trial7')])
        helper = rclpy.create_node('test_helper')
        clock_pub = helper.create_publisher(Clock, '/clock', 10)
        truth = []
        helper.create_subscription(String, '/ids/attack_truth',
                                   lambda m: truth.append(json.loads(m.data)), 10)
        sim_t = [100.0]

        def tick():
            sim_t[0] += 0.1
            sec = int(sim_t[0])
            clock_pub.publish(Clock(clock=Time(sec=sec, nanosec=int((sim_t[0] - sec) * 1e9))))

        helper.create_timer(0.1, tick)
        executor = SingleThreadedExecutor()
        executor.add_node(injector)
        executor.add_node(helper)
        end = time.monotonic() + 15.0
        while not injector.done() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        assert injector.done(), 'attacker did not finish'
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        states = [t['state'] for t in truth]
        assert states == ['start', 'end'], truth
        assert all(t['run'] == 'trial7' for t in truth)
        assert 0.9 <= truth[1]['t'] - truth[0]['t'] <= 1.3
        injector.destroy_node()
        helper.destroy_node()
    finally:
        rclpy.try_shutdown()
