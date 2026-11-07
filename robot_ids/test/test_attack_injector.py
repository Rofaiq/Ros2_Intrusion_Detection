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


def run_attack(attack, setup, seconds=4.0, **params):
    """Run one attack against a helper node under a 10 Hz sim clock."""
    rclpy.init()
    try:
        overrides = [Parameter('use_sim_time', value=True), Parameter('attack', value=attack),
                     Parameter('start_delay_s', value=0.0), Parameter('duration_s', value=0.0)]
        overrides += [Parameter(k, value=v) for k, v in params.items()]
        injector = AttackInjectorNode(parameter_overrides=overrides)
        helper = rclpy.create_node('test_helper')
        clock_pub = helper.create_publisher(Clock, '/clock', 10)
        sim_t = [100.0]

        def tick():
            sim_t[0] += 0.1
            sec = int(sim_t[0])
            clock_pub.publish(Clock(clock=Time(sec=sec, nanosec=int((sim_t[0] - sec) * 1e9))))

        helper.create_timer(0.1, tick)
        received = setup(helper, sim_t)
        executor = SingleThreadedExecutor()
        executor.add_node(injector)
        executor.add_node(helper)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        injector.destroy_node()
        helper.destroy_node()
        return received
    finally:
        rclpy.try_shutdown()


def test_cmd_bias_adds_a_small_turn_to_real_commands():
    msg_type = TwistStamped if default_cmd_vel_stamped() else Twist

    def setup(helper, sim_t):
        pub = helper.create_publisher(msg_type, '/cmd_vel_in', 10)

        def send():
            msg = msg_type()
            (msg.twist if default_cmd_vel_stamped() else msg).linear.x = 0.15
            pub.publish(msg)

        helper.create_timer(0.1, send)
        got = []
        helper.create_subscription(msg_type, '/cmd_vel', got.append, 100)
        return got

    got = run_attack('cmd_bias', setup, bias_angular=0.3)
    twists = [m.twist if default_cmd_vel_stamped() else m for m in got]
    assert len(twists) >= 5, 'cmd_bias sent too few commands'
    assert all(abs(t.linear.x - 0.15) < 1e-9 and abs(t.angular.z - 0.3) < 1e-9 for t in twists)


def test_scan_drift_shortens_ranges_slowly_with_increasing_stamps():
    import math
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    def setup(helper, sim_t):
        pub = helper.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)

        def send():
            msg = LaserScan()
            sec = int(sim_t[0])
            msg.header.stamp.sec, msg.header.stamp.nanosec = sec, int((sim_t[0] - sec) * 1e9)
            msg.header.frame_id = 'base_scan'
            msg.range_min, msg.range_max = 0.12, 3.5
            msg.angle_min, msg.angle_increment = -math.pi, 2 * math.pi / 360
            msg.ranges = [2.0] * 360
            pub.publish(msg)

        helper.create_timer(0.2, send)  # real LiDAR at 5 Hz
        got = []
        helper.create_subscription(
            LaserScan, '/scan', lambda m: got.append(m) if m.ranges[0] < 2.0 else None,
            qos_profile_sensor_data)
        return got

    fakes = run_attack('scan_drift', setup, seconds=5.0, drift_rate=0.05)
    assert 2 <= len(fakes) <= 7, f'{len(fakes)} fake scans in 5 s (expected about 1 per second)'
    offsets = [2.0 - f.ranges[0] for f in fakes]
    assert all(0.0 < o <= 0.3 for o in offsets) and offsets == sorted(offsets)
