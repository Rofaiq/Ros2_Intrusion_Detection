"""Unit tests for the pure-Python detectors (no ROS needed)."""

import math

from robot_ids.detectors import (CmdConflictDetector, CmdOdomConsistencyDetector,
                                 OdomReplayDetector, RateDetector, ScanSanityDetector,
                                 check_publishers)


def feed_rate(detector, hz, start, seconds):
    n = int(hz * seconds)
    for i in range(n):
        detector.add(start + i / hz)
    return start + seconds


def test_rate_normal_stream_is_quiet():
    d = RateDetector('/scan', max_hz=8.0, min_hz=2.0)
    now = feed_rate(d, 5.0, 0.0, 3.0)
    assert d.check(now) == []


def test_rate_doubled_stream_alerts():
    d = RateDetector('/scan', max_hz=8.0)
    feed_rate(d, 5.0, 0.0, 3.0)
    feed_rate(d, 5.0, 0.1, 3.0)  # second, interleaved publisher
    alerts = d.check(3.0)
    assert len(alerts) == 1 and 'above' in alerts[0].message


def test_rate_silent_topic_alerts_after_warmup():
    d = RateDetector('/odom', max_hz=60.0, min_hz=5.0)
    d.add(0.0)
    assert d.check(0.5) == []  # still warming up
    assert d.check(2.0)[0].detector == 'rate'


def test_consistency_quiet_when_robot_follows_command():
    d = CmdOdomConsistencyDetector()
    alerts = []
    for i in range(100):
        t = i * 0.033
        d.add_cmd(t, 0.2, 0.0)
        alerts += d.add_odom(t, 0.2, 0.0)
    assert alerts == []


def test_consistency_alerts_when_commands_disagree_with_motion():
    d = CmdOdomConsistencyDetector()
    alerts = []
    for i in range(100):
        t = i * 0.033
        d.add_cmd(t, 0.2, 0.0)           # what the legitimate driver sent
        alerts += d.add_odom(t, 0.0, 1.5)  # robot is spinning instead
    assert alerts and alerts[0].detector == 'cmd_odom_consistency'


def test_consistency_ignores_stale_commands():
    d = CmdOdomConsistencyDetector(cmd_timeout_s=0.5)
    d.add_cmd(0.0, 0.2, 0.0)
    assert d.add_odom(5.0, 0.0, 0.0) == []


def room_scan(n=360, rotation=0):
    """A smooth fake room profile, optionally rotated by some beams."""
    return [2.0 + 0.8 * math.sin(2 * math.pi * ((i + rotation) % n) / n * 3)
            for i in range(n)]


def test_scan_consecutive_real_scans_are_quiet():
    d = ScanSanityDetector()
    inc = 2 * math.pi / 360
    assert d.update(0.0, room_scan(), 0.12, 3.5, inc) == []
    assert d.update(0.2, room_scan(), 0.12, 3.5, inc) == []


def test_scan_rotation_is_not_an_attack():
    d = ScanSanityDetector(max_angular_speed=3.0)
    inc = 2 * math.pi / 360
    d.update(0.0, room_scan(), 0.12, 3.5, inc)
    # 2.5 rad/s for 0.2 s is about 29 beams of rotation.
    assert d.update(0.2, room_scan(rotation=29), 0.12, 3.5, inc) == []


def test_scan_phantom_wall_alerts():
    d = ScanSanityDetector()
    inc = 2 * math.pi / 360
    d.update(0.0, room_scan(), 0.12, 3.5, inc)
    alerts = d.update(0.1, [0.4] * 360, 0.12, 3.5, inc)
    assert any('jump' in a.message for a in alerts)


def test_scan_out_of_range_values_alert():
    d = ScanSanityDetector()
    ranges = room_scan()
    ranges[:60] = [0.01] * 60  # closer than the sensor can measure
    alerts = d.update(0.0, ranges, 0.12, 3.5, 0.0)
    assert any('outside sensor limits' in a.message for a in alerts)


def test_scan_infinite_ranges_are_valid():
    d = ScanSanityDetector()
    ranges = room_scan()
    ranges[:100] = [math.inf] * 100  # nothing within range: normal
    assert d.update(0.0, ranges, 0.12, 3.5, 0.0) == []


def test_odom_normal_motion_is_quiet():
    d = OdomReplayDetector(max_speed=1.0)
    alerts = []
    for i in range(50):
        alerts += d.update(i * 0.033, 0.2 * i * 0.033, 0.0)
    assert alerts == []


def test_odom_replay_old_stamp_alerts():
    d = OdomReplayDetector()
    d.update(10.0, 1.0, 0.0)
    alerts = d.update(4.0, 0.0, 0.0)
    assert alerts and 'backwards' in alerts[0].message


def test_odom_teleport_alerts():
    d = OdomReplayDetector(max_speed=1.0)
    d.update(1.0, 0.0, 0.0)
    alerts = d.update(1.033, 2.0, 0.0)
    assert alerts and 'implied speed' in alerts[0].message


def test_publishers_allowlist():
    assert check_publishers('/cmd_vel', ['/safety_gate'], ['/safety_gate']) == []
    alerts = check_publishers('/cmd_vel', ['/safety_gate', '/attack_injector'],
                              ['/safety_gate'])
    kinds = {a.detector for a in alerts}
    assert kinds == {'publisher_allowlist', 'publisher_count'}


def test_publishers_count_without_allowlist():
    assert check_publishers('/scan', ['/bridge']) == []
    assert check_publishers('/scan', ['/bridge', '/x'])[0].detector == 'publisher_count'


def test_conflict_quiet_for_smooth_teleop():
    d = CmdConflictDetector()
    alerts = []
    v = w = 0.0
    for i in range(100):  # operator ramps up speed in small steps at 10 Hz
        v, w = min(v + 0.005, 0.22), min(w + 0.05, 1.0)
        alerts += d.add_cmd(i * 0.1, v, w)
    alerts += d.add_cmd(10.1, 0.0, 0.0)  # one emergency stop is fine
    assert alerts == []


def test_conflict_detects_interleaved_streams():
    d = CmdConflictDetector()
    alerts = []
    for i in range(30):  # teleop (0.06, 0.6) and attacker (0.2, 1.5) interleaved
        t = i * 0.033
        alerts += d.add_cmd(t, *((0.06, 0.6) if i % 2 else (0.2, 1.5)))
    assert alerts and alerts[0].detector == 'cmd_conflict'
