"""Scoring of experiment trials and the driver's scan helpers."""

import math

from robot_ids.experiment import score_trial, summarize


def ev(t, detector='rate'):
    return {'t': t, 'detector': detector, 'topic': '/x', 'message': ''}


def test_score_trial_first_delay_and_per_detector():
    r = score_trial('flood', 0, 100.0, 110.0,
                    [ev(99.0), ev(100.4, 'rate'), ev(100.9, 'cmd_conflict'), ev(101.0, 'rate')])
    assert r.detected
    assert math.isclose(r.first_delay, 0.4)
    assert math.isclose(r.detector_delays['cmd_conflict'], 0.9)
    assert set(r.detector_delays) == {'rate', 'cmd_conflict'}


def test_event_before_attack_does_not_count():
    r = score_trial('flood', 0, 100.0, 110.0, [ev(99.5)])
    assert not r.detected


def test_event_long_after_attack_does_not_count():
    r = score_trial('flood', 0, 100.0, 110.0, [ev(115.0)], post_s=2.0)
    assert not r.detected


def test_attack_that_never_started():
    r = score_trial('flood', 0, None, None, [ev(1.0)])
    assert r.start is None and not r.detected


def test_summary_table():
    results = [score_trial('flood', i, 10.0 * i, 10.0 * i + 5, [ev(10.0 * i + 0.5)])
               for i in range(3)]
    results.append(score_trial('scan_spoof', 0, 50.0, 55.0, []))
    text = summarize(results, baseline_s=1800.0, false_alarms=[ev(1.0, 'scan_sanity')])
    assert '| `flood` | 3/3 | 0.50 s | 0.50 s | rate (3/3) |' in text
    assert '| `scan_spoof` | 0/1 | - | - | - |' in text
    assert '1 false alarm(s) (2.0 per hour); caused by scan_sanity (1)' in text


def test_driver_scan_helpers():
    import pytest
    pytest.importorskip('rclpy')
    from robot_ids.driver_node import freer_side, front_clearance, ramp
    inc = 2 * math.pi / 360
    ranges = [3.0] * 360
    ranges[0] = 0.3          # obstacle straight ahead
    ranges[40:50] = [0.2] * 10  # wall on the front-left
    assert front_clearance(ranges, 0.0, inc, math.radians(30)) == 0.3
    assert freer_side(ranges, 0.0, inc) == -1.0
    assert ramp(0.0, 1.0, 0.2) == 0.2 and ramp(1.0, 0.0, 0.2) == 0.8


def test_ros_modules_import():
    import importlib

    import pytest
    pytest.importorskip('rclpy')
    for name in ('driver_node', 'experiment_node', 'event_logger_node'):
        importlib.import_module(f'robot_ids.{name}')
