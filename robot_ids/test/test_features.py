"""Feature extraction: normal driving looks calm, attacks stand out."""

import math

from robot_ids.features import FEATURES, FeatureExtractor


def drive(fx, t0, t1, v=0.15, extra_cmd=None, odom_stamp_offset=0.0):
    """Feed a robot driving straight at v from t0 to t1 (10 Hz cmd, 30 Hz odom, 5 Hz scan)."""
    t = t0
    step = 1.0 / 30.0
    i = 0
    while t < t1:
        if i % 3 == 0:
            fx.add_cmd(t, t, v, 0.0)
            if extra_cmd is not None:
                fx.add_cmd(t + 0.001, t + 0.001, *extra_cmd)
        fx.add_odom(t, t + odom_stamp_offset, v * t, 0.0, v, 0.0)
        if i % 6 == 0:
            wall = 3.0 - v * t  # wall straight ahead, getting closer
            fx.add_scan(t, t, [wall] * 360, 0.12, 3.5, -math.pi, 2 * math.pi / 360)
        t += step
        i += 1
    return t


def as_dict(fx, t):
    return dict(zip(FEATURES, fx.features(t, t)))


def test_normal_driving_is_calm():
    fx = FeatureExtractor()
    t = drive(fx, 0.0, 3.0)
    f = as_dict(fx, t)
    assert 9 <= f['cmd_hz'] <= 11 and 28 <= f['odom_hz'] <= 32 and 4 <= f['scan_hz'] <= 6
    assert f['cmd_jumps'] == 0 and f['res_lin'] < 1e-6 and f['odom_backsteps'] == 0
    assert f['odom_speed_gap'] < 0.01 and f['scan_change'] < 0.05 and f['scan_front_gap'] < 0.01


def test_competing_commands_show_up():
    fx = FeatureExtractor()
    t = drive(fx, 0.0, 3.0, extra_cmd=(0.2, 1.5))
    f = as_dict(fx, t)
    assert f['cmd_hz'] > 18 and f['cmd_jumps'] >= 15 and f['cmd_ang_std'] > 0.5


def test_old_odometry_stamps_are_counted():
    fx = FeatureExtractor()
    t = drive(fx, 0.0, 2.0)
    fx.add_odom(t, t - 3.0, 0.0, 0.0, 0.15, 0.0)  # a replayed message
    f = as_dict(fx, t)
    assert f['odom_backsteps'] == 1 and f['odom_max_step'] > 0.2


def test_window_forgets_old_messages():
    fx = FeatureExtractor(window_s=1.0)
    drive(fx, 0.0, 2.0, extra_cmd=(0.2, 1.5))
    t = drive(fx, 2.0, 4.0)
    assert as_dict(fx, t)['cmd_jumps'] == 0
