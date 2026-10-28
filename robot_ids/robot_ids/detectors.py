"""Attack detectors for a ROS 2 mobile robot.

This module has no ROS imports on purpose: every detector takes plain
Python values (times in seconds, speeds in m/s, lists of ranges), so the
logic can be unit-tested with pytest without a ROS installation. The ROS
node in ``monitor_node.py`` only converts messages and calls these classes.

Each ``check``/``update`` method returns a list of ``Alert`` objects, which
is empty when nothing looks wrong.
"""

from collections import deque
from dataclasses import dataclass
import math
import statistics


@dataclass
class Alert:
    """One detection event."""

    detector: str
    topic: str
    message: str
    score: float = 1.0


class RateDetector:
    """Flag a topic whose message rate leaves [min_hz, max_hz].

    Catches flooding (DoS) and injected duplicate streams, which roughly
    double the rate of a sensor topic. The rate is measured over a sliding
    window, and no alert is raised until one full window has been observed.
    """

    def __init__(self, topic, max_hz, min_hz=0.0, window_s=1.0):
        self.topic = topic
        self.max_hz = max_hz
        self.min_hz = min_hz
        self.window_s = window_s
        self._stamps = deque()
        self._first_seen = None

    def add(self, t):
        if self._first_seen is None:
            self._first_seen = t
        self._stamps.append(t)

    def rate(self, now):
        while self._stamps and now - self._stamps[0] > self.window_s:
            self._stamps.popleft()
        return len(self._stamps) / self.window_s

    def check(self, now):
        hz = self.rate(now)
        if self._first_seen is None or now - self._first_seen < self.window_s:
            return []
        if hz > self.max_hz:
            return [Alert('rate', self.topic,
                          f'rate {hz:.1f} Hz above limit {self.max_hz:.1f} Hz',
                          hz / self.max_hz)]
        if self.min_hz > 0.0 and hz < self.min_hz:
            return [Alert('rate', self.topic,
                          f'rate {hz:.1f} Hz below limit {self.min_hz:.1f} Hz',
                          self.min_hz / max(hz, 1e-6))]
        return []


class CmdOdomConsistencyDetector:
    """Compare what the robot was told to do with what it actually did.

    If a second publisher injects velocity commands, the robot follows a
    mix of the two streams, so the commands this node sees stop matching
    the motion measured by odometry. The residual is smoothed with an
    exponential moving average so normal acceleration lag does not trigger.
    """

    def __init__(self, topic='/cmd_vel', threshold=0.15, alpha=0.1,
                 angular_weight=0.3, cmd_timeout_s=0.5):
        self.topic = topic
        self.threshold = threshold
        self.alpha = alpha
        self.angular_weight = angular_weight
        self.cmd_timeout_s = cmd_timeout_s
        self.residual = 0.0
        self._cmd = None  # (t, v, w)

    def add_cmd(self, t, v, w):
        self._cmd = (t, v, w)

    def add_odom(self, t, v, w):
        if self._cmd is None or t - self._cmd[0] > self.cmd_timeout_s:
            # No fresh command: decay towards zero instead of guessing.
            self.residual *= (1.0 - self.alpha)
            return []
        _, v_cmd, w_cmd = self._cmd
        r = abs(v_cmd - v) + self.angular_weight * abs(w_cmd - w)
        self.residual = self.alpha * r + (1.0 - self.alpha) * self.residual
        if self.residual > self.threshold:
            return [Alert('cmd_odom_consistency', self.topic,
                          f'command/odometry residual {self.residual:.3f} '
                          f'above {self.threshold:.3f}',
                          self.residual / self.threshold)]
        return []


class ScanSanityDetector:
    """Check each LaserScan for physically impossible content.

    Two checks:
    * too many finite readings outside [range_min, range_max];
    * the median change between consecutive scans is larger than the robot
      could have moved in that time (spoofed scans interleaved with real
      ones make the range profile jump).

    Rotation shifts the whole scan sideways, so the second check compares
    the scans at every beam offset the robot could have turned through and
    keeps the best match.
    """

    def __init__(self, topic='/scan', max_invalid_fraction=0.05,
                 max_speed=0.5, max_angular_speed=3.0, margin_m=0.05):
        self.topic = topic
        self.max_invalid_fraction = max_invalid_fraction
        self.max_speed = max_speed
        self.max_angular_speed = max_angular_speed
        self.margin_m = margin_m
        self._prev = None  # (t, ranges)

    def _median_change(self, prev, ranges, max_shift):
        n = len(ranges)
        best = None
        for shift in range(-max_shift, max_shift + 1):
            diffs = [abs(ranges[i] - prev[(i + shift) % n]) for i in range(n)
                     if math.isfinite(ranges[i]) and math.isfinite(prev[(i + shift) % n])]
            if len(diffs) < n // 4:
                continue
            change = statistics.median(diffs)
            if best is None or change < best:
                best = change
        return best

    def update(self, t, ranges, range_min, range_max, angle_increment):
        alerts = []
        finite = [r for r in ranges if math.isfinite(r)]
        if finite:
            invalid = sum(1 for r in finite if r < range_min or r > range_max)
            frac = invalid / len(ranges)
            if frac > self.max_invalid_fraction:
                alerts.append(Alert(
                    'scan_sanity', self.topic,
                    f'{frac:.0%} of beams outside sensor limits',
                    frac / self.max_invalid_fraction))

        if self._prev is not None and len(self._prev[1]) == len(ranges):
            dt = t - self._prev[0]
            if dt <= 0.0:
                alerts.append(Alert('scan_sanity', self.topic,
                                    f'scan stamp did not increase (dt={dt:.3f} s)'))
            else:
                max_shift = 0
                if angle_increment > 0.0:
                    max_shift = math.ceil(self.max_angular_speed * dt / angle_increment)
                    max_shift = min(max_shift, len(ranges) // 2)
                change = self._median_change(self._prev[1], ranges, max_shift)
                if change is not None:
                    allowed = self.max_speed * dt + self.margin_m
                    if change > allowed:
                        alerts.append(Alert(
                            'scan_sanity', self.topic,
                            f'median range jump {change:.2f} m in {dt:.3f} s '
                            f'(allowed {allowed:.2f} m)',
                            change / allowed))
        self._prev = (t, list(ranges))
        return alerts


class OdomReplayDetector:
    """Detect replayed or forged odometry.

    Replayed messages carry old header stamps, so stamps stop increasing;
    forged poses make the robot appear to teleport.
    """

    def __init__(self, topic='/odom', max_speed=1.0):
        self.topic = topic
        self.max_speed = max_speed
        self._prev = None  # (stamp, x, y)

    def update(self, stamp, x, y):
        alerts = []
        if self._prev is not None:
            p_stamp, px, py = self._prev
            dt = stamp - p_stamp
            if dt <= 0.0:
                alerts.append(Alert('odom_replay', self.topic,
                                    f'odometry stamp went backwards by {-dt:.3f} s'))
                # Keep the newest stamp so one replayed message does not
                # hide the next one.
                return alerts
            speed = math.hypot(x - px, y - py) / dt
            if speed > self.max_speed:
                alerts.append(Alert('odom_replay', self.topic,
                                    f'implied speed {speed:.2f} m/s above '
                                    f'{self.max_speed:.2f} m/s',
                                    speed / self.max_speed))
        self._prev = (stamp, x, y)
        return alerts


def check_publishers(topic, publisher_names, allowed=None, max_publishers=1):
    """Flag unexpected publishers on a topic.

    ``publisher_names`` are fully qualified node names as reported by the
    ROS graph. With an ``allowed`` list, any other name is an alert; without
    one, only the publisher count is checked.
    """
    alerts = []
    if allowed:
        for name in publisher_names:
            if name not in allowed:
                alerts.append(Alert('publisher_allowlist', topic,
                                    f'unexpected publisher {name}'))
    if len(publisher_names) > max_publishers:
        alerts.append(Alert('publisher_count', topic,
                            f'{len(publisher_names)} publishers '
                            f'(expected at most {max_publishers})',
                            len(publisher_names) / max_publishers))
    return alerts
