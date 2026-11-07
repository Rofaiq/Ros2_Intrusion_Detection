"""Turn the robot's topics into feature vectors for a learned detector.

Pure Python, no ROS imports, like ``detectors.py``. The monitor feeds every
``/cmd_vel``, ``/odom`` and ``/scan`` message into a ``FeatureExtractor``
and asks it for one feature vector per time step. Each vector summarises
the last ``window_s`` seconds: message rates, how jumpy the commands are,
how well commands, odometry and scans agree with each other, and whether
stamps run backwards.

The same code produces the training data (recorded by the experiment
runner) and the vectors the trained model sees online, so the two always
match.
"""

from collections import deque
import math
import statistics

FEATURES = (
    'cmd_hz',          # /cmd_vel messages per second (wall clock)
    'odom_hz',         # /odom messages per second
    'scan_hz',         # /scan messages per second
    'cmd_jumps',       # command changes larger than a smooth driver makes
    'cmd_lin_std',     # spread of commanded linear speed
    'cmd_ang_std',     # spread of commanded angular speed
    'res_lin',         # |commanded - measured| linear speed
    'res_ang',         # |commanded - measured| angular speed
    'odom_backsteps',  # odometry stamps that went backwards
    'odom_speed_gap',  # |speed implied by poses - reported speed|
    'odom_max_step',   # largest pose jump between two odometry messages
    'scan_backsteps',  # scan stamps that went backwards
    'scan_change',     # largest median range change between two scans
    'scan_invalid',    # largest fraction of beams outside sensor limits
    'scan_front_gap',  # |change in front distance - change predicted by odometry|
)


def _std(values):
    return statistics.pstdev(values) if len(values) > 1 else 0.0


def _mean(values):
    return sum(values) / len(values) if values else 0.0


class FeatureExtractor:
    """Sliding-window features over the three robot topics.

    Times ``t`` are in (simulation) seconds; ``wall`` is a wall-clock time
    used only for the command rate, because commands are sent on wall-clock
    timers (see the note in ``monitor_node.py``).
    """

    def __init__(self, window_s=1.0, linear_jump=0.05, angular_jump=0.2,
                 cmd_timeout_s=0.5, front_half_angle=math.radians(15.0)):
        self.window_s = window_s
        self.linear_jump = linear_jump
        self.angular_jump = angular_jump
        self.cmd_timeout_s = cmd_timeout_s
        self.front_half_angle = front_half_angle
        self._cmd = deque()    # (wall, t, v, w, jumped)
        self._odom = deque()   # (t, backstep, step, speed_gap, res_v, res_w)
        self._scan = deque()   # (t, backstep, change, invalid, front_gap)
        self._last_cmd = None   # (t, v, w)
        self._last_odom = None  # (stamp, x, y, v, w)
        self._last_scan = None  # (stamp, ranges, front)

    # --- inputs ----------------------------------------------------------

    def add_cmd(self, t, wall, v, w):
        jumped = False
        if self._last_cmd is not None:
            _, pv, pw = self._last_cmd
            jumped = abs(v - pv) > self.linear_jump or abs(w - pw) > self.angular_jump
        self._last_cmd = (t, v, w)
        self._cmd.append((wall, t, v, w, jumped))

    def add_odom(self, t, stamp, x, y, v, w):
        backstep, step, gap = False, 0.0, 0.0
        if self._last_odom is not None:
            p_stamp, px, py, pv, _ = self._last_odom
            dt = stamp - p_stamp
            step = math.hypot(x - px, y - py)
            if dt <= 0.0:
                backstep = True
            else:
                gap = abs(step / dt - abs(pv))
        res_v = res_w = 0.0
        if self._last_cmd is not None and t - self._last_cmd[0] <= self.cmd_timeout_s:
            res_v = abs(self._last_cmd[1] - v)
            res_w = abs(self._last_cmd[2] - w)
        if not backstep:
            # Keep the newest stamp, as OdomReplayDetector does.
            self._last_odom = (stamp, x, y, v, w)
        self._odom.append((t, backstep, step, gap, res_v, res_w))

    def add_scan(self, t, stamp, ranges, range_min, range_max, angle_min, angle_increment):
        finite = [r for r in ranges if math.isfinite(r)]
        invalid = (sum(1 for r in finite if r < range_min or r > range_max) / len(ranges)
                   if ranges else 0.0)
        front = math.inf
        for i, r in enumerate(ranges):
            if math.isfinite(r) and range_min <= r <= range_max:
                a = angle_min + i * angle_increment
                a = math.atan2(math.sin(a), math.cos(a))
                if abs(a) <= self.front_half_angle:
                    front = min(front, r)
        backstep, change, front_gap = False, 0.0, 0.0
        if self._last_scan is not None:
            p_stamp, prev, p_front = self._last_scan
            dt = stamp - p_stamp
            if dt <= 0.0:
                backstep = True
            if len(prev) == len(ranges):
                diffs = [abs(a - b) for a, b in zip(ranges, prev)
                         if math.isfinite(a) and math.isfinite(b)]
                change = statistics.median(diffs) if diffs else 0.0
            # Driving straight towards a wall at speed v shortens the front
            # distance by about v * dt; turning makes this meaningless.
            if (dt > 0.0 and self._last_odom is not None and abs(self._last_odom[4]) < 0.2
                    and math.isfinite(front) and math.isfinite(p_front)):
                front_gap = min(abs((front - p_front) + self._last_odom[3] * dt), 1.0)
        if not backstep:
            self._last_scan = (stamp, list(ranges), front)
        self._scan.append((t, backstep, change, invalid, front_gap))

    # --- output ----------------------------------------------------------

    def features(self, t, wall):
        """Feature vector for the window ending now, in ``FEATURES`` order."""
        while self._cmd and wall - self._cmd[0][0] > self.window_s:
            self._cmd.popleft()
        for q in (self._odom, self._scan):
            while q and t - q[0][0] > self.window_s:
                q.popleft()
        cmd, odom, scan = list(self._cmd), list(self._odom), list(self._scan)
        return [
            len(cmd) / self.window_s,
            len(odom) / self.window_s,
            len(scan) / self.window_s,
            float(sum(1 for c in cmd if c[4])),
            _std([c[2] for c in cmd]),
            _std([c[3] for c in cmd]),
            _mean([o[4] for o in odom]),
            _mean([o[5] for o in odom]),
            float(sum(1 for o in odom if o[1])),
            _mean([o[3] for o in odom]),
            max((o[2] for o in odom), default=0.0),
            float(sum(1 for s in scan if s[1])),
            max((s[2] for s in scan), default=0.0),
            max((s[3] for s in scan), default=0.0),
            _mean([s[4] for s in scan]),
        ]
