"""Experiment runner: measures detection rate, delay and false alarms.

Run it with ``ros2 launch robot_ids experiment.launch.py`` (which also
starts the simulation, the monitor, the gate and the scripted driver).
It first drives without attacks for ``baseline_s`` seconds, counting false
alarms, then runs each attack ``trials`` times in round-robin order. Each
attack runs as a separate ``attack_injector`` process, as a real attacker
would. All phases are timed in simulation time, so the experiment gives the
same results on a slow computer, where Gazebo runs below real time; it just
takes longer. Results go to ``~/ids_results/<date-time>/``:

* ``summary.md``: the results table, ready to paste into the README;
* ``trials.csv``: one row per attack trial;
* ``events.csv``: every monitor event, labelled with phase and trial.
"""

import csv
from datetime import datetime
import json
import os
import signal
import subprocess
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_ids.attack_injector_node import ATTACKS
from robot_ids.experiment import score_trial, summarize, trial_over


class ExperimentNode(Node):

    def __init__(self, **kwargs):
        super().__init__('ids_experiment', **kwargs)
        self.declare_parameter('attacks', list(ATTACKS))
        self.declare_parameter('trials', 10)
        self.declare_parameter('baseline_s', 300.0)
        self.declare_parameter('settle_s', 8.0)
        self.declare_parameter('start_delay_s', 3.0)
        self.declare_parameter('duration_s', 10.0)
        self.declare_parameter('post_s', 2.0)
        # Wall-clock limit per trial, in case the simulation freezes.
        self.declare_parameter('trial_timeout_s', 300.0)
        self.declare_parameter('output_dir', '~/ids_results')
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.p = p

        self.plan = [(a, i) for i in range(p('trials')) for a in p('attacks')]
        out = os.path.expanduser(p('output_dir'))
        self.out_dir = os.path.join(out, datetime.now().strftime('%Y%m%d-%H%M%S'))
        os.makedirs(self.out_dir, exist_ok=True)

        self.events = []         # every event, labelled
        self.false_alarms = []   # events that raised an alarm while driving normally
        self.results = []
        self.truth = {}
        self.proc = None
        self.launches = 0        # attacker processes started so far
        self.run_id = None       # tag of the current attacker's truth messages
        self.phase = 'warmup'
        self.trial = None
        self.phase_end = None    # simulation time; set once the robot is up
        self.ignore_until = 0.0
        self.baseline_start = None  # (simulation time, wall time)
        self.baseline_s = 0.0
        self.real_time_factor = None

        self.create_subscription(String, '/ids/events', self.on_event, 100)
        self.create_subscription(String, '/ids/attack_truth', self.on_truth, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.reset_client = self.create_client(Trigger, '/ids/reset')
        # A wall-clock timer keeps ticking even if /clock is slow or stalls.
        self.create_timer(0.1, self.on_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        minutes = (p('baseline_s') + len(self.plan) * (
            p('settle_s') + p('start_delay_s') + p('duration_s') + p('post_s') + 1)) / 60
        self.get_logger().info(f'Experiment: {len(self.plan)} attack trials after '
                               f'{p("baseline_s"):.0f} s of normal driving, about '
                               f'{minutes:.0f} min of simulation time (longer if the '
                               f'simulation runs slower than real time). '
                               f'Results: {self.out_dir}')

    def sim_now(self):
        """Simulation time in seconds, or None before /clock arrives."""
        t = self.get_clock().now().nanoseconds * 1e-9
        return t if t > 0.0 else None

    # --- callbacks -------------------------------------------------------

    def on_event(self, msg):
        e = json.loads(msg.data)
        e['phase'] = self.phase
        e['trial'] = '' if self.trial is None else f'{self.trial[0]}#{self.trial[1]}'
        self.events.append(e)
        now = self.sim_now()
        if self.phase == 'baseline' and now is not None and now >= self.ignore_until:
            self.false_alarms.append(e)
            self.get_logger().warn(f"False alarm: {e['detector']} on {e['topic']}: "
                                   f"{e['message']}")
            self.reset_alarm()
            self.ignore_until = now + self.p('settle_s')

    def on_odom(self, msg):
        # Gazebo can take a while to start; begin on the first odometry
        # message, once the clock runs and the monitor's reset service is up.
        now = self.sim_now()
        if self.phase_end is None and now is not None and self.reset_client.service_is_ready():
            self.phase_end = now + self.p('settle_s') + 5.0
            self.get_logger().info('Robot is up, warming up')

    def on_truth(self, msg):
        t = json.loads(msg.data)
        # Only trust the attacker this trial started.
        if t.get('run') != self.run_id:
            self.get_logger().warn(f'Ignoring ground truth from another attacker: {msg.data}')
            return
        state = 'end' if t['state'] == 'aborted' else t['state']
        self.truth[state] = t['t']

    # --- state machine ---------------------------------------------------

    def on_tick(self):
        now = self.sim_now()
        if now is None or self.phase_end is None:
            return
        if self.phase == 'warmup' and now >= self.phase_end:
            self.check_no_attackers()
            self.reset_alarm()
            self.phase = 'baseline'
            self.baseline_start = (now, time.monotonic())
            self.ignore_until = now + self.p('settle_s')
            self.phase_end = now + self.p('baseline_s')
            self.get_logger().info('Baseline: driving without attacks')
        elif self.phase == 'baseline' and now >= self.phase_end:
            sim0, wall0 = self.baseline_start
            self.baseline_s = now - sim0
            self.real_time_factor = self.baseline_s / max(time.monotonic() - wall0, 1e-6)
            self.get_logger().info(f'Baseline done: the simulation runs at '
                                   f'{self.real_time_factor:.2f}x real time')
            self.next_trial(now)
        elif self.phase == 'settle' and now >= self.phase_end:
            self.start_attack(now)
        elif self.phase == 'attack' and (
                trial_over(now, self.launched, self.truth, self.p('start_delay_s'),
                           self.p('duration_s'), self.p('post_s'))
                or time.monotonic() > self.wall_deadline):
            self.finish_trial(now)

    def check_no_attackers(self):
        """Stop if attacker processes from an earlier run are still alive.

        They would attack during this run and spoil every result.
        """
        stray = sorted(n for n in self.get_node_names() if n.startswith('attack_injector'))
        if stray:
            self.get_logger().error(
                f'Attacker nodes from an earlier run are still running ({", ".join(stray)}). '
                'Stop them with "pkill -f attack_injector" and start the experiment again.')
            raise SystemExit(1)

    def next_trial(self, now):
        self.reset_alarm()
        if not self.plan:
            self.finish()
            return
        self.trial = self.plan.pop(0)
        self.phase = 'settle'
        self.phase_end = now + self.p('settle_s')

    def start_attack(self, now):
        attack, i = self.trial
        self.launches += 1
        self.run_id = f'trial{self.launches}'
        self.truth = {}
        self.events_before = len(self.events)
        self.phase = 'attack'
        self.launched = now
        self.wall_deadline = time.monotonic() + self.p('trial_timeout_s')
        self.get_logger().info(f'Trial {i + 1}/{self.p("trials")}: {attack}')
        # A new session puts "ros2 run" and the attacker it starts in their
        # own process group, so stop_attacker() can signal both.
        self.proc = subprocess.Popen([
            'ros2', 'run', 'robot_ids', 'attack_injector', '--ros-args',
            '-r', f'__node:=attack_injector_{self.launches}',
            '-p', 'use_sim_time:=true',
            '-p', f'attack:={attack}',
            '-p', f'start_delay_s:={self.p("start_delay_s")}',
            '-p', f'duration_s:={self.p("duration_s")}',
            '-p', f'run_id:={self.run_id}',
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

    def finish_trial(self, now):
        self.stop_attacker()
        attack, i = self.trial
        result = score_trial(attack, i, self.truth.get('start'), self.truth.get('end'),
                             self.events[self.events_before:], self.p('post_s'))
        self.results.append(result)
        if result.start is None:
            self.get_logger().error('  attack never started (check the injector)')
        elif result.detected:
            self.get_logger().info(f'  detected after {result.first_delay:.2f} s by '
                                   f'{", ".join(result.detector_delays)}')
        else:
            self.get_logger().warn('  NOT detected')
        self.next_trial(now)

    def stop_attacker(self):
        """Stop the attacker process and everything it started."""
        if self.proc is None:
            return
        # The attacker normally exits by itself when its attack is over.
        # Signalling only "ros2 run" would not reach the attacker it started,
        # so signal the whole process group.
        if self.proc.poll() is None:
            self.signal_attacker(signal.SIGINT)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('Attacker did not stop on SIGINT, killing it')
        self.signal_attacker(signal.SIGKILL)
        self.proc.wait()
        self.proc = None

    def signal_attacker(self, sig):
        try:
            os.killpg(self.proc.pid, sig)
        except ProcessLookupError:
            pass  # already gone

    def reset_alarm(self):
        if self.reset_client.service_is_ready():
            self.reset_client.call_async(Trigger.Request())

    # --- output ----------------------------------------------------------

    def finish(self):
        self.phase = 'done'
        with open(os.path.join(self.out_dir, 'events.csv'), 'w', newline='') as f:
            fields = ['t', 'phase', 'trial', 'detector', 'topic', 'message', 'score']
            w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
            w.writeheader()
            w.writerows(self.events)
        with open(os.path.join(self.out_dir, 'trials.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['attack', 'trial', 'start', 'end', 'detected', 'first_delay_s',
                        'detector_delays'])
            for r in self.results:
                w.writerow([r.attack, r.trial, r.start, r.end, r.detected,
                            '' if r.first_delay is None else round(r.first_delay, 3),
                            json.dumps({k: round(v, 3) for k, v in r.detector_delays.items()})])
        summary = summarize(self.results, self.baseline_s, self.false_alarms,
                            self.real_time_factor)
        with open(os.path.join(self.out_dir, 'summary.md'), 'w') as f:
            f.write(summary + '\n')
        self.get_logger().info('\n' + summary)
        self.get_logger().info(f'Results written to {self.out_dir}')
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.stop_attacker()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
