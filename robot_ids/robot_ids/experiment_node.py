"""Experiment runner: measures detection rate, delay and false alarms.

Run it with ``ros2 launch robot_ids experiment.launch.py`` (which also
starts the simulation, the monitor, the gate and the scripted driver).
It first drives without attacks for ``baseline_s`` seconds, counting false
alarms, then runs each attack ``trials`` times in round-robin order. Each
attack runs as a separate ``attack_injector`` process, as a real attacker
would. Results go to ``~/ids_results/<date-time>/``:

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
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_ids.attack_injector_node import ATTACKS
from robot_ids.experiment import score_trial, summarize


class ExperimentNode(Node):

    def __init__(self):
        super().__init__('ids_experiment')
        self.declare_parameter('attacks', list(ATTACKS))
        self.declare_parameter('trials', 10)
        self.declare_parameter('baseline_s', 300.0)
        self.declare_parameter('settle_s', 8.0)
        self.declare_parameter('start_delay_s', 3.0)
        self.declare_parameter('duration_s', 10.0)
        self.declare_parameter('post_s', 2.0)
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
        self.phase = 'warmup'
        self.trial = None
        self.phase_end = None  # set once the simulated robot is up
        self.ignore_until = 0.0
        self.baseline_start = None
        self.baseline_s = 0.0

        self.create_subscription(String, '/ids/events', self.on_event, 100)
        self.create_subscription(String, '/ids/attack_truth', self.on_truth, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.reset_client = self.create_client(Trigger, '/ids/reset')
        self.create_timer(0.1, self.on_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        minutes = (p('baseline_s') + len(self.plan) * (
            p('settle_s') + p('start_delay_s') + p('duration_s') + p('post_s') + 2)) / 60
        self.get_logger().info(f'Experiment: {len(self.plan)} attack trials after '
                               f'{p("baseline_s"):.0f} s of normal driving, '
                               f'about {minutes:.0f} min. Results: {self.out_dir}')

    # --- callbacks -------------------------------------------------------

    def on_event(self, msg):
        e = json.loads(msg.data)
        e['phase'] = self.phase
        e['trial'] = '' if self.trial is None else f'{self.trial[0]}#{self.trial[1]}'
        self.events.append(e)
        if self.phase == 'baseline' and time.monotonic() >= self.ignore_until:
            self.false_alarms.append(e)
            self.get_logger().warn(f"False alarm: {e['detector']} on {e['topic']}: "
                                   f"{e['message']}")
            self.reset_alarm()
            self.ignore_until = time.monotonic() + self.p('settle_s')

    def on_odom(self, msg):
        # Gazebo can take a while to start; begin the clock on the first
        # odometry message, once the monitor's reset service is up too.
        if self.phase_end is None and self.reset_client.service_is_ready():
            self.phase_end = time.monotonic() + self.p('settle_s') + 5.0
            self.get_logger().info('Robot is up, warming up')

    def on_truth(self, msg):
        t = json.loads(msg.data)
        self.truth[t['state']] = t['t']

    # --- state machine ---------------------------------------------------

    def on_tick(self):
        now = time.monotonic()
        if self.phase_end is None:
            return
        if self.phase == 'warmup' and now >= self.phase_end:
            self.reset_alarm()
            self.phase = 'baseline'
            self.baseline_start = now
            self.ignore_until = now + self.p('settle_s')
            self.phase_end = now + self.p('baseline_s')
            self.get_logger().info('Baseline: driving without attacks')
        elif self.phase == 'baseline' and now >= self.phase_end:
            self.baseline_s = now - self.baseline_start
            self.next_trial(now)
        elif self.phase == 'settle' and now >= self.phase_end:
            self.start_attack(now)
        elif self.phase == 'attack' and now >= self.phase_end:
            self.finish_trial(now)

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
        self.truth = {}
        self.events_before = len(self.events)
        self.phase = 'attack'
        self.phase_end = now + self.p('start_delay_s') + self.p('duration_s') + self.p('post_s') + 2
        self.get_logger().info(f'Trial {i + 1}/{self.p("trials")}: {attack}')
        self.proc = subprocess.Popen([
            'ros2', 'run', 'robot_ids', 'attack_injector', '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', f'attack:={attack}',
            '-p', f'start_delay_s:={self.p("start_delay_s")}',
            '-p', f'duration_s:={self.p("duration_s")}',
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

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
        if self.proc is None:
            return
        self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        self.proc = None

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
        summary = summarize(self.results, self.baseline_s, self.false_alarms)
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
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.stop_attacker()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
