"""Scoring for detection experiments (pure Python, no ROS imports).

An experiment has attack trials, each with a ground-truth start and end
time, and attack-free baseline driving. Events are the monitor's
``/ids/events`` messages, all in the same (simulation) time.
"""

from dataclasses import dataclass, field
import statistics


@dataclass
class TrialResult:
    attack: str
    trial: int
    start: float | None = None  # None: the attack never started
    end: float | None = None
    first_delay: float | None = None  # None: not detected
    detector_delays: dict = field(default_factory=dict)

    @property
    def detected(self):
        return self.first_delay is not None


def score_trial(attack, trial, start, end, events, post_s=2.0):
    """Score one trial from its truth window and the events seen around it.

    An event counts if it falls between the attack start and ``post_s``
    seconds after the attack end.
    """
    result = TrialResult(attack, trial, start, end)
    if start is None:
        return result
    stop = (end if end is not None else start) + post_s
    for e in sorted(events, key=lambda e: e['t']):
        if start <= e['t'] <= stop:
            delay = e['t'] - start
            result.detector_delays.setdefault(e['detector'], delay)
            if result.first_delay is None:
                result.first_delay = delay
    return result


def summarize(results, baseline_s, false_alarms):
    """Markdown report: one row per attack, plus the false-alarm rate.

    ``false_alarms`` is a list of event dicts that raised an alarm during
    attack-free driving.
    """
    lines = ['| Attack | Detected | Median delay | Worst delay | Detectors that fired |',
             '|---|---|---|---|---|']
    for attack in sorted({r.attack for r in results}):
        rs = [r for r in results if r.attack == attack and r.start is not None]
        hits = [r for r in rs if r.detected]
        delays = [r.first_delay for r in hits]
        fired = {}
        for r in hits:
            for d in r.detector_delays:
                fired[d] = fired.get(d, 0) + 1
        detectors = ', '.join(f'{d} ({n}/{len(rs)})'
                              for d, n in sorted(fired.items(), key=lambda x: -x[1]))
        lines.append(
            f'| `{attack}` | {len(hits)}/{len(rs)} '
            f"| {f'{statistics.median(delays):.2f} s' if delays else '-'} "
            f"| {f'{max(delays):.2f} s' if delays else '-'} "
            f'| {detectors or "-"} |')
    hours = baseline_s / 3600.0
    rate = len(false_alarms) / hours if hours > 0 else float('nan')
    causes = {}
    for e in false_alarms:
        causes[e['detector']] = causes.get(e['detector'], 0) + 1
    cause_text = ', '.join(f'{d} ({n})' for d, n in sorted(causes.items(), key=lambda x: -x[1]))
    lines += ['',
              f'Attack-free driving: {baseline_s / 60:.1f} min, '
              f'{len(false_alarms)} false alarm(s) ({rate:.1f} per hour)'
              + (f'; caused by {cause_text}' if cause_text else '') + '.',
              '',
              'Delays are in simulation time. Gazebo Classic advances its clock in '
              '0.1 s steps, so 0.00 s means detected within the same step.']
    return '\n'.join(lines)
