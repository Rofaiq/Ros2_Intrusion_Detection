"""Train the learned detectors from recorded feature windows.

  ros2 run robot_ids train_detector ~/ids_results/<date-time>/features.csv

Record the data first with
``ros2 launch robot_ids experiment.launch.py detectors:=none record:=true``
(see the README). This trains two models and writes them, with a report,
next to the data (or to ``--out``):

* an MLP classifier on normal and attack windows;
* an autoencoder on normal windows only, which flags anything unusual.

Trials 0-6 of each attack and the first 70 % of the attack-free driving
are used for training; trials 7-9 and the rest of the driving are kept for
testing. To see how the MLP copes with an attack it has never seen, it is
also retrained once per attack with that attack left out
("leave-one-attack-out").

Needs scikit-learn on the training computer (``sudo apt install
python3-sklearn``); the robot itself only needs numpy (see ``learned.py``).
"""

import argparse
import csv
import json
import os
import statistics

import numpy as np

from robot_ids.features import FEATURES
from robot_ids.learned import Model, transform

NORMAL = 'normal'
IGNORED = 'transition'  # just after an attack: neither clean nor attacked


def load_rows(paths):
    """Read features.csv files into a list of dicts (one per window)."""
    rows = []
    for k, path in enumerate(paths):
        with open(path, newline='') as f:
            for r in csv.DictReader(f):
                row = {'file': k, 't': float(r['t']), 'label': r['label'],
                       'trial': r['trial'], 'phase': r['phase'],
                       'attack_start': float(r['attack_start']) if r['attack_start'] else None,
                       'x': [float(r[name]) for name in FEATURES]}
                rows.append(row)
    rows.sort(key=lambda r: (r['file'], r['t']))
    return rows


def trial_index(row):
    return int(row['trial'].split('#')[1]) if '#' in row['trial'] else None


def assign_split(rows, test_from=7, baseline_train_fraction=0.7):
    """Mark each row 'train' or 'test' (by trial, and by time for the baseline)."""
    for k in {r['file'] for r in rows}:
        base = [r for r in rows if r['file'] == k and r['phase'] == 'baseline']
        cut = int(len(base) * baseline_train_fraction)
        for i, r in enumerate(base):
            r['split'] = 'train' if i < cut else 'test'
    for r in rows:
        if r['phase'] != 'baseline':
            i = trial_index(r)
            r['split'] = 'test' if i is not None and i >= test_from else 'train'
    return rows


def scaling(x_normal):
    z = np.log1p(np.maximum(x_normal, 0.0))
    mean = z.mean(axis=0)
    std = z.std(axis=0)
    # Features that hardly vary in normal driving (such as odometry stamps
    # going backwards, which never happens) get a floor on their spread, so
    # one odd value does not become an enormous input.
    std = np.maximum(std, 0.1)
    return mean, std


def export(kind, network, mean, std, threshold):
    return {'kind': kind, 'features': list(FEATURES),
            'mean': mean.tolist(), 'std': std.tolist(),
            'weights': [w.tolist() for w in network.coefs_],
            'biases': [b.tolist() for b in network.intercepts_],
            'threshold': float(threshold)}


def fit_autoencoder(x_normal, quantile=0.995, seed=0):
    from sklearn.neural_network import MLPRegressor
    mean, std = scaling(x_normal)
    z = transform(x_normal, mean, std)
    net = MLPRegressor(hidden_layer_sizes=(10, 4, 10), activation='relu',
                       max_iter=3000, random_state=seed)
    net.fit(z, z)
    spec = export('autoencoder', net, mean, std, 0.0)
    errors = Model(spec).score(x_normal)
    spec['threshold'] = float(np.quantile(errors, quantile))
    return spec


def fit_mlp(x, y, x_normal, threshold=0.9, seed=0):
    from sklearn.neural_network import MLPClassifier
    mean, std = scaling(x_normal)
    z = transform(x, mean, std)
    net = MLPClassifier(hidden_layer_sizes=(16, 8), activation='relu',
                        max_iter=3000, random_state=seed)
    net.fit(z, y)
    return export('mlp', net, mean, std, threshold)


def evaluate(spec, rows, consecutive=2):
    """Replay test rows through a model as the monitor would.

    Returns per-attack (detected, trials, delays) and the false alarms on
    normal rows. An alarm is ``consecutive`` windows above the threshold.
    """
    model = Model(spec)
    per_attack, false_alarms, normal_rows = {}, 0, 0
    for k in sorted({r['file'] for r in rows}):
        stream = [r for r in rows if r['file'] == k]
        scores = model.score(np.array([r['x'] for r in stream])) if stream else []
        run, alarm_prev, detected = 0, False, set()
        for r, s in zip(stream, scores):
            run = run + 1 if s > model.threshold else 0
            alarm = run >= consecutive
            if r['label'] == NORMAL:
                normal_rows += 1
                if alarm and not alarm_prev:
                    false_alarms += 1
            elif r['label'] != IGNORED:
                trials = per_attack.setdefault(r['label'], {})
                trials.setdefault(r['trial'], None)
                if alarm and r['trial'] not in detected:
                    detected.add(r['trial'])
                    trials[r['trial']] = max(r['t'] - r['attack_start'], 0.0)
            alarm_prev = alarm
    return per_attack, false_alarms, normal_rows


def step_s(rows):
    dts = [b['t'] - a['t'] for a, b in zip(rows, rows[1:])
           if a['file'] == b['file'] and b['t'] > a['t']]
    return statistics.median(dts) if dts else 0.25


def table(per_attack):
    lines = ['| Attack | Detected | Median delay | Worst delay |', '|---|---|---|---|']
    for attack in sorted(per_attack):
        delays = [d for d in per_attack[attack].values() if d is not None]
        n = len(per_attack[attack])
        lines.append(f'| `{attack}` | {len(delays)}/{n} '
                     f"| {f'{statistics.median(delays):.2f} s' if delays else '-'} "
                     f"| {f'{max(delays):.2f} s' if delays else '-'} |")
    return lines


def fa_line(false_alarms, normal_rows, step):
    minutes = normal_rows * step / 60.0
    return (f'Attack-free test windows: {minutes:.1f} min, {false_alarms} false alarm(s)'
            + (f' ({false_alarms / (minutes / 60.0):.1f} per hour).' if minutes > 0 else '.'))


def train_and_report(rows, mlp_threshold=0.9, ae_quantile=0.995, consecutive=2):
    assign_split(rows)
    usable = [r for r in rows if r['label'] != IGNORED]
    train = [r for r in usable if r['split'] == 'train']
    test = [r for r in rows if r['split'] == 'test']
    x_train = np.array([r['x'] for r in train])
    y_train = np.array([0 if r['label'] == NORMAL else 1 for r in train])
    x_normal = x_train[y_train == 0]
    if len(x_normal) == 0 or y_train.sum() == 0:
        raise SystemExit('Need both attack-free and attack windows to train.')
    step = step_s(rows)

    models = {'mlp': fit_mlp(x_train, y_train, x_normal, mlp_threshold),
              'autoencoder': fit_autoencoder(x_normal, ae_quantile)}
    report = ['# Learned detector report', '',
              f'Training windows: {len(train)} ({int(y_train.sum())} attack), '
              f'test windows: {len(test)}, one window every {step:.2f} s. '
              f'An alarm needs {consecutive} windows in a row above the threshold.', '']
    for name in ('mlp', 'autoencoder'):
        per_attack, fa, n = evaluate(models[name], test, consecutive)
        report += [f'## {name} (threshold {models[name]["threshold"]:.3f})', '']
        report += table(per_attack) + ['', fa_line(fa, n, step), '']

    # How does the MLP do on attacks it never saw in training?
    attacks = sorted({r['label'] for r in usable} - {NORMAL})
    unseen = {}
    for attack in attacks:
        keep = [r for r in train if r['label'] != attack]
        x = np.array([r['x'] for r in keep])
        y = np.array([0 if r['label'] == NORMAL else 1 for r in keep])
        if y.sum() == 0:
            continue
        spec = fit_mlp(x, y, x[y == 0], mlp_threshold)
        per_attack, _, _ = evaluate(spec, [r for r in test if r['label'] in (attack, NORMAL)],
                                    consecutive)
        if attack in per_attack:
            unseen[attack] = per_attack[attack]
    report += ['## mlp on unseen attacks (leave-one-attack-out)', '',
               'Each row is a separate MLP trained without that attack.', '']
    report += table(unseen) + ['']
    return models, '\n'.join(report)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('features', nargs='+', help='features.csv file(s) from record runs')
    parser.add_argument('--out', help='model file (default: model.json next to the data)')
    parser.add_argument('--mlp-threshold', type=float, default=0.9)
    parser.add_argument('--ae-quantile', type=float, default=0.995)
    parser.add_argument('--consecutive', type=int, default=2)
    a = parser.parse_args(args)
    paths = [os.path.expanduser(p) for p in a.features]
    rows = load_rows(paths)
    models, report = train_and_report(rows, a.mlp_threshold, a.ae_quantile, a.consecutive)
    out = os.path.expanduser(a.out) if a.out else os.path.join(
        os.path.dirname(os.path.abspath(paths[0])), 'model.json')
    models['consecutive'] = a.consecutive
    with open(out, 'w') as f:
        json.dump(models, f)
    report_path = os.path.splitext(out)[0] + '_report.md'
    with open(report_path, 'w') as f:
        f.write(report + '\n')
    print(report)
    print(f'\nModel written to {out}\nReport written to {report_path}')


if __name__ == '__main__':
    main()
