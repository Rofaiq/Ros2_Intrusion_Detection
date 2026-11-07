"""Training and numpy inference of the learned detectors."""

import random

import pytest

np = pytest.importorskip('numpy')
pytest.importorskip('sklearn')

from robot_ids.experiment import TrialResult, label_windows  # noqa: E402
from robot_ids.features import FEATURES  # noqa: E402
from robot_ids.learned import LearnedDetector, Model  # noqa: E402
from robot_ids import train  # noqa: E402

N = len(FEATURES)


def normal_x(rng):
    x = [10.0, 30.0, 5.0] + [abs(rng.gauss(0.0, 0.02)) for _ in range(N - 3)]
    return x


def attack_x(rng):
    x = normal_x(rng)
    x[0] = 30.0 + rng.random() * 5   # extra command stream
    x[3] = 10.0 + rng.random() * 5   # command jumps
    return x


def synthetic_rows(seed=0):
    """A fake recording: baseline driving, then 10 trials of one attack."""
    rng = random.Random(seed)
    rows, t = [], 0.0
    for _ in range(400):
        rows.append({'file': 0, 't': t, 'label': 'normal', 'trial': '', 'phase': 'baseline',
                     'attack_start': None, 'x': normal_x(rng)})
        t += 0.25
    for i in range(10):
        trial = f'cmd_vel_spoof#{i}'
        for _ in range(20):
            rows.append({'file': 0, 't': t, 'label': 'normal', 'trial': trial,
                         'phase': 'settle', 'attack_start': None, 'x': normal_x(rng)})
            t += 0.25
        start = t
        for _ in range(20):
            rows.append({'file': 0, 't': t, 'label': 'cmd_vel_spoof', 'trial': trial,
                         'phase': 'attack', 'attack_start': start, 'x': attack_x(rng)})
            t += 0.25
    return rows


def test_numpy_forward_pass_matches_sklearn():
    from sklearn.neural_network import MLPClassifier, MLPRegressor
    rng = np.random.default_rng(0)
    x = rng.random((200, N))
    y = (x[:, 0] > 0.5).astype(int)
    mean, std = train.scaling(x)
    z = (np.log1p(x) - mean) / std
    clf = MLPClassifier(hidden_layer_sizes=(16, 8), max_iter=500, random_state=0).fit(z, y)
    m = Model(train.export('mlp', clf, mean, std, 0.5))
    assert np.allclose(m.score(x), clf.predict_proba(z)[:, 1], atol=1e-9)
    reg = MLPRegressor(hidden_layer_sizes=(10, 4, 10), max_iter=500, random_state=0).fit(z, z)
    a = Model(train.export('autoencoder', reg, mean, std, 1.0))
    assert np.allclose(a.score(x), np.mean((reg.predict(z) - z) ** 2, axis=1), atol=1e-9)


def test_training_detects_synthetic_attacks():
    models, report = train.train_and_report(synthetic_rows())
    assert '## mlp' in report and '## autoencoder' in report and 'leave-one-attack-out' in report
    assert '| `cmd_vel_spoof` | 3/3 |' in report   # trials 7-9 are the test set
    rng = random.Random(1)
    for name in ('mlp', 'autoencoder'):
        det = LearnedDetector(name, models[name], consecutive=2)
        assert not any(det.check(normal_x(rng)) for _ in range(50))
        alerts = [det.check(attack_x(rng)) for _ in range(3)]
        assert alerts[-1] and alerts[-1][0].detector == name


def test_label_windows():
    results = [TrialResult('flood', 0, start=100.0, end=110.0),
               TrialResult('flood', 1)]  # never started
    windows = [{'t': 99.0, 'phase': 'attack', 'trial': 'flood#0'},
               {'t': 105.0, 'phase': 'attack', 'trial': 'flood#0'},
               {'t': 111.0, 'phase': 'attack', 'trial': 'flood#0'},
               {'t': 112.5, 'phase': 'settle', 'trial': 'flood#1'},
               {'t': 120.0, 'phase': 'attack', 'trial': 'flood#1'},
               {'t': 130.0, 'phase': 'settle', 'trial': 'flood#2'}]
    labels = [r['label'] for r in label_windows(windows, results, duration_s=10.0)]
    assert labels == ['normal', 'flood', 'transition', 'transition', 'transition', 'normal']
