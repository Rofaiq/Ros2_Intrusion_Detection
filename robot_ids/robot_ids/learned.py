"""Run a trained detector on feature vectors (numpy only).

Models are trained offline by ``train.py`` with scikit-learn and saved as
plain JSON: the feature names, the scaling, the weight matrices and an
alarm threshold. Running them needs nothing but numpy, so the robot does
not need scikit-learn, and the file is easy to read and check.

Two kinds of model:

* ``mlp``: a classifier trained on normal and attack windows; its score is
  the probability that a window is an attack.
* ``autoencoder``: trained on normal windows only to reproduce its input;
  its score is the reconstruction error, which grows for anything unlike
  normal driving, including attacks it has never seen.
"""

import json

import numpy as np

from robot_ids.detectors import Alert


Z_CLIP = 50.0  # keeps wildly abnormal inputs from overflowing the network


def transform(x, mean, std):
    """Scale raw features the way the models were trained.

    log1p tames heavy-tailed features such as the 1 kHz flood rate; then each
    feature is standardised with the mean and spread seen in normal driving.
    """
    z = (np.log1p(np.maximum(np.asarray(x, dtype=float), 0.0)) - mean) / std
    return np.clip(z, -Z_CLIP, Z_CLIP)


def load_models(path):
    with open(path) as f:
        return json.load(f)


class Model:
    """Forward pass of a small fully connected network saved by train.py."""

    def __init__(self, spec):
        self.kind = spec['kind']
        self.features = list(spec['features'])
        self.mean = np.asarray(spec['mean'], dtype=float)
        self.std = np.asarray(spec['std'], dtype=float)
        self.weights = [np.asarray(w, dtype=float) for w in spec['weights']]
        self.biases = [np.asarray(b, dtype=float) for b in spec['biases']]
        self.threshold = float(spec['threshold'])

    def scale(self, x):
        return transform(x, self.mean, self.std)

    def forward(self, z):
        a = z
        for i, (w, b) in enumerate(zip(self.weights, self.biases)):
            a = a @ w + b
            if i < len(self.weights) - 1:
                a = np.maximum(a, 0.0)  # ReLU in the hidden layers
        return a

    def score(self, x):
        """Score one feature vector (or a 2-D array of them)."""
        z = self.scale(x)
        out = self.forward(z)
        if self.kind == 'mlp':
            return 1.0 / (1.0 + np.exp(-out[..., 0]))
        return np.mean((out - z) ** 2, axis=-1)


class LearnedDetector:
    """Raise an alert when a model's score stays above its threshold.

    Requiring ``consecutive`` windows in a row trades a little delay for
    far fewer false alarms from one odd window.
    """

    def __init__(self, name, spec, topic='model', consecutive=2):
        self.name = name
        self.model = Model(spec)
        self.topic = topic
        self.consecutive = consecutive
        self._run = 0

    def check(self, x):
        s = float(self.model.score(x))
        self._run = self._run + 1 if s > self.model.threshold else 0
        if self._run >= self.consecutive:
            return [Alert(self.name, self.topic,
                          f'{self.name} score {s:.3f} above {self.model.threshold:.3f} '
                          f'for {self._run} windows', s / self.model.threshold)]
        return []
