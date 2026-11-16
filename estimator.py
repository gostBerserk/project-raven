"""Navigation: turn noisy position measurements into position + velocity estimates."""
from collections import deque

import numpy as np

from . import params


class TrackedPoint:
    """Keeps recent (x, y, t) samples of one object and estimates its velocity."""

    def __init__(self, maxlen=params.TRACK_HISTORY):
        self.history = deque(maxlen=maxlen)

    def update(self, x, y, t):
        self.history.append((x, y, t))

    def position(self):
        if not self.history:
            return None
        x, y, _ = self.history[-1]
        return x, y

    def last_time(self):
        return self.history[-1][2] if self.history else None

    def velocity(self):
        """Least-squares slope over the history; far less jittery than differencing
        the first and last sample."""
        n = len(self.history)
        if n < 2:
            return 0.0, 0.0
        xs, ys, ts = (np.array(v, dtype=float) for v in zip(*self.history))
        ts = ts - ts[0]
        if ts[-1] <= 0:
            return 0.0, 0.0
        if n == 2:
            return (xs[1] - xs[0]) / ts[1], (ys[1] - ys[0]) / ts[1]
        return float(np.polyfit(ts, xs, 1)[0]), float(np.polyfit(ts, ys, 1)[0])

