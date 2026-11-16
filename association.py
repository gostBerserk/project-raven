"""Navigation: match anonymous detections from one frame to the next and estimate each
object's velocity. Positions can be in any consistent length unit (metres or pixels)."""
import math

from . import params
from .estimator import TrackedPoint


class ThreatTracker:
    """Greedy nearest-neighbour association with a distance gate.

    update(points, t) takes the detections of one frame, [(x, y), ...], and returns
    [(x, y, vx, vy), ...] for every object seen in that frame. Tracks that have not
    been seen for `stale_s` seconds are dropped."""

    def __init__(self, gate, stale_s=0.5, history=params.TRACK_HISTORY):
        self.gate = gate
        self.stale_s = stale_s
        self.history = history
        self.tracks = {}
        self._next_id = 0

    def update(self, points, t):
        assigned = set()
        for (x, y) in points:
            best_id, best_d = None, self.gate
            for tid, tr in self.tracks.items():
                if tid in assigned:
                    continue
                px, py = tr.position()
                d = math.hypot(px - x, py - y)
                if d < best_d:
                    best_id, best_d = tid, d
            if best_id is None:
                best_id = self._next_id
                self._next_id += 1
                self.tracks[best_id] = TrackedPoint(maxlen=self.history)
            self.tracks[best_id].update(x, y, t)
            assigned.add(best_id)
        for tid in [k for k, tr in self.tracks.items() if t - tr.last_time() > self.stale_s]:
            del self.tracks[tid]
        out = []
        for tid in assigned:
            tr = self.tracks[tid]
            x, y = tr.position()
            vx, vy = tr.velocity()
            out.append((x, y, vx, vy))
        return out
