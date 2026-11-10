"""Minimum-snap trajectories through a planned route.

The planner's route is a polyline: the drone would have to stop at every corner to follow it exactly.
:func:`min_snap` replaces it with one smooth curve per axis that passes through the same waypoints,
starts and ends at rest, and minimises snap (the fourth derivative of position). A quadrotor's thrust
and tilt rates map to jerk and snap, so a low-snap curve is one the drone can track without sharp
motor changes (Mellinger and Kumar, ICRA 2011).

Each segment between two waypoints is a 7th-order polynomial. The minimum-snap optimum is exactly the
7th-order spline whose first six derivatives are continuous at every interior waypoint, so the
coefficients come from one square linear system per axis, with no optimiser.

:func:`fit` adds the two things a plain fit lacks:

- **time allocation**: how long each segment takes. Segment times are scaled until the curve's peak
  speed and acceleration meet the limits, then shortened one at a time while the limits still hold.
- **clearance**: a smooth curve can bulge away from the polyline. Wherever it comes closer than the
  required clearance to a building, the offending segment is split at its midpoint (which lies on
  the safe polyline) and the curve is refitted (Richter, Bry and Roy, ISRR 2013).
"""

from __future__ import annotations

from dataclasses import dataclass
from math import factorial

import numpy as np

from planner import Obstacles

ORDER = 7                      # polynomial degree per segment
N_COEF = ORDER + 1
V_MAX = 2.0                    # [m/s] default speed limit
A_MAX = 2.0                    # [m/s^2] default acceleration limit (0.2 rad of tilt; the controller allows 0.35)
SAMPLE_DT = 0.01               # [s] sampling step for limit and clearance checks
MAX_SEGMENT = 1.0              # [m] longer straight runs get extra waypoints along them
CURVE_MARGIN = 0.36            # [m] beyond DRONE_RADIUS. The route touches DRONE_RADIUS + MARGIN (0.59 m) at
                               # its corners; the curve may come 4 cm closer than that, never more.


def _deriv_row(s: float, r: int) -> np.ndarray:
    """d^r/ds^r of [1, s, s^2, ..., s^7] at s."""
    row = np.zeros(N_COEF)
    for k in range(r, N_COEF):
        row[k] = factorial(k) / factorial(k - r) * s ** (k - r)
    return row


@dataclass
class Trajectory:
    """Piecewise polynomial in D dimensions (2 for the planner's routes at cruise height, 3 for race paths). Segment i
    runs for durations[i] seconds; within it, with s = (t - start_i) / durations[i] in [0, 1], position is
    sum_k coeffs[axis, i, k] * s**k."""
    waypoints: np.ndarray      # (M + 1, D)
    durations: np.ndarray      # (M,)
    coeffs: np.ndarray         # (D, M, 8)

    @property
    def total(self) -> float:
        return float(self.durations.sum())

    def evaluate(self, t: np.ndarray, r: int = 0) -> np.ndarray:
        """r-th time derivative at times t (clipped to the trajectory), shape (len(t), D)."""
        t = np.clip(np.atleast_1d(np.asarray(t, float)), 0.0, self.total)
        starts = np.concatenate([[0.0], np.cumsum(self.durations)[:-1]])
        seg = np.clip(np.searchsorted(starts, t, side="right") - 1, 0, len(self.durations) - 1)
        T = self.durations[seg]
        s = (t - starts[seg]) / T
        k = np.arange(r, N_COEF)
        basis = (np.array([factorial(j) / factorial(j - r) for j in k]) * s[:, None] ** (k - r))   # (n, 8 - r)
        out = np.einsum("nk,ank->na", basis, self.coeffs[:, seg][:, :, r:])
        return out / T[:, None] ** r

    def sample(self, dt: float = SAMPLE_DT) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        t = np.arange(0.0, self.total + dt / 2, dt)
        return t, self.evaluate(t), self.evaluate(t, 1), self.evaluate(t, 2)

    def peaks(self) -> tuple[float, float]:
        """Peak speed [m/s] and peak acceleration [m/s^2]."""
        _, _, v, a = self.sample()
        return float(np.linalg.norm(v, axis=1).max()), float(np.linalg.norm(a, axis=1).max())

    def clearance(self, obstacles: Obstacles) -> np.ndarray:
        """Distance to the nearest building at each sample."""
        return obstacles.distance(self.sample()[1])

    def length(self) -> float:
        p = self.sample()[1]
        return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def min_snap(waypoints: np.ndarray, durations: np.ndarray) -> Trajectory:
    """The minimum-snap curve through the waypoints, at rest at both ends, with the given segment times."""
    w = np.asarray(waypoints, float)
    T = np.asarray(durations, float)
    m = len(T)
    if len(w) != m + 1:
        raise ValueError("need one more waypoint than segment durations")
    n = N_COEF * m
    A = np.zeros((n, n))
    rows = 0

    def put(seg, s, r, scale=1.0):
        A[rows, seg * N_COEF:(seg + 1) * N_COEF] += scale * _deriv_row(s, r) / T[seg] ** r

    b = np.zeros((n, w.shape[1]))
    for i in range(m):                              # pass through both ends of every segment
        put(i, 0.0, 0); b[rows] = w[i]; rows += 1
        put(i, 1.0, 0); b[rows] = w[i + 1]; rows += 1
    for r in (1, 2, 3):                             # start and end at rest: velocity, accel, jerk zero
        put(0, 0.0, r); rows += 1
        put(m - 1, 1.0, r); rows += 1
    for i in range(m - 1):                          # derivatives 1..6 continuous at interior waypoints
        for r in range(1, ORDER):
            put(i, 1.0, r)
            put(i + 1, 0.0, r, -1.0)
            rows += 1
    assert rows == n
    c = np.linalg.solve(A, b)                       # (n, D)
    return Trajectory(w, T, c.T.reshape(w.shape[1], m, N_COEF))


def _scale_to_limits(w: np.ndarray, T: np.ndarray, v_max: float, a_max: float) -> Trajectory:
    """Stretch or shrink all segment times together until the tighter limit is exactly met.
    Uniform scaling by f divides speed by f and acceleration by f^2, so one step is exact."""
    tr = min_snap(w, T)
    v, a = tr.peaks()
    f = max(v / v_max, np.sqrt(a / a_max))
    return min_snap(w, T * f)


def _initial_durations(w: np.ndarray, v_max: float, a_max: float) -> np.ndarray:
    d = np.linalg.norm(np.diff(w, axis=0), axis=1)
    # Time to cover d from rest to rest with a trapezoidal speed profile: a guess, refined below.
    return np.where(d > v_max ** 2 / a_max, d / v_max + v_max / a_max, 2 * np.sqrt(d / a_max))


def _allocate(w: np.ndarray, v_max: float, a_max: float, passes: int = 4) -> Trajectory:
    """Segment times: scale to the limits, then try shortening each segment while the limits hold."""
    best = _scale_to_limits(w, _initial_durations(w, v_max, a_max), v_max, a_max)
    if len(w) == 2:
        return best
    for _ in range(passes):
        improved = False
        for i in range(len(best.durations)):
            for factor in (0.8, 1.25):
                T = best.durations.copy()
                T[i] *= factor
                cand = _scale_to_limits(w, T, v_max, a_max)
                if cand.total < best.total - 1e-3:
                    best, improved = cand, True
        if not improved:
            break
    return best


def subdivide(waypoints: np.ndarray, max_segment: float = MAX_SEGMENT) -> np.ndarray:
    """Add evenly spaced points along every run longer than max_segment.

    One polynomial from rest to rest has a bell-shaped speed profile that peaks at about 2.2 times its
    average, so a long straight run under a speed limit is flown slowly. Extra knots on the line let the
    speed plateau near the limit, and they also keep the curve on the (safe) polyline."""
    w = np.asarray(waypoints, float)
    out = [w[0]]
    for a, b in zip(w, w[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / max_segment)))
        out += [a + (b - a) * k / n for k in range(1, n + 1)]
    return np.array(out)


def fit(waypoints: np.ndarray, obstacles: Obstacles, clearance: float,
        v_max: float = V_MAX, a_max: float = A_MAX, max_segment: float = MAX_SEGMENT,
        max_splits: int = 40) -> Trajectory:
    """Fastest minimum-snap curve through the route that respects the limits and keeps `clearance`
    from every building. Raises if splitting segments cannot make it clear."""
    w = subdivide(waypoints, max_segment)
    for _ in range(max_splits + 1):
        tr = _allocate(w, v_max, a_max)
        t, p, _, _ = tr.sample()
        d = obstacles.distance(p)
        if d.min() >= clearance:
            return tr
        # Split every segment whose curve dips below the clearance at its midpoint.
        starts = np.concatenate([[0.0], np.cumsum(tr.durations)])
        bad = sorted({int(np.clip(np.searchsorted(starts, ti, side="right") - 1, 0, len(tr.durations) - 1))
                      for ti in t[d < clearance]})
        new = [w[0]]
        for i in range(len(w) - 1):
            if i in bad:
                new.append((w[i] + w[i + 1]) / 2)
            new.append(w[i + 1])
        w = np.array(new)
    raise ValueError(f"curve still within {clearance} m of a building after {max_splits} rounds of splitting")
