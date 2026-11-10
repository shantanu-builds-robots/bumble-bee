"""2-D path planning around a course's buildings at cruise height.

The drone transits at a fixed height, so planning is in the x/y plane: every building taller than the
cruise height is a rotated rectangle to keep clear of. :func:`plan` runs A* on an occupancy grid
whose obstacles are the building footprints grown by the drone's radius plus a safety margin, then
pulls the path taut into as few straight segments as the clearance allows. :func:`plan_mission`
plans every leg of a course's delivery mission.
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass

import numpy as np

from course import Course

DRONE_RADIUS = 0.19   # [m] props reach 0.184 m from the centre (hub 0.12 m + 0.064 m half-prop)
MARGIN = 0.40         # [m] extra clearance for tracking error and corner cutting
RESOLUTION = 0.05     # [m] grid cell size
BORDER = 2.0          # [m] free space planned around the course's extent
CHECK_STEP = 0.02     # [m] sampling step when checking a straight segment


@dataclass(frozen=True)
class Obstacles:
    """Building footprints that block flight at a given height."""
    centres: np.ndarray   # (n, 2)
    halves: np.ndarray    # (n, 2) half sizes along each building's own axes
    yaws: np.ndarray      # (n,) radians

    @classmethod
    def from_course(cls, course: Course, height: float) -> "Obstacles":
        bs = [b for b in course.buildings if b.size[2] > height - DRONE_RADIUS]
        return cls(np.array([b.xyz[:2] for b in bs]).reshape(-1, 2),
                   np.array([b.size[:2] / 2 for b in bs]).reshape(-1, 2),
                   np.radians([b.yaw for b in bs]))

    def distance(self, pts: np.ndarray) -> np.ndarray:
        """Distance [m] from each point (…, 2) to the nearest footprint; 0 inside one."""
        pts = np.asarray(pts, dtype=float)
        if len(self.centres) == 0:
            return np.full(pts.shape[:-1], np.inf)
        d = pts[..., None, :] - self.centres                      # (…, n, 2)
        c, s = np.cos(self.yaws), np.sin(self.yaws)
        local = np.stack([c * d[..., 0] + s * d[..., 1], -s * d[..., 0] + c * d[..., 1]], axis=-1)
        outside = np.maximum(np.abs(local) - self.halves, 0.0)
        return np.linalg.norm(outside, axis=-1).min(axis=-1)

    def segment_clear(self, a: np.ndarray, b: np.ndarray, clearance: float) -> bool:
        n = max(2, int(np.ceil(np.linalg.norm(b - a) / CHECK_STEP)) + 1)
        pts = a + np.linspace(0.0, 1.0, n)[:, None] * (b - a)
        return bool(self.distance(pts).min() >= clearance)


class Grid:
    def __init__(self, obstacles: Obstacles, lo: np.ndarray, hi: np.ndarray, clearance: float):
        self.lo = lo - BORDER
        shape = np.ceil((hi + BORDER - self.lo) / RESOLUTION).astype(int) + 1
        ix, iy = np.meshgrid(np.arange(shape[0]), np.arange(shape[1]), indexing="ij")
        centres = self.lo + np.stack([ix, iy], axis=-1) * RESOLUTION
        self.blocked = obstacles.distance(centres) < clearance

    def cell(self, p: np.ndarray) -> tuple[int, int]:
        i, j = np.round((p - self.lo) / RESOLUTION).astype(int)
        return int(i), int(j)

    def point(self, c: tuple[int, int]) -> np.ndarray:
        return self.lo + np.array(c) * RESOLUTION


_MOVES = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
          (1, 1, np.sqrt(2)), (1, -1, np.sqrt(2)), (-1, 1, np.sqrt(2)), (-1, -1, np.sqrt(2))]


def _astar(grid: Grid, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
    def h(c):  # octile distance: exact on an 8-connected grid without obstacles
        dx, dy = abs(c[0] - goal[0]), abs(c[1] - goal[1])
        return max(dx, dy) + (np.sqrt(2) - 1) * min(dx, dy)

    nx, ny = grid.blocked.shape
    g = {start: 0.0}
    parent = {start: None}
    heap = [(h(start), 0.0, start)]
    while heap:
        _, gc, c = heapq.heappop(heap)
        if c == goal:
            path = []
            while c is not None:
                path.append(c)
                c = parent[c]
            return path[::-1]
        if gc > g[c]:
            continue
        for dx, dy, w in _MOVES:
            n = (c[0] + dx, c[1] + dy)
            if not (0 <= n[0] < nx and 0 <= n[1] < ny) or grid.blocked[n]:
                continue
            ng = gc + w
            if ng < g.get(n, np.inf):
                g[n], parent[n] = ng, c
                heapq.heappush(heap, (ng + h(n), ng, n))
    raise ValueError("no path: the goal is walled off at this clearance")


def _shortcut(pts: list[np.ndarray], obstacles: Obstacles, clearance: float) -> list[np.ndarray]:
    """Greedy string pulling: from each kept point, jump to the farthest point still in clear view."""
    out, i = [pts[0]], 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not obstacles.segment_clear(pts[i], pts[j], clearance):
            j -= 1
        out.append(pts[j])
        i = j
    return out


def plan(course: Course, start, goal, height: float, margin: float = MARGIN) -> np.ndarray:
    """Waypoints (N, 2) from start to goal (both included) keeping DRONE_RADIUS + margin from buildings."""
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    obstacles = Obstacles.from_course(course, height)
    clearance = DRONE_RADIUS + margin
    for name, p in (("start", start), ("goal", goal)):
        if obstacles.distance(p) < clearance:
            raise ValueError(f"{name} {p} is within {clearance} m of a building")
    if obstacles.segment_clear(start, goal, clearance):
        return np.array([start, goal])
    pts = np.vstack([start, goal] + [c for c in obstacles.centres])
    grid = Grid(obstacles, pts.min(axis=0), pts.max(axis=0), clearance)
    cells = _astar(grid, grid.cell(start), grid.cell(goal))
    raw = [start] + [grid.point(c) for c in cells[1:-1]] + [goal]
    return np.array(_shortcut(raw, obstacles, clearance))


def plan_mission(course: Course, margin: float = MARGIN) -> list[dict]:
    """Every transit leg of the course's mission: home → pickup → drop → … → home."""
    m = course.mission
    if m is None:
        raise ValueError(f"course {course.name} has no mission")
    stops = [("home", m.home)]
    for k, (pick, drop) in enumerate(m.deliveries):
        stops += [(f"pickup_{k}", pick), (f"drop_{k}", drop)]
    stops.append(("home", m.home))
    obstacles = Obstacles.from_course(course, m.cruise)
    legs = []
    for (a_name, a), (b_name, b) in zip(stops, stops[1:]):
        wps = plan(course, a, b, m.cruise, margin)
        path_clear = min(obstacles.distance(np.linspace(p, q, 200)).min() for p, q in zip(wps, wps[1:]))
        legs.append({"from": a_name, "to": b_name, "waypoints": wps,
                     "length": float(np.linalg.norm(np.diff(wps, axis=0), axis=1).sum()),
                     "straight_length": float(np.linalg.norm(b - a)),
                     "straight_clearance": float(obstacles.distance(np.linspace(a, b, 400)).min()),
                     "clearance": float(path_clear)})
    return legs
