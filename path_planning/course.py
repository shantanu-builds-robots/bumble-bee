"""Course description: one YAML file per course, with the buildings, pads and delivery mission the planner reads.

Coordinates are metres in the world frame (z up); yaw is in degrees about z.

```yaml
name: race_gates
description: Six gates on a figure-eight.
spawn: {xyz: [0.5, 0.5, 0.1], yaw: 0}
pads:      [{xyz: [0.5, 0.5]}]                       # 1 x 1 x 0.1 m landing pads
parcels:   [{model: box_0, xyz: [0.5, 2.0]}]         # 12 cm, 0.1 kg boxes from bumble-bee's models
buildings: [{xyz: [3, 4], size: [2, 2, 6], yaw: 0}]  # boxes standing on the ground
gates:     [{xyz: [4, 0, 1.5], yaw: 90, opening: [0.8, 0.8], frame: 0.1, depth: 0.05}]
mission:                                             # optional: what bumble-bee's deliver node flies
  home: [0.5, 0.5]
  cruise: 0.7                                        # transit height [m]
  deliveries: [{pickup: [0.5, 2.0], drop: [0.5, 9.5]}]
```

A gate's opening is centred on ``xyz``; the drone passes through it along the gate's local x axis
(yaw 0 means flying in +x). Gates are listed in race order.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

COURSE_DIR = Path(__file__).resolve().parent / "courses"
PAD_SIZE = (1.0, 1.0, 0.1)
PARCEL_SIZE = 0.12
PARCEL_MASS = 0.1


@dataclass(frozen=True)
class Gate:
    xyz: np.ndarray
    yaw: float                 # degrees
    opening: tuple[float, float]  # inner width (local y), inner height (z) [m]
    frame: float = 0.1         # frame bar thickness [m]
    depth: float = 0.05        # frame depth along the flight direction [m]

    @property
    def normal(self) -> np.ndarray:
        """Unit vector of the flight direction through the gate."""
        a = np.radians(self.yaw)
        return np.array([np.cos(a), np.sin(a), 0.0])

    def bars(self) -> list[tuple[np.ndarray, np.ndarray]]:
        """The four frame bars as (centre in gate frame, half sizes)."""
        w, h = self.opening
        f, d = self.frame, self.depth
        return [
            (np.array([0.0, 0.0, h / 2 + f / 2]), np.array([d / 2, w / 2 + f, f / 2])),   # top
            (np.array([0.0, 0.0, -h / 2 - f / 2]), np.array([d / 2, w / 2 + f, f / 2])),  # bottom
            (np.array([0.0, w / 2 + f / 2, 0.0]), np.array([d / 2, f / 2, h / 2])),       # left
            (np.array([0.0, -w / 2 - f / 2, 0.0]), np.array([d / 2, f / 2, h / 2])),      # right
        ]


@dataclass(frozen=True)
class Building:
    xyz: np.ndarray            # centre of the footprint on the ground
    size: np.ndarray           # x, y, height [m]
    yaw: float = 0.0


@dataclass(frozen=True)
class Mission:
    home: np.ndarray                                    # x, y of the home pad centre
    cruise: float                                       # transit height [m]
    deliveries: tuple[tuple[np.ndarray, np.ndarray], ...]  # (pickup x/y, drop x/y), in order


@dataclass(frozen=True)
class Course:
    name: str
    description: str
    spawn_xyz: np.ndarray
    spawn_yaw: float
    pads: tuple[np.ndarray, ...] = ()
    parcels: tuple[tuple[str, np.ndarray], ...] = ()
    buildings: tuple[Building, ...] = ()
    gates: tuple[Gate, ...] = ()
    mission: Mission | None = None
    extra: dict = field(default_factory=dict)


def _xyz(v, z=0.0) -> np.ndarray:
    v = [float(x) for x in v]
    return np.array(v if len(v) == 3 else v + [z])


def _mission(m: dict) -> Mission:
    xy = lambda v: np.array([float(v[0]), float(v[1])])  # noqa: E731
    return Mission(home=xy(m["home"]), cruise=float(m.get("cruise", 0.7)),
                   deliveries=tuple((xy(d["pickup"]), xy(d["drop"])) for d in m["deliveries"]))


def load(name_or_path: str | Path) -> Course:
    """Load a course by name (``courses/<name>.yaml``) or path."""
    path = Path(name_or_path)
    if not path.suffix:
        path = COURSE_DIR / f"{name_or_path}.yaml"
    d = yaml.safe_load(path.read_text())
    spawn = d.get("spawn", {})
    return Course(
        name=d["name"],
        description=d.get("description", ""),
        spawn_xyz=_xyz(spawn.get("xyz", [0, 0, 0.1])),
        spawn_yaw=float(spawn.get("yaw", 0.0)),
        pads=tuple(_xyz(p["xyz"]) for p in d.get("pads", [])),
        parcels=tuple((p["model"], _xyz(p["xyz"])) for p in d.get("parcels", [])),
        buildings=tuple(Building(_xyz(b["xyz"]), np.array(b["size"], float), float(b.get("yaw", 0)))
                        for b in d.get("buildings", [])),
        gates=tuple(Gate(_xyz(g["xyz"]), float(g.get("yaw", 0)), tuple(g.get("opening", [0.8, 0.8])),
                         float(g.get("frame", 0.1)), float(g.get("depth", 0.05)))
                    for g in d.get("gates", [])),
        mission=_mission(d["mission"]) if "mission" in d else None,
        extra={k: v for k, v in d.items()
               if k not in {"name", "description", "spawn", "pads", "parcels", "buildings", "gates", "mission"}},
    )
