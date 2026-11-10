#!/usr/bin/env python3
"""How close did a flown trajectory get to the course's buildings?

    python3 path_planning/check_clearance.py delivery_city pose.csv      # CSV with columns t,x,y,z

Reports the closest horizontal distance between the drone's centre and any building wall while the
drone was below that building's roof, and whether the props (DRONE_RADIUS) ever touched one.
"""

import argparse
import csv

import numpy as np

import course, planner


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("course")
    ap.add_argument("csv")
    args = ap.parse_args()
    c = course.load(args.course)
    rows = list(csv.DictReader(open(args.csv)))
    t, p = np.array([float(r["t"]) for r in rows]), np.array([[float(r[k]) for k in "xyz"] for r in rows])
    worst, when, which = np.inf, None, None
    for i, b in enumerate(c.buildings):
        obs = planner.Obstacles(b.xyz[None, :2], b.size[None, :2] / 2, np.radians([b.yaw]))
        below = p[:, 2] < b.size[2]
        if not below.any():
            continue
        d = obs.distance(p[below, :2])
        k = int(np.argmin(d))
        if d[k] < worst:
            worst, when, which = float(d[k]), float(t[below][k]), i
    print(f"{len(t)} samples over {t[-1] - t[0]:.0f} s")
    print(f"closest approach: {worst:.2f} m from building {which} at t = {when:.1f} s "
          f"(planned clearance {planner.DRONE_RADIUS + planner.MARGIN:.2f} m, drone radius {planner.DRONE_RADIUS} m)")
    print("COLLISION" if worst < planner.DRONE_RADIUS else "no contact")


if __name__ == "__main__":
    main()
