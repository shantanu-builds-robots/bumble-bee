#!/usr/bin/env python3
"""Draw a flown trajectory over a course's planned mission (top-down).

    python3 path_planning/plot_flight.py delivery_city pose.csv -o flight.png      # CSV columns t,x,y,z
"""

import argparse
import csv
from pathlib import Path

import numpy as np
from PIL import ImageDraw, Image

import course, planner

import plan_mission  # same folder: reuse the mission picture as the background


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("course")
    ap.add_argument("csv")
    ap.add_argument("-o", "--out", type=Path, default=Path("flight.png"))
    args = ap.parse_args()
    c = course.load(args.course)
    legs = planner.plan_mission(c)
    plan_mission.draw(c, legs, args.out)

    # Same frame as plan_mission.draw: recompute its bounds to place the flown path.
    obstacles = planner.Obstacles.from_course(c, c.mission.cruise)
    pts = np.vstack([c.mission.home] + [p for d in c.mission.deliveries for p in d] + list(obstacles.centres))
    lo, hi = pts.min(axis=0) - 1.5, pts.max(axis=0) + 1.5
    to_px = lambda p: ((p[0] - lo[0]) * plan_mission.PX, (hi[1] - p[1]) * plan_mission.PX)  # noqa: E731

    rows = list(csv.DictReader(open(args.csv)))
    xy = np.array([[float(r["x"]), float(r["y"])] for r in rows])
    img = Image.open(args.out)
    d = ImageDraw.Draw(img)
    d.line([to_px(p) for p in xy], fill=(30, 110, 60), width=2)
    d.rectangle([0, 0, img.width, 28], fill=(236, 238, 232))
    d.text((10, 10), f"{c.name}: flown in Gazebo (green) over the planned route (yellow); "
                     "red dashes = straight lines through buildings", fill=(12, 26, 14))
    img.save(args.out)
    print(args.out)


if __name__ == "__main__":
    main()
