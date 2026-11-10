"""The planner's paths keep the promised clearance and stay close to the shortest route."""

import numpy as np
import pytest

import course, planner

CLEAR = planner.DRONE_RADIUS + planner.MARGIN


@pytest.fixture(scope="module")
def city():
    return course.load("delivery_city")


def segment_clearance(obstacles, wps):
    return min(obstacles.distance(np.linspace(a, b, 400)).min() for a, b in zip(wps, wps[1:]))


def test_distance_to_a_rotated_building():
    obs = planner.Obstacles(np.array([[0.0, 0.0]]), np.array([[1.0, 0.5]]), np.radians([90.0]))
    # Rotated 90 degrees: the long side now runs along y.
    assert obs.distance(np.array([0.0, 0.0])) == 0.0
    assert obs.distance(np.array([0.0, 1.5])) == pytest.approx(0.5)
    assert obs.distance(np.array([1.5, 0.0])) == pytest.approx(1.0)


def test_mission_legs_keep_clearance(city):
    legs = planner.plan_mission(city)
    obstacles = planner.Obstacles.from_course(city, city.mission.cruise)
    assert len(legs) == 2 * len(city.mission.deliveries) + 1
    for leg in legs:
        assert segment_clearance(obstacles, leg["waypoints"]) >= CLEAR - 0.01, leg["from"] + "->" + leg["to"]
        # Detours stay short: no leg is more than 25% longer than the straight line.
        assert leg["length"] <= 1.25 * leg["straight_length"] + 1e-9


def test_course_actually_needs_the_planner(city):
    """Most straight-line legs of the original mission run through a building."""
    legs = planner.plan_mission(city)
    assert sum(leg["straight_clearance"] == 0.0 for leg in legs) >= 5


def test_clear_line_is_kept_straight(city):
    wps = planner.plan(city, [0.5, 0.5], [0.5, 2.0], city.mission.cruise)
    assert len(wps) == 2


def test_unreachable_goal_raises(city):
    with pytest.raises(ValueError):
        planner.plan(city, [0.5, 0.5], [5.8, 6.0], city.mission.cruise)  # inside a building
