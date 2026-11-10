"""Minimum-snap trajectories: pass through the route, start and end at rest, stay smooth, respect the
limits and keep clear of the buildings."""

import numpy as np
import pytest

import course, planner
import trajectory as tj

CURVE_CLEAR = planner.DRONE_RADIUS + tj.CURVE_MARGIN


@pytest.fixture(scope="module")
def city():
    return course.load("delivery_city")


def test_passes_through_waypoints_and_rests_at_the_ends():
    w = np.array([[0.0, 0.0], [1.0, 1.0], [2.5, 0.5], [3.0, -1.0]])
    tr = tj.min_snap(w, np.array([1.0, 1.5, 0.8]))
    knots = np.concatenate([[0.0], np.cumsum(tr.durations)])
    np.testing.assert_allclose(tr.evaluate(knots), w, atol=1e-9)
    for r in (1, 2, 3):
        np.testing.assert_allclose(tr.evaluate([0.0, tr.total], r), 0.0, atol=1e-9)


def test_smooth_through_interior_waypoints():
    """Velocity to snap are continuous across segment boundaries (the minimum-snap optimum is C6)."""
    w = np.array([[0.0, 0.0], [1.0, 2.0], [3.0, 2.0], [4.0, 0.0]])
    tr = tj.min_snap(w, np.array([1.2, 1.0, 1.4]))
    eps = 1e-7
    for knot in np.cumsum(tr.durations)[:-1]:
        for r in (1, 2, 3, 4):
            before, after = tr.evaluate(knot - eps, r), tr.evaluate(knot + eps, r)
            np.testing.assert_allclose(before, after, atol=1e-4 * max(1.0, np.abs(before).max()))


def test_straight_route_stays_on_the_line():
    tr = tj.min_snap(np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]), np.array([1.0, 1.0]))
    _, p, _, _ = tr.sample()
    assert np.abs(p[:, 1]).max() < 1e-9


def test_fit_respects_limits(city):
    obstacles = planner.Obstacles.from_course(city, city.mission.cruise)
    wps = planner.plan(city, city.mission.deliveries[1][0], city.mission.deliveries[1][1], city.mission.cruise)
    tr = tj.fit(wps, obstacles, CURVE_CLEAR, v_max=1.2, a_max=1.0)
    v, a = tr.peaks()
    assert v <= 1.2 + 1e-6 and a <= 1.0 + 1e-6
    assert max(v / 1.2, a / 1.0) > 0.99      # and is as fast as they allow


def test_mission_curves_keep_clearance(city):
    obstacles = planner.Obstacles.from_course(city, city.mission.cruise)
    for leg in planner.plan_mission(city):
        tr = tj.fit(leg["waypoints"], obstacles, CURVE_CLEAR)
        assert tr.clearance(obstacles).min() >= CURVE_CLEAR, leg["from"] + "->" + leg["to"]
        np.testing.assert_allclose(tr.evaluate([0.0, tr.total]), leg["waypoints"][[0, -1]], atol=1e-9)


def test_subdivide_keeps_points_on_the_route():
    w = tj.subdivide(np.array([[0.0, 0.0], [3.0, 0.0], [3.0, 0.5]]), max_segment=1.0)
    np.testing.assert_allclose(w, [[0, 0], [1, 0], [2, 0], [3, 0], [3, 0.5]])
