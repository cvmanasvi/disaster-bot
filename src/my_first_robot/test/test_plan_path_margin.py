"""Grid planner corridor margin vs the live straight-path check (wall end on the 0.20 m edge)."""

import math
import random

from my_first_robot.autonomous_search import AutonomousSearch, straight_path_blocker

import pytest

import rclpy

# Wall end (Wall_99 north end, recorded geometry): end face 0.15 m long, starting 0.33 m ahead,
# 0.205 m to the left of the planned line (recorded truth 0.201-0.203) -- inside 0.20 + 0.04,
# outside 0.20.
WALL_END = [(0.33 + 0.005 * i, 0.205) for i in range(31)]


@pytest.fixture(scope='module')
def node():
    rclpy.init()
    n = AutonomousSearch()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_defaults(node):
    assert node.p.explore_path_half_width_m == pytest.approx(0.20)
    assert node.p.grid_plan_path_margin_m == pytest.approx(0.04)


def test_free_travel_default_half_width_unchanged(node):
    # Default call (sector planner, escape moves) still uses explore_path_half_width_m.
    assert node._explore_free_travel(WALL_END, 0.0) == pytest.approx(1.0)


def test_planner_corridor_rejects_wall_end_inside_margin(node):
    half = node.p.explore_path_half_width_m + node.p.grid_plan_path_margin_m
    assert node._explore_free_travel(WALL_END, 0.0, half) == pytest.approx(0.0)
    # Turning 10 deg away from the wall end clears the widened corridor.
    assert node._explore_free_travel(WALL_END, math.radians(-10.0), half) == pytest.approx(1.0)


def test_live_check_on_noisy_scans_matches_planner():
    """Live check (unchanged 0.20 m) on noisy rays: the wall end at 0.205 m trips it, the
    10-deg-away heading (wall end >= 0.25 m lateral) never does."""
    rng = random.Random(3)

    def noisy(points, heading):
        c, s = math.cos(heading), math.sin(heading)
        out = []
        for x, y in points:
            r, a = math.hypot(x, y) + rng.gauss(0.0, 0.01), math.atan2(y, x)
            px, py = r * math.cos(a), r * math.sin(a)
            out.append((px * c + py * s, -px * s + py * c))
        return out

    def trips(heading):
        return straight_path_blocker(noisy(WALL_END, heading), 1.0, 0.20, 0.40) is not None

    tripped_on_line = sum(trips(0.0) for _ in range(50))
    tripped_away = sum(trips(math.radians(-10.0)) for _ in range(50))
    assert tripped_on_line > 25
    assert tripped_away == 0
