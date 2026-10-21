"""Synthetic unit tests for the persistent exploration grid."""

import math

from my_first_robot.exploration_grid import (
    ExplorationGrid, FREE, OCCUPIED, UNKNOWN, VISITED, evaluate_frontiers, in_portal,
    interior_region, move_target)

INF = float('inf')
BINS = 360


def box_profile(x, y, x0, x1, y0, y1, gap=None, range_max=3.5):
    """Robot-centred 1-degree profile inside an axis-aligned box room.
    gap=(bearing_from_deg, bearing_to_deg): no return (max-range) there."""
    profile = []
    for k in range(BINS):
        b = math.radians(k + 0.5)
        c, s = math.cos(b), math.sin(b)
        t = INF
        if c > 1e-9:
            t = min(t, (x1 - x) / c)
        if c < -1e-9:
            t = min(t, (x0 - x) / c)
        if s > 1e-9:
            t = min(t, (y1 - y) / s)
        if s < -1e-9:
            t = min(t, (y0 - y) / s)
        if gap and gap[0] <= k + 0.5 <= gap[1]:
            t = INF
        profile.append(t if t <= range_max else INF)
    rays = [10] * BINS
    maxrange = [10 if not math.isfinite(r) else 0 for r in profile]
    return profile, rays, maxrange


def single_ray(k, r):
    profile = [INF] * BINS
    rays = [0] * BINS
    maxrange = [0] * BINS
    profile[k] = r
    rays[k] = 10
    return profile, rays, maxrange


def test_empty_grid_has_nothing():
    g = ExplorationGrid()
    assert g.state((0, 0)) == UNKNOWN
    assert g.frontier_cells() == []
    ev = evaluate_frontiers(g, 0.1, 0.1, 0.0)
    assert ev['clusters'] == []


def test_ray_free_then_occupied_nothing_beyond():
    g = ExplorationGrid(0.2)
    g.integrate_profile(0.1, 0.1, *single_ray(0, 1.0), range_max=3.5)
    # bearing 0.5 deg from (0.1, 0.1): endpoint x = 1.1 -> cell 5
    assert g.state((5, 0)) == OCCUPIED
    for i in range(0, 5):
        assert g.state((i, 0)) == FREE
    for i in range(6, 20):
        assert g.state((i, 0)) == UNKNOWN


def test_no_return_free_only_to_range_max():
    g = ExplorationGrid(0.2)
    profile, rays, maxrange = single_ray(0, INF)
    maxrange[0] = 10
    g.integrate_profile(0.1, 0.1, profile, rays, maxrange, range_max=3.5)
    assert g.state((17, 0)) == FREE          # 3.5 m reaches x = 3.6 -> cell 17
    assert g.state((18, 0)) == UNKNOWN
    assert all(g.state((i, 0)) != OCCUPIED for i in range(18))


def test_bin_without_clean_max_range_adds_nothing():
    g = ExplorationGrid(0.2)
    profile, rays, maxrange = single_ray(0, INF)
    maxrange[0] = 2                          # 2 of 10 rays: unreliable
    g.integrate_profile(0.1, 0.1, profile, rays, maxrange, range_max=3.5)
    assert g.cells == {}


def test_occupied_survives_one_later_miss():
    g = ExplorationGrid(0.2)
    g.integrate_profile(0.1, 0.1, *single_ray(0, 1.0), range_max=3.5)
    profile, rays, maxrange = single_ray(0, INF)
    maxrange[0] = 10
    g.integrate_profile(0.1, 0.1, profile, rays, maxrange, range_max=3.5)
    assert g.state((5, 0)) == OCCUPIED       # 1 hit vs 1 miss -> still occupied
    g.integrate_profile(0.1, 0.1, profile, rays, maxrange, range_max=3.5)
    assert g.state((5, 0)) == FREE           # 1 hit vs 2 misses


def test_visited_distinct_from_free_and_sticky():
    g = ExplorationGrid(0.2, robot_radius=0.11)
    g.integrate_profile(1.0, 1.0, *box_profile(1.0, 1.0, 0.0, 3.0, 0.0, 2.0), range_max=3.5)
    assert g.state((5, 5)) == FREE
    g.mark_visited(1.0, 1.0)                 # on a cell corner -> 4 cells
    assert {c for c in g.cells if g.state(c) == VISITED} == {(4, 4), (4, 5), (5, 4), (5, 5)}
    assert g.state((7, 5)) == FREE
    g.integrate_profile(1.0, 1.0, *box_profile(1.0, 1.0, 0.0, 3.0, 0.0, 2.0), range_max=3.5)
    assert g.state((5, 5)) == VISITED
    counts = g.counts()
    assert counts['VISITED'] == 4 and counts['FREE'] > 0


def test_frontier_definition_and_clusters():
    g = ExplorationGrid(0.2)
    # Closed 3 x 2 m room with a 30-degree doorway toward +x (no return there).
    g.integrate_profile(1.0, 1.0, *box_profile(1.0, 1.0, 0.0, 3.0, 0.0, 2.0, gap=(-1, 15)),
                        range_max=3.5)
    g.integrate_profile(1.0, 1.0, *box_profile(1.0, 1.0, 0.0, 3.0, 0.0, 2.0, gap=(345, 361)),
                        range_max=3.5)
    g.mark_visited(1.0, 1.0)
    frontier = g.frontier_cells()
    assert frontier
    for i, j in frontier:
        assert g.state((i, j)) == FREE
        assert any(g.state((i + di, j + dj)) == UNKNOWN
                   for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)))
    # all frontier is beyond the east wall (x > 3.0)
    assert all(g.center(c)[0] > 3.0 for c in frontier)
    clusters = ExplorationGrid.cluster(frontier)
    assert len(clusters) == 1


def test_clustering_separate_and_diagonal():
    groups = ExplorationGrid.cluster([(0, 0), (1, 0), (5, 5), (6, 6), (20, 0)])
    assert sorted(len(c) for c in groups) == [1, 2, 2]


def test_exclusion_and_selection():
    g = ExplorationGrid(0.2)
    g.integrate_profile(1.0, 1.0, *box_profile(1.0, 1.0, -2.0, 4.0, 0.0, 2.0, gap=(60, 120)),
                        range_max=3.5)
    g.mark_visited(1.0, 1.0)
    ev = evaluate_frontiers(g, 1.0, 1.0, 0.0)
    assert [c['status'] for c in ev['clusters']][0] == 'candidate'
    assert ev['clusters'][0]['centroid'][1] > 2.0           # the north opening
    scores = [c['score'] for c in ev['clusters'] if c['status'] == 'candidate']
    assert scores == sorted(scores, reverse=True)
    ev2 = evaluate_frontiers(g, 1.0, 1.0, 0.0, excluded=lambda c: g.center(c)[1] > 2.0)
    assert all(c['status'] != 'candidate' or c['centroid'][1] <= 2.0 for c in ev2['clusters'])
    assert ev2['frontier_cells_excluded'] > 0


def test_no_frontier_when_everything_visited():
    g = ExplorationGrid(0.2)
    g.integrate_profile(1.0, 1.0, *box_profile(1.0, 1.0, 0.0, 3.0, 0.0, 2.0), range_max=3.5)
    assert g.frontier_cells() == []           # closed room: walls everywhere
    g2 = ExplorationGrid(0.2)
    g2.integrate_profile(1.0, 1.0, *box_profile(1.0, 1.0, 0.0, 3.0, 0.0, 2.0, gap=(-1, 15)),
                         range_max=3.5)
    assert g2.frontier_cells()
    for cell in list(g2.cells):
        if g2.state(cell) == FREE:
            g2.cells[cell][2] = True
    assert g2.frontier_cells() == []
    ev = evaluate_frontiers(g2, 1.0, 1.0, 0.0)
    assert not [c for c in ev['clusters'] if c['status'] == 'candidate']


def test_move_target_never_overshoots_aim():
    assert move_target(1.0, 2.0, 0.32, 0.10)[0] == 0.32 - 0.10
    assert move_target(1.0, 2.0, 0.32, 0.10)[1] == 'aim'
    assert move_target(1.0, 2.0, 4.0, 0.10) == (1.0, 'step')
    assert move_target(1.0, 0.4, 4.0, 0.10) == (0.4, 'free_travel')
    assert move_target(1.0, 2.0, 0.0, 0.10)[0] == 0.0


def test_portal_band():
    door = {'x': 0.0, 'y': 0.0, 'normal': math.pi / 2, 'width': 1.0}
    assert in_portal(door, 0.0, 0.0)
    assert in_portal(door, 0.75, 0.2)            # jamb margin
    assert not in_portal(door, 0.0, 0.5)         # deeper inside
    assert not in_portal(door, 0.0, -0.5)        # further outside
    assert not in_portal(door, 1.0, 0.0)         # along the wall, beyond the jamb margin


def test_interior_region_stops_at_portal():
    # Room 0..3 x 0..2 (inside), doorway in the south wall at x 1.2..1.8, yard below.
    g = ExplorationGrid(0.2)
    for i in range(-5, 20):
        for j in range(-8, 11):
            x, y = (i + 0.5) * 0.2, (j + 0.5) * 0.2
            wall = (abs(y) <= 0.1 and not 1.2 < x < 1.8) or x < 0.0 or x > 3.0 or y > 2.0
            g.cells[(i, j)] = [1, 0, False] if wall else [0, 1, False]
    door = {'x': 1.5, 'y': -0.1, 'normal': math.pi / 2, 'width': 0.6}

    def blocked(c):
        return in_portal(door, *g.center(c))

    region = interior_region(g, [g.cell_of(1.5, 1.0)], blocked)
    assert region and all(g.center(c)[1] > 0.0 for c in region)
    assert g.cell_of(1.5, -1.0) not in region
    assert g.cell_of(0.3, 1.9) in region
