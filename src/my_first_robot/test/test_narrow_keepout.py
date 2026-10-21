"""
Narrow-obstacle keep-out, its release on contradicting evidence and the IMU tilt guard.

Synthetic geometry plus two recorded Gazebo fixtures (test/data): the run-2 west-wing door jamb
falsely kept as a narrow post (wallend_mismatch_20261004) and the cafe_table_0 base-plate
collision (full_house_exploration_20261003_121952).
"""

import json
import math
import os

from builtin_interfaces.msg import Time
from my_first_robot.autonomous_search import (
    AutonomousSearch, contiguous_surface_length, detect_narrow_obstacles, narrow_keepout_travel,
    Pose2D, State, tilt_exceeded)
from my_first_robot.exploration_grid import ExplorationGrid
import pytest
import rclpy
from sensor_msgs.msg import Imu, LaserScan

KEEPOUT = 0.50 + 0.13          # keep-out to swept body + swept radius (robot centre)


def ray_box(ox, oy, a, x0, x1, y0, y1):
    dx, dy = math.cos(a), math.sin(a)
    tmin, tmax = -math.inf, math.inf
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return math.inf
            continue
        t0, t1 = (lo - o) / d, (hi - o) / d
        tmin, tmax = max(tmin, min(t0, t1)), min(tmax, max(t0, t1))
    return tmin if tmax >= tmin > 0 else math.inf


def sweep(boxes, room=(-2.0, 2.5, -2.0, 2.0), n=360):
    """Robot-centred 1-degree sweep inside a room with extra boxes."""
    out = []
    for k in range(n):
        a = math.radians(k + 0.5)
        x0, x1, y0, y1 = room
        dx, dy = math.cos(a), math.sin(a)
        t = min([(x1 / dx) if dx > 1e-9 else math.inf, (x0 / dx) if dx < -1e-9 else math.inf,
                 (y1 / dy) if dy > 1e-9 else math.inf, (y0 / dy) if dy < -1e-9 else math.inf])
        for b in boxes:
            t = min(t, ray_box(0.0, 0.0, a, *b))
        out.append((a, t if t <= 3.5 else math.inf))
    return out


def column(x, y, half=0.021):
    return (x - half, x + half, y - half, y + half)


def free_travel(polar, heading, half_width=0.20, margin=0.40, step=1.0):
    """Same straight-path test as AutonomousSearch._explore_free_travel."""
    c, s = math.cos(heading), math.sin(heading)
    best = step
    for a, r in polar:
        if not math.isfinite(r):
            continue
        x, y = r * math.cos(a), r * math.sin(a)
        ahead, side = x * c + y * s, -x * s + y * c
        if ahead > 0.0 and abs(side) < half_width:
            best = min(best, ahead - margin)
    return max(0.0, best)


# ---------------------------------------------------------------- A
def test_A_narrow_isolated_obstacle_rejects_previously_clear_path():
    polar = sweep([column(0.75, 0.30)])
    obs = detect_narrow_obstacles(polar)
    assert len(obs) == 1 and obs[0]['width'] <= 0.15
    assert abs(obs[0]['x'] - 0.73) < 0.05 and abs(obs[0]['y'] - 0.30) < 0.05
    assert free_travel(polar, 0.0) == pytest.approx(1.0)      # old check: clear
    travel, o = narrow_keepout_travel(obs, 0.0, 1.0, KEEPOUT)
    assert o is obs[0] and travel < 0.30                      # rejected (< grid_min_step_m)


# ---------------------------------------------------------------- B
def test_B_wider_obstacle_left_to_normal_handling():
    polar = sweep([(0.55, 0.95, 0.10, 0.50)])                 # 0.40 m wide box
    assert detect_narrow_obstacles(polar) == []
    assert narrow_keepout_travel([], 0.0, 1.0, KEEPOUT) == (1.0, None)
    polar = sweep([(0.70, 0.90, -0.10, 0.10)])                # dead ahead: free travel stops it
    assert detect_narrow_obstacles(polar) == []
    assert free_travel(polar, 0.0) < 0.35


def test_B_partly_hidden_cluster_not_narrow():
    # A thin post right in front of a wider box: the post's flank is the
    # nearer box -> not an isolated narrow obstacle.
    polar = sweep([column(1.20, 0.0), (0.80, 1.00, -0.60, -0.02)])
    assert all(abs(o['y']) > 0.05 or o['x'] < 1.0 for o in detect_narrow_obstacles(polar))


# ---------------------------------------------------------------- D
def test_D_far_narrow_obstacle_path_allowed():
    polar = sweep([column(0.50, 1.20)])
    obs = detect_narrow_obstacles(polar)
    assert len(obs) == 1
    assert narrow_keepout_travel(obs, 0.0, 1.0, KEEPOUT) == (1.0, None)


def test_keepout_allows_moving_away_from_inside():
    obs = [{'x': 0.30, 'y': 0.0, 'range': 0.30, 'bearing': 0.0, 'width': 0.04, 'rays': 5}]
    assert narrow_keepout_travel(obs, math.pi, 0.5, KEEPOUT)[0] == pytest.approx(0.5)
    assert narrow_keepout_travel(obs, 0.0, 0.5, KEEPOUT)[0] == 0.0


def test_keepout_tolerance_band_just_inside():
    # 0.61 m from a post (inside 0.63 by jitter): a tangential move may pass
    # as long as the centre stays >= 0.58 m; it may not dive in further.
    obs = [{'x': 0.0, 'y': 0.61, 'range': 0.61, 'bearing': math.pi / 2, 'width': 0.04, 'rays': 5}]
    assert narrow_keepout_travel(obs, 0.0, 1.0, KEEPOUT)[0] == pytest.approx(1.0)
    travel, o = narrow_keepout_travel(obs, math.radians(60), 1.0, KEEPOUT)
    assert o is obs[0] and travel < 0.1


def test_detection_range_limit():
    polar = sweep([column(2.3, 0.0)], room=(-3.0, 3.4, -3.0, 3.0))
    assert detect_narrow_obstacles(polar, max_range_m=2.0) == []
    assert len(detect_narrow_obstacles(polar, max_range_m=3.0)) == 1


# ---------------------------------------------------------------- E (pure)
@pytest.mark.parametrize('roll,pitch,stop', [
    (0.0, 0.0, False), (0.0, 9.0, False), (0.0, 10.0, True), (0.0, 12.0, True),
    (0.0, -10.5, True), (11.0, 0.0, True), (-15.0, 0.0, True), (9.9, -9.9, False),
])
def test_E_tilt_threshold(roll, pitch, stop):
    assert tilt_exceeded(roll, pitch, 10.0) is stop


# ---------------------------------------------------------------- node level
@pytest.fixture(scope='module')
def node():
    rclpy.init()
    n = AutonomousSearch()
    n.p.scan_record_dir = ''
    yield n
    n.destroy_node()
    rclpy.shutdown()


def laser(boxes, yaw=0.0):
    """Raw LaserScan (360 rays, LiDAR at x=-0.032) of the synthetic room, robot at origin."""
    m = LaserScan()
    m.header.stamp = Time(sec=5, nanosec=0)
    m.angle_min, m.angle_increment = 0.0, 2 * math.pi / 360
    m.angle_max = m.angle_min + 359 * m.angle_increment
    m.range_min, m.range_max = 0.12, 3.5
    lx = -0.032
    ranges = []
    for i in range(360):
        a = i * m.angle_increment
        x0, x1, y0, y1 = (-2.0, 2.5, -2.0, 2.0)
        dx, dy = math.cos(a), math.sin(a)
        t = min([((x1 - lx) / dx) if dx > 1e-9 else math.inf,
                 ((x0 - lx) / dx) if dx < -1e-9 else math.inf,
                 (y1 / dy) if dy > 1e-9 else math.inf, (y0 / dy) if dy < -1e-9 else math.inf])
        for b in boxes:
            t = min(t, ray_box(lx, 0.0, a, *b))
        ranges.append(float(t) if t <= 3.5 else math.inf)
    m.ranges = ranges
    return m


def prime(node, boxes):
    node.latest_scan = laser(boxes)
    node.latest_scan_stamp = node.latest_scan.header.stamp
    node.nav_pose = Pose2D(0.0, 0.0, 0.0)
    node.imu_yaw = 0.0
    node.keepout_escapes = 0
    node.state = State.STOP_AND_SCAN


def test_C_rotation_refused_inside_keepout_escapes_away(node):
    prime(node, [column(0.45, 0.10)])
    assert node._keepout_before_rotation(1.0) is False
    assert node.state == State.EXPLORE_TURN
    plan = node.explore_plan
    assert plan['sector'] == 'ESCAPE'
    away = math.degrees(plan['heading'])
    assert abs(abs(away) - 180.0) < 25.0                      # away from the post
    assert 0.15 < plan['target'] < 0.35                       # 0.63 - 0.45 + 0.05


def test_C_rotation_allowed_outside_keepout(node):
    prime(node, [column(0.80, 0.30)])
    assert node._keepout_before_rotation(1.0) is True


def test_C_rotation_hold_after_escape_attempts(node):
    prime(node, [column(0.45, 0.10)])
    node.keepout_escapes = node.p.narrow_obstacle_max_escapes
    assert node._keepout_before_rotation(1.0) is False
    assert node.state == State.INSIDE_HOLD


def imu(roll_deg, pitch_deg):
    m = Imu()
    m.header.stamp = Time(sec=6, nanosec=0)
    r, p = math.radians(roll_deg) / 2, math.radians(pitch_deg) / 2
    m.orientation.w = math.cos(r) * math.cos(p)
    m.orientation.x = math.sin(r) * math.cos(p)
    m.orientation.y = math.cos(r) * math.sin(p)
    m.orientation.z = -math.sin(r) * math.sin(p)
    return m


@pytest.mark.parametrize('roll,pitch,stop', [
    (0.0, 0.0, False), (0.0, 9.0, False), (0.0, 10.5, True), (0.0, -12.0, True), (11.0, 0.0, True),
])
def test_E_tilt_guard_node(node, roll, pitch, stop):
    sent = []
    node.cmd_publisher.publish = lambda msg: sent.append((msg.twist.linear.x, msg.twist.angular.z))
    node.tilt_latched = None
    node.state = State.EXPLORE_MOVE
    node.imu_callback(imu(roll, pitch))
    assert abs(math.degrees(node.imu_pitch) - pitch) < 0.01
    assert abs(math.degrees(node.imu_roll) - roll) < 0.01
    assert (node.tilt_latched is not None) is stop
    if stop:
        assert sent and sent[-1] == (0.0, 0.0)
        node.control_loop()
        assert node.state == State.FAULT_HOLD and node.fault_terminal
        assert sent[-1] == (0.0, 0.0)
    node.tilt_latched = None
    node.fault_terminal = False


# ================================================ release on contradicting evidence
DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
ROOM = (-2.0, 2.5, -2.0, 2.0)
# Wall end: a 0.15 m thick wall whose end face (x = 0.40, y 0.50..0.65) is 0.15 m wide -- exactly
# what a door jamb looks like edge-on. The remembered "post" centre is the end-face centre pushed
# back by half its chord (as detect_narrow_obstacles reports it).
WALL_END = (0.40, 1.40, 0.50, 0.65)
JAMB = (0.475, 0.575, 0.149)
# Cabinet-like panel standing on the wall's far side (forms an L with the wall end).
PANEL = (0.20, 0.22, 0.65, 1.10)


def view(boxes, vx, vy):
    """Return the merged 1-degree profile (robot-centred, world-aligned) seen from (vx, vy)."""
    room = (ROOM[0] - vx, ROOM[1] - vx, ROOM[2] - vy, ROOM[3] - vy)
    rel = [(x0 - vx, x1 - vx, y0 - vy, y1 - vy) for x0, x1, y0, y1 in boxes]
    return [r for _, r in sweep(rel, room=room)]


def polar_of(profile):
    return [(math.radians(k + 0.5), r) for k, r in enumerate(profile)]


def reset_release(node, memory):
    node.narrow_memory = [{'x': x, 'y': y, 'width': w} for x, y, w in memory]
    node.narrow_released = []
    node.narrow_plan = []
    node.scan_count = 100


def release_scan(node, boxes, vx, vy, fresh=()):
    """One _narrow_release_check from (vx, vy); fresh = this scan's narrow detections (nav)."""
    node.scan_count += 1
    node.narrow_plan = [{'x': x - vx, 'y': y - vy, 'range': math.hypot(x - vx, y - vy),
                         'bearing': math.atan2(y - vy, x - vx), 'width': 0.04, 'rays': 5}
                        for x, y in fresh]
    node._narrow_release_check(Pose2D(vx, vy, 0.0), view(boxes, vx, vy))


# ---------------------------------------------------------------- A / B (pure)
@pytest.mark.parametrize('boxes,vx,vy', [
    ([WALL_END], 0.0, 0.0), ([WALL_END], 0.0, -0.3), ([WALL_END, PANEL], 0.6, 1.4),
    ([WALL_END, PANEL], -0.3, 0.3),
])
def test_A_wall_end_contour_is_contradicting_evidence(boxes, vx, vy):
    polar = polar_of(view(boxes, vx, vy))
    length = contiguous_surface_length(polar, JAMB[0] - vx, JAMB[1] - vy)
    assert length > 0.30


@pytest.mark.parametrize('vx,vy', [(0.0, 0.0), (0.0, 0.6), (0.6, -0.5), (1.5, 0.3)])
def test_B_isolated_post_is_not_contradicting_evidence(vx, vy):
    post = column(0.75, 0.30)
    polar = polar_of(view([post], vx, vy))
    assert len(detect_narrow_obstacles(polar)) == 1
    assert contiguous_surface_length(polar, 0.75 - vx, 0.30 - vy) < 0.10


def test_B_surface_must_pass_near_the_point():
    # A long wall 0.30 m from the point is no evidence about the point.
    polar = polar_of(view([(1.05, 1.20, -1.5, 1.5)], 0.0, 0.0))
    assert contiguous_surface_length(polar, 0.75, 0.0) == 0.0
    assert contiguous_surface_length(polar, 1.00, 0.0) > 0.30


# ---------------------------------------------------------------- node level C / D / E / F / G
def test_release_defaults(node):
    assert node.p.narrow_release_range_m == pytest.approx(3.0)
    assert node.p.narrow_release_min_width_m == pytest.approx(0.06)
    assert node.p.narrow_release_min_surface_m == pytest.approx(0.30)
    assert node.p.narrow_release_contradictions == 2
    assert node.p.narrow_release_suppress_radius_m == pytest.approx(0.20)


def test_C_one_contradiction_does_not_release(node):
    reset_release(node, [JAMB])
    release_scan(node, [WALL_END], 0.0, 0.0)
    assert len(node.narrow_memory) == 1 and node.narrow_memory[0]['contradictions'] == 1
    assert node.narrow_released == []
    # Same view point again (no-move rescan): not an independent observation.
    release_scan(node, [WALL_END], 0.03, 0.0)
    assert len(node.narrow_memory) == 1 and node.narrow_memory[0]['contradictions'] == 1
    assert node.narrow_released == []


def test_D_two_contradictions_release(node):
    reset_release(node, [JAMB, (-1.0, -1.0, 0.04)])
    before = node.keepout_stats['released']
    release_scan(node, [WALL_END], 0.0, 0.0)
    release_scan(node, [WALL_END], 0.0, -0.3)
    assert [(m['x'], m['y']) for m in node.narrow_memory] == [(-1.0, -1.0)]
    assert [(r['x'], r['y']) for r in node.narrow_released] == [(JAMB[0], JAMB[1])]
    assert node.keepout_stats['released'] == before + 1


def test_D_redetection_as_isolated_resets_count(node):
    reset_release(node, [JAMB])
    release_scan(node, [WALL_END], 0.0, 0.0)
    release_scan(node, [WALL_END], 0.0, -0.3, fresh=[(JAMB[0], JAMB[1])])
    assert node.narrow_memory[0]['contradictions'] == 0
    release_scan(node, [WALL_END], 0.3, -0.3)
    assert node.narrow_released == [] and node.narrow_memory[0]['contradictions'] == 1


def test_D_out_of_range_view_ignored(node):
    reset_release(node, [JAMB])
    far = (JAMB[0] - 3.2, JAMB[1])
    node.scan_count += 1
    node.narrow_plan = []
    node._narrow_release_check(Pose2D(far[0], far[1], 0.0), view([WALL_END], 0.0, 0.0))
    assert node.narrow_memory[0].get('contradictions', 0) == 0


@pytest.mark.parametrize('width', [0.025, 0.042, 0.055, 0.06])
def test_E_thin_object_never_released(node, width):
    reset_release(node, [(JAMB[0], JAMB[1], width)])
    for vx, vy in ((0.0, 0.0), (0.0, -0.3), (0.3, -0.3), (-0.3, 0.0)):
        release_scan(node, [WALL_END], vx, vy)
    assert len(node.narrow_memory) == 1 and node.narrow_released == []
    assert node.narrow_memory[0].get('contradictions', 0) == 0


def test_F_released_position_not_remembered_again(node):
    reset_release(node, [JAMB])
    release_scan(node, [WALL_END], 0.0, 0.0)
    release_scan(node, [WALL_END], 0.0, -0.3)
    assert node.narrow_memory == []
    before = node.keepout_stats['suppressed_released']
    node._narrow_remember(JAMB[0] + 0.05, JAMB[1] - 0.12, 0.149)        # 0.13 m away
    node._narrow_remember(JAMB[0], JAMB[1] + 0.19, 0.120)               # 0.19 m away
    assert node.narrow_memory == []
    assert node.keepout_stats['suppressed_released'] == before + 2
    node._narrow_remember(JAMB[0] + 0.30, JAMB[1] - 0.30, 0.04)         # a different object
    assert len(node.narrow_memory) == 1


@pytest.mark.parametrize('half', [0.02, 0.07])
def test_G_post_in_front_of_wall_not_released(node, half):
    # Post 0.30 m in front of a long wall (wall face x = 1.20, post centre x = 0.90).
    wall = (1.20, 1.35, -1.5, 1.5)
    post = column(0.90, 0.0, half=half)
    views = ((0.0, 0.0), (0.0, 0.6), (0.0, -0.6), (0.4, 0.8), (0.4, -0.8), (-0.5, 0.0))
    for vx, vy in views:
        polar = polar_of(view([wall, post], vx, vy))
        assert contiguous_surface_length(polar, 0.90 - vx, 0.0 - vy) <= 0.30
    reset_release(node, [(0.90, 0.0, 2 * half + 0.004)])
    for vx, vy in views:
        release_scan(node, [wall, post], vx, vy)           # no fresh re-detection passed
    assert len(node.narrow_memory) == 1 and node.narrow_released == []
    assert node.narrow_memory[0].get('contradictions', 0) == 0


# ------------------------------------------------ H / I: existing keep-out unchanged
def test_H_keepout_radius_and_reachability_raster_unchanged(node):
    assert node.p.narrow_obstacle_keepout_m == pytest.approx(0.50)
    assert node.p.robot_swept_radius_m == pytest.approx(0.13)
    assert node._keepout_radius() == pytest.approx(KEEPOUT)
    assert node.p.narrow_obstacle_width_threshold_m == pytest.approx(0.15)
    assert node.p.narrow_obstacle_background_m == pytest.approx(0.20)
    assert node.p.narrow_obstacle_structure_cells == 5
    assert node.p.narrow_obstacle_tolerance_m == pytest.approx(0.05)
    node.grid = ExplorationGrid(node.p.grid_resolution_m, node.p.grid_robot_radius_m)
    node.narrow_memory = [{'x': 0.31, 'y': -0.47, 'width': 0.04}]
    node._grid_update_keepout_cells()
    res = node.grid.res
    want = {(i, j) for i in range(-10, 11) for j in range(-10, 11)
            if math.hypot((i + 0.5) * res - 0.31, (j + 0.5) * res + 0.47) < KEEPOUT + 0.10}
    assert node.grid_keepout_cells == want
    node.grid = None
    node.narrow_memory = []


def test_H_remembered_post_keeps_full_keepout_until_released(node):
    reset_release(node, [JAMB])
    release_scan(node, [WALL_END], 0.0, 0.0)                    # contradiction #1 only
    node.grid = ExplorationGrid(node.p.grid_resolution_m, node.p.grid_robot_radius_m)
    node._grid_update_keepout_cells()
    cells = set(node.grid_keepout_cells)
    assert cells and all(
        math.hypot(node.grid.center(c)[0] - JAMB[0], node.grid.center(c)[1] - JAMB[1])
        < KEEPOUT + 0.10
        for c in cells)
    release_scan(node, [WALL_END], 0.0, -0.3)                   # contradiction #2 -> released
    node._grid_update_keepout_cells()
    assert node.grid_keepout_cells == set()
    node.grid = None


def test_I_live_and_rotation_checks_still_act_at_released_position(node):
    # Release does not filter detections: a post that really stands at a released position
    # is still cut by the live check and still refuses the pre-rotation check.
    reset_release(node, [])
    node.narrow_released = [{'x': 0.45, 'y': 0.10, 'width': 0.149, 'scan': 1, 'suppressed': 0}]
    prime(node, [column(0.45, 0.10)])
    live = node._narrow_from_live_scan()
    assert len(live) == 1
    travel, o = narrow_keepout_travel(live, 0.0, 1.0, node._keepout_radius(),
                                      tolerance_m=node.p.narrow_obstacle_tolerance_m)
    assert o is live[0] and travel < 0.05
    assert node._keepout_before_rotation(1.0) is False
    assert node.state == State.EXPLORE_TURN and node.explore_plan['sector'] == 'ESCAPE'
    prime(node, [column(0.80, 0.30)])
    assert node._keepout_before_rotation(1.0) is True
    node.narrow_released = []


# ---------------------------------------------------------------- recorded fixtures
def grid_node_reset(node, entrance):
    node.grid = ExplorationGrid(node.p.grid_resolution_m, node.p.grid_robot_radius_m)
    node.entrance = {'x': entrance['x'], 'y': entrance['y'],
                     'normal': math.radians(entrance['normal_deg']), 'width': entrance['width']}
    node.grid_interior_seed = None
    node.grid_no_move_count = 0
    node.narrow_memory = []
    node.narrow_released = []
    node.narrow_plan = []
    node.keepout_escapes = 0


def fixture_update(node, scan, range_max=3.5):
    prof = [math.inf if v is None else v for v in scan['profile']]
    data = {'analysis': {'profile': prof, 'rays_per_bin': scan['rays_per_bin'],
                         'maxrange_per_bin': scan['maxrange_per_bin']},
            'samples': [(0, 0, 0, 0, 0, range_max)]}
    node.scan_count = scan['scan']
    yaw = math.radians(scan['yaw_deg']) if 'yaw_deg' in scan else scan['yaw']
    pose = Pose2D(scan['nav'][0], scan['nav'][1], yaw)
    node._grid_update(pose, data)
    return pose, data


def west_wing(node):
    """Return the south west-wing frontier cluster (nav: x < -3.23, y < 1.35, >= 3 cells)."""
    return [c for c in node.grid_last_eval['clusters']
            if c['centroid'][0] < -3.23 and c['centroid'][1] < 1.35 and c['size'] >= 3]


def near(m, x, y, d=0.3):
    return math.hypot(m['x'] - x, m['y'] - y) < d


def test_J_run2_jamb_released_and_west_wing_reachable(node):
    fx = json.load(open(os.path.join(DATA, 'run2_jamb_release.json')))
    grid_node_reset(node, fx['entrance'])
    for i, j, h, m, v in fx['grid_after_scan33']:
        node.grid.cells[(i, j)] = [h, m, bool(v)]
    node.grid_interior_seed = tuple(fx['seed_cell'])
    node.narrow_memory = [{'x': x, 'y': y, 'width': w} for x, y, w in fx['memory_before_scan34']]
    jx, jy = -4.885, 1.373                                    # Wall_98 west jamb end (nav)
    others = [(m['x'], m['y']) for m in node.narrow_memory if not near(m, jx, jy)]
    assert len(others) == 11
    status, prev = {}, None
    for scan in fx['scans']:
        if prev is not None:
            node.grid.mark_path_visited(prev[0], prev[1], *scan['start_nav'])
        fixture_update(node, scan, scan['range_max'])
        prev = scan['nav']
        ww = west_wing(node)
        assert len(ww) == 1 and ww[0]['size'] == 10
        jamb = [m for m in node.narrow_memory if near(m, jx, jy)]
        count = jamb[0].get('contradictions', 0) if jamb else None
        status[scan['scan']] = (ww[0]['status'], count,
                                node.grid_last_eval['clusters'][0] is ww[0])
    assert status[34] == ('unreachable', 0, False)             # as recorded: cut off by keep-out
    assert status[35] == ('unreachable', 1, False)             # one contradiction: still kept
    assert status[36] == ('candidate', None, True)             # released -> reachable and selected
    assert all(status[k][0] == 'candidate' for k in range(36, 41))
    assert [(r['x'], r['y']) for r in node.narrow_released] == [(jx, jy)]
    # Every other remembered post is still there (positions are averaged on re-detection).
    assert len(node.narrow_memory) == len(others)
    assert all(any(near(m, x, y, 0.15) for m in node.narrow_memory) for x, y in others)


def test_K_cafe_table_0_protection_active_in_run2_replay(node):
    fx = json.load(open(os.path.join(DATA, 'run2_jamb_release.json')))
    # cafe_table / cafe_table_0 columns
    cafe = [(x, y, w) for x, y, w in fx['memory_before_scan34'] if x > 8.0]
    assert len(cafe) == 2 and all(w <= 0.06 for _, _, w in cafe)
    # Even a cafe column standing right against a wall (its returns merge with the wall's)
    # is never released: it is thinner than the release width floor.
    wall = (0.80, 0.95, -1.5, 1.5)
    col = column(0.759, 0.0)
    for vx, vy in ((0.0, 0.0), (0.0, 0.5)):
        polar = polar_of(view([wall, col], vx, vy))
        assert contiguous_surface_length(polar, 0.759 - vx, -vy) > 0.30
    reset_release(node, [(0.759, 0.0, 0.042)])
    for vx, vy in ((0.0, 0.0), (0.0, 0.5), (0.0, -0.5), (-0.4, 0.2)):
        release_scan(node, [wall, col], vx, vy)
    assert len(node.narrow_memory) == 1 and node.narrow_released == []
    node.grid = ExplorationGrid(node.p.grid_resolution_m, node.p.grid_robot_radius_m)
    node._grid_update_keepout_cells()
    assert len(node.grid_keepout_cells) > 30
    node.grid = None


def test_L_cafe_collision_replay_still_protected(node):
    fx = json.load(open(os.path.join(DATA, 'cafe_collision_replay.json')))
    grid_node_reset(node, fx['entrance'])
    ox, oy = fx['nav_to_world']
    cx, cy = fx['column_world']
    half = fx['plate_half']

    def plate_clearance(x, y, swept=0.124):
        dx = max(cx - half - x, 0.0, x - cx - half)
        dy = max(cy - half - y, 0.0, y - cy - half)
        return math.hypot(dx, dy) - swept

    prev = None
    for scan in fx['scans']:
        if prev is not None and node.grid is not None:
            node.grid.mark_path_visited(prev[0], prev[1], *scan['start_nav'])
        pose, data = fixture_update(node, scan)
        prev = scan['nav']
    assert any(math.hypot(pose.x + o['x'] + ox - cx, pose.y + o['y'] + oy - cy) < 0.15
               for o in node.narrow_plan)
    assert any(math.hypot(m['x'] + ox - cx, m['y'] + oy - cy) < 0.15 for m in node.narrow_memory)
    assert node.narrow_released == []
    node.explore_step = 10
    node.latest_scan_stamp = None
    node.state = State.STOP_AND_SCAN
    node._grid_select(0.0, pose, data)
    plan = node.explore_plan
    gx, gy = fx['gt_scan10']
    old_h, old_t = math.radians(fx['old_step10']['heading_deg']), fx['old_step10']['target_m']
    assert plate_clearance(gx + old_t * math.cos(old_h), gy + old_t * math.sin(old_h)) < 0.02
    nx = gx + plan['target'] * math.cos(plan['heading'])
    ny = gy + plan['target'] * math.sin(plan['heading'])
    assert node.state == State.EXPLORE_TURN
    assert plate_clearance(nx, ny) > 0.05
    assert math.hypot(nx - cx, ny - cy) >= 0.60
    # Rotation layer at the recorded failure pose (first raw scan of the old scan #11).
    s = fx['failure_raw_scan']
    m = LaserScan()
    m.header.stamp = Time(sec=int(s['stamp']), nanosec=0)
    m.angle_min, m.angle_increment = s['angle_min'], s['angle_increment']
    m.range_min, m.range_max = s['range_min'], s['range_max']
    m.ranges = [math.inf if v is None else float(v) for v in s['ranges']]
    node.latest_scan, node.latest_scan_stamp = m, m.header.stamp
    node.nav_pose = Pose2D(s['nav_x'], s['nav_y'], s['imu_yaw'])
    node.imu_yaw = s['imu_yaw']
    node.keepout_escapes = 0
    node.state = State.STOP_AND_SCAN
    assert node._keepout_before_rotation(0.0) is False
    assert node.state == State.EXPLORE_TURN and node.explore_plan['sector'] == 'ESCAPE'


# ================================================ release eligibility: width history
def remember_series(node, x, y, widths):
    """Remember the same object once per width (first detection, then merged re-detections)."""
    node.narrow_memory = []
    node.narrow_released = []
    node.narrow_plan = []
    node.scan_count = 200
    for w in widths:
        node._narrow_remember(x, y, w)
    assert len(node.narrow_memory) == 1
    return node.narrow_memory[0]


def two_contradictions(node):
    release_scan(node, [WALL_END], 0.0, 0.0)
    release_scan(node, [WALL_END], 0.0, -0.3)


@pytest.mark.parametrize('widths', [
    [0.062, 0.028, 0.035, 0.048],        # recorded divider pattern: one wide first value
    [0.028, 0.062],                      # order does not matter
    [0.062, 0.048],
    [0.149, 0.140, 0.055],               # a single thin observation later is enough
])
def test_WA_width_outlier_does_not_make_releasable(node, widths):
    m = remember_series(node, JAMB[0], JAMB[1], widths)
    assert m['width'] == widths[0]                             # first value kept for logs
    assert m['width_min'] == pytest.approx(min(widths))
    assert m['width_max'] == pytest.approx(max(widths))
    assert m['width_n'] == len(widths)
    assert not node._narrow_release_eligible(m)
    two_contradictions(node)
    assert len(node.narrow_memory) == 1 and node.narrow_released == []
    assert node.narrow_memory[0].get('contradictions', 0) == 0


@pytest.mark.parametrize('widths', [[0.04, 0.05, 0.045, 0.058], [0.06, 0.06], [0.03]])
def test_WB_stable_thin_object_never_releasable(node, widths):
    m = remember_series(node, JAMB[0], JAMB[1], widths)
    assert not node._narrow_release_eligible(m)
    two_contradictions(node)
    release_scan(node, [WALL_END], 0.3, -0.3)
    assert len(node.narrow_memory) == 1 and node.narrow_released == []


@pytest.mark.parametrize('widths', [[0.149, 0.134, 0.146], [0.13, 0.12], [0.07]])
def test_WC_consistently_wide_object_still_released(node, widths):
    m = remember_series(node, JAMB[0], JAMB[1], widths)
    assert node._narrow_release_eligible(m)
    release_scan(node, [WALL_END], 0.0, 0.0)
    assert node.narrow_memory and node.narrow_memory[0]['contradictions'] == 1
    release_scan(node, [WALL_END], 0.0, -0.3)
    assert node.narrow_memory == []
    assert [(r['x'], r['y']) for r in node.narrow_released] == [(JAMB[0], JAMB[1])]
    assert node.narrow_released[0]['width_min'] == pytest.approx(min(widths))


def test_WD_jamb_single_observation_still_releasable(node):
    # Run 2: the jamb was remembered from ONE live cut (0.149 m) and never re-detected
    # before the contradicting views; it must stay releasable.
    m = remember_series(node, JAMB[0], JAMB[1], [0.149])
    assert m['width_n'] == 1 and node._narrow_release_eligible(m)
    two_contradictions(node)
    assert node.narrow_memory == [] and len(node.narrow_released) == 1


@pytest.mark.parametrize('widths', [[0.042, 0.065], [0.034, 0.07, 0.09], [0.058, 0.07]])
def test_WE_post_with_upward_width_noise_never_released(node, widths):
    # Cafe column (0.042) / table leg (0.058) whose width is once over-measured, standing
    # right against a wall so that contradicting geometry IS present.
    wall = (0.80, 0.95, -1.5, 1.5)
    col = column(0.759, 0.0)
    m = remember_series(node, 0.759, 0.0, widths)
    assert not node._narrow_release_eligible(m)
    for vx, vy in ((0.0, 0.0), (0.0, 0.5), (0.0, -0.5), (-0.4, 0.2)):
        release_scan(node, [wall, col], vx, vy)
    assert len(node.narrow_memory) == 1 and node.narrow_released == []


@pytest.mark.parametrize('width', [0.149, 0.03])
def test_WF_released_object_not_remembered_again(node, width):
    remember_series(node, JAMB[0], JAMB[1], [0.149, 0.140])
    two_contradictions(node)
    assert node.narrow_memory == []
    node._narrow_remember(JAMB[0] + 0.10, JAMB[1] - 0.10, width)
    node._narrow_remember(JAMB[0] - 0.05, JAMB[1] + 0.15, width)
    assert node.narrow_memory == []


def divider_replay(node, observations):
    fx = json.load(open(os.path.join(DATA, 'cabinet_divider_replay.json')))
    node.narrow_memory, node.narrow_released, node.narrow_plan = [], [], []
    node.scan_count = 18
    for o in fx['observations'][:observations]:
        node._narrow_remember(o['nav'][0], o['nav'][1], o['width'])
    assert len(node.narrow_memory) == 1
    for view in (fx['scan19'], fx['synthetic_view']):
        node.scan_count += 1
        node.narrow_plan = []
        prof = [math.inf if v is None else v for v in view['profile']]
        node._narrow_release_check(Pose2D(view['nav'][0], view['nav'][1], 0.0), prof)


def test_W_cabinet_divider_replay(node):
    # Gazebo release_validation_20261005: cabinet/cabinet_0 divider, first width 0.062 m
    # (live cut), then 0.033 m (pre-rotation refusal). Recorded scan-19 view (0.55 m
    # surface) + a second ray-cast view 0.30 m away (0.33 m surface).
    divider_replay(node, observations=1)           # first-width-only rule (previous code)
    assert node.narrow_memory == [] and len(node.narrow_released) == 1   # would be released
    divider_replay(node, observations=2)           # full recorded width history
    assert len(node.narrow_memory) == 1 and node.narrow_released == []
    m = node.narrow_memory[0]
    assert m['width_min'] == pytest.approx(0.0328, abs=1e-3) and m['width_n'] == 2
    assert m.get('contradictions', 0) == 0
