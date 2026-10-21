"""
VERIFY_INSIDE gap window built from the minimum detected doorway width.

The doorway detector tends to place a jamb end slightly beyond the real jamb,
so the smoothed e['width'] can exceed the opening (archived failed entry V2a:
1.00 m smoothed, 0.90 m real) and pull the real jamb face into the gap window.
VERIFY_INSIDE now builds only its gap window from e['width_min'], the
smallest matched, unoccluded single-scan width; everything else still uses
e['width']. Recorded fixtures: test/data/verify_inside_replay.json (raw
/scan of the VERIFY_INSIDE checks plus the logged entrance estimate, pose and
detection widths of archived Gazebo runs).
"""

import json
import math
import os
import re

from builtin_interfaces.msg import Time
from my_first_robot.autonomous_search import AutonomousSearch, detect_entrances, Pose2D, State
import pytest
import rclpy
from sensor_msgs.msg import LaserScan

LIDAR_X = -0.032
DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
REPLAY = json.load(open(os.path.join(DATA, 'verify_inside_replay.json')))
V2A = REPLAY['V2a_entry_verify_fail']
SUCCESSES = [k for k in REPLAY if k != 'V2a_entry_verify_fail']


@pytest.fixture(scope='module')
def node():
    rclpy.init()
    n = AutonomousSearch()
    n.p.scan_record_dir = ''
    n.p.scan_after_inside = False
    yield n
    n.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def logs(node):
    out = []
    node._entry_log = lambda msg, level='info': out.append(msg)
    yield out
    del node._entry_log


_stamp = [100]


def laser(ranges, angle_min=0.0, inc=2 * math.pi / 360, rmin=0.12, rmax=3.5):
    m = LaserScan()
    _stamp[0] += 1
    m.header.stamp = Time(sec=_stamp[0], nanosec=0)
    m.angle_min, m.angle_increment = angle_min, inc
    m.angle_max = angle_min + (len(ranges) - 1) * inc
    m.range_min, m.range_max = rmin, rmax
    m.ranges = [math.inf if r is None else float(r) for r in ranges]
    return m


def recorded(scan):
    return laser(scan['ranges'], scan['angle_min'], scan['angle_increment'],
                 scan['range_min'], scan['range_max'])


def ray_box(ox, oy, ang, x0, x1, y0, y1):
    dx, dy = math.cos(ang), math.sin(ang)
    lo, hi = -math.inf, math.inf
    for o, d, a, b in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            if not a <= o <= b:
                return math.inf
            continue
        t0, t1 = sorted(((a - o) / d, (b - o) / d))
        lo, hi = max(lo, t0), min(hi, t1)
    return lo if hi >= lo > 0.0 else math.inf


# Synthetic doorway, door frame (a along the normal, 0 = outer wall face;
# lat + = left): wall 0.15 m thick, true opening |lat| < 0.45 (0.90 m), a
# room behind it. Nav frame = door frame (entrance normal 0).
TRUE_HALF = 0.45
WALLS = [(0.0, 0.15, TRUE_HALF, 3.0), (0.0, 0.15, -3.0, -TRUE_HALF),
         (0.15, 3.0, 2.5, 2.6), (0.15, 3.0, -2.6, -2.5), (3.0, 3.1, -2.6, 2.6)]


def synthetic_scan(robot_a, robot_lat, obstacles=()):
    """Robot facing into the house (yaw 0) at door-frame (robot_a, robot_lat)."""
    boxes = [(a0 - robot_a, a1 - robot_a, l0 - robot_lat, l1 - robot_lat)
             for a0, a1, l0, l1 in list(WALLS) + list(obstacles)]
    ranges = []
    for i in range(360):
        ang = math.radians(i)
        t = min(ray_box(LIDAR_X, 0.0, ang, *b) for b in boxes)
        ranges.append(t if t <= 3.5 else math.inf)
    return laser(ranges)


def doorway_object(lat, width, a0=0.0, depth=0.10):
    return (a0, a0 + depth, lat - 0.5 * width, lat + 0.5 * width)


def entrance(x, y, normal_deg, width, width_min=None):
    e = {'x': x, 'y': y, 'normal': math.radians(normal_deg), 'width': width}
    if width_min is not None:
        e['width_min'] = width_min
    return e


def verify(node, logs, ent, a, lat, yaw_deg, scans, est_lat=None):
    """
    Run handle_verify_inside on `scans`; returns (state, per-scan log rows).

    The robot is at door-frame (a, lat) of `ent`, the node's own estimate.
    """
    node.entrance = ent
    n = ent['normal']
    yaw = math.radians(yaw_deg)
    node.nav_pose = Pose2D(ent['x'] + a * math.cos(n) - lat * math.sin(n),
                           ent['y'] + a * math.sin(n) + lat * math.cos(n), yaw)
    node.imu_yaw = yaw
    node.state = State.VERIFY_INSIDE
    node.mission_phase = 'ENTERING'
    node.entry_inside_verified = False
    node.entry_verify_scans = 0
    node.entry_phase_start = 0.0
    node.entry_settle_scan_key = None
    del logs[:]
    for k, msg in enumerate(scans):
        node.latest_scan, node.latest_scan_stamp = msg, msg.header.stamp
        node.handle_verify_inside(None, 10.0 + k)
        if node.state != State.VERIFY_INSIDE:
            break
    rows = [parse(m) for m in logs if '[VERIFY_INSIDE] scan' in m]
    return node.state, rows


def parse(msg):
    m = re.search(r'L=(\d+) R=(\d+).*?gap_returns=(\d+).*?used=([\d.]+) -> (.*)$', msg)
    return {'L': int(m.group(1)), 'R': int(m.group(2)), 'gap': int(m.group(3)),
            'used': float(m.group(4)), 'result': m.group(5)}


def synthetic_verify(node, logs, obstacles=(), bias=0.0, width=0.98, width_min=0.90,
                     robot_lat=0.0):
    """Estimate centre shifted by `bias` (door-frame lat) from the true centre."""
    ent = entrance(0.0, bias, 0.0, width, width_min)
    scan = synthetic_scan(0.66, robot_lat, obstacles)
    scans = [laser(scan.ranges) for _ in range(node.p.entry_verify_max_scans)]
    return verify(node, logs, ent, 0.66, robot_lat - bias, 0.0, scans)


def replay(node, logs, case, width_min='recorded'):
    e = case['entrance']
    wmin = min(case['track_widths'] + case['refine_widths']) if width_min == 'recorded' \
        else width_min
    ent = entrance(e['x'], e['y'], e['normal_deg'], e['width'], wmin)
    s0 = case['scans'][0]
    return verify(node, logs, ent, s0['a'], s0['lat'], s0['yaw_deg'],
                  [recorded(s) for s in case['scans']])


# ---------------------------------------------------------------- 1. V2a
def test_v2a_recorded_detections_minimum(node):
    widths = V2A['track_widths'] + V2A['refine_widths']
    assert V2A['entrance']['width'] == pytest.approx(1.00)
    assert min(widths) == pytest.approx(0.90)
    assert len(widths) == 56 and max(widths) <= 1.30


def test_v2a_old_window_reproduces_failure(node, logs):
    state, rows = replay(node, logs, V2A, width_min=None)
    assert [r['used'] for r in rows] == [1.0] * 5
    assert all(r['gap'] > 1 for r in rows)            # logged 7, 8, 7, 8, 6
    assert all(min(r['L'], r['R']) >= 3 for r in rows)
    assert state == State.INSIDE_HOLD and node.entry_inside_verified is False
    assert node.mission_phase == 'ENTERING'


def test_v2a_min_width_window_verifies_on_scan_1(node, logs):
    state, rows = replay(node, logs, V2A)
    assert len(rows) == 1 and rows[0]['used'] == pytest.approx(0.90)
    assert rows[0]['gap'] == 0 and rows[0]['result'] == 'INSIDE'
    assert min(rows[0]['L'], rows[0]['R']) >= 3
    assert state == State.INSIDE_HOLD and node.entry_inside_verified is True
    assert node.mission_phase == 'INSIDE'


@pytest.mark.parametrize('k', range(5))
def test_v2a_every_recorded_scan_verifies(node, logs, k):
    case = dict(V2A, scans=[V2A['scans'][k]])
    _, rows = replay(node, logs, case)
    assert rows[0]['gap'] <= 1 and rows[0]['result'] == 'INSIDE'


# ---------------------------------------------------------------- 2. historical successes
@pytest.mark.parametrize('run', SUCCESSES)
def test_historical_success_still_verifies(node, logs, run):
    case = REPLAY[run]
    _, old = replay(node, logs, case, width_min=None)
    assert old[0]['result'] == 'INSIDE'
    state, new = replay(node, logs, case)
    assert new[0]['result'] == 'INSIDE' and new[0]['gap'] == 0
    assert new[0]['L'] >= 3 and new[0]['R'] >= 3
    assert state == State.INSIDE_HOLD and node.entry_inside_verified is True


# ---------------------------------------------------------------- 3./4. synthetic doorway
def test_synthetic_unobstructed_doorway_inside(node, logs):
    state, rows = synthetic_verify(node, logs)
    assert rows[0]['gap'] == 0 and rows[0]['L'] >= 3 and rows[0]['R'] >= 3
    assert state == State.INSIDE_HOLD and node.entry_inside_verified is True


def test_synthetic_v2a_like_biased_estimate(node, logs):
    # Estimate 0.058 m east of the real centre, smoothed width 1.00 m (V2a).
    _, old = synthetic_verify(node, logs, bias=-0.058, width=1.00, width_min=None,
                              robot_lat=-0.045)
    assert old[0]['gap'] > 1
    state, new = synthetic_verify(node, logs, bias=-0.058, width=1.00, width_min=0.90,
                                  robot_lat=-0.045)
    assert new[0]['gap'] == 0 and state == State.INSIDE_HOLD
    assert node.entry_inside_verified is True


@pytest.mark.parametrize('lat', [0.0, 0.25, -0.25, 0.32, -0.32])
@pytest.mark.parametrize('width', [0.05, 0.10])
def test_synthetic_obstruction_rejected(node, logs, lat, width):
    state, rows = synthetic_verify(node, logs, obstacles=[doorway_object(lat, width)])
    assert all(r['gap'] > 1 and r['result'] == 'not verified' for r in rows)
    assert state == State.INSIDE_HOLD and node.entry_inside_verified is False


@pytest.mark.parametrize('lat', [0.0, 0.25, -0.25, 0.32, -0.32])
def test_synthetic_obstruction_rejected_with_biased_estimate(node, logs, lat):
    # Positions relative to the node's (biased) doorway estimate.
    bias = -0.058
    state, rows = synthetic_verify(node, logs, obstacles=[doorway_object(lat + bias, 0.05)],
                                   bias=bias, width=1.00, width_min=0.90, robot_lat=-0.045)
    assert rows[0]['gap'] > 1 and node.entry_inside_verified is False


def test_synthetic_blocked_doorway_rejected(node, logs):
    state, rows = synthetic_verify(node, logs, obstacles=[doorway_object(0.0, 0.90)])
    assert all(r['gap'] > 1 for r in rows)
    assert state == State.INSIDE_HOLD and node.entry_inside_verified is False


def test_jamb_count_requirement_unchanged(node, logs):
    # Gap window narrowed, but a missing wall on one side still fails.
    walls = list(WALLS)
    WALLS[:] = [w for w in walls if w[2] < 0.0]
    try:
        state, rows = synthetic_verify(node, logs)
    finally:
        WALLS[:] = walls
    assert all(r['L'] < 3 and r['result'] == 'not verified' for r in rows)
    assert node.entry_inside_verified is False


# ---------------------------------------------------------------- 5. minimum-width tracker
def cand(x, y, width, occluded=False):
    """Robot-frame detect_entrances() candidate facing +x."""
    return {'center': (x, y), 'normal': 0.0, 'width': width, 'jambs': None,
            'line_distance': x, 'through': 10, 'occluded_rays': 0, 'blocked': 0,
            'occluded': occluded}


def outside(node):
    node.entrance = None
    node.entry_tracks = []
    node.entry_retry_after = 0.0
    node.mission_phase = 'OUTSIDE_SEARCH'
    node.state = State.EXPLORING
    node.nav_pose = Pose2D(0.0, 0.0, 0.0)
    node.imu_yaw = 0.0
    node._entry_start_approach = lambda now, why: None


def feed(node, tick, per_scan):
    for k, cands in enumerate(per_scan):
        node.latest_scan = laser([math.inf] * 360)
        node.latest_scan_stamp = node.latest_scan.header.stamp
        node._detect_entrances_now = lambda c=cands: c
        tick(k)
    del node._detect_entrances_now


def test_tracker_confirmation_minimum_matched_only(node, logs):
    outside(node)
    try:
        feed(node, lambda k: node._entry_outside_search_tick(1.0 + 0.1 * k), [
            [cand(2.0, 0.0, 0.95), cand(1.0, 2.0, 0.75)],              # + unrelated gap
            [cand(2.02, 0.0, 0.72, occluded=True), cand(2.01, 0.01, 0.92)],
            [cand(2.0, -0.01, 0.97), cand(1.0, 2.0, 0.71)],
        ])
    finally:
        del node._entry_start_approach
    e = node.entrance
    assert e is not None and e['width_min'] == pytest.approx(0.92)
    w = 0.95
    for x in (0.92, 0.97):
        w += 0.3 * (x - w)
    assert e['width'] == pytest.approx(w)                     # smoothing unchanged


def test_tracker_refine_updates_only_on_matched_unoccluded(node, logs):
    node.entrance = entrance(2.0, 0.0, 0.0, 0.98, 0.93)
    pose = Pose2D(0.0, 0.0, 0.0)

    def refine(cands):
        node._entry_last_scan_key = None
        feed(node, lambda k: node._entry_refine(pose, 1.0, 'align'), [cands])

    refine([cand(1.0, 2.0, 0.75)])                            # unmatched
    assert node.entrance['width_min'] == pytest.approx(0.93)
    refine([cand(2.0, 0.0, 0.72, occluded=True)])            # occluded
    assert node.entrance['width_min'] == pytest.approx(0.93)
    refine([cand(2.0, 0.0, 0.89)])                            # matched, valid
    assert node.entrance['width_min'] == pytest.approx(0.89)
    assert node.entrance['width'] == pytest.approx(0.98 + 0.3 * (0.89 - 0.98))
    refine([cand(2.0, 0.0, 1.05)])                            # larger: min kept
    assert node.entrance['width_min'] == pytest.approx(0.89)


def wall_with_gap(gap_half, x=2.0, blocked=False):
    boxes = [(x, x + 0.15, gap_half, 3.0), (x, x + 0.15, -3.0, -gap_half)]
    if blocked:
        boxes.append((x + 0.05, x + 0.15, -gap_half, gap_half))
    ranges = []
    for i in range(360):
        t = min(ray_box(LIDAR_X, 0.0, math.radians(i), *b) for b in boxes)
        ranges.append(t if t <= 3.5 else math.inf)
    return ranges


@pytest.mark.parametrize('gap_half, blocked', [(0.30, False), (0.45, True), (0.75, False)])
def test_tracker_invalid_detection_never_updates(node, logs, gap_half, blocked):
    # Too narrow (0.60 m), blocked, too wide (1.50 m): detect_entrances
    # rejects them, so the real _entry_refine path sees nothing.
    ranges = wall_with_gap(gap_half, blocked=blocked)
    assert detect_entrances(ranges, 0.0, 2 * math.pi / 360, 0.12, 3.5, x_offset=LIDAR_X,
                            min_width_m=node.p.entry_min_width_m,
                            max_width_m=node.p.entry_max_width_m) == []
    node.entrance = entrance(2.0, 0.0, 0.0, 0.98, 0.93)
    node._entry_last_scan_key = None
    node.latest_scan = laser(ranges)
    node.latest_scan_stamp = node.latest_scan.header.stamp
    node._entry_refine(Pose2D(0.0, 0.0, 0.0), 1.0, 'align')
    assert node.entrance['width_min'] == pytest.approx(0.93)
    assert node.entrance['width'] == pytest.approx(0.98)


def test_valid_detection_through_real_detector_updates(node, logs):
    ranges = wall_with_gap(0.45)
    node.entrance = entrance(2.0, 0.0, 0.0, 1.00, 0.97)
    node._entry_last_scan_key = None
    node.latest_scan = laser(ranges)
    node.latest_scan_stamp = node.latest_scan.header.stamp
    node._entry_refine(Pose2D(0.0, 0.0, 0.0), 1.0, 'align')
    assert node.entrance['width_min'] < 0.97
    assert node.entrance['width_min'] >= node.p.entry_min_width_m


def test_verify_width_floor_and_cap(node):
    node.entrance = entrance(0.0, 0.0, 0.0, 0.98, 0.60)
    assert node._entry_verify_width() == pytest.approx(node.p.entry_min_width_m) == 0.70
    node.entrance = entrance(0.0, 0.0, 0.0, 0.98, 1.05)
    assert node._entry_verify_width() == pytest.approx(0.98)     # never wider than before
    node.entrance = entrance(0.0, 0.0, 0.0, 0.98, 0.90)
    assert node._entry_verify_width() == pytest.approx(0.90)


# ---------------------------------------------------------------- 6. fallback
@pytest.mark.parametrize('wmin', [None, float('nan')])
def test_fallback_without_minimum_uses_averaged_width(node, logs, wmin):
    node.entrance = entrance(0.0, 0.0, 0.0, 0.98, wmin)
    assert node._entry_verify_width() == pytest.approx(0.98)
    ent = entrance(0.0, 0.0, 0.0, 0.98, wmin)
    _, rows = verify(node, logs, ent, 0.66, 0.0, 0.0, [synthetic_scan(0.66, 0.0)])
    assert rows[0]['used'] == pytest.approx(0.98) and rows[0]['result'] == 'INSIDE'


# ---------------------------------------------------------------- 7. regression
def test_alignment_still_uses_averaged_width(node, logs):
    out = {}
    for wmin in (None, 0.80):
        node.entrance = entrance(0.0, 0.0, 0.0, 1.00, wmin)
        node.nav_pose = Pose2D(-0.60, 0.0, 0.0)
        node.imu_yaw = 0.0
        node.entry_sidesteps = 0
        node.entry_attempts = 0
        node.entry_align_start = 0.0
        node._entry_last_scan_key = None
        node.latest_scan = laser([math.inf] * 360)           # no corridor: nominal width
        node.latest_scan_stamp = node.latest_scan.header.stamp
        node.state = State.ENTRANCE_ALIGN
        del logs[:]
        node._entry_measure_and_decide(node._entry_pose(), 5.0)
        meas = next(m for m in logs if 'measure:' in m)
        assert 'door_width=1.00m' in meas
        assert 'L=+0.500(nominal) R=-0.500(nominal) width=1.00m' in meas
        out[wmin] = (node.state, node.entry_stop_a, node.entry_lateral_target,
                     [m for m in logs if 'width_min' not in m])
    assert out[None] == out[0.80]


def test_crossing_unchanged_by_minimum(node, logs):
    cmds = {}
    for wmin in (None, 0.80):
        node.entrance = entrance(0.0, 0.0, 0.0, 1.00, wmin)
        node.nav_pose = Pose2D(0.10, 0.02, 0.02)
        node.imu_yaw = 0.02
        node.state = State.ENTRANCE_CROSS
        node.entry_stop_a = 0.70
        node.entry_lateral_target = 0.0
        node.entry_cross_start = 0.0
        node._entry_progress_reset(0.0, 0.10)
        node._cross_log_time = 0.0
        node.latest_scan = synthetic_scan(0.10, 0.02)
        node.latest_scan_stamp = node.latest_scan.header.stamp
        sent = []
        node.publish_velocity = lambda lin, ang: sent.append((lin, ang))
        try:
            node.handle_entrance_cross(None, 1.0)
        finally:
            del node.publish_velocity
        cmds[wmin] = (node.state, sent)
    assert cmds[None] == cmds[0.80] and cmds[None][1][0][0] > 0.0


@pytest.mark.parametrize('lat, expect', [(0.85, True), (0.95, True), (1.05, False)])
def test_entrance_guard_unchanged(node, lat, expect):
    for wmin in (None, 0.80):
        node.entrance = entrance(0.0, 0.0, 0.0, 1.00, wmin)
        # Lateral 0.95: inside 0.5*avg+0.5 = 1.00, outside 0.5*min+0.5 = 0.90.
        assert node._explore_toward_entrance(Pose2D(-1.0, lat, 0.0), 0.0, 1.2) is expect
