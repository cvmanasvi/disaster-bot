"""
Victim prioritizer (victim_prioritization_enabled) above the existing victim approach.

Pure ranking tests on my_first_robot.victim_prioritizer plus node tests that drive
AutonomousSearch by direct calls (no simulator), as in test_victim_approach.py.
"""

import ast
import inspect
import json
import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from my_first_robot import victim_prioritizer as vp
from my_first_robot.autonomous_search import AutonomousSearch, Pose2D, State, VictimMemory
import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

LIDAR_X = -0.032
STOP = 0.80


@pytest.fixture
def make_node():
    made = []

    def make(approach=True, prioritize=True):
        args = ['--ros-args',
                '-p', f'victim_approach_enabled:={"true" if approach else "false"}',
                '-p', f'victim_prioritization_enabled:={"true" if prioritize else "false"}']
        rclpy.init(args=args)
        n = AutonomousSearch()
        n.p.scan_record_dir = ''
        made.append(n)
        return n
    yield make
    for n in made:
        n.destroy_node()
    rclpy.shutdown()


def reading(distance, angle_deg=0.0, confidence=0.9, age=0.05, victim_id=1):
    m = String()
    m.data = json.dumps({'victim_id': victim_id, 'class': 'person', 'confidence': confidence,
                         'distance_m': distance, 'camera_x': 160.0, 'angle_deg': angle_deg,
                         'detection_age_s': age})
    return m


def inside(node, x=0.0, y=0.0, yaw=0.0, state=State.EXPLORE_MOVE, rotating=False):
    node.mission_phase = 'INSIDE'
    node.nav_pose = Pose2D(x, y, yaw)
    node.imu_yaw = yaw
    node.imu_history.clear()
    end = yaw + (0.5 if rotating else 0.0)
    for k in range(13):
        node.imu_history.append((0.1 * k, yaw + (end - yaw) * k / 12))
    node.state = state
    node.entry_phase = 'SETTLE' if state == State.STOP_AND_SCAN else None
    node.narrow_plan = []


def scan_data(r=4.0):
    return {'analysis': {'profile': [r] * 360}, 'complete': True, 'reason': 'complete',
            'result': {}}


def profile_with(points, r=4.0):
    data = scan_data(r)
    profile = data['analysis']['profile']
    for x, y in points:
        k = int(math.floor(math.degrees(math.atan2(y, x)))) % 360
        profile[k] = min(profile[k], math.hypot(x, y))
    return data


def laser_empty():
    m = LaserScan()
    m.header.stamp = Time(sec=7, nanosec=0)
    m.angle_min, m.angle_increment = 0.0, 2 * math.pi / 360
    m.angle_max = m.angle_min + 359 * m.angle_increment
    m.range_min, m.range_max = 0.12, 3.5
    m.ranges = [3.0] * 360
    return m


def fresh_live(node):
    node.latest_scan = laser_empty()
    node.latest_scan.header.stamp = node.get_clock().now().to_msg()
    node.latest_scan_stamp = node.latest_scan.header.stamp


def confirm(node, distance, angle_deg=0.0, confidence=0.9, n=3, yaw=0.0):
    inside(node, yaw=yaw)
    for _ in range(n):
        node.victim_callback(reading(distance, angle_deg, confidence))


def recorder(node):
    events = []
    node._victim_record = events.append
    return events


def decide(node, data=None):
    inside(node, state=State.STOP_AND_SCAN)
    return node._victim_after_scan(node._now_sec(), node._entry_pose(), data or scan_data())


def track_near(node, x, y):
    return min(node.victim_memory.tracks, key=lambda t: math.hypot(t.x - x, t.y - y))


def make_target(node, track, moves=0):
    node.victim_memory.tracks.remove(track)
    node.victim_target = {'track': track, 'missed': 0, 'moves': moves, 'checked': -1.0,
                          'offsets_used': []}


def plan_bearing_deg(node):
    return math.degrees(node.explore_plan['heading'])


def cand(cid, conf, rng, offset=0.0, risk=0.0):
    return vp.VictimCandidate(cid, conf, rng, offset, risk, STOP)


BLOCK_AHEAD = [(0.10, -0.22), (0.10, 0.22)]      # rejects all 5 headings toward +x


# ---------------------------------------------------------------- 1 one victim
def test_01_one_victim_selected(make_node):
    node = make_node()
    events = recorder(node)
    confirm(node, 2.5, 0.0, 0.85)
    assert decide(node)
    assert node.state == State.EXPLORE_TURN and node.explore_plan['sector'] == 'VICTIM'
    assert node.victim_target['track'].id == 1
    rec = [e for e in events if e['event'] == 'prioritized']
    assert len(rec) == 1 and rec[0]['chosen'] == 1 and len(rec[0]['ranking']) == 1
    assert rec[0]['ranking'][0]['access_offset_deg'] == 0.0


def test_01_pure_single_candidate():
    c = cand(1, 0.85, 2.5)
    assert vp.choose([c]) == (c, 'best')
    assert vp.choose([cand(1, 0.85, 2.5, offset=None)]) == (None, 'none_feasible')
    assert vp.choose([]) == (None, 'none_feasible')


# ---------------------------------------------------------------- 2 two victims
def test_02_pure_combined_evidence_wins():
    a = cand(1, 0.92, 3.0)                 # confident but far
    b = cand(2, 0.75, 1.5)                 # less confident but close
    assert b.score > a.score
    assert a.score == pytest.approx(0.35 * 0.8 + 0.35 * (1 - 2.2 / 2.7) + 0.30, abs=1e-6)
    assert b.score == pytest.approx(0.35 * 0.375 + 0.35 * (1 - 0.7 / 2.7) + 0.30, abs=1e-6)
    assert vp.choose([a, b])[0] is b and vp.choose([b, a])[0] is b
    assert vp.rank([a, b]) == [b, a]


def test_02_node_two_victims_higher_priority_selected(make_node):
    node = make_node()
    events = recorder(node)
    confirm(node, 3.0, 0.0, 0.92)          # A straight ahead
    confirm(node, 1.5, 60.0, 0.75)         # B 60 deg left
    assert len(node.victim_memory.tracks) == 2
    assert decide(node)
    tb = node.victim_target['track']
    assert tb.id == 2 and plan_bearing_deg(node) == pytest.approx(
        math.degrees(math.atan2(tb.y, tb.x)), abs=1e-6)
    rec = [e for e in events if e['event'] == 'prioritized'][-1]
    assert [r['id'] for r in rec['ranking']] == [2, 1]
    # The other victim stays a candidate (not abandoned, not cleared).
    assert [t.id for t in node.victim_memory.tracks] == [1] and not node.victim_abandoned


# ---------------------------------------------------------------- 3 inaccessible victim
def test_03_pure_inaccessible_loses():
    c = cand(1, 0.95, 2.0, offset=None)    # no safe heading
    b = cand(2, 0.70, 2.0)
    assert not c.feasible and vp.choose([c, b]) == (b, 'best')
    detour = cand(3, 0.88, 2.0, offset=20.0, risk=0.5)   # reachable only by +-20, risky
    assert detour.access_score == 0.5 and detour.score < cand(4, 0.88, 2.0).score
    assert vp.rank([c, detour, b])[-1] is c


def test_03_node_accessible_lower_confidence_wins(make_node):
    node = make_node()
    fresh_live(node)
    events = recorder(node)
    confirm(node, 2.0, 0.0, 0.95)                    # C ahead (+x): corridor blocked
    confirm(node, 2.0, 0.0, 0.70, yaw=math.pi)       # B behind (-x): clear
    data = profile_with(BLOCK_AHEAD)
    assert decide(node, data)
    tb = node.victim_target['track']
    assert tb.x < 0.0 and tb.mean_confidence() == pytest.approx(0.70)
    assert abs(plan_bearing_deg(node)) == pytest.approx(180.0, abs=1e-6)
    rank = [e for e in events if e['event'] == 'prioritized'][-1]['ranking']
    assert rank[0]['id'] == tb.id and rank[1]['feasible'] is False
    assert rank[1]['checks'] == ['+0:corridor_blocked', '+10:corridor_blocked',
                                 '-10:corridor_blocked', '+20:corridor_blocked',
                                 '-20:corridor_blocked']
    # The blocked victim is kept for a later decision, not abandoned.
    assert len(node.victim_memory.tracks) == 1 and not node.victim_abandoned


def test_03_risk_ignores_person_and_counts_obstacles_near_line():
    keep = 0.63
    # Obstacle 0.5 m beside the middle of a 2 m approach: inside keep-out -> risk 1.
    assert vp.risk_score([(1.0, 0.5)], 2.0, 0.0, keep) == 1.0
    # 0.78 m away (keep-out + 0.15): half risk; 1.0 m: none.
    assert vp.risk_score([(1.0, keep + 0.15)], 2.0, 0.0, keep) == pytest.approx(0.5)
    assert vp.risk_score([(1.0, 1.0)], 2.0, 0.0, keep) == 0.0
    # The person's own legs (within 0.5 m of the victim) are not a risk.
    assert vp.risk_score([(2.75, 0.1)], 2.0, 0.0, keep, (2.8, 0.0), 0.5) == 0.0


def test_03_node_risk_from_narrow_obstacle_lowers_score(make_node):
    node = make_node()
    confirm(node, 2.5, 0.0, 0.85)
    tr = node.victim_memory.tracks[0]
    inside(node, state=State.STOP_AND_SCAN)
    clean, _ = node._victim_assess(node._entry_pose(), scan_data(), tr)
    node.narrow_plan = [{'x': 0.8, 'y': 0.70, 'range': 1.06, 'bearing': 0.7, 'width': 0.04}]
    risky, _ = node._victim_assess(node._entry_pose(), scan_data(), tr)
    assert clean.risk == 0.0 and risky.risk > 0.0 and risky.score < clean.score


# ---------------------------------------------------------------- 4 close/low vs far/high
def test_04_close_low_vs_far_high_deterministic():
    close = cand(1, 0.65, 1.0)
    far = cand(2, 0.95, 3.0)
    expect_close = 0.35 * 0.125 + 0.35 * (1 - 0.2 / 2.7) + 0.30
    expect_far = 0.35 * 0.875 + 0.35 * (1 - 2.2 / 2.7) + 0.30
    assert close.score == pytest.approx(expect_close, abs=1e-6)
    assert far.score == pytest.approx(expect_far, abs=1e-6)
    winner = far if expect_far > expect_close else close
    for order in ([close, far], [far, close]):
        for _ in range(3):
            assert vp.choose(order)[0] is winner
    # Neither factor alone decides: a much closer low-confidence victim wins.
    assert vp.choose([cand(3, 0.65, 0.9), far])[0].cid == 3


# ---------------------------------------------------------------- 5 tie
def test_05_tie_break_deterministic():
    a, b = cand(5, 0.80, 2.0), cand(2, 0.80, 2.0)
    assert a.score == b.score
    assert vp.choose([a, b])[0] is b and vp.choose([b, a])[0] is b          # lower id
    # Equal score, different range -> the shorter range wins.
    near, far = cand(9, 0.70, 1.34), cand(1, 0.80, 2.015)
    assert near.score == far.score
    assert vp.choose([far, near])[0] is near and vp.choose([near, far])[0] is near
    # Equal score and range -> higher confidence (only possible via risk) -> id.
    x, y = cand(4, 0.80, 2.0, risk=0.0), cand(3, 0.80, 2.0, risk=0.0)
    assert vp.rank([x, y])[0] is y


# ---------------------------------------------------------------- 6 no oscillation
def test_06_pure_small_fluctuation_keeps_target():
    cur = cand(1, 0.80, 2.0)
    for conf in (0.82, 0.84, 0.86, 0.88, 0.84, 0.81):
        other = cand(2, conf, 2.0)
        assert other.score - cur.score < vp.SWITCH_MARGIN
        assert vp.choose([cur, other], current_id=1) == (cur, 'hysteresis')


def test_06_node_small_fluctuation_no_switch(make_node):
    node = make_node()
    events = recorder(node)
    confirm(node, 2.0, 0.0, 0.80)
    make_target(node, node.victim_memory.tracks[0])
    for k, conf in enumerate((0.86, 0.84, 0.88, 0.83)):
        confirm(node, 2.0, 90.0, conf)              # B alongside, slightly more confident
        node.victim_target.update(moves=0, checked=-1.0)
        assert decide(node)
        assert node.victim_target['track'].id == 1, f'switched at decision {k}'
        assert abs(plan_bearing_deg(node)) < 1e-6
    rr = [e for e in events if e['event'] == 'reprioritized']
    assert len(rr) == 4 and all(e['reason'] == 'hysteresis' for e in rr)
    assert not [e for e in events if e['event'] == 'switched']


# ---------------------------------------------------------------- 7 meaningful improvement
def test_07_pure_meaningful_improvement_switches():
    cur = cand(1, 0.62, 3.4)
    better = cand(2, 0.95, 1.2)
    assert better.score - cur.score >= vp.SWITCH_MARGIN
    assert vp.choose([cur, better], current_id=1) == (better, 'margin')
    # Current target became infeasible -> best feasible other.
    stuck = cand(1, 0.95, 1.2, offset=None)
    assert vp.choose([stuck, cur.__class__(2, 0.62, 3.4, 0.0, 0.0, STOP)],
                     current_id=1)[1] == 'current_infeasible'


def test_07_node_switch_at_safe_decision_point(make_node):
    node = make_node()
    events = recorder(node)
    confirm(node, 3.4, 0.0, 0.62)                   # weak, far target A
    make_target(node, node.victim_memory.tracks[0])
    confirm(node, 1.2, 90.0, 0.95)                  # strong, close B
    assert decide(node)
    tb = node.victim_target['track']
    assert tb.id == 2 and node.victim_target['moves'] == 1
    assert plan_bearing_deg(node) == pytest.approx(math.degrees(math.atan2(tb.y, tb.x)),
                                                   abs=1e-6)
    sw = [e for e in events if e['event'] == 'switched']
    assert len(sw) == 1 and sw[0]['from'] == 1 and sw[0]['to'] == 2
    assert [t.id for t in node.victim_memory.tracks] == [1] and not node.victim_abandoned


# ---------------------------------------------------------------- 8 target lock
def test_08_pure_locked_never_switches():
    cur, better = cand(1, 0.62, 3.4), cand(2, 0.95, 1.2)
    assert vp.choose([cur, better], current_id=1, locked=True) == (cur, 'locked')


def test_08_node_active_approach_cannot_switch(make_node):
    node = make_node()
    events = recorder(node)
    confirm(node, 3.4, 0.0, 0.62)
    make_target(node, node.victim_memory.tracks[0], moves=1)      # approach already moving
    confirm(node, 1.2, 90.0, 0.95)
    assert decide(node)
    assert node.victim_target['track'].id == 1 and node.victim_target['moves'] == 2
    assert abs(plan_bearing_deg(node)) < 1e-6
    assert not [e for e in events if e['event'] in ('reprioritized', 'switched')]


def test_08_readings_during_turn_and_move_never_switch(make_node):
    node = make_node()
    confirm(node, 3.4, 0.0, 0.62)
    make_target(node, node.victim_memory.tracks[0])
    calls = []
    node._victim_prioritize = lambda *a: calls.append('p')
    node._victim_maybe_switch = lambda *a: calls.append('s')
    for state in (State.EXPLORE_TURN, State.EXPLORE_MOVE):
        inside(node, state=state)
        for _ in range(5):
            node.victim_callback(reading(1.2, 90.0, 0.95))
        inside(node, state=state, rotating=True)
        node.victim_callback(reading(1.2, 90.0, 0.95))
    assert node.victim_target['track'].id == 1 and not calls


# ---------------------------------------------------------------- 9 entrance
@pytest.mark.parametrize('state,phase', [
    (State.ENTRANCE_APPROACH, 'ENTERING'), (State.ENTRANCE_ALIGN, 'ENTERING'),
    (State.ENTRANCE_CROSS, 'ENTERING'), (State.VERIFY_INSIDE, 'INSIDE'),
    (State.EXPLORING, 'OUTSIDE_SEARCH')])
def test_09_entrance_unaffected(make_node, state, phase):
    node = make_node()
    calls = []
    node._victim_prioritize = lambda *a: calls.append('p')
    node._victim_assess = lambda *a: calls.append('a')
    node._victim_rank_cues = lambda *a: calls.append('c')
    sent = []
    node.cmd_publisher.publish = sent.append
    for rotating in (False, True):
        inside(node, state=state, rotating=rotating)
        node.mission_phase = phase
        for angle, conf in ((0.0, 0.9), (60.0, 0.7)):
            for _ in range(4):
                node.victim_callback(reading(2.0, angle, conf))
    assert node.state == state and node.victim_target is None
    assert not node.victim_memory.tracks and not node.victim_cues.tracks
    assert not calls and not sent


# ---------------------------------------------------------------- 10 rotating scan
def test_10_rotating_scan_collects_cues_only(make_node):
    node = make_node()
    events = recorder(node)
    confirm(node, 3.4, 0.0, 0.62)
    make_target(node, node.victim_memory.tracks[0])
    calls = []
    node._victim_prioritize = lambda *a: calls.append('p')
    node._victim_maybe_switch = lambda *a: calls.append('s')
    sent = []
    node.cmd_publisher.publish = sent.append
    events.clear()                                       # setup readings above
    inside(node, state=State.STOP_AND_SCAN)
    node.entry_phase = 'ROTATE'
    for angle, conf in ((90.0, 0.95), (-60.0, 0.80)):
        for _ in range(3):
            node.victim_callback(reading(1.5, angle, conf))
    assert node.victim_stats['cues'] == 6 and len(node.victim_cues.tracks) == 2
    assert not node.victim_memory.tracks                 # cues never become positions
    assert node.victim_target['track'].id == 1 and node.victim_target['moves'] == 0
    assert node.state == State.STOP_AND_SCAN and node.explore_plan is None
    assert not calls and not sent
    assert {e['event'] for e in events} == {'cue'}


def test_10_cue_groups_ranked_for_look(make_node):
    node = make_node()
    fresh_live(node)                                        # pre-rotation check input
    events = recorder(node)
    inside(node, state=State.STOP_AND_SCAN)
    node.entry_phase = 'ROTATE'
    for _ in range(5):
        node.victim_callback(reading(3.2, -60.0, 0.70))    # more cues, weak and far
    for _ in range(3):
        node.victim_callback(reading(1.4, 60.0, 0.93))     # fewer cues, strong and close
    assert decide(node)
    assert node.explore_plan['sector'] == 'VICTIM_LOOK' and node.explore_plan['target'] == 0.0
    assert plan_bearing_deg(node) > 0.0
    rec = [e for e in events if e['event'] == 'cues_prioritized']
    assert rec and rec[0]['chosen'] == 2


# ---------------------------------------------------------------- 11 stale / invalid
def test_11_stale_and_invalid_candidates_removed(make_node):
    node = make_node()
    events = recorder(node)
    now = node._now_sec()
    for _ in range(3):                                      # stale (older than 30 s)
        node.victim_memory.observe(now - 40.0, 1.5, 0.0, 0.95, 1.5)
    confirm(node, 2.5, 60.0, 0.80)                          # fresh
    confirm(node, 2.0, -60.0, 0.40)                         # below the confidence bar
    confirm(node, 2.0, -60.0, 0.90, n=1)                    # one reading: not confirmed
    assert decide(node)
    rec = [e for e in events if e['event'] == 'prioritized'][-1]
    assert [r['id'] for r in rec['ranking']] == [2]
    assert node.victim_target['track'].id == 2
    assert all(t.id != 1 for t in node.victim_memory.tracks)
    assert node.victim_stats['rejected_invalid'] == 3


def test_11_only_stale_candidates_no_selection(make_node):
    node = make_node()
    now = node._now_sec()
    for _ in range(3):
        node.victim_memory.observe(now - 31.0, 1.5, 0.0, 0.95, 1.5)
    assert not decide(node)
    assert node.victim_target is None and not node.victim_memory.tracks


# ---------------------------------------------------------------- 12 unreliable YOLO ids
def test_12_spatial_association_ignores_yolo_id():
    def run(order, ids):
        node_mem = VictimMemory(0.5, 3, 30.0)
        t = 0.0
        for who, vid in zip(order, ids):
            x, y = {'P': (2.0, 0.0), 'Q': (1.0, 1.7)}[who]
            node_mem.observe(t, x, y, 0.9, 2.0)
            t += 0.1
        return [(tr.id, round(tr.x, 6), round(tr.y, 6), tr.count) for tr in node_mem.tracks]

    seq = 'PQPQQP'
    assert run(seq, [1] * 6) == run(seq, [1] * 6) == run(seq, [7, 3, 1, 1, 9, 2])
    assert run(seq, [1] * 6) == [(1, 2.0, 0.0, 3), (2, 1.0, 1.7, 3)]
    assert run('QPQP', [1] * 4)[0][1:3] == (1.0, 1.7)            # first seen -> id 1
    # Equidistant reading -> the earlier (lower id) track.
    mem = VictimMemory(0.5, 3, 30.0)
    mem.observe(0.0, 0.0, 0.0, 0.9, 1.0)
    mem.observe(0.0, 0.6, 0.0, 0.9, 1.0)
    assert mem.observe(0.1, 0.3, 0.0, 0.9, 1.0).id == 1


def test_12_node_same_yolo_id_two_people(make_node):
    node = make_node()
    inside(node)
    for _ in range(3):
        node.victim_callback(reading(2.0, 0.0, 0.9, victim_id=1))
        node.victim_callback(reading(2.0, 60.0, 0.8, victim_id=1))
    ids = sorted((t.id, round(math.degrees(math.atan2(t.y, t.x)))) for t in
                 node.victim_memory.tracks)
    assert [i for i, _ in ids] == [1, 2] and all(t.count == 3 for t in node.victim_memory.tracks)


# ---------------------------------------------------------------- 13 single-victim regression
def test_13_single_victim_plan_matches_existing(make_node):
    node = make_node()
    events = recorder(node)
    confirm(node, 2.5)
    assert decide(node)
    plan = node.explore_plan                         # same as test_victim_approach test_24
    assert plan['sector'] == 'VICTIM' and plan['offset_deg'] == 0.0
    assert plan['heading'] == pytest.approx(0.0) and plan['target'] == pytest.approx(1.0)
    cands = [e for e in events if e['event'] == 'approach_candidates'][-1]['candidates']
    assert [(c['offset_deg'], c['result']) for c in cands] == [(0.0, 'ok')]


def test_13_single_victim_hold_report_keys(make_node):
    node = make_node()
    confirm(node, 0.80, 0.0, 0.85)
    assert decide(node)
    rep = node.victim_report
    assert node.state == State.VICTIM_HOLD and rep['state'] == 'VICTIM_HOLD'
    assert rep['victim_x'] == pytest.approx(0.80 - 0.032, abs=1e-3)
    for key in ('victim_x', 'victim_y', 'frame', 'distance_m', 'last_measured_distance_m',
                'confidence', 'observations', 'robot_x', 'robot_y', 'timestamp', 'state',
                'reason'):
        assert key in rep
    assert list(rep)[:12] == ['victim_x', 'victim_y', 'frame', 'distance_m',
                              'last_measured_distance_m', 'confidence', 'observations',
                              'robot_x', 'robot_y', 'timestamp', 'state', 'reason']
    assert rep['candidate_id'] == 1 and rep['candidates_considered'] == 1
    assert rep['priority_score'] == pytest.approx(
        0.35 * 0.625 + 0.35 * 1.0 + 0.30, abs=1e-3)


def test_13_abandon_keeps_other_candidates_and_next_ranked_follows(make_node):
    node = make_node()
    fresh_live(node)
    confirm(node, 1.5, 0.0, 0.95)                    # best, ahead
    confirm(node, 2.0, 0.0, 0.70, yaw=math.pi)       # second, behind
    assert decide(node) and node.victim_target['track'].id == 1
    # Now the way to #1 is blocked on every heading -> the existing rule abandons it.
    assert not decide(node, profile_with(BLOCK_AHEAD))
    assert node.victim_target is None and node.victim_abandoned
    assert [t.id for t in node.victim_memory.tracks] == [2]
    assert decide(node) and node.victim_target['track'].id == 2


# ---------------------------------------------------------------- 14 flag off
def test_14_flag_off_completely_inactive(make_node):
    node = make_node(prioritize=False)
    assert node.p.victim_prioritization_enabled is False and not node.victim_prioritizing

    def boom(*a):
        raise AssertionError('prioritizer called with the flag off')
    for name in ('_victim_prioritize', '_victim_maybe_switch', '_victim_rank_cues',
                 '_victim_assess'):
        setattr(node, name, boom)
    confirm(node, 3.0, 0.0, 0.92)
    confirm(node, 1.5, 60.0, 0.75)
    confirm(node, 1.5, 60.0, 0.75)                    # more readings -> max count wins
    assert decide(node)
    assert node.victim_target['track'].id == 2        # existing rule: most readings
    node.victim_target['moves'] = 0
    assert decide(node)
    # Existing abandon clears all candidates.
    node._victim_abandon(node._now_sec(), node._entry_pose(), 'test')
    assert not node.victim_memory.tracks
    confirm(node, 0.80)
    assert decide(node) and node.state == State.VICTIM_HOLD
    assert 'candidate_id' not in node.victim_report
    assert 'priority_score' not in node.victim_report


def test_14_prioritization_needs_victim_approach(make_node):
    node = make_node(approach=False, prioritize=True)
    assert not node.victim_prioritizing and node.victim_found_publisher is None
    assert '/victim_distance' not in [s.topic_name for s in node.subscriptions]


# ---------------------------------------------------------------- safety: never commands motion
def test_prioritizer_module_has_no_ros_or_motion():
    tree = ast.parse(inspect.getsource(vp))
    imported = {a.name.split('.')[0] for n in ast.walk(tree)
                if isinstance(n, (ast.Import, ast.ImportFrom))
                for a in (n.names if isinstance(n, ast.Import) else [ast.alias(n.module)])}
    assert imported == {'math'}


def test_prioritizer_adds_no_publishers(make_node):
    node = make_node()
    assert [p.topic_name for p in node.publishers if p.msg_type is TwistStamped] == [
        '/cmd_vel_raw']
    sent = []
    node.cmd_publisher.publish = sent.append
    confirm(node, 3.0, 0.0, 0.92)
    confirm(node, 1.5, 60.0, 0.75)
    tracks = node.victim_memory.confirmed_all(node._now_sec())
    inside(node, state=State.STOP_AND_SCAN)
    for tr in tracks:
        node._victim_assess(node._entry_pose(), scan_data(), tr)
    assert node._victim_prioritize(node._now_sec(), node._entry_pose(), scan_data()) is not None
    assert not sent and node.state == State.STOP_AND_SCAN and node.explore_plan is None


# ================================================================ hardening
def laser_with_post(node, post_x, post_y, half=0.022):
    """Fresh raw scan from the robot at the origin: 3 m room + one thin square post."""
    m = laser_empty()
    ranges = []
    for i in range(360):
        a = i * m.angle_increment
        dx, dy = math.cos(a), math.sin(a)
        t = 3.0
        tmin, tmax = -math.inf, math.inf
        for o, d, lo, hi in ((LIDAR_X, dx, post_x - half, post_x + half),
                             (0.0, dy, post_y - half, post_y + half)):
            if abs(d) < 1e-12:
                if o < lo or o > hi:
                    tmin = math.inf
                continue
            t0, t1 = (lo - o) / d, (hi - o) / d
            tmin, tmax = max(tmin, min(t0, t1)), min(tmax, max(t0, t1))
        if tmax >= tmin > 0:
            t = min(t, tmin)
        ranges.append(t)
    m.ranges = ranges
    m.header.stamp = node.get_clock().now().to_msg()
    node.latest_scan = m
    node.latest_scan_stamp = m.header.stamp


DIRECT_BLOCKED = [(0.6, -0.15)]                  # direct corridor blocked, +10 deg passes


def test_15_access_table_matches_heading_offsets():
    from my_first_robot.autonomous_search import VICTIM_HEADING_OFFSETS_DEG
    assert {abs(o) for o in VICTIM_HEADING_OFFSETS_DEG} == set(vp.ACCESS_BY_OFFSET)
    scores = [vp.accessibility_score(o) for o in VICTIM_HEADING_OFFSETS_DEG]
    assert all(s > 0.0 for s in scores)            # every approach heading counts as access
    by_abs = sorted(vp.ACCESS_BY_OFFSET.items())
    assert all(a[1] > b[1] for a, b in zip(by_abs, by_abs[1:]))   # wider offset = lower access
    assert vp.accessibility_score(10.0) == vp.accessibility_score(-10.0)
    assert vp.accessibility_score(20.0) == vp.accessibility_score(-20.0)


def test_16_node_current_infeasible_switches(make_node):
    node = make_node()
    fresh_live(node)
    events = recorder(node)
    confirm(node, 2.0, 0.0, 0.95)                    # current target C ahead
    make_target(node, node.victim_memory.tracks[0])
    confirm(node, 2.0, 0.0, 0.70, yaw=math.pi)       # B behind, lower confidence
    assert decide(node, profile_with(BLOCK_AHEAD))   # every heading to C blocked
    rr = [e for e in events if e['event'] == 'reprioritized'][-1]
    assert rr['reason'] == 'current_infeasible'
    assert node.victim_target['track'].x < 0.0 and node.victim_target['moves'] == 1
    assert abs(plan_bearing_deg(node)) == pytest.approx(180.0, abs=1e-6)
    # C goes back to the candidates, it is not abandoned.
    assert [t.id for t in node.victim_memory.tracks] == [1] and not node.victim_abandoned


def test_16_node_current_infeasible_without_alternative_abandons(make_node):
    node = make_node()
    fresh_live(node)
    events = recorder(node)
    confirm(node, 2.0, 0.0, 0.95)
    make_target(node, node.victim_memory.tracks[0])
    assert not decide(node, profile_with(BLOCK_AHEAD))
    assert node.victim_target is None and node.victim_abandoned     # existing abandon rule
    assert not [e for e in events if e['event'] in ('reprioritized', 'switched')]
    ab = [e for e in events if e['event'] == 'abandoned']
    assert ab and 'no safe approach heading' in ab[0]['reason']


@pytest.mark.parametrize('lead,expect', [
    (0.0, 'hysteresis'), (0.05, 'hysteresis'), (0.0999, 'hysteresis'),
    (0.09999, 'hysteresis'), (0.1, 'margin'), (0.1001, 'margin'), (0.11, 'margin')])
def test_17_switch_margin_boundary(lead, expect):
    cur, other = cand(1, 0.80, 2.0), cand(2, 0.80, 2.0)
    cur.score, other.score = 0.70, round(0.70 + lead, vp.SCORE_DECIMALS)
    assert vp.SWITCH_MARGIN == 0.10
    chosen, reason = vp.choose([cur, other], current_id=1)
    assert reason == expect and chosen is (other if expect == 'margin' else cur)
    # Locked: never, whatever the lead.
    assert vp.choose([cur, other], current_id=1, locked=True) == (cur, 'locked')


@pytest.mark.parametrize('points,offset', [
    ([], 0.0), (DIRECT_BLOCKED, 10.0), ([(0.29, 0.23)], -10.0),
    ([(0.10, -0.22)], 20.0), ([(0.10, 0.22)], -20.0)])
def test_18_assessed_heading_matches_approach_heading(make_node, points, offset):
    node = make_node()
    fresh_live(node)
    events = recorder(node)
    confirm(node, 2.5)
    assert decide(node, profile_with(points))
    rank = [e for e in events if e['event'] == 'prioritized'][-1]['ranking']
    assert rank[0]['access_offset_deg'] == offset
    assert node.explore_plan['offset_deg'] == offset
    assert rank[0]['terms']['access'] == vp.accessibility_score(offset)


def test_19_offset_access_keeps_direct_line_risk(make_node):
    node = make_node()
    fresh_live(node)
    confirm(node, 2.5, 0.0, 0.85)
    tr = node.victim_memory.tracks[0]
    inside(node, state=State.STOP_AND_SCAN)
    beside = [{'x': 1.0, 'y': -0.75, 'range': 1.25, 'bearing': -0.64, 'width': 0.04}]
    node.narrow_plan = list(beside)
    direct, res_d = node._victim_assess(node._entry_pose(), profile_with([]), tr)
    node.narrow_plan = list(beside)
    offset, res_o = node._victim_assess(node._entry_pose(), profile_with(DIRECT_BLOCKED), tr)
    assert res_d == ['+0:ok'] and res_o == ['+0:corridor_blocked', '+10:ok']
    assert direct.access_offset_deg == 0.0 and offset.access_offset_deg == 10.0
    # Risk is the direct-line risk in both cases: 0.75 m beside the line, keep-out 0.63.
    assert direct.risk == pytest.approx(1.0 - (0.75 - 0.63) / 0.30, abs=1e-6)
    assert offset.risk == pytest.approx(direct.risk, abs=1e-12)
    # So the offset candidate differs only by the accessibility term: 0.30 * (1 - 0.75).
    assert direct.score - offset.score == pytest.approx(0.075, abs=1e-6)


def test_19_risk_counts_obstacle_near_robot_in_any_direction():
    keep = 0.63
    # Segment distance from the robot's own position: an obstacle 0.70 m BEHIND counts.
    behind = vp.risk_score([(-0.70, 0.0)], 2.0, 0.0, keep)
    assert behind == pytest.approx(1.0 - (0.70 - keep) / 0.30)
    assert vp.risk_score([(-0.70, 0.0)], 0.0, 2.0, keep) == pytest.approx(behind)
    assert vp.risk_score([(-1.0, 0.0)], 2.0, 0.0, keep) == 0.0
    # Bounded in [0, 1] for any placement.
    for ox in (-2.0, -0.5, 0.0, 0.5, 1.0, 2.0, 3.0):
        for oy in (-1.0, -0.63, 0.0, 0.3, 0.93, 2.0):
            r = vp.risk_score([(ox, oy)], 2.0, 0.0, keep)
            assert 0.0 <= r <= 1.0


def test_20_abandoned_victim_exclusion(make_node):
    node = make_node()
    events = recorder(node)
    confirm(node, 2.0, 0.0, 0.90)                    # A
    assert decide(node) and node.victim_target['track'].id == 1
    ax, ay = node.victim_target['track'].x, node.victim_target['track'].y
    node._victim_abandon(node._now_sec(), node._entry_pose(), 'test')
    assert node.victim_abandoned == [(ax, ay)]
    confirm(node, 2.0, 0.0, 0.90)                    # A seen again: never retried
    confirm(node, 2.0, 25.0, 0.95)                   # B within 1.0 m of A: excluded too
    bx, by = node.victim_memory.tracks[-1].x, node.victim_memory.tracks[-1].y
    assert math.hypot(bx - ax, by - ay) < 2.0 * node.p.victim_confirm_radius_m
    confirm(node, 2.0, 90.0, 0.70)                   # C far from A: selectable
    assert decide(node)
    rec = [e for e in events if e['event'] == 'prioritized'][-1]
    assert len(rec['ranking']) == 1
    tc = node.victim_target['track']
    assert math.hypot(tc.x - ax, tc.y - ay) > 2.0 * node.p.victim_confirm_radius_m
    assert tc.mean_confidence() == pytest.approx(0.70)


def test_21_quiet_assessment_leaves_keepout_stats_unchanged(make_node):
    node = make_node()
    node._narrow_on_structure = lambda x, y: True    # every live narrow detection is counted
    laser_with_post(node, 1.5, -1.0)                 # post seen by the offset live check
    confirm(node, 2.5)
    tr = node.victim_memory.tracks[0]
    inside(node, state=State.STOP_AND_SCAN)
    sent = []
    node.cmd_publisher.publish = sent.append
    escapes = node.keepout_escapes
    before = dict(node.keepout_stats)
    c, res = node._victim_assess(node._entry_pose(), profile_with(DIRECT_BLOCKED), tr)
    assert res == ['+0:corridor_blocked', '+10:ok'] and c.access_offset_deg == 10.0
    assert node.keepout_stats == before               # M2: diagnostics untouched
    assert not sent and node.state == State.STOP_AND_SCAN and node.explore_plan is None
    assert node.keepout_escapes == escapes
    # Sensitivity: the same live detection does count outside the quiet assessment.
    node._narrow_from_live_scan()
    assert node.keepout_stats['structure_filtered'] == before['structure_filtered'] + 1


def test_21_decision_counts_only_the_approach_own_live_check(make_node):
    node = make_node()
    node._narrow_on_structure = lambda x, y: True
    deltas = {}
    for prioritizing in (False, True):               # same decision, prioritizer off / on
        node.victim_prioritizing = prioritizing
        node.victim_target, node.explore_plan = None, None
        node.victim_memory.clear()
        laser_with_post(node, 1.5, -1.0)
        confirm(node, 2.5)
        before = node.keepout_stats['structure_filtered']
        assert decide(node, profile_with(DIRECT_BLOCKED))
        assert node.explore_plan['offset_deg'] == 10.0
        deltas[prioritizing] = node.keepout_stats['structure_filtered'] - before
    # The approach's own live check and pre-rotation check count; scoring adds nothing.
    assert deltas[False] == 2 and deltas[True] == deltas[False]
