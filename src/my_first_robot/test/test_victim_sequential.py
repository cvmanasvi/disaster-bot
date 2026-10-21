"""
Sequential search (B1) and pending unreachable victims (B2) in autonomous_search.

Node driven by direct calls (no simulator), as in test_victim_approach.py and
test_victim_prioritization.py. Both features are off by default: the first tests
pin the unchanged single-victim behaviour.
"""

import json
import math

from my_first_robot.autonomous_search import AutonomousSearch, Pose2D, State
import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

STOP = 0.80
BLOCK_AHEAD = [(0.10, -0.22), (0.10, 0.22)]      # rejects all 5 headings toward +x


@pytest.fixture
def make_node():
    made = []

    def make(sequential=True, pending=True, prioritize=True, **extra):
        params = {'victim_approach_enabled': 'true',
                  'victim_prioritization_enabled': 'true' if prioritize else 'false',
                  'victim_sequential_search_enabled': 'true' if sequential else 'false',
                  'victim_pending_enabled': 'true' if pending else 'false',
                  'explore_max_steps': '10'}
        params.update({k: str(v) for k, v in extra.items()})
        args = ['--ros-args']
        for k, v in params.items():
            args += ['-p', f'{k}:={v}']
        rclpy.init(args=args)
        n = AutonomousSearch()
        n.p.scan_record_dir = ''
        made.append(n)
        return n
    yield make
    for n in made:
        n.destroy_node()
    rclpy.shutdown()


def reading(distance, angle_deg=0.0, confidence=0.9, age=0.05):
    m = String()
    m.data = json.dumps({'victim_id': 1, 'class': 'person', 'confidence': confidence,
                         'distance_m': distance, 'camera_x': 160.0, 'angle_deg': angle_deg,
                         'detection_age_s': age})
    return m


def live_scan(node):
    m = LaserScan()
    m.angle_min, m.angle_increment = 0.0, 2 * math.pi / 360
    m.angle_max = m.angle_min + 359 * m.angle_increment
    m.range_min, m.range_max = 0.12, 3.5
    m.ranges = [3.0] * 360
    m.header.stamp = node.get_clock().now().to_msg()             # fresh for the live check
    node.latest_scan = m
    node.latest_scan_stamp = m.header.stamp


def inside(node, x=0.0, y=0.0, yaw=0.0, state=State.EXPLORE_MOVE):
    live_scan(node)
    node.mission_phase = 'INSIDE'
    node.nav_pose = Pose2D(x, y, yaw)
    node.imu_yaw = yaw
    node.imu_history.clear()
    for k in range(13):
        node.imu_history.append((0.1 * k, yaw))
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


def feed(node, distance, angle_deg=0.0, n=3, x=0.0, y=0.0, yaw=0.0, conf=0.9):
    inside(node, x=x, y=y, yaw=yaw)
    for _ in range(n):
        node.victim_callback(reading(distance, angle_deg, conf))


def decide(node, data=None, x=0.0, y=0.0, yaw=0.0, now=None):
    inside(node, x=x, y=y, yaw=yaw, state=State.STOP_AND_SCAN)
    return node._victim_after_scan(node._now_sec() if now is None else now,
                                   node._entry_pose(), data or scan_data())


def events(node):
    got = []
    node._victim_record = got.append
    return got


def hold_at(node, distance=0.82, x=0.0, y=0.0):
    feed(node, distance, x=x, y=y)
    assert decide(node, x=x, y=y)
    assert node.state == State.VICTIM_HOLD
    node.victim_hold['facing'] = False
    return node.victim_hold['start']


# ================================================================ defaults unchanged
def test_defaults_are_off_and_hold_is_terminal(make_node):
    node = make_node(sequential=False, pending=False)
    assert node.victim_sequential is False and node.victim_pending_on is False
    assert 'rejected_found' not in node.victim_stats
    start = hold_at(node)
    for dt in (5.0, 30.0, 300.0):
        node.handle_victim_hold(None, start + dt)
    assert node.state == State.VICTIM_HOLD and node.victim_report is not None
    assert node.victims_found == []
    assert 'found_index' not in node.victim_report


def test_feature_params_have_safe_defaults():
    rclpy.init(args=['--ros-args', '-p', 'victim_approach_enabled:=true'])
    try:
        n = AutonomousSearch()
        assert n.p.victim_sequential_search_enabled is False
        assert n.p.victim_pending_enabled is False
        assert n.p.victim_found_exclusion_m == pytest.approx(0.60)
        assert n.victim_sequential is False and n.victim_pending_on is False
        n.destroy_node()
    finally:
        rclpy.shutdown()


def test_sequential_needs_victim_approach():
    rclpy.init(args=['--ros-args', '-p', 'victim_sequential_search_enabled:=true',
                     '-p', 'victim_pending_enabled:=true'])
    try:
        n = AutonomousSearch()
        assert n.victim_sequential is False and n.victim_pending_on is False
        n.destroy_node()
    finally:
        rclpy.shutdown()


# ================================================================ B1 sequential search
def test_hold_marks_found_and_report_has_index(make_node):
    node = make_node()
    ev = events(node)
    hold_at(node)
    assert len(node.victims_found) == 1
    assert node.victim_report['found_index'] == 1
    assert any(e['event'] == 'victim_marked_found' for e in ev)


def test_hold_waits_duration_then_resumes_with_scan(make_node):
    node = make_node()
    ev = events(node)
    start = hold_at(node)
    sent = []
    node.cmd_publisher.publish = sent.append
    node.handle_victim_hold(None, start + 5.0)              # < 10 s: still holding
    assert node.state == State.VICTIM_HOLD
    assert all(m.twist.linear.x == 0.0 and m.twist.angular.z == 0.0 for m in sent)
    node.handle_victim_hold(None, start + 10.5)
    assert node.state == State.STOP_AND_SCAN                 # fresh scan, existing checks
    assert node.victim_report is None and node.victim_target is None
    assert node.victim_hold is None
    assert any(e['event'] == 'search_resumed' for e in ev)
    assert all(m.twist.linear.x == 0.0 for m in sent)      # resuming never drives forward


def test_resume_waits_for_facing_turn(make_node):
    node = make_node()
    start = hold_at(node)
    node.victim_hold.update({'facing': True, 'heading': math.radians(40.0)})
    node.latest_imu_stamp = node.get_clock().now().to_msg()
    node.handle_victim_hold(None, start + 11.0)              # still turning to face the person
    assert node.state == State.VICTIM_HOLD


def test_found_victim_not_retargeted_under_drift(make_node):
    node = make_node()
    hold_at(node)                                            # found at nav (0.788, 0)
    node.handle_victim_hold(None, node.victim_hold['start'] + 11.0)
    fx = node.victims_found[0]['x']
    # Same person re-seen from elsewhere with 0.25 m and 0.5 m drift: ignored.
    for dx, dy in ((0.25, 0.0), (0.0, 0.45), (-0.3, 0.3)):
        before = node.victim_stats['rejected_found']
        feed(node, 1.0, n=3, x=fx + dx - 1.0 + 0.032, y=dy)
        assert node.victim_stats['rejected_found'] == before + 3
    assert node.victim_memory.tracks == []
    assert decide(node) is False                             # nothing to approach


def test_nearby_distinct_person_beyond_exclusion_is_tracked(make_node):
    node = make_node()
    hold_at(node)
    node.handle_victim_hold(None, node.victim_hold['start'] + 11.0)
    fx, fy = node.victims_found[0]['x'], node.victims_found[0]['y']
    # A second person 0.9 m from the found one (outside the 0.60 m exclusion).
    feed(node, 2.0, angle_deg=math.degrees(math.atan2(fy + 0.9, fx + 1.2)), n=3)
    assert len(node.victim_memory.tracks) == 1
    tr = node.victim_memory.tracks[0]
    assert math.hypot(tr.x - fx, tr.y - fy) > node.p.victim_found_exclusion_m


def test_second_distinct_victim_found_and_standoff_kept(make_node):
    node = make_node()
    hold_at(node)
    node.handle_victim_hold(None, node.victim_hold['start'] + 11.0)
    # Second person straight ahead of a new pose, 2.0 m away.
    feed(node, 2.0, n=3, x=0.0, y=3.0, yaw=0.0)
    assert decide(node, x=0.0, y=3.0)                        # approach move started
    assert node.explore_plan['sector'] == 'VICTIM'
    # one bounded step, never past the 0.80 m stop distance
    assert 0.0 < node.explore_plan['target'] <= 2.0 - STOP + 1e-6
    track = node.victim_target['track']
    # After the move: within the stop distance, seen -> hold #2.
    feed(node, 0.81, n=1, x=1.2, y=3.0)
    assert track.count >= 4
    assert decide(node, x=1.2, y=3.0)
    assert node.state == State.VICTIM_HOLD
    assert len(node.victims_found) == 2 and node.victim_report['found_index'] == 2
    a, b = node.victims_found
    assert math.hypot(a['x'] - b['x'], a['y'] - b['y']) > 2.0
    assert a['id'] != b['id']


def test_budget_exhausted_makes_hold_final(make_node):
    node = make_node()
    ev = events(node)
    start = hold_at(node)
    node.explore_step = node.p.explore_max_steps
    node.handle_victim_hold(None, start + 11.0)
    assert node.state == State.VICTIM_HOLD and node.victim_final
    assert any(e['event'] == 'final_hold' and 'budget' in e['reason'] for e in ev)
    node.handle_victim_hold(None, start + 500.0)
    assert node.state == State.VICTIM_HOLD


def test_max_found_makes_hold_final(make_node):
    node = make_node(victim_max_found=1)
    start = hold_at(node)
    node.handle_victim_hold(None, start + 11.0)
    assert node.state == State.VICTIM_HOLD and node.victim_final


def test_exploration_budget_still_terminates_after_resume(make_node):
    node = make_node()
    start = hold_at(node)
    node.handle_victim_hold(None, start + 11.0)
    node.explore_step = node.p.explore_max_steps
    node.scan_data = dict(scan_data(), result={'complete_360': True, 'rotation_deg': 360,
                                               'duration_s': 1, 'valid_rays': 360,
                                               'total_rays': 360, 'bins_with_returns': 36})
    node.p.explore_planner = 'sector'                       # skip the grid stop check
    node.explore_move = None
    node._explore_after_scan(node._now_sec())
    assert node.state == State.INSIDE_HOLD


@pytest.mark.parametrize('payload', [
    {'distance_m': 2.0, 'angle_deg': 0.0, 'confidence': 0.9, 'detection_age_s': 0.9},   # stale
    {'distance_m': float('nan'), 'angle_deg': 0.0, 'confidence': 0.9},
    {'distance_m': 2.0, 'angle_deg': 0.0, 'confidence': 0.3},
    {'angle_deg': 0.0, 'confidence': 0.9}])
def test_bad_measurements_after_resume_rejected(make_node, payload):
    node = make_node()
    start = hold_at(node)
    node.handle_victim_hold(None, start + 11.0)
    inside(node)
    before = node.victim_stats['rejected_invalid']
    m = String()
    m.data = json.dumps(payload)
    node.victim_callback(m)
    assert node.victim_stats['rejected_invalid'] == before + 1
    assert node.victim_memory.tracks == []


def test_readings_during_hold_are_ignored(make_node):
    node = make_node()
    hold_at(node)
    inside(node, state=State.VICTIM_HOLD)
    node.victim_report = node.victim_report or {}
    before = node.victim_stats['rejected_state']
    node.victim_callback(reading(1.5, 30.0))
    assert node.victim_stats['rejected_state'] == before + 1


# ================================================================ B2 pending unreachable
def test_blocked_trusted_target_becomes_pending_and_exploration_continues(make_node):
    node = make_node()
    ev = events(node)
    feed(node, 2.0, n=6)
    assert decide(node, profile_with(BLOCK_AHEAD)) is False     # explore as usual
    assert len(node.victim_pending) == 1 and node.victim_target is None
    assert node.victim_memory.tracks == []                       # moved, not duplicated
    assert any(e['event'] == 'pending_added' for e in ev)
    assert node.victim_abandoned == []                           # not abandoned


def test_untrusted_blocked_track_is_not_pending(make_node):
    node = make_node()
    feed(node, 2.0, n=3)                                         # < 6 readings
    assert decide(node, profile_with(BLOCK_AHEAD)) is False
    assert node.victim_pending == []


def test_pending_disabled_keeps_old_behaviour(make_node):
    node = make_node(pending=False)
    feed(node, 2.0, n=6)
    assert decide(node, profile_with(BLOCK_AHEAD)) is False
    assert node.victim_pending == []
    assert len(node.victim_memory.tracks) == 1                   # expires as before


def test_temporarily_blocked_then_reachable_is_approached(make_node):
    node = make_node()
    ev = events(node)
    feed(node, 2.0, n=6)
    decide(node, profile_with(BLOCK_AHEAD))
    feed(node, 2.0, n=1)                                         # still seen while pending
    assert node.victim_pending[0]['track'].count == 7
    assert decide(node, scan_data())                             # route clear now
    assert node.victim_target is not None and 'revived' in node.victim_target
    assert node.explore_plan['sector'] == 'VICTIM'
    assert 0.0 < node.explore_plan['target'] <= 2.0 - STOP + 1e-6
    assert any(e['event'] == 'pending_revived' for e in ev)


def test_safe_alternate_heading_used_for_revived_target(make_node):
    node = make_node()
    feed(node, 2.0, n=6)
    decide(node, profile_with(BLOCK_AHEAD))
    feed(node, 2.0, n=1)
    # Only the direct heading is blocked now: an offset heading passes every check.
    assert decide(node, profile_with([(0.6, -0.15)]))            # blocker on the right
    assert node.explore_plan['offset_deg'] == pytest.approx(10.0)


def test_revived_but_not_seen_again_is_never_held(make_node):
    node = make_node()
    ev = events(node)
    feed(node, 2.0, n=6)
    decide(node, profile_with(BLOCK_AHEAD))
    assert len(node.victim_pending) == 1
    # Later at a pose 0.80 m from it with a clear route, but no reading since it became
    # pending: never held on old evidence.
    assert decide(node, scan_data(), x=1.2) is False
    assert node.state != State.VICTIM_HOLD and node.victim_report is None
    assert any(e['event'] == 'pending_requeued' for e in ev)
    assert len(node.victim_pending) == 1


def test_revived_and_seen_again_is_held(make_node):
    node = make_node()
    feed(node, 2.0, n=6)
    decide(node, profile_with(BLOCK_AHEAD))
    feed(node, 0.80, n=1, x=1.2)                                 # seen again while pending
    assert decide(node, scan_data(), x=1.2)
    assert node.state == State.VICTIM_HOLD


def test_persistently_blocked_pending_expires_after_max_checks(make_node):
    node = make_node(victim_pending_max_checks=3)
    ev = events(node)
    feed(node, 2.0, n=6)
    for _ in range(5):
        feed(node, 2.0, n=1)                                     # keep it fresh
        decide(node, profile_with(BLOCK_AHEAD))
    assert node.victim_pending == []
    assert any(e['event'] == 'pending_expired' and 'assessments' in e['reason'] for e in ev)
    assert node.state != State.VICTIM_HOLD


def test_stale_pending_target_rejected(make_node):
    node = make_node(victim_pending_timeout_sec=60.0)
    ev = events(node)
    feed(node, 2.0, n=6)
    decide(node, profile_with(BLOCK_AHEAD))
    later = node._now_sec() + 61.0
    assert decide(node, scan_data(), now=later) is False         # clear, but stale -> dropped
    assert node.victim_pending == [] and node.victim_target is None
    assert any(e['event'] == 'pending_expired' and 'not seen' in e['reason'] for e in ev)


def test_pending_does_not_monopolize_exploration(make_node):
    node = make_node()
    feed(node, 2.0, n=6)
    for _ in range(3):
        assert decide(node, profile_with(BLOCK_AHEAD)) is False  # each decision -> frontier
    assert node.victim_target is None


def test_pending_near_found_victim_dropped(make_node):
    node = make_node()
    feed(node, 2.0, n=6, y=0.0)
    decide(node, profile_with(BLOCK_AHEAD))
    tr = node.victim_pending[0]['track']
    node.victims_found.append({'x': tr.x + 0.2, 'y': tr.y, 'id': 99, 't': 0.0, 'report': {}})
    node._victim_pending_prune(node._now_sec())
    assert node.victim_pending == []


def test_two_pending_people_keep_distinct_tracks(make_node):
    node = make_node()
    feed(node, 2.0, angle_deg=20.0, n=6)
    feed(node, 2.0, angle_deg=-20.0, n=6)
    blocked = profile_with([(0.10, y) for y in (-0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4)])
    decide(node, blocked)
    ids = sorted(e['track'].id for e in node.victim_pending)
    assert len(ids) == 2 and ids[0] != ids[1]
    a, b = (e['track'] for e in node.victim_pending)
    assert math.hypot(a.x - b.x, a.y - b.y) > 1.0
