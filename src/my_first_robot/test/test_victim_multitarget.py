"""
Multi-target navigation input (B3): victim_input = 'localizations'.

autonomous_search reads per-person /victim_localizations (victim_localizer, RVC on)
instead of the one-person-per-frame /victim_distance. Pure parser tests plus node tests
driven by direct calls (no simulator). Navigation identity = the spatial VictimMemory
track; the localizer's track_id is advisory and only recorded.
"""

import json
import math

from my_first_robot.autonomous_search import (
    AutonomousSearch, Pose2D, State, victim_localization_readings)
import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

SEP = 1.0


def loc(x, y, conf=0.9, idx=0, tid=None, status='localized', usable=True):
    return {'index': idx, 'status': status, 'usable': usable, 'x_m': x, 'y_m': y,
            'distance_m': math.hypot(x + 0.032, y), 'confidence': conf, 'track_id': tid}


def frame(*people, age=0.3, rvc=True):
    return {'schema': 'victim_localizations/1', 'image_age_s': age, 'rvc_enabled': rvc,
            'localizations': [dict(p, index=i) for i, p in enumerate(people)]}


def msg(d):
    m = String()
    m.data = json.dumps(d)
    return m


@pytest.fixture
def make_node():
    made = []

    def make(victim_input='localizations', **extra):
        params = {'victim_approach_enabled': 'true', 'victim_prioritization_enabled': 'true',
                  'victim_input': victim_input}
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


def live_scan(node):
    m = LaserScan()
    m.angle_min, m.angle_increment = 0.0, 2 * math.pi / 360
    m.angle_max = m.angle_min + 359 * m.angle_increment
    m.range_min, m.range_max = 0.12, 3.5
    m.ranges = [3.0] * 360
    m.header.stamp = node.get_clock().now().to_msg()
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


def events(node):
    got = []
    node._victim_record = got.append
    return got


def decide(node, data=None):
    inside(node, state=State.STOP_AND_SCAN)
    return node._victim_after_scan(node._now_sec(), node._entry_pose(), data or scan_data())


A = (2.0, 0.8)       # person A in the base frame (x forward, y left)
B = (2.4, -0.9)      # person B, 1.75 m from A


def two(node, n=3, a=A, b=B, conf_a=0.9, conf_b=0.85):
    inside(node)
    for _ in range(n):
        node.victim_localizations_callback(msg(frame(loc(*a, conf_a, tid=4), loc(*b, conf_b, tid=5))))


# ================================================================ parser
def test_parser_keeps_distinct_people_and_advisory_ids():
    r, w = victim_localization_readings(frame(loc(*A, tid=4), loc(*B, tid=5)), SEP)
    assert len(r) == 2 and w == []
    assert {x['track_id'] for x in r} == {4, 5}
    assert r[0]['angle_rad'] == pytest.approx(math.atan2(A[1], A[0]))


@pytest.mark.parametrize('bad,why', [
    (dict(frame(loc(*A)), image_age_s=1.5), 'stale_image'),
    (dict(frame(loc(*A)), image_age_s=float('nan')), 'stale_image'),
    (dict(frame(loc(*A)), rvc_enabled=False), 'rvc_disabled'),
    ({'localizations': []}, 'malformed'),
    ('not a dict', 'malformed'),
])
def test_parser_rejects_bad_frames(bad, why):
    r, w = victim_localization_readings(bad, SEP)
    assert r == [] and w == [(None, why)]


def test_parser_withholds_close_pair_both():
    r, w = victim_localization_readings(frame(loc(2.0, 0.2), loc(2.1, -0.5)), SEP)   # 0.71 m
    assert r == []
    assert sorted(w) == [(0, 'ambiguous_close_pair'), (1, 'ambiguous_close_pair')]


def test_parser_close_pair_counts_low_confidence_neighbour():
    # The neighbour is below the confidence gate but still a person close by: both withheld.
    r, w = victim_localization_readings(frame(loc(2.0, 0.2), loc(2.1, -0.5, conf=0.4)), SEP)
    assert r == [] and len(w) == 2


def test_parser_separated_pair_accepted():
    r, w = victim_localization_readings(frame(loc(2.0, 0.65), loc(2.0, -0.65)), SEP)  # 1.3 m
    assert len(r) == 2


def test_parser_per_person_gates():
    r, w = victim_localization_readings(frame(
        loc(*A, conf=0.5), loc(*B, status='edge_truncated', usable=False), loc(4.0, 3.0)), SEP)
    assert r == []
    assert dict(w) == {0: 'confidence', 1: 'edge_truncated', 2: 'range'}


# ================================================================ node wiring
def test_default_input_is_distance_only(make_node):
    node = make_node(victim_input='distance')
    topics = [s.topic_name for s in node.subscriptions]
    assert '/victim_distance' in topics and '/victim_localizations' not in topics


def test_localizations_input_replaces_distance(make_node):
    node = make_node()
    topics = [s.topic_name for s in node.subscriptions]
    assert '/victim_localizations' in topics and '/victim_distance' not in topics


def test_invalid_input_name_refused():
    rclpy.init(args=['--ros-args', '-p', 'victim_approach_enabled:=true',
                     '-p', 'victim_input:=both'])
    try:
        with pytest.raises(ValueError):
            AutonomousSearch()
    finally:
        rclpy.shutdown()


# ================================================================ tracks
def test_two_people_get_two_distinct_tracks(make_node):
    node = make_node()
    two(node)
    tracks = node.victim_memory.tracks
    assert len(tracks) == 2
    assert tracks[0].id != tracks[1].id                     # not a constant placeholder
    assert all(t.count == 3 for t in tracks)
    assert math.hypot(tracks[0].x - tracks[1].x, tracks[0].y - tracks[1].y) > 1.5


def test_one_persons_readings_do_not_move_the_other(make_node):
    node = make_node()
    two(node)
    ta = min(node.victim_memory.tracks, key=lambda t: math.hypot(t.x - A[0], t.y - A[1]))
    before = (ta.x, ta.y, ta.count)
    inside(node)
    for k in range(5):                                       # only B seen, with jitter
        node.victim_localizations_callback(msg(frame(loc(B[0] + 0.05 * k, B[1]))))
    assert (ta.x, ta.y, ta.count) == before


def test_two_readings_of_one_frame_never_join_one_track(make_node):
    node = make_node(victim_multi_min_separation_m=0.5)
    ev = events(node)
    inside(node)
    node.victim_localizations_callback(msg(frame(loc(2.0, 0.0))))     # existing track at y 0
    # Next frame: two people 0.6 m apart, both within 0.5 m of that track.
    node.victim_localizations_callback(msg(frame(loc(2.0, 0.3), loc(2.0, -0.3))))
    assert node.victim_memory.tracks[0].count == 1
    assert sum(e['event'] == 'reading_withheld_same_track' for e in ev) == 2


def test_tracks_expire_and_reappearing_person_policy(make_node):
    node = make_node()
    two(node, n=1)
    first_ids = sorted(t.id for t in node.victim_memory.tracks)
    t_old = node.victim_memory.tracks[0].last_time
    # Reappears within the 30 s lifetime -> same spatial track.
    tr = node.victim_memory.observe(t_old + 10.0, A[0] - 0.032 + 0.0, A[1], 0.9, 2.1)
    assert tr.id in first_ids
    # Unseen for > 30 s -> expired; reappearing then starts a NEW track id.
    node.victim_memory.prune(t_old + 45.0)
    assert node.victim_memory.tracks == []
    tr2 = node.victim_memory.observe(t_old + 45.0, A[0], A[1], 0.9, 2.1)
    assert tr2.id not in first_ids


def test_withheld_or_stale_frames_create_nothing(make_node):
    node = make_node()
    inside(node)
    for d in (dict(frame(loc(*A)), image_age_s=2.0), dict(frame(loc(*A)), rvc_enabled=False),
              frame(loc(2.0, 0.2), loc(2.1, -0.5))):
        node.victim_localizations_callback(msg(d))
    assert node.victim_memory.tracks == []
    assert node.victim_stats['loc_withheld_ambiguous_close_pair'] == 2


def test_rotating_frames_become_cues_only(make_node):
    node = make_node()
    inside(node, state=State.STOP_AND_SCAN)
    node.entry_phase = 'ROTATE'
    node.victim_localizations_callback(msg(frame(loc(*A), loc(*B))))
    assert node.victim_memory.tracks == [] and len(node.victim_cues.tracks) == 2


# ================================================================ prioritizer
def test_two_valid_candidates_reach_the_prioritizer_together(make_node):
    node = make_node()
    ev = events(node)
    two(node)
    assert decide(node)
    rank = [e for e in ev if e['event'] == 'prioritized'][-1]
    assert len(rank['ranking']) == 2
    assert len({c['id'] for c in rank['ranking']}) == 2
    assert rank['chosen'] in {c['id'] for c in rank['ranking']}


def test_selected_target_keeps_its_own_track(make_node):
    node = make_node()
    two(node, conf_a=0.95, conf_b=0.65)
    decide(node)
    target = node.victim_target['track']
    other = node.victim_memory.tracks[0]
    tx, ty, tc = target.x, target.y, target.count
    oc = other.count
    two(node, n=2)
    assert target.count == tc + 2 and other.count == oc + 2
    assert math.hypot(target.x - tx, target.y - ty) < 0.05


def test_switch_follows_existing_margin_before_first_move(make_node):
    node = make_node()
    ev = events(node)
    two(node, conf_a=0.80, conf_b=0.78)
    decide(node)
    first = node.victim_target['track'].id
    node.victim_target['moves'] = 0
    two(node, n=1, conf_a=0.80, conf_b=0.79)                 # small change: no switch
    decide(node)
    assert node.victim_target['track'].id == first
    assert not any(e['event'] == 'switched' for e in ev)


def test_no_switch_once_approach_started(make_node):
    node = make_node()
    ev = events(node)
    two(node, conf_a=0.62, conf_b=0.61)
    decide(node)
    node.victim_target['moves'] = 1                          # approach under way
    first = node.victim_target['track'].id
    for _ in range(6):
        inside(node)
        node.victim_localizations_callback(msg(frame(loc(*A, 0.62), loc(B[0] - 1.2, B[1], 1.0))))
    decide(node)
    assert node.victim_target is None or node.victim_target['track'].id == first
    assert not any(e['event'] == 'switched' for e in ev)
