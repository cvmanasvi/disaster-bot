"""Navigation-facing protection: detection guard -> frozen estimator -> autonomous_search."""

import json
import math
import os

from builtin_interfaces.msg import Time
from my_first_robot import victim_detection_guard as guard_mod
from my_first_robot import victim_distance_estimator as est_mod
from my_first_robot import victim_localizer_core as core
from my_first_robot.autonomous_search import victim_measurement, VictimMemory
import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import test_victim_localizer_rvc as W

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GUARD = os.path.join(PKG, 'my_first_robot', 'victim_detection_guard.py')
LAUNCH = os.path.join(PKG, 'launch', 'disaster_bot_posture.launch.py')


def det(b, conf=0.9, stamp=100.0):
    """Return a /victim_detections box in the detector's schema."""
    return {'victim_id': 1, 'class': 'person', 'confidence': conf,
            'bbox': {'x1': b[0], 'y1': b[1], 'x2': b[2], 'y2': b[3]},
            'center': {'x': (b[0] + b[2]) / 2, 'y': (b[1] + b[3]) / 2},
            'timestamp': 1000.0, 'image_stamp': stamp}


def guard(dets, scan, age=0.2):
    return core.guard_detections(dets, scan, age)


def statuses(decisions):
    return [d.get('reason', d['status']) for d in decisions]


# ------------------------------------------------------------------ scenes (LiDAR frame)
def standing(r=2.0, y=0.0):
    return [det(W.box(r, y))], W.make_scan(W.legs(r, y))


def behind_lying():
    lying = W.body(1.5, 0.0, math.pi / 2)
    b = W.box(2.4, 0.0, y2=W.image_row(1.5, 0.0, 0.25))
    return [det(b)], W.make_scan(lying + W.legs(2.4, 0.0))


def behind_debris():
    scan = W.make_scan(W.legs(2.5, 0.0), segments=[((1.7, -0.6), (1.7, 0.6))])
    return [det(W.box(2.5, 0.0, y2=W.image_row(1.7, 0.0, 0.45)))], scan


def behind_beam():
    scan = W.make_scan(W.legs(2.8, 0.0), segments=[((2.2, -1.2), (2.2, 1.2))])
    return [det(W.box(2.8, 0.0, y2=W.image_row(2.2, 0.0, 0.40)))], scan


def debris_beside():
    b = W.box(2.0, 0.0)
    g = core.candidate_geometry({'bbox': b})
    edge = g['unpadded'][1] + math.radians(1.0)
    obj = (1.7 * math.cos(edge), 1.7 * math.sin(edge), 0.05)
    return [det(b)], W.make_scan(W.legs(2.0, 0.0) + [obj])


# ------------------------------------------------------------------ A-I, K, L, P: guard decisions
def test_a_valid_standing_person_forwarded_verbatim():
    dets, scan = standing()
    fwd, dec = guard(dets, scan)
    assert fwd == dets and fwd[0] is dets[0]
    assert statuses(dec) == ['forwarded']


@pytest.mark.parametrize('scene,reason', [(behind_lying, 'lidar_hit_below_box'),
                                          (behind_debris, 'lidar_hit_below_box'),
                                          (behind_beam, 'lidar_hit_below_box'),
                                          (debris_beside, 'no_central_support')])
def test_b_to_e_unreliable_associations_are_withheld_with_reason(scene, reason):
    dets, scan = scene()
    # without the guard the frozen rule would measure something (the occluder)
    assert core.localize_frame(dets, scan)[0]['status'] == core.LOCALIZED
    fwd, dec = guard(dets, scan)
    assert fwd == [] and statuses(dec) == [reason]
    assert 'rvc' in dec[0] and 'distance_m' not in dec[0]


def test_f_two_people_at_different_distances_both_forwarded():
    dets = [det(W.box(1.2, -0.4)), det(W.box(2.8, 0.8), conf=0.8)]
    scan = W.make_scan(W.legs(1.2, -0.4) + W.legs(2.8, 0.8))
    fwd, dec = guard(dets, scan)
    assert fwd == dets and statuses(dec) == ['forwarded', 'forwarded']


@pytest.mark.parametrize('sep', [0.45, 0.6])
def test_g_close_pair_forwarded_like_the_frozen_single_box_rule(sep):
    a, b = (1.8, sep / 2), (1.8, -sep / 2)
    dets = [det(W.box(*a)), det(W.box(*b), conf=0.8)]
    scan = W.make_scan(W.legs(*a) + W.legs(*b))
    fwd, dec = guard(dets, scan)
    single = [core.localize_frame([d], scan)[0]['status'] == core.LOCALIZED for d in dets]
    assert [d['status'] == 'forwarded' for d in dec] == single


def test_h_empty_frame_forwards_nothing():
    assert guard([], W.make_scan([])) == ([], [])


def test_i_detected_but_withheld_is_visible_in_decisions():
    dets, scan = behind_debris()
    fwd, dec = guard(dets, scan)
    assert fwd == [] and len(dec) == 1 and dec[0]['status'] == 'withheld'


@pytest.mark.parametrize('age,ok', [(0.2, True), (1.0, True), (1.0001, False), (5.0, False),
                                    (-0.4, True), (-0.6, False), (None, False),
                                    (float('nan'), False)])
def test_k_stale_or_unstamped_images_are_withheld(age, ok):
    dets, scan = standing()
    fwd, dec = guard(dets, scan, age)
    assert bool(fwd) is ok
    if not ok:
        assert statuses(dec)[0] in ('stale_image', 'no_image_stamp')


def test_l_missing_scan_and_malformed_inputs():
    dets, _ = standing()
    assert statuses(guard(dets, None)[1]) == ['no_fresh_scan']
    assert guard('nope', W.make_scan([]))[0] == []
    fwd, dec = guard([{'bbox': 'x'}, dets[0]], standing()[1])
    assert fwd == [dets[0]] and statuses(dec) == ['invalid_candidate', 'forwarded']
    empty = W.make_scan([])
    assert statuses(guard(dets, empty)[1]) == ['no_supported_cluster']


def test_p_one_candidate_withheld_other_forwarded():
    scan = W.make_scan(W.legs(1.8, 0.8) + W.legs(2.6, -0.6), segments=[((1.9, -1.1), (1.9, -0.1))])
    ok = det(W.box(1.8, 0.8), conf=0.7)
    hidden = det(W.box(2.6, -0.6, y2=W.image_row(1.9, -0.6, 0.45)), conf=0.95)
    fwd, dec = guard([hidden, ok], scan)
    assert fwd == [ok] and statuses(dec) == ['lidar_hit_below_box', 'forwarded']


def _bad(field, value):
    d, _ = standing()
    if value is _DROP:
        d[0].pop(field)
    else:
        d[0][field] = value
    return d[0]


_DROP = object()


@pytest.mark.parametrize('field,value', [
    ('victim_id', _DROP), ('center', _DROP), ('center', {'y': 1.0}), ('center', {'x': 'a'}),
    ('confidence', _DROP), ('confidence', 'high'), ('confidence', float('nan')),
    ('confidence', 1.5)])
def test_n_boxes_the_estimator_cannot_read_are_withheld(field, value):
    # the frozen estimator raises on these and drops the WHOLE frame (incl. valid boxes)
    fwd, dec = guard([_bad(field, value), standing(2.0, 0.0)[0][0]], standing()[1])
    assert len(fwd) == 1 and fwd[0].get(field) == standing()[0][0].get(field)
    assert dec[0]['reason'] in ('estimator_contract', 'invalid_candidate')


def test_n_center_must_match_the_validated_bbox():
    # the estimator measures around center.x; the guard validated the bbox
    dets, scan = standing()
    off = dict(dets[0], center={'x': dets[0]['center']['x'] + 30.0, 'y': 1.0})
    fwd, dec = guard([off], scan)
    assert fwd == [] and dec[0]['reason'] == 'estimator_contract'
    near = dict(dets[0], center={'x': round(dets[0]['center']['x'], 1), 'y': 1.0})
    assert guard([near], scan)[0] == [near]                 # detector rounding (0.1 px)


def test_n_list_bbox_is_withheld_for_the_estimator():
    dets, scan = standing()
    b = dets[0]['bbox']
    lst = dict(dets[0], bbox=[b['x1'], b['y1'], b['x2'], b['y2']])
    assert core.localize_frame([lst], scan)[0]['status'] == core.LOCALIZED   # core accepts it
    fwd, dec = guard([lst], scan)
    assert fwd == [] and dec[0]['reason'] == 'estimator_contract'


def test_n_each_box_must_carry_the_frame_image_stamp():
    dets, scan = standing()
    other = dict(standing(2.0, 0.0)[0][0])
    for stamp in (None, 50.0):
        mixed = [dets[0], dict(other, image_stamp=stamp)]
        fwd, dec = guard(mixed, scan)
        assert fwd == [dets[0]] and dec[1]['reason'] == 'image_stamp_mismatch'


def test_n_malformed_scans_never_forward():
    dets, _ = standing()
    nan_scan = dict(W.make_scan(W.legs(2.0, 0.0)), angle_increment=float('nan'))
    assert guard(dets, nan_scan)[0] == []
    bad = dict(W.make_scan(W.legs(2.0, 0.0)), ranges=['x'] * 10)
    with pytest.raises(ValueError):          # the node turns this into a 'guard_error' frame
        guard(dets, bad)


# ------------------------------------------------- Q: optional close-pair check (default off)
CP = {'exclusive_cluster_check': True}


def test_q_close_pair_check_is_off_by_default():
    assert core.GUARD_DEFAULTS['exclusive_cluster_check'] is False
    dets = [det(W.box(1.8, 0.2)), det(W.box(1.8, -0.2), conf=0.8)]
    scan = W.make_scan(W.legs(1.8, 0.2) + W.legs(1.8, -0.2))
    assert guard(dets, scan) == core.guard_detections(dets, scan, 0.2, {})


@pytest.mark.parametrize('sep', [0.30, 0.35, 0.40])
def test_q_close_pair_mixed_cluster_is_withheld(sep):
    a, b = (1.8, sep / 2), (1.8, -sep / 2)
    dets = [det(W.box(*a)), det(W.box(*b), conf=0.8)]
    scan = W.make_scan(W.legs(*a) + W.legs(*b))
    assert statuses(guard(dets, scan)[1]) == ['forwarded', 'forwarded']   # default: both pass
    fwd, dec = core.guard_detections(dets, scan, 0.2, CP)
    assert fwd == [] and statuses(dec) == ['ambiguous_close_pair'] * 2
    # the single-box cluster the estimator would measure contains the NEIGHBOUR's rays
    rays = {r['i']: r for r in core.scan_rays(scan)}
    for own, other in ((0, 1), (1, 0)):
        single = core.localize_frame([dets[own]], scan)[0]
        span = core.candidate_geometry({'bbox': core._bbox(dets[other]['bbox'])})['unpadded']
        assert any(core._inside(rays[i]['cam_bearing'], span)
                   for i in single['cluster_ray_indices'])


@pytest.mark.parametrize('sep', [0.45, 0.6])
def test_q_close_pair_with_exclusive_clusters_still_forwarded(sep):
    a, b = (1.8, sep / 2), (1.8, -sep / 2)
    dets = [det(W.box(*a)), det(W.box(*b), conf=0.8)]
    scan = W.make_scan(W.legs(*a) + W.legs(*b))
    assert core.guard_detections(dets, scan, 0.2, CP) == guard(dets, scan)


@pytest.mark.parametrize('scene', [lambda: standing(2.0), lambda: standing(1.2, 0.5),
                                   behind_lying, behind_debris, behind_beam, debris_beside])
def test_q_single_person_scenes_unchanged(scene):
    dets, scan = scene()
    assert core.guard_detections(dets, scan, 0.2, CP) == guard(dets, scan)


def test_q_separated_people_both_forwarded():
    dets = [det(W.box(1.2, -0.4)), det(W.box(2.8, 0.8), conf=0.8)]
    scan = W.make_scan(W.legs(1.2, -0.4) + W.legs(2.8, 0.8))
    fwd, dec = core.guard_detections(dets, scan, 0.2, CP)
    assert fwd == dets and statuses(dec) == ['forwarded', 'forwarded']


def test_q_empty_and_frame_level_rules_unchanged():
    assert core.guard_detections([], W.make_scan([]), 0.2, CP) == ([], [])
    dets, scan = standing()
    assert statuses(core.guard_detections(dets, None, 0.2, CP)[1]) == ['no_fresh_scan']
    assert statuses(core.guard_detections(dets, scan, 5.0, CP)[1]) == ['stale_image']


def test_q_check_only_ever_removes_boxes():
    # monotone: with the check on, the forwarded set is a subset of the default forwarded set
    for sep in (0.3, 0.4, 0.45, 0.6, 1.0):
        for dr in (0.0, 0.3):
            a, b = (1.8, sep / 2), (1.8 + dr, -sep / 2)
            dets = [det(W.box(*a)), det(W.box(*b), conf=0.8)]
            scan = W.make_scan(W.legs(*a) + W.legs(*b))
            on = core.guard_detections(dets, scan, 0.2, CP)[0]
            off = guard(dets, scan)[0]
            assert all(any(x is y for y in off) for x in on)


def test_q_node_parameter_parsing():
    for args, expected in (([], False), (['-p', 'exclusive_cluster_check:=true'], True),
                           (['-p', 'exclusive_cluster_check:=false'], False)):
        rclpy.init(args=['--ros-args', *args])
        try:
            g = guard_mod.VictimDetectionGuard()
            assert g.g['exclusive_cluster_check'] is expected
            assert isinstance(g.g['max_image_age_s'], float)
            g.destroy_node()
        finally:
            rclpy.shutdown()


def test_m_guard_does_not_depend_on_the_pose_stream():
    src = open(GUARD).read()
    assert '/victim_posture' not in src and 'ultralytics' not in src
    assert 'cmd_vel' not in src and 'Twist' not in src


# ------------------------------------------------------- end to end with the frozen estimator
def _scan_msg(d, stamp):
    m = LaserScan()
    m.header.stamp = Time(sec=int(stamp), nanosec=int(round((stamp % 1) * 1e9)))
    m.angle_min, m.angle_increment = d['angle_min'], d['angle_increment']
    m.range_min, m.range_max = d['range_min'], d['range_max']
    m.ranges = [float('inf') if r is None else float(r) for r in d['ranges']]
    return m


@pytest.fixture
def chain():
    """Guard node and the UNMODIFIED estimator node, wired by hand, clocks controlled."""
    rclpy.init()
    g = guard_mod.VictimDetectionGuard()
    e = est_mod.VictimDistanceEstimator()
    t = {'now': 100.0}
    g._now = lambda: t['now']
    e._now_sec = lambda: t['now']
    out = {'nav': [], 'audit': [], 'distance': []}
    g.pub.publish = lambda m: (out['nav'].append(json.loads(m.data)), e.detection_callback(m))
    g.audit.publish = lambda m: out['audit'].append(json.loads(m.data))
    e.distance_publisher.publish = lambda m: out['distance'].append(json.loads(m.data))

    def feed(dets, scan, stamp):
        t['now'] = stamp
        msg = _scan_msg(scan, stamp)
        g._scan(msg)
        e.scan_callback(msg)
        for d in dets:
            d['image_stamp'] = stamp - 0.2
        g._detections(String(data=json.dumps(dets)))
    feed.guard, feed.clock = g, t
    yield feed, out, e
    g.destroy_node()
    e.destroy_node()
    rclpy.shutdown()


def test_e2e_valid_standing_distance_is_published_unchanged(chain):
    feed, out, e = chain
    dets, scan = standing(2.0)
    feed(dets, scan, 100.0)
    assert len(out['distance']) == 1
    d = out['distance'][0]
    raw, _ = e.get_lidar_distance(_scan_msg(scan, 100.0), *_span(dets[0], e))
    assert d['distance_m'] == round(raw, 3) and victim_measurement(d) is not None


def _span(dt, e):
    b = dt['bbox']
    lo, hi = est_mod.pixel_to_bearing(b['x2']), est_mod.pixel_to_bearing(b['x1'])
    c = est_mod.pixel_to_bearing(dt['center']['x'])
    gate = max(0.20, e.person_min_width_m / (hi - lo))
    pad = math.radians(2)
    return (min(lo, c - pad) - pad, max(hi, c + pad) + pad, gate)


@pytest.mark.parametrize('scene', [behind_lying, behind_debris, behind_beam, debris_beside])
def test_e2e_unreliable_scenes_publish_no_navigation_distance(chain, scene):
    feed, out, _ = chain
    dets, scan = scene()
    for k in range(5):
        feed([dict(d) for d in dets], scan, 100.0 + 0.2 * k)
    assert out['distance'] == [] and out['nav'] == []
    assert all(a['withheld'] == 1 for a in out['audit'])


def test_e2e_withheld_after_valid_does_not_republish_old_distance(chain):
    feed, out, _ = chain
    dets, scan = standing(2.0)
    feed(dets, scan, 100.0)
    n = len(out['distance'])
    hd, hs = behind_debris()
    for k in range(5):
        feed([dict(d) for d in hd], hs, 100.2 + 0.2 * k)
    assert len(out['distance']) == n                  # nothing new while withheld


def test_e2e_rejected_box_does_not_hide_valid_lower_confidence_person(chain):
    feed, out, _ = chain
    scan = W.make_scan(W.legs(1.8, 0.8) + W.legs(2.6, -0.6), segments=[((1.9, -1.1), (1.9, -0.1))])
    ok = det(W.box(1.8, 0.8), conf=0.7)
    hidden = det(W.box(2.6, -0.6, y2=W.image_row(1.9, -0.6, 0.45)), conf=0.95)
    feed([hidden, ok], scan, 100.0)
    assert len(out['distance']) == 1 and out['distance'][0]['confidence'] == 0.7
    assert abs(out['distance'][0]['distance_m'] - (math.hypot(1.8, 0.8) - 0.06)) < 0.08


def test_e2e_stale_frames_never_reach_the_estimator(chain):
    feed, out, _ = chain
    dets, scan = standing()
    feed(dets, scan, 100.0)
    n_nav, n_dist = len(out['nav']), len(out['distance'])
    feed.clock['now'] = 100.4
    feed.guard._detections(String(data=json.dumps([dict(dets[0], image_stamp=90.0)])))
    assert len(out['nav']) == n_nav and len(out['distance']) == n_dist
    assert out['audit'][-1]['decisions'][0]['reason'] == 'stale_image'


def test_e2e_unreadable_box_does_not_suppress_a_valid_one(chain):
    feed, out, _ = chain
    dets, scan = standing(2.0)
    bad = dict(det(W.box(2.0, 0.0), conf=0.99))
    bad.pop('victim_id')
    feed([bad, dets[0]], scan, 100.0)
    assert len(out['distance']) == 1 and out['distance'][0]['confidence'] == 0.9
    assert out['audit'][-1]['decisions'][0]['reason'] == 'estimator_contract'


def test_e2e_guard_error_forwards_nothing(chain):
    feed, out, _ = chain
    dets, scan = standing(2.0)
    feed.clock['now'] = 100.0
    feed.guard.latest_scan = dict(scan, ranges=['x'] * 10)   # unreadable scan
    feed.guard.latest_scan_time = 100.0
    for d in dets:
        d['image_stamp'] = 99.8
    feed.guard._detections(String(data=json.dumps(dets)))
    assert out['nav'] == [] and out['distance'] == []
    assert out['audit'][-1]['decisions'][0]['reason'].startswith('guard_error')


def test_e2e_estimator_smoothing_state_survives_short_withholding(chain):
    # Documents the FROZEN estimator: the guard can only withhold, not reset its EMA.
    # A withheld (wrong) distance never enters the EMA, but the last valid one is blended
    # into the next valid reading if it arrives within detection_timeout (1 s) and < 0.5 m away.
    feed, out, e = chain
    d1, s1 = standing(2.0)
    feed(d1, s1, 100.0)
    first = e.previous_distance                               # unrounded EMA state
    hd, hs = behind_debris()
    feed([dict(d) for d in hd], hs, 100.2)                    # withheld: estimator hears nothing
    d2, s2 = standing(2.3)
    feed(d2, s2, 100.6)
    raw2, _ = e.get_lidar_distance(_scan_msg(s2, 100.6), *_span(d2[0], e))
    assert out['distance'][-1]['distance_m'] == round(0.35 * raw2 + 0.65 * first, 3)
    e.check_detection_timeout()
    feed.clock['now'] = 101.8                                 # > 1 s without forwarded frames
    e.check_detection_timeout()
    assert e.previous_distance is None
    feed([dict(d) for d in d2], s2, 101.8)
    assert out['distance'][-1]['distance_m'] == round(raw2, 3)


def test_stale_sequence_withheld_then_watchdog_then_fresh(chain):
    # 1 valid published; 2 later candidate rejected; 3 smoothing state stays; 4 ages out on the
    # estimator watchdog (detection_timeout 1.0 s); 5 nothing is (re)published meanwhile;
    # 6 a new valid candidate is published unblended.
    feed, out, e = chain
    d1, s1 = standing(2.0)
    feed(d1, s1, 100.0)                                                       # 1
    assert len(out['distance']) == 1
    raw1 = e.previous_distance
    hd, hs = behind_debris()
    feed([dict(d) for d in hd], hs, 100.2)                                    # 2
    assert out['audit'][-1]['decisions'][0]['reason'] == 'lidar_hit_below_box'
    assert len(out['distance']) == 1 and e.previous_distance == raw1         # 3
    for t in (100.4, 100.6, 100.8, 100.99):                                   # 5: timer ticks
        feed.clock['now'] = t
        e.check_detection_timeout()
        assert e.previous_distance == raw1 and len(out['distance']) == 1
    feed.clock['now'] = 101.01
    e.check_detection_timeout()                                               # 4
    assert e.previous_distance is None and not e.detection_active
    assert len(out['distance']) == 1                                          # 5
    d2, s2 = standing(2.3)
    feed(d2, s2, 101.2)                                                       # 6
    raw2, _ = e.get_lidar_distance(_scan_msg(s2, 101.2), *_span(d2[0], e))
    assert len(out['distance']) == 2 and out['distance'][-1]['distance_m'] == round(raw2, 3)


def test_stale_large_jump_is_not_blended(chain):
    # replacement path 2: a new valid reading > 0.5 m from the smoothed value resets the EMA
    feed, out, e = chain
    feed(*standing(2.0), 100.0)
    feed([dict(d) for d in behind_debris()[0]], behind_debris()[1], 100.2)   # withheld
    d2, s2 = standing(2.7)
    feed(d2, s2, 100.4)
    raw2, _ = e.get_lidar_distance(_scan_msg(s2, 100.4), *_span(d2[0], e))
    assert out['distance'][-1]['distance_m'] == round(raw2, 3)


def test_stale_consumer_age_reads_receipt_not_image(chain):
    # 0.9 s old image passes the guard (limit 1.0 s); the estimator reports detection_age_s 0.0
    # (receipt time), so the consumer's 0.5 s age gate accepts it as a fresh observation.
    feed, out, e = chain
    dets, scan = standing(2.0)
    feed.clock['now'] = 100.0
    msg = _scan_msg(scan, 100.0)
    feed.guard._scan(msg)
    e.scan_callback(msg)
    feed.guard._detections(String(data=json.dumps([dict(dets[0], image_stamp=99.1)])))
    d = out['distance'][-1]
    assert d['detection_age_s'] == 0.0 and victim_measurement(d) is not None
    assert out['audit'][-1]['image_age_s'] == 0.9


def test_stale_two_candidates_share_one_smoothing_state(chain):
    # 7 the guard decides each box independently, but the estimator keeps ONE EMA for whichever
    # box it measures: when person A is withheld and person B forwarded within 1 s, B's first
    # reading is blended with A's last distance (people < 0.5 m apart in range).
    feed, out, e = chain
    a, b = (2.2, -0.6), (2.0, 0.6)
    A, B = det(W.box(*a), conf=0.95), det(W.box(*b), conf=0.7)
    clear = W.make_scan(W.legs(*a) + W.legs(*b))
    feed([dict(A), dict(B)], clear, 100.0)
    assert [x['status'] for x in out['audit'][-1]['decisions']] == ['forwarded', 'forwarded']
    assert len(out['distance']) == 1 and out['distance'][0]['confidence'] == 0.95   # A only
    prev_a = e.previous_distance
    hidden = det(W.box(*a, y2=W.image_row(1.6, -0.6, 0.45)), conf=0.95)
    occl = W.make_scan(W.legs(*a) + W.legs(*b), segments=[((1.6, -1.0), (1.6, -0.2))])
    feed([hidden, dict(B)], occl, 100.2)
    assert statuses(out['audit'][-1]['decisions']) == ['lidar_hit_below_box', 'forwarded']
    raw_b, _ = e.get_lidar_distance(_scan_msg(occl, 100.2), *_span(B, e))
    assert abs(raw_b - prev_a) <= e.smoothing_reset_jump
    pub = out['distance'][-1]
    assert pub['confidence'] == 0.7
    assert pub['distance_m'] == round(0.35 * raw_b + 0.65 * prev_a, 3)      # cross-person blend
    # without the guard the estimator would have measured the debris for A (max confidence)
    assert core.localize_frame([hidden], occl)[0]['status'] == core.LOCALIZED


def test_e2e_close_pair_option_withholds_mixed_reading(chain):
    feed, out, e = chain
    a, b = (1.8, 0.175), (1.8, -0.175)
    dets = [det(W.box(*a)), det(W.box(*b), conf=0.8)]
    scan = W.make_scan(W.legs(*a) + W.legs(*b))
    feed([dict(d) for d in dets], scan, 100.0)
    assert len(out['distance']) == 1                       # default: estimator measures box A
    feed.guard.g['exclusive_cluster_check'] = True
    feed.clock['now'] = 102.0                               # past the EMA watchdog
    e.check_detection_timeout()
    feed([dict(d) for d in dets], scan, 102.0)
    assert len(out['distance']) == 1 and out['nav'][-1:] == out['nav'][:1]  # nothing new
    assert statuses(out['audit'][-1]['decisions']) == ['ambiguous_close_pair'] * 2


def test_e2e_guard_absent_means_no_distance(chain):
    # while remapped the estimator only hears /victim_detections_nav: no guard -> no distance
    _, out, e = chain
    dets, scan = standing()
    e.scan_callback(_scan_msg(scan, 100.0))
    assert out['distance'] == []


# ------------------------------------- J: consumer behaviour (frozen autonomous_search)
AS_SRC = os.path.join(PKG, 'my_first_robot', 'autonomous_search.py')


def _as_default(name):
    import re
    m = re.search(rf"'{name}': ([0-9.]+),", open(AS_SRC).read())
    return float(m.group(1))


def test_j_consumer_keeps_no_current_distance_and_tracks_expire():
    # production values: frozen autonomous_search defaults (the mission launch does not
    # override them)
    radius = _as_default('victim_confirm_radius_m')
    count = int(_as_default('victim_confirm_count'))
    timeout = _as_default('victim_memory_timeout_sec')
    assert (radius, count, timeout) == (0.5, 3, 30.0)
    mem = VictimMemory(radius_m=radius, count=count, timeout_s=timeout)
    mem.observe(10.0, 1.0, 0.0, 0.9, 1.0)
    mem.observe(10.2, 1.0, 0.0, 0.9, 1.0)
    assert mem.confirmed(10.3) is None                 # 2 readings: not confirmed
    # readings withheld from now on: no observe() calls. Each /victim_distance message is one
    # event; there is no "current distance". But accumulated readings persist for 30 s:
    assert len(mem.tracks) == 1 and mem.confirmed(39.0) is None
    mem.observe(39.5, 1.05, 0.0, 0.9, 1.0)            # one new reading 29 s later confirms
    assert mem.confirmed(39.6) is not None
    mem2 = VictimMemory(radius_m=radius, count=count, timeout_s=timeout)
    mem2.observe(10.0, 1.0, 0.0, 0.9, 1.0)
    assert mem2.confirmed(40.3) is None and mem2.tracks == []   # expired after 30 s


def test_j_consumer_age_gate_cannot_detect_stale_images():
    # the estimator reports detection_age_s from RECEIPT time (always ~0), so the age gate
    # passes stale images,
    # so image staleness must be rejected upstream (guard stale_image)
    r = victim_measurement({'distance_m': 2.0, 'angle_deg': 0.0, 'confidence': 0.9,
                            'detection_age_s': 0.0})
    assert r is not None


# ------------------------------------------------------------------ launch wiring (static)
def test_launch_remaps_estimator_input_only_with_nav_guard():
    import importlib.util
    from launch import LaunchContext
    from launch.actions import ExecuteProcess, IncludeLaunchDescription
    from launch_ros.actions import SetRemap
    try:
        spec = importlib.util.spec_from_file_location('posture_launch', LAUNCH)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        ents = mod.generate_launch_description().entities
    except Exception as exc:  # noqa: B902
        pytest.skip(f'my_first_robot share not resolvable here: {exc}')
    kinds = [type(e).__name__ for e in ents]
    remap = [e for e in ents if isinstance(e, SetRemap)]
    assert len(remap) == 1
    assert kinds.index('SetRemap') < kinds.index('IncludeLaunchDescription')   # applies to it
    assert len([e for e in ents if isinstance(e, IncludeLaunchDescription)]) == 1
    for value, expected in (('true', [('/victim_detections', '/victim_detections_nav')]),
                            ('false', None)):
        ctx = LaunchContext()
        ctx.launch_configurations['nav_guard'] = value
        remap[0].visit(ctx)
        assert ctx.launch_configurations.get('ros_remaps') == expected
    # ExecuteProcess.cmd holds lists of Substitution objects (str() is an object repr),
    # so resolve each part before matching
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    defaults = LaunchContext()
    for e in ents:
        if isinstance(e, DeclareLaunchArgument):
            defaults.launch_configurations[e.name] = perform_substitutions(
                defaults, e.default_value)
    resolved = [(e, [perform_substitutions(defaults, part) for part in e.cmd])
                for e in ents if isinstance(e, ExecuteProcess)]
    guard = [(e, cmd) for e, cmd in resolved
             if any(c.endswith('victim_detection_guard.py') for c in cmd)]
    assert len(guard) == 1
    guard_proc, guard_cmd = guard[0]
    assert guard_cmd[0] == 'python3' and 'use_sim_time:=true' in guard_cmd
    for value, expected in (('true', True), ('false', False)):
        ctx = LaunchContext()
        ctx.launch_configurations['nav_guard'] = value
        assert guard_proc.condition.evaluate(ctx) is expected
    src = open(LAUNCH).read()
    assert "'nav_guard', default_value='false'" in src
