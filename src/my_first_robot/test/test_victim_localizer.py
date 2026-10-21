"""Tests for the optional per-person victim localizer (pure core + static node checks)."""

import ast
import hashlib
import json
import math
import os

from my_first_robot import victim_localizer_core as core
from my_first_robot import victim_posture_core as posture
import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NODE = os.path.join(PKG, 'my_first_robot', 'victim_localizer.py')
YOLO = os.path.join(PKG, 'my_first_robot', 'yolo_detector.py')
LAUNCH = os.path.join(PKG, 'launch', 'disaster_bot_posture.launch.py')
N_RAYS = 360
OFF = core.DEFAULTS['camera_offset_x_m']


# ------------------------------------------------------------------ synthetic world
def leg_circles(x, y, lateral=0.10, radius=0.06):
    """Return the two leg circles of an upright person centred at (x, y) (LiDAR frame)."""
    n = math.hypot(x, y)
    ux, uy = -y / n, x / n
    return [(x + ux * lateral, y + uy * lateral, radius),
            (x - ux * lateral, y - uy * lateral, radius)]


def body_circles(x, y, heading, length=1.6, radius=0.12):
    """Return a lying body (chain of circles) centred at (x, y)."""
    k = 9
    return [(x + math.cos(heading) * length * (i / (k - 1) - 0.5),
             y + math.sin(heading) * length * (i / (k - 1) - 0.5), radius) for i in range(k)]


def make_scan(circles, segments=(), rmax=3.5):
    ranges = []
    for i in range(N_RAYS):
        a = i * 2 * math.pi / N_RAYS
        dx, dy = math.cos(a), math.sin(a)
        best = math.inf
        for cx, cy, r in circles:
            b = dx * cx + dy * cy
            c = cx * cx + cy * cy - r * r
            disc = b * b - c
            if disc >= 0:
                t = b - math.sqrt(disc)
                if t > 0:
                    best = min(best, t)
        for (x1, y1), (x2, y2) in segments:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) > 1e-12:
                t = (x1 * ey - y1 * ex) / den
                u = (x1 * dy - y1 * dx) / den
                if t > 0 and 0 <= u <= 1:
                    best = min(best, t)
        ranges.append(best if best <= rmax else math.inf)
    return {'angle_min': 0.0, 'angle_increment': 2 * math.pi / N_RAYS, 'range_min': 0.12,
            'range_max': rmax, 'ranges': ranges}


def bearing_to_pixel(b):
    return 160.0 + 160.0 * math.tan(-b / 4.0) / math.tan(core.LENS_HFOV_RAD / 8.0)


def box_for(points_xy, y1=40.0, y2=200.0):
    """Return the image box spanning LiDAR-frame points, seen from the camera (OFF ahead)."""
    bs = [math.atan2(py, px - OFF) for px, py in points_xy]
    xs = sorted(bearing_to_pixel(b) for b in bs)
    return [round(xs[0], 1), y1, round(xs[-1], 1), y2]


def person_box(x, y, half=0.22, **kw):
    n = math.hypot(x, y)
    ux, uy = -y / n, x / n
    return box_for([(x + ux * half, y + uy * half), (x - ux * half, y - uy * half)], **kw)


def cand(box, source='yolo11n', det=0.85, pose=None):
    return {'source': source, 'bbox': box, 'detection_confidence': det, 'pose_confidence': pose}


def own_rays(scan, circles):
    """Return the ray indices that hit the given circles (ground truth for attribution)."""
    own = set()
    for i, r in enumerate(scan['ranges']):
        if not math.isfinite(r):
            continue
        a = i * scan['angle_increment']
        px, py = r * math.cos(a), r * math.sin(a)
        if any(abs(math.hypot(px - cx, py - cy) - rad) < 1e-6 for cx, cy, rad in circles):
            own.add(i)
    return own


def nearest_surface(circles):
    return min(math.hypot(cx, cy) - r for cx, cy, r in circles)


# ------------------------------------------------------------------ geometry
def test_pixel_to_bearing_matches_frozen_estimator():
    est = pytest.importorskip('my_first_robot.victim_distance_estimator')
    for px in (0.0, 37.5, 160.0, 250.0, 320.0):
        assert core.pixel_to_bearing(px) == est.pixel_to_bearing(px)


def test_parallax_transform_of_a_ray():
    scan = {'angle_min': math.pi / 2, 'angle_increment': 0.01, 'range_min': 0.1,
            'range_max': 3.5, 'ranges': [1.0]}
    r = core.scan_rays(scan, dict(core.DEFAULTS, camera_offset_x_m=0.108))[0]
    assert r['angle'] == pytest.approx(math.pi / 2)
    assert r['cam_bearing'] == pytest.approx(math.atan2(1.0, -0.108))
    r0 = core.scan_rays(scan)[0]
    assert core.DEFAULTS['camera_offset_x_m'] == 0.0
    assert r0['cam_bearing'] == pytest.approx(math.pi / 2)


def test_invalid_rays_are_ignored():
    scan = {'angle_min': 0.0, 'angle_increment': 0.01, 'range_min': 0.12, 'range_max': 3.5,
            'ranges': [None, math.nan, math.inf, 0.05, 9.0, 1.0]}
    assert [r['i'] for r in core.scan_rays(scan)] == [5]


# ------------------------------------------------------------------ single person
def test_single_standing_person_matches_production_rule():
    legs = leg_circles(1.5, 0.2)
    scan = make_scan(legs)
    res = core.localize_frame([cand(person_box(1.5, 0.2))], scan)
    assert len(res) == 1 and res[0]['usable'] and res[0]['status'] == 'localized'
    assert res[0]['distance_m'] == pytest.approx(nearest_surface(legs), abs=0.06)
    assert set(res[0]['cluster_ray_indices']) <= own_rays(scan, legs)


def test_equivalent_to_frozen_estimator_cluster_when_alone():
    pytest.importorskip('rclpy')
    est = pytest.importorskip('my_first_robot.victim_distance_estimator')
    from types import SimpleNamespace
    for x, y in ((1.0, 0.0), (2.0, 0.8), (1.6, -1.2), (2.5, 2.4)):
        legs = leg_circles(x, y)
        scan = make_scan(legs)
        box = person_box(x, y)
        res = core.localize_frame([cand(box)], scan)[0]
        g = core.candidate_geometry({'bbox': tuple(box)})
        stub = SimpleNamespace(minimum_victim_distance=0.20, cluster_tolerance=0.15,
                               minimum_support=2)
        ns = SimpleNamespace(**scan)
        span = g['padded']
        d, _ = est.VictimDistanceEstimator.get_lidar_distance(
            stub, ns, span[0], span[1], g['size_gate_m'])
        assert res['usable'] and d is not None
        assert res['distance_m'] == pytest.approx(d, abs=0.03)


def test_pose_only_and_unassociated_candidates_are_localized():
    legs = leg_circles(2.0, -0.5)
    scan = make_scan(legs)
    for src in ('pose_only', 'pose_unassociated'):
        r = core.localize_frame([cand(person_box(2.0, -0.5), src, det=None, pose=0.7)], scan)[0]
        assert r['usable'] and r['source'] == src and r['confidence'] == 0.7
        assert r['detection_confidence'] is None and r['pose_confidence'] == 0.7


def test_lying_person_alone_horizontal_box():
    body = body_circles(1.8, 0.3, heading=1.4)
    scan = make_scan(body)
    pts = [(cx, cy) for cx, cy, _ in body]
    r = core.localize_frame([cand(box_for(pts, 150, 190), 'pose_only', None, 0.66)], scan)[0]
    assert r['usable']
    assert r['distance_m'] == pytest.approx(nearest_surface(body), abs=0.08)
    assert set(r['cluster_ray_indices']) <= own_rays(scan, body)


def test_no_person_no_candidates_gives_no_records():
    assert core.localize_frame([], make_scan([])) == []
    assert core.localize_frame(None, make_scan([])) == []


# ------------------------------------------------------------------ multiple people
def test_two_separated_people_get_distinct_distances():
    a, b = leg_circles(1.5, 0.6), leg_circles(2.4, -0.7)
    scan = make_scan(a + b)
    res = core.localize_frame([cand(person_box(1.5, 0.6)),
                               cand(person_box(2.4, -0.7), det=0.6)], scan)
    assert [r['usable'] for r in res] == [True, True]
    assert res[0]['distance_m'] == pytest.approx(nearest_surface(a), abs=0.06)
    assert res[1]['distance_m'] == pytest.approx(nearest_surface(b), abs=0.06)
    assert set(res[0]['cluster_ray_indices']) <= own_rays(scan, a)
    assert set(res[1]['cluster_ray_indices']) <= own_rays(scan, b)


def test_lower_confidence_person_is_not_discarded():
    a, b = leg_circles(1.2, 0.5), leg_circles(2.2, -0.6)
    scan = make_scan(a + b)
    res = core.localize_frame([cand(person_box(1.2, 0.5), det=0.95),
                               cand(person_box(2.2, -0.6), 'pose_only', None, 0.52)], scan)
    assert all(r['usable'] for r in res)


@pytest.mark.parametrize('sep', [0.40, 0.45, 0.50, 0.60])
def test_close_pair_never_uses_the_neighbours_rays(sep):
    # Stage-1 failure geometry: overlapping boxes, legs of A inside B's span.
    a, b = leg_circles(1.8, sep / 2), leg_circles(1.85, -sep / 2)
    scan = make_scan(a + b)
    res = core.localize_frame([cand(person_box(1.8, sep / 2)),
                               cand(person_box(1.85, -sep / 2))], scan)
    ra, rb = own_rays(scan, a), own_rays(scan, b)
    for r, own in zip(res, (ra, rb)):
        if r['usable']:
            assert set(r['cluster_ray_indices']) <= own, r
        else:
            assert r['status'] in ('no_exclusive_lidar_evidence', 'surface_shared', 'same_object',
                                   'no_supported_cluster')


def test_standing_with_lying_in_front_both_detected():
    stand = leg_circles(2.4, 0.0)
    lying = body_circles(1.4, 0.35, heading=0.0, length=1.2)
    scan = make_scan(stand + lying)
    pts = [(cx, cy) for cx, cy, _ in lying]
    res = core.localize_frame([cand(person_box(2.4, 0.0), det=0.9),
                               cand(box_for(pts, 150, 200), 'pose_only', None, 0.6)], scan)
    rs, rl = own_rays(scan, stand), own_rays(scan, lying)
    for r, own in zip(res, (rs, rl)):
        if r['usable']:
            assert set(r['cluster_ray_indices']) <= own
    assert res[1]['usable']


def test_missed_lying_person_in_front_is_known_failure_of_single_box():
    # Without a candidate for the lying person, the upright box measures the nearer body:
    # documented limitation (IVR Experiment B1); the fused pose candidate is the mitigation.
    stand = leg_circles(2.4, 0.0)
    lying = body_circles(1.8, 0.0, heading=math.pi / 2, length=1.2)
    scan = make_scan(stand + lying)
    r = core.localize_frame([cand(person_box(2.4, 0.0))], scan)[0]
    assert r['usable'] and set(r['cluster_ray_indices']) <= own_rays(scan, lying)
    # Nearer than the person-width size gate, the same body is excluded and nothing is published.
    lying = body_circles(1.4, 0.0, heading=math.pi / 2, length=1.2)
    r = core.localize_frame([cand(person_box(2.4, 0.0))], make_scan(stand + lying))[0]
    assert not r['usable'] and r['status'] == 'no_supported_cluster'


def test_wall_spanning_two_candidates_is_withheld():
    wall = [((2.0, -2.0), (2.0, 2.0))]
    scan = make_scan([], wall)
    res = core.localize_frame([cand(person_box(2.0, 0.4)), cand(person_box(2.0, -0.4))], scan)
    assert [r['status'] for r in res] == ['surface_shared', 'surface_shared']
    assert not any(r['usable'] for r in res)


def test_background_surface_behind_a_standing_person_is_withheld():
    # A second box over an empty wall region next to a person standing in front of the same wall:
    # the wall is background behind the person, so it cannot be attributed to the second box.
    legs = leg_circles(1.6, 0.0)
    wall = [((2.6, -2.0), (2.6, 2.0))]
    scan = make_scan(legs, wall)
    res = core.localize_frame([cand(person_box(1.6, 0.0)), cand(person_box(2.6, -0.75))], scan)
    assert res[0]['usable'] and set(res[0]['cluster_ray_indices']) <= own_rays(scan, legs)
    assert res[1]['status'] == 'surface_shared' and not res[1]['usable']


def test_lying_in_front_occludes_standing_box_and_is_kept():
    stand = leg_circles(2.3, 0.0)
    lying = body_circles(1.45, 0.05, heading=math.pi / 2 - 0.3, length=1.4)
    scan = make_scan(stand + lying)
    pts = [(cx + dx * r, cy + dy * r) for cx, cy, r in lying for dx in (-1, 1) for dy in (-1, 1)]
    res = core.localize_frame([cand(person_box(2.3, 0.0), det=0.85),
                               cand(box_for(pts, 90, 130), 'pose_only', None, 0.66)], scan)
    assert not res[0]['usable']
    assert res[1]['usable'] and set(res[1]['cluster_ray_indices']) <= own_rays(scan, lying)
    assert res[1].get('occludes') == [0]


def test_body_wider_than_the_box_that_measured_it_is_withheld():
    # IVR B1_05 v2 failure: a standing box measures part of a lying body whose own (wide)
    # box is also present; the body extends past the standing box, so it is not attributed.
    stand = leg_circles(2.4, -0.9)
    lying = body_circles(1.5, -0.2, heading=-0.5, length=1.6)
    scan = make_scan(stand + lying)
    pts = [(cx + dx * r, cy + dy * r) for cx, cy, r in lying for dx in (-1, 1) for dy in (-1, 1)]
    box_l = box_for(pts, 150, 200)
    res = core.localize_frame([cand(person_box(2.4, -0.9), det=0.85),
                               cand(box_l, 'pose_only', None, 0.6)], scan)
    stand_rays, lying_rays = own_rays(scan, stand), own_rays(scan, lying)
    for r, own in zip(res, (stand_rays, lying_rays)):
        if r['usable']:
            assert set(r['cluster_ray_indices']) <= own, r


def test_debris_nearer_inside_span_is_measured_known_limitation():
    legs = leg_circles(2.2, 0.0)
    debris = [(1.3, 0.0, 0.08)]
    scan = make_scan(legs + debris)
    r = core.localize_frame([cand(person_box(2.2, 0.0))], scan)[0]
    # Same as the production nearest-cluster rule: the nearer object wins.
    assert r['usable'] and r['distance_m'] < 1.3


def test_two_boxes_on_one_person_resolve_to_same_object():
    legs = leg_circles(1.6, 0.0, lateral=0.12)
    scan = make_scan(legs)
    n = person_box(1.6, 0.0)
    mid = (n[0] + n[2]) / 2
    left, right = [n[0] - 6, 40, mid - 1, 200], [mid + 1, 40, n[2] + 6, 200]
    res = core.localize_frame([cand(left), cand(right, 'pose_only', None, 0.6)], scan)
    assert not any(r['usable'] for r in res)
    assert {r['status'] for r in res} <= {'same_object', 'surface_shared',
                                          'no_exclusive_lidar_evidence'}


def test_exact_duplicate_boxes_give_one_record():
    legs = leg_circles(1.6, 0.3)
    scan = make_scan(legs)
    b = person_box(1.6, 0.3)
    res = core.localize_frame([cand(b, det=0.7),
                               cand([v + 0.5 for v in b], 'pose_only', None, 0.8)], scan)
    assert sorted(r['status'] for r in res) == ['duplicate_box', 'localized']
    assert res[0]['duplicate_of'] == 1 and res[1]['usable']


# ------------------------------------------------------------------ partial / edge / gates
def test_edge_touching_box_is_withheld():
    scan = make_scan(leg_circles(1.0, 0.0))
    r = core.localize_frame([cand([0.0, 40, 30, 200])], scan)[0]
    assert r['status'] == 'edge_truncated' and not r['usable']


def test_head_only_box_far_size_gate_withholds_near_person():
    legs = leg_circles(1.2, 0.0)
    scan = make_scan(legs)
    c = 160.0
    r = core.localize_frame([cand([c - 3, 150, c + 3, 160])], scan)[0]
    assert r['size_gate_m'] > 1.2
    assert not r['usable']


def test_fused_head_plus_body_union_box_is_localized():
    legs = leg_circles(1.5, 0.0)
    scan = make_scan(legs)
    body = person_box(1.5, 0.0)
    r = core.localize_frame([cand(body, 'yolo11n+pose', 0.6, 0.7)], scan)[0]
    assert r['usable'] and r['source'] == 'yolo11n+pose'


def test_no_scan_and_empty_scan():
    b = person_box(1.5, 0.0)
    assert core.localize_frame([cand(b)], None)[0]['status'] == 'no_scan'
    assert core.localize_frame([cand(b)], make_scan([]))[0]['status'] == 'no_supported_cluster'


@pytest.mark.parametrize('bad', [
    {'bbox': [10, 10, 5, 50], 'source': 'yolo11n', 'detection_confidence': 0.9},
    {'bbox': [10, 10, math.nan, 50], 'source': 'yolo11n', 'detection_confidence': 0.9},
    {'bbox': [10, 10, 400, 50], 'source': 'yolo11n', 'detection_confidence': 0.9},
    {'bbox': [10, 10, 50, 50], 'source': 'pose_only'},
    {'bbox': [10, 10, 50, 50], 'source': 'yolo11n', 'detection_confidence': 1.5},
    {'source': 'yolo11n', 'detection_confidence': 0.9},
    'not a dict',
])
def test_malformed_candidate_rejected_others_kept(bad):
    legs = leg_circles(1.5, 0.0)
    res = core.localize_frame([bad, cand(person_box(1.5, 0.0))], make_scan(legs))
    assert res[0]['status'] == 'invalid_candidate' and not res[0]['usable']
    assert res[1]['usable']


def test_detection_format_input_is_accepted():
    legs = leg_circles(1.5, 0.0)
    b = person_box(1.5, 0.0)
    det = {'victim_id': 1, 'class': 'person', 'confidence': 0.8,
           'bbox': {'x1': b[0], 'y1': b[1], 'x2': b[2], 'y2': b[3]},
           'center': {'x': 0, 'y': 0}, 'timestamp': 1.0, 'image_stamp': 2.0}
    r = core.localize_frame([det], make_scan(legs))[0]
    assert r['usable'] and r['source'] == 'yolo11n' and r['confidence'] == 0.8


def test_never_derives_distance_without_lidar():
    for scan in (None, make_scan([])):
        for r in core.localize_frame([cand(person_box(1.5, 0.0), 'pose_only', None, 0.9)], scan):
            assert 'distance_m' not in r and not r['usable']


# ------------------------------------------------------------------ parsing / timing
def test_parse_posture_variants():
    ok = json.dumps({'status': 'ok', 'image_stamp': 12.5, 'candidates': [{'bbox': [1, 2, 3, 4]}]})
    assert core.parse_posture(ok) == ('ok', [{'bbox': [1, 2, 3, 4]}], 12.5)
    assert core.parse_posture(json.dumps({'status': 'error', 'image_stamp': 3.0}))[0] == 'error'
    assert core.parse_posture('{')[0] == 'malformed'
    assert core.parse_posture(json.dumps({'status': 'ok', 'candidates': 5}))[0] == 'malformed'
    assert core.parse_posture(json.dumps([1]))[0] == 'malformed'


def test_parse_detections_stamps():
    d, stamp, wall = core.parse_detections(json.dumps([{'confidence': 0.9, 'timestamp': 100.0,
                                                        'image_stamp': 7.25}]))
    assert stamp == 7.25 and wall == 100.0 and len(d) == 1
    d, stamp, wall = core.parse_detections(json.dumps([{'confidence': 0.9, 'timestamp': 100.0}]))
    assert stamp is None and wall == 100.0
    assert core.parse_detections('nope') == (None, None, None)
    assert core.parse_detections('{}') == (None, None, None)


def test_select_scan_nearest_and_stale():
    scans = [(10.0, 'a'), (10.2, 'b'), (10.4, 'c')]
    assert core.select_scan(scans, 10.25, 0.15) == ('b', pytest.approx(-0.05))
    assert core.select_scan(scans, 11.0, 0.15) == (None, None)
    assert core.select_scan([], 10.0, 0.15) == (None, None)


def test_tracker_ids_one_to_one_and_expiry():
    t = core.PersonTracker(gate_m=0.5, timeout_s=1.0)
    assert t.update(0.0, [(1.0, 0.0), (2.0, 0.0)]) == [1, 2]
    assert t.update(0.5, [(2.05, 0.0), (1.02, 0.0)]) == [2, 1]
    # never two records on one track
    assert t.update(0.6, [(1.0, 0.0), (1.05, 0.0)]) == [1, 3]
    assert t.update(5.0, [(1.0, 0.0)]) == [4]                        # expired


# ------------------------------------------------------------------ fusion stamp matching
def test_posture_core_prefers_same_frame_detections():
    same = [{'bbox': [0, 0, 1, 1], 'confidence': 0.9, 'image_stamp': 5.0}]
    other = [{'bbox': [0, 0, 1, 1], 'confidence': 0.8, 'image_stamp': 5.1}]
    hist = [(100.00, other), (100.30, same)]
    dets, dt = posture.select_detection_set(hist, 100.0, 0.5, image_stamp=5.0)
    assert dets is same and dt == pytest.approx(0.3)
    dets, _ = posture.select_detection_set(hist, 100.0, 0.5)
    assert dets is other                                              # original wall-time rule
    dets, _ = posture.select_detection_set(hist, 100.0, 0.5, image_stamp=9.0)
    assert dets is other                                              # no same-frame set: fallback


def test_posture_parse_detections_keeps_image_stamp():
    msg = json.dumps([{'bbox': {'x1': 1, 'y1': 2, 'x2': 3, 'y2': 4}, 'confidence': 0.5,
                       'victim_id': 1, 'timestamp': 1.0, 'image_stamp': 3.5}])
    assert posture.parse_detections(msg)[0]['image_stamp'] == 3.5
    old = json.dumps([{'bbox': {'x1': 1, 'y1': 2, 'x2': 3, 'y2': 4}, 'confidence': 0.5}])
    assert posture.parse_detections(old)[0]['image_stamp'] is None


def test_pose_failure_frame_yields_no_candidates_for_localizer():
    def boom(_):
        raise RuntimeError('model crashed')
    msg = posture.process_frame(boom, None, [], 0.0, {}, {'image_stamp': 1.0})
    status, cands, _ = core.parse_posture(json.dumps(msg))
    assert status == 'error' and cands == []


# ------------------------------------------------------------------ static node / interface checks
def _tree(path):
    with open(path) as f:
        return ast.parse(f.read())


def _published_topics(path):
    out = []
    for n in ast.walk(_tree(path)):
        if isinstance(n, ast.Call) and getattr(n.func, 'attr', '') == 'create_publisher':
            out.append(n.args[1].value)
    return out


def test_localizer_publishes_only_its_advisory_topic():
    assert _published_topics(NODE) == ['/victim_localizations']


def test_localizer_does_not_import_ultralytics_or_motion():
    src = open(NODE).read()
    assert 'ultralytics' not in src and 'Twist' not in src and 'cmd_vel' not in src


def test_yolo_detector_keeps_schema_and_adds_image_stamp():
    src = open(YOLO).read()
    for key in ("'victim_id'", "'class'", "'confidence'", "'bbox'", "'center'", "'timestamp'",
                "'image_stamp'"):
        assert key in src
    assert _published_topics(YOLO) == ['/victim_detections']


def test_launch_declares_optional_localizer():
    src = open(LAUNCH).read()
    assert "'localizer_enabled'" in src and 'victim_localizer.py' in src
    assert 'disaster_bot_full.launch.py' in src


def test_frozen_production_files_unchanged():
    ws = os.path.abspath(os.path.join(PKG, '..', '..'))
    sums = os.path.join(ws, 'evidence', 'SHA256SUMS.frozen')
    if not os.path.isfile(sums):
        pytest.skip('frozen checksum list not found')
    for line in open(sums):
        digest, rel = line.split()
        path = os.path.join(ws, rel)
        assert hashlib.sha256(open(path, 'rb').read()).hexdigest() == digest, rel
