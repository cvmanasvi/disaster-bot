"""Tests for the optional RVC withhold checks of victim_localizer_core (step 6)."""

import math
import os

from my_first_robot import victim_localizer_core as core
import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NODE = os.path.join(PKG, 'my_first_robot', 'victim_localizer.py')
LAUNCH = os.path.join(PKG, 'launch', 'disaster_bot_posture.launch.py')
N_RAYS = 360
ON = {'rvc_enabled': True}
RVC_STATUSES = ('lidar_hit_below_box', 'no_central_support', 'rvc_unevaluable')


# ------------------------------------------------------- synthetic world (LiDAR frame, x forward)
def legs(x, y, lateral=0.10, radius=0.06):
    n = math.hypot(x, y)
    ux, uy = -y / n, x / n
    return [(x + ux * lateral, y + uy * lateral, radius),
            (x - ux * lateral, y - uy * lateral, radius)]


def body(x, y, heading, length=1.6, radius=0.12):
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
            disc = b * b - (cx * cx + cy * cy - r * r)
            if disc >= 0 and b - math.sqrt(disc) > 0:
                best = min(best, b - math.sqrt(disc))
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


def image_row(px, py, z):
    """Row of a LiDAR-frame point (px, py) at height z (m), calibrated camera as in the core."""
    c = core.DEFAULTS
    fwd = c['lidar_offset_x_m'] + px - c['camera_x_m']
    up = z - c['camera_height_m']
    th = math.atan2(math.hypot(py, up), fwd)
    r = 160.0 / math.tan(core.LENS_HFOV_RAD / 8.0) * math.tan(th / 4.0)
    return 120.0 - r * math.sin(math.atan2(up, py))


def col(bearing):
    return 160.0 + 160.0 * math.tan(-bearing / 4.0) / math.tan(core.LENS_HFOV_RAD / 8.0)


def box(x, y, half=0.22, y1=40.0, y2=None):
    """Box over a person at (x, y); y2 defaults to the projected feet row at that position."""
    n = math.hypot(x, y)
    ux, uy = -y / n, x / n
    ends = ((x + ux * half, y + uy * half), (x - ux * half, y - uy * half))
    cols = sorted(col(math.atan2(py, px)) for px, py in ends)
    bottom = image_row(x, y, 0.0) if y2 is None else y2
    return [round(cols[0], 1), y1, round(cols[1], 1), round(bottom, 1)]


def cand(b, source='yolo11n', det=0.85):
    return {'source': source, 'bbox': b, 'detection_confidence': det, 'pose_confidence': None}


def both(cands, scan):
    return core.localize_frame(cands, scan), core.localize_frame(cands, scan, ON)


def assert_only_withholds(off, on):
    """RVC never changes an accepted distance, adds a record or revives a withheld one."""
    assert len(off) == len(on)
    for a, b in zip(off, on):
        assert b['usable'] <= a['usable']
        if b['usable']:
            assert b['distance_m'] == a['distance_m']
            assert b['cluster_ray_indices'] == a['cluster_ray_indices']
        if b['status'] in RVC_STATUSES:
            assert a['status'] == core.LOCALIZED and not b['usable']
            assert 'distance_m' not in b and 'x_m' not in b and 'rvc' in b


# ------------------------------------------------------------------ geometry
def test_defaults_disabled_and_calibrated():
    d = core.DEFAULTS
    assert d['rvc_enabled'] is False
    assert (d['rv_tolerance_px'], d['rc_central_fraction']) == (0.0, 0.25)
    assert (d['camera_x_m'], d['camera_height_m'], d['lidar_height_m']) == (0.040, 0.120, 0.181)


def test_lidar_hits_project_above_horizon_and_above_feet():
    for r in (0.5, 1.0, 2.0, 3.5):
        v = core.lidar_hit_row(r, 0.0)
        assert v < 120.0
        assert image_row(r, 0.0, 0.0) - v >= 4.9
    assert core.lidar_hit_row(0.05, math.pi / 2) is None        # beside / behind the camera


# ------------------------------------------------------------------ single person
def test_valid_standing_person_kept_with_identical_distance():
    scan = make_scan(legs(2.0, 0.0))
    off, on = both([cand(box(2.0, 0.0))], scan)
    assert on[0]['status'] == core.LOCALIZED and on[0]['usable']
    ev = on[0]['rvc']
    assert ev['central_rays'] >= 2 and ev['lidar_row_px'] < ev['box_bottom_px']
    assert_only_withholds(off, on)


@pytest.mark.parametrize('r', [0.8, 1.5, 2.5, 3.3])
def test_standing_person_kept_at_all_lidar_ranges(r):
    off, on = both([cand(box(r, 0.2 * r))], make_scan(legs(r, 0.2 * r)))
    assert on[0]['usable']
    assert_only_withholds(off, on)


def test_disabled_equals_v3_behaviour():
    scan = make_scan(legs(2.0, 0.0), segments=[((1.7, -0.6), (1.7, 0.6))])
    b = box(2.0, 0.0, y2=image_row(1.7, 0.0, 0.45))
    off = core.localize_frame([cand(b)], scan)
    explicit_off = core.localize_frame([cand(b)], scan, {'rvc_enabled': False})
    assert off == explicit_off
    assert off[0]['status'] == core.LOCALIZED and 'rvc' not in off[0]


def test_hit_below_box_bottom_is_withheld():
    # low obstacle (0.45 m high) at 1.7 m hides the legs; the box ends at the obstacle's top edge
    scan = make_scan(legs(2.5, 0.0), segments=[((1.7, -0.6), (1.7, 0.6))])
    b = box(2.5, 0.0, y2=image_row(1.7, 0.0, 0.45))
    off, on = both([cand(b)], scan)
    # v3 measures the obstacle
    assert off[0]['usable'] and abs(off[0]['distance_m'] - 1.7) < 0.02
    assert on[0]['status'] == 'lidar_hit_below_box' and not on[0]['usable']
    assert on[0]['rvc']['lidar_row_px'] > on[0]['rvc']['box_bottom_px']
    assert_only_withholds(off, on)


def test_beam_across_the_view_hiding_lower_body_is_withheld():
    scan = make_scan(legs(2.8, 0.0), segments=[((2.2, -1.2), (2.2, 1.2))])
    b = box(2.8, 0.0, y2=image_row(2.2, 0.0, 0.40))
    off, on = both([cand(b)], scan)
    assert off[0]['usable'] and on[0]['status'] == 'lidar_hit_below_box'
    assert_only_withholds(off, on)


def test_object_only_in_outer_margin_is_withheld():
    # nearer object enters only the edge padding of the box; the person's legs are at the centre
    b = box(2.0, 0.0)
    g = core.candidate_geometry({'bbox': b})
    edge = g['unpadded'][1] + math.radians(1.0)
    ox, oy = 1.7 * math.cos(edge), 1.7 * math.sin(edge)
    scan = make_scan(legs(2.0, 0.0) + [(ox, oy, 0.05)])
    off, on = both([cand(b)], scan)
    assert off[0]['usable'] and off[0]['distance_m'] < 1.75
    assert on[0]['status'] == 'no_central_support' and on[0]['rvc']['central_rays'] == 0
    assert_only_withholds(off, on)


def test_points_in_central_half_are_accepted():
    b = box(2.0, 0.0, half=0.5)                     # wide box, legs at its centre
    off, on = both([cand(b)], make_scan(legs(2.0, 0.0)))
    assert on[0]['usable'] and on[0]['rvc']['central_rays'] >= 2
    assert_only_withholds(off, on)


def test_lying_proxy_alone_is_kept():
    bx, by = 1.4, 0.0
    circles = body(bx, by, math.pi / 2)
    near = min(math.hypot(cx, cy) - r for cx, cy, r in circles)
    cols = sorted(col(math.atan2(cy, cx)) for cx, cy, _ in circles)
    b = [round(cols[0] - 4, 1), round(image_row(bx, by, 0.30), 1), round(cols[-1] + 4, 1),
         round(image_row(near, 0.0, 0.0), 1)]
    pose = cand(b, source='pose_only', det=None) | {'pose_confidence': 0.7}
    off, on = both([pose], make_scan(circles))
    assert on[0]['usable'] and on[0]['source'] == 'pose_only'
    assert_only_withholds(off, on)


def test_partially_occluded_standing_box_over_missed_lying_person_is_withheld():
    # YOLO-only fallback failure: an unboxed lying person in front hides the standing legs
    # (lying body placed beyond the box's person-width size gate, so v3 measures it)
    lying = body(1.5, 0.0, math.pi / 2)
    scan = make_scan(lying + legs(2.4, 0.0))
    b = box(2.4, 0.0, y2=image_row(1.5, 0.0, 0.25))
    off, on = both([cand(b)], scan)
    # v3 reports the lying body's distance
    assert off[0]['usable'] and off[0]['distance_m'] < 1.5
    assert on[0]['status'] == 'lidar_hit_below_box'
    assert_only_withholds(off, on)


# ------------------------------------------------------------------ several people
def test_standing_beside_lying_proxy_both_kept():
    lying = body(1.5, -0.9, math.pi / 2, length=1.0)
    standing = legs(2.0, 0.9)
    near = min(math.hypot(cx, cy) - r for cx, cy, r in lying)
    cols = sorted(col(math.atan2(cy, cx)) for cx, cy, _ in lying)
    lb = [round(cols[0] - 3, 1), round(image_row(1.5, -0.9, 0.30), 1), round(cols[-1] + 3, 1),
          round(image_row(near, -0.9, 0.0), 1)]
    pose = cand(lb, source='pose_only', det=None) | {'pose_confidence': 0.7}
    off, on = both([cand(box(2.0, 0.9)), pose], make_scan(lying + standing))
    assert [r['usable'] for r in on] == [True, True]
    assert_only_withholds(off, on)


def test_two_people_at_different_distances_both_kept():
    off, on = both([cand(box(1.2, -0.4)), cand(box(2.8, 0.8))],
                   make_scan(legs(1.2, -0.4) + legs(2.8, 0.8)))
    assert [r['usable'] for r in on] == [True, True]
    assert abs(on[0]['distance_m'] - off[0]['distance_m']) == 0
    assert_only_withholds(off, on)


@pytest.mark.parametrize('sep', [0.45, 0.6, 0.9])
def test_close_pair_rvc_adds_no_withholding(sep):
    a, b = (1.8, sep / 2), (1.8, -sep / 2)
    off, on = both([cand(box(*a)), cand(box(*b))], make_scan(legs(*a) + legs(*b)))
    assert [r['usable'] for r in on] == [r['usable'] for r in off]
    assert_only_withholds(off, on)


def test_only_the_occluded_person_is_withheld():
    # person A clear at the left; person B behind a low obstacle at the right
    scan = make_scan(legs(1.8, 0.8) + legs(2.6, -0.6), segments=[((1.9, -1.1), (1.9, -0.1))])
    cands = [cand(box(1.8, 0.8)), cand(box(2.6, -0.6, y2=image_row(1.9, -0.6, 0.45)))]
    off, on = both(cands, scan)
    assert on[0]['usable'] and on[0]['distance_m'] == off[0]['distance_m']
    assert on[1]['status'] == 'lidar_hit_below_box'
    assert_only_withholds(off, on)


# ------------------------------------------------------------------ degenerate inputs
def test_no_scan_empty_scan_and_missing_returns_unchanged():
    c = [cand(box(2.0, 0.0))]
    assert core.localize_frame(c, None, ON)[0]['status'] == 'no_scan'
    empty = make_scan([])
    assert core.localize_frame(c, empty, ON)[0]['status'] == 'no_supported_cluster'
    dropped = make_scan(legs(2.0, 0.0))
    dropped['ranges'] = [None if i % 2 else r for i, r in enumerate(dropped['ranges'])]
    off, on = both(c, dropped)
    assert_only_withholds(off, on)


@pytest.mark.parametrize('bad', [{'bbox': [10, 10, 5, 20], 'confidence': 0.9}, {'bbox': 'x'},
                                 'nope', {'bbox': [0, 0, 400, 300], 'confidence': 0.9},
                                 {'bbox': [1, 1, 50, 50]}])
def test_malformed_candidate_rejected_and_others_still_checked(bad):
    scan = make_scan(legs(2.0, 0.0))
    out = core.localize_frame([bad, cand(box(2.0, 0.0))], scan, ON)
    assert out[0]['status'] == 'invalid_candidate' and not out[0]['usable']
    assert out[1]['usable'] and 'rvc' in out[1]


# ------------------------------------------------------------------ exact boundaries
def _check(rows_from, y2, angles=(0.0, 0.01), lo=-0.1, hi=0.1):
    cluster = [{'range': rows_from, 'angle': a} for a in angles]
    return core.rvc_check({'bbox': (100.0, 40.0, 200.0, y2), 'unpadded': (lo, hi),
                           'cluster': cluster})


def test_rv_threshold_boundary_is_strictly_greater():
    v = sorted(core.lidar_hit_row(1.9, a) for a in (0.0, 0.01))[1]   # upper median as in the core
    assert _check(1.9, v)[0] is None                    # exactly at the box bottom: kept
    assert _check(1.9, v - 1e-6)[0] == 'lidar_hit_below_box'          # just below: withheld
    cluster = [{'range': 1.9, 'angle': a} for a in (0.0, 0.01)]
    kept = core.rvc_check({'bbox': (100.0, 40.0, 200.0, v - 1.0), 'unpadded': (-0.1, 0.1),
                           'cluster': cluster}, dict(core.DEFAULTS, rv_tolerance_px=1.0))
    assert kept[0] is None


def test_rc_boundary_ray_exactly_at_quarter_width_is_central():
    lo, hi = -0.2, 0.2
    # 0.1 rad = 0.25 x width 0.4 rad
    assert _check(1.9, 200.0, angles=(0.1, 0.15), lo=lo, hi=hi)[0] is None
    assert _check(1.9, 200.0, angles=(0.1 + 1e-9, 0.15), lo=lo, hi=hi)[0] == 'no_central_support'


def test_unevaluable_hits_are_withheld_not_published():
    why, ev = core.rvc_check({'bbox': (100.0, 40.0, 200.0, 200.0), 'unpadded': (-0.1, 0.1),
                              'cluster': [{'range': 0.05, 'angle': math.pi / 2}] * 2})
    assert why == 'rvc_unevaluable' and ev['lidar_row_px'] is None


# ------------------------------------------------------------------ node / launch
def test_node_reports_rvc_state_and_launch_exposes_switch():
    src = open(NODE).read()
    assert "'rvc_enabled': bool(self.cfg['rvc_enabled'])" in src
    launch = open(LAUNCH).read()
    assert "'localizer_rvc', default_value='false'" in launch and "'rvc_enabled:='" in launch
    for status in RVC_STATUSES:
        assert status in core.WITHHELD_REASONS
