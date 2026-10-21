"""
Pure per-person LiDAR localization of fused person candidates (no ROS, no Ultralytics).

Optional and advisory. The validated single-target victim_distance_estimator
(frozen) is NOT replaced: it keeps publishing /victim_distance for
autonomous_search. This module serves the separate victim_localizer node,
which publishes /victim_localizations and is consumed by nothing in the
mission.

Input: the person candidates of ONE camera frame. These are either the fused
'candidates' of a /victim_posture message (sources yolo11n+pose, yolo11n,
pose_only, pose_unassociated) or the boxes of a /victim_detections message
(YOLO11n only, used when the optional pose node is off or unhealthy), plus
the LaserScan closest in time to that frame.

Per candidate, independently:

  1. geometry   production lens model (pixel_to_bearing), 2 px edge gate,
                person-width size gate, +-2 deg padding, >= 2 deg half-span
                (all copied unchanged from victim_distance_estimator.py).
  2. parallax   every LiDAR ray's hit point can be transformed into the camera
                position (camera_offset_x_m ahead of the LiDAR) before testing
                whether it lies inside the box's bearing span. Default 0.0:
                on 3,861 recorded YOLO11n boxes (2026-10-15) the production
                lens bearings matched the LiDAR bearings best with no offset
                (mean |residual| 2.87 deg at 0.0 vs 3.15 deg at 0.108 m, the
                model.sdf camera-to-LiDAR distance), i.e. as the validated
                estimator assumes. No range is guessed.
  3. exclusive  rays that also fall inside ANOTHER candidate's span (padded
                by exclusion_padding_deg) are 'shared' and are never used.
                The production nearest-supported-cluster walk (>= 2 rays
                within 0.15 m, median) runs on the exclusive rays only.
                This is the fix for the 2026-10-13 Stage-1 failure, where
                shared leg rays of a neighbour blended two people.
  4. surface    the selected cluster is grown along neighbouring scan rays
                (|range step| <= surface_jump_m) into a surface S. If S reaches
                into another candidate j's unpadded span, the candidate is
                withheld as 'surface_shared' when (a) j is localized on S too
                (one surface, two boxes: e.g. a wall behind two false boxes),
                or (b) the nearest supported surface in j's direction is not S
                (S is background passing behind whatever stands in front of
                it there), or (c) S extends outside this candidate's own padded
                span (a surface wider than the box, e.g. a lying body that a
                standing person's box only crosses, may belong to j). Only if
                S lies inside this box's span AND is the nearest surface in
                j's direction does S occlude j (e.g. a lying person, boxed
                whole, in front of a standing one); the candidate then keeps
                its reading ('occludes' lists j).
                History (2026-10-15, offline replay, disclosed in the report):
                v1 withheld whenever S touched another span (IVR B2 lying +
                standing: 0/96 published); v2 added the occlusion exception
                without (c) and published 47 wrong-person readings in IVR
                B1_05; v3 = this rule.
  5. pairwise   two localized candidates whose cluster centroids are closer
                than same_object_m, or that share a ray, are both withheld as
                'same_object' (never merged silently).
  6. RVC        optional (rvc_enabled, default False), applied last to each
                still-localized candidate on its own; it only withholds and
                never changes a distance (2026-10-16 offline investigation,
                localization_investigation_20261016_232940, rules
                pre-registered as sha 09fc572e...):
                RV  'lidar_hit_below_box': the median image row of the selected
                    cluster's hits (range, LiDAR bearing, lidar_height_m),
                    projected with the calibrated camera (camera_x_m,
                    camera_height_m, production lens), lies more than
                    rv_tolerance_px below the box bottom y2: the measured
                    surface is not inside the visible person (lower body hidden
                    by a low object or another person);
                RC  'no_central_support': no cluster ray within
                    rc_central_fraction x width of the centre of the box's
                    unpadded bearing span (object only in the edge padding);
                'rvc_unevaluable': no cluster hit projects into the image
                    (withheld rather than published unchecked).
                Calibration: camera x 0.040 m, z 0.120 m matched live box
                bottoms to 0.6 px (geometry_check); model.sdf's chain does not.

Metric distance comes only from LiDAR rays; never from a box, a posture or
keypoints. A candidate without valid exclusive LiDAR evidence is reported
with usable = false and a reason. There is no cross-person smoothing: each
distance is the raw cluster median of this frame.
"""

import json
import math

SCHEMA = 'victim_localizations/1'

# ---- copied unchanged from victim_distance_estimator.py ----------------------
IMAGE_CENTER_X = 160.0
IMAGE_HALF_WIDTH = 160.0
LENS_HFOV_RAD = 3.183


def pixel_to_bearing(pixel_x):
    """burger_cam wide-angle lens model (+ = left), as in the estimator."""
    normalized = (pixel_x - IMAGE_CENTER_X) / IMAGE_HALF_WIDTH
    return -4.0 * math.atan(normalized * math.tan(LENS_HFOV_RAD / 8.0))


DEFAULTS = {
    # production values (victim_distance_estimator.py)
    'span_padding_deg': 2.0,
    'min_half_span_deg': 2.0,
    'min_range_m': 0.20,
    'min_support': 2,
    'cluster_tolerance_m': 0.15,
    'person_min_width_m': 0.25,
    'image_edge_margin_px': 2.0,
    # additions (module docstring)
    'camera_offset_x_m': 0.0,
    'lidar_offset_x_m': -0.032,
    'exclusion_padding_deg': 2.0,
    'surface_jump_m': 0.10,
    'same_object_m': 0.25,
    'dedup_iou': 0.70,
    'image_width_px': 320.0,
    'image_height_px': 240.0,
    # RVC (step 6); disabled by default for rollout
    'rvc_enabled': False,
    'rv_tolerance_px': 0.0,
    'rc_central_fraction': 0.25,
    'camera_x_m': 0.040,
    'camera_height_m': 0.120,
    'lidar_height_m': 0.181,
}

LOCALIZED = 'localized'
WITHHELD_REASONS = (
    'invalid_candidate', 'duplicate_box', 'edge_truncated', 'no_scan',
    'no_supported_cluster', 'no_exclusive_lidar_evidence', 'surface_shared',
    'same_object', 'lidar_hit_below_box', 'no_central_support', 'rvc_unevaluable',
)


# ------------------------------------------------------------------ candidates
def _finite(v):
    f = float(v)
    if not math.isfinite(f):
        raise ValueError('non-finite value')
    return f


def _bbox(raw):
    if isinstance(raw, dict):
        return tuple(_finite(raw[k]) for k in ('x1', 'y1', 'x2', 'y2'))
    if isinstance(raw, (list, tuple)) and len(raw) == 4:
        return tuple(_finite(v) for v in raw)
    raise TypeError('bbox must be {x1,y1,x2,y2} or [x1, y1, x2, y2]')


def _opt_conf(v):
    if v is None:
        return None
    f = _finite(v)
    if not 0.0 <= f <= 1.0:
        raise ValueError('confidence outside [0, 1]')
    return f


def parse_candidate(entry, index, cfg=None):
    """
    Normalize one candidate; raise (ValueError, KeyError, TypeError) if malformed.

    Accepts a /victim_posture 'candidates' entry (bbox list, source,
    detection_confidence, pose_confidence) or a /victim_detections entry
    (bbox dict, confidence). 'confidence' is the strongest model confidence.
    """
    cfg = cfg or DEFAULTS
    if not isinstance(entry, dict):
        raise TypeError('candidate is not an object')
    box = _bbox(entry['bbox'])
    x1, y1, x2, y2 = box
    w, h = cfg['image_width_px'], cfg['image_height_px']
    if not (x2 > x1 and y2 > y1):
        raise ValueError('empty or inverted bbox')
    if x1 < -1.0 or y1 < -1.0 or x2 > w + 1.0 or y2 > h + 1.0:
        raise ValueError('bbox outside the image')
    if 'source' in entry:
        source = str(entry['source'])
        det_c = _opt_conf(entry.get('detection_confidence'))
        pose_c = _opt_conf(entry.get('pose_confidence'))
    else:
        source = 'yolo11n'
        det_c = _opt_conf(entry['confidence'])
        pose_c = None
    confs = [c for c in (det_c, pose_c) if c is not None]
    if not confs:
        raise ValueError('no confidence')
    return {'index': index, 'source': source, 'bbox': box, 'confidence': max(confs),
            'detection_confidence': det_c, 'pose_confidence': pose_c}


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0.0 else 0.0


def candidate_geometry(cand, cfg=None):
    """Bearings (camera frame, rad) and gates of one candidate; production arithmetic."""
    cfg = cfg or DEFAULTS
    x1, _, x2, _ = cand['bbox']
    center = pixel_to_bearing((x1 + x2) / 2.0)
    hi, lo = pixel_to_bearing(x1), pixel_to_bearing(x2)
    width = hi - lo
    gate = cfg['min_range_m']
    if width > 0.0:
        gate = max(gate, cfg['person_min_width_m'] / width)
    m = cfg['image_edge_margin_px']
    edge = x1 <= m or x2 >= 2.0 * IMAGE_HALF_WIDTH - m
    half = math.radians(cfg['min_half_span_deg'])
    pad = math.radians(cfg['span_padding_deg'])
    return {'center': center, 'unpadded': (lo, hi),
            'padded': (min(lo, center - half) - pad, max(hi, center + half) + pad),
            'size_gate_m': gate, 'edge': edge}


# ------------------------------------------------------------------ scan
def _scan_get(scan, name):
    return scan[name] if isinstance(scan, dict) else getattr(scan, name)


def scan_rays(scan, cfg=None):
    """
    Return the valid rays of a LaserScan(-like) as dicts {i, angle, range, cam_bearing}.

    angle: LiDAR bearing (rad, + = left). cam_bearing: bearing of the hit
    point seen from the camera, camera_offset_x_m ahead of the LiDAR.
    """
    cfg = cfg or DEFAULTS
    a0 = float(_scan_get(scan, 'angle_min'))
    da = float(_scan_get(scan, 'angle_increment'))
    rmin = float(_scan_get(scan, 'range_min'))
    rmax = float(_scan_get(scan, 'range_max'))
    off = cfg['camera_offset_x_m']
    out = []
    for i, r in enumerate(_scan_get(scan, 'ranges')):
        if r is None:
            continue
        r = float(r)
        if not math.isfinite(r) or r < rmin or r > rmax:
            continue
        a = math.atan2(math.sin(a0 + i * da), math.cos(a0 + i * da))
        x, y = r * math.cos(a), r * math.sin(a)
        out.append({'i': i, 'angle': a, 'range': r, 'cam_bearing': math.atan2(y, x - off)})
    return out


def _inside(b, span):
    return span[0] <= b <= span[1]


def nearest_supported_cluster(rays, cfg=None):
    """Production walk: nearest range with >= min_support rays within tolerance; returns rays."""
    cfg = cfg or DEFAULTS
    srt = sorted(rays, key=lambda r: (r['range'], r['i']))
    for k, near in enumerate(srt):
        cluster = [r for r in srt[k:] if r['range'] - near['range'] <= cfg['cluster_tolerance_m']]
        if len(cluster) >= cfg['min_support']:
            return cluster
    return []


def grow_surface(cluster, scan, cfg=None):
    """Scan indices of the contiguous surface containing the cluster (range steps <= jump)."""
    cfg = cfg or DEFAULTS
    ranges = list(_scan_get(scan, 'ranges'))
    n = len(ranges)
    rmin = float(_scan_get(scan, 'range_min'))
    rmax = float(_scan_get(scan, 'range_max'))

    def ok(j):
        r = ranges[j]
        return r is not None and math.isfinite(float(r)) and rmin <= float(r) <= rmax

    members = {r['i'] for r in cluster}
    for start in list(members):
        for step in (-1, 1):
            j = start
            while True:
                k = (j + step) % n
                if k in members or not ok(k) or not ok(j):
                    break
                if abs(float(ranges[k]) - float(ranges[j])) > cfg['surface_jump_m']:
                    break
                members.add(k)
                j = k
    return members


# ------------------------------------------------------------------ RVC
def lidar_hit_row(rng, bearing, cfg=None):
    """Image row (v, down = +) of a LiDAR hit at lidar_height_m, or None if behind the camera."""
    cfg = cfg or DEFAULTS
    fwd = cfg['lidar_offset_x_m'] + rng * math.cos(bearing) - cfg['camera_x_m']
    left = rng * math.sin(bearing)
    up = cfg['lidar_height_m'] - cfg['camera_height_m']
    if fwd <= 0.01:
        return None
    theta = math.atan2(math.hypot(left, up), fwd)
    r_px = IMAGE_HALF_WIDTH / math.tan(LENS_HFOV_RAD / 8.0) * math.tan(theta / 4.0)
    return cfg['image_height_px'] / 2.0 - r_px * math.sin(math.atan2(up, left))


def rvc_check(cand, cfg=None):
    """
    Return (withhold reason or None, evidence) for one localized candidate.

    cand needs 'bbox', 'unpadded' (candidate_geometry) and 'cluster' (rays).
    """
    cfg = cfg or DEFAULTS
    rows = sorted(v for v in (lidar_hit_row(r['range'], r['angle'], cfg) for r in cand['cluster'])
                  if v is not None)
    lo, hi = cand['unpadded']
    centre, width = (lo + hi) / 2.0, hi - lo
    central = sum(1 for r in cand['cluster']
                  if abs(r['angle'] - centre) <= cfg['rc_central_fraction'] * width)
    y2 = cand['bbox'][3]
    ev = {'lidar_row_px': None if not rows else round(rows[len(rows) // 2], 2),
          'box_bottom_px': round(y2, 1), 'central_rays': central}
    if not rows:
        return 'rvc_unevaluable', ev
    if rows[len(rows) // 2] - y2 > cfg['rv_tolerance_px']:
        return 'lidar_hit_below_box', ev
    if central == 0:
        return 'no_central_support', ev
    return None, ev


# ------------------------------------------------------------------ main
def localize_frame(entries, scan, cfg=None):
    """
    Localize every candidate of one frame. Never raises on bad candidates.

    entries: list of candidate dicts (see parse_candidate); scan: LaserScan-like
    object or dict, or None. Returns one result dict per input entry, in input
    order, each with 'status' ('localized' or a WITHHELD_REASONS value),
    'usable' and the evidence used.
    """
    cfg = dict(DEFAULTS, **(cfg or {}))
    results = []
    cands = []
    for idx, e in enumerate(entries if isinstance(entries, list) else []):
        try:
            c = parse_candidate(e, idx, cfg)
        except (KeyError, TypeError, ValueError) as exc:
            results.append({'index': idx, 'status': 'invalid_candidate', 'usable': False,
                            'reason': f'{type(exc).__name__}: {exc}'})
            continue
        c.update(candidate_geometry(c, cfg))
        cands.append(c)
    # exact-duplicate boxes only (fusion upstream already de-duplicates people)
    kept = []
    for c in sorted(cands, key=lambda c: (-c['confidence'], c['index'])):
        dup = next((k for k in kept if iou(c['bbox'], k['bbox']) >= cfg['dedup_iou']), None)
        if dup is not None:
            c['status'], c['duplicate_of'] = 'duplicate_box', dup['index']
        else:
            kept.append(c)
    rays = scan_rays(scan, cfg) if scan is not None else None
    excl_pad = math.radians(cfg['exclusion_padding_deg'])
    for c in kept:
        if c['edge']:
            c['status'] = 'edge_truncated'
            continue
        if rays is None:
            c['status'] = 'no_scan'
            continue
        others = [(o['unpadded'][0] - excl_pad, o['unpadded'][1] + excl_pad)
                  for o in kept if o is not c]
        own = [r for r in rays if _inside(r['cam_bearing'], c['padded'])
               and r['range'] >= max(c['size_gate_m'], cfg['min_range_m'])]
        excl = [r for r in own if not any(_inside(r['cam_bearing'], s) for s in others)]
        c['own_rays'], c['exclusive_rays'] = len(own), len(excl)
        cluster = nearest_supported_cluster(excl, cfg)
        if not cluster:
            c['status'] = ('no_exclusive_lidar_evidence'
                           if nearest_supported_cluster(own, cfg) else 'no_supported_cluster')
            continue
        c['cluster'] = cluster
        c['status'] = LOCALIZED
    by_i = {r['i']: r for r in rays} if rays is not None else {}
    for c in [k for k in kept if k.get('status') == LOCALIZED]:
        surface = grow_surface(c['cluster'], scan, cfg)
        shared, occludes = [], []
        for o in kept:
            if o is c or not any(j in by_i and _inside(by_i[j]['cam_bearing'], o['unpadded'])
                                 for j in surface):
                continue
            o_on_surface = 'cluster' in o and bool({r['i'] for r in o['cluster']} & surface)
            near = nearest_supported_cluster(
                [r for r in rays if _inside(r['cam_bearing'], o['padded'])
                 and r['range'] >= cfg['min_range_m']], cfg)
            in_front = bool(near) and not ({r['i'] for r in near} & surface)
            wider = any(j in by_i and not _inside(by_i[j]['cam_bearing'], c['padded'])
                        for j in surface)
            (shared if o_on_surface or in_front or wider else occludes).append(o['index'])
        if shared:
            c['status'], c['surface_touches'] = 'surface_shared', shared
        elif occludes:
            c['occludes'] = occludes
    loc = [c for c in kept if c.get('status') == LOCALIZED]
    for c in loc:
        rs = sorted(c['cluster'], key=lambda r: r['range'])
        med = rs[len(rs) // 2]
        ang = sorted(r['angle'] for r in rs)
        bearing = ang[len(ang) // 2]
        x = cfg['lidar_offset_x_m'] + med['range'] * math.cos(bearing)
        y = med['range'] * math.sin(bearing)
        c.update(distance_m=med['range'], bearing_lidar=bearing, x=x, y=y,
                 range_spread_m=rs[-1]['range'] - rs[0]['range'], support=len(rs),
                 cluster_i=sorted(r['i'] for r in rs))
    for a_k, a in enumerate(loc):
        for b in loc[a_k + 1:]:
            if (math.hypot(a['x'] - b['x'], a['y'] - b['y']) < cfg['same_object_m']
                    or set(a['cluster_i']) & set(b['cluster_i'])):
                for p, q in ((a, b), (b, a)):
                    p['status'] = 'same_object'
                    p.setdefault('same_object_with', []).append(q['index'])
    if cfg['rvc_enabled']:
        for c in loc:
            if c['status'] != LOCALIZED:
                continue
            why, c['rvc'] = rvc_check(c, cfg)
            if why:
                c['status'] = why
                for k in ('distance_m', 'bearing_lidar', 'x', 'y', 'range_spread_m', 'support',
                          'cluster_i'):
                    c.pop(k, None)
    for c in cands:
        results.append(_result(c))
    results.sort(key=lambda r: r['index'])
    return results


def _r(v, n=3):
    return None if v is None else round(v, n)


def _result(c):
    loc = c['status'] == LOCALIZED
    out = {
        'index': c['index'], 'source': c['source'], 'status': c['status'], 'usable': loc,
        'bbox': [round(v, 1) for v in c['bbox']], 'confidence': _r(c['confidence']),
        'detection_confidence': _r(c['detection_confidence']),
        'pose_confidence': _r(c['pose_confidence']),
        'bearing_camera_deg': _r(math.degrees(c['center']), 2),
        'size_gate_m': _r(c['size_gate_m']),
        'own_rays': c.get('own_rays'), 'exclusive_rays': c.get('exclusive_rays'),
    }
    for k in ('duplicate_of', 'surface_touches', 'occludes', 'same_object_with', 'rvc'):
        if k in c:
            out[k] = c[k]
    if 'distance_m' in c:
        out.update({
            'distance_m': _r(c['distance_m']),
            'bearing_lidar_deg': _r(math.degrees(c['bearing_lidar']), 2),
            'x_m': _r(c['x']), 'y_m': _r(c['y']),
            'support': c['support'], 'range_spread_m': _r(c['range_spread_m']),
            'cluster_ray_indices': c['cluster_i'],
        })
    return out


# ------------------------------------------------------------------ messages
def parse_posture(text):
    """
    Parse a /victim_posture payload. Returns (status, candidates, image_stamp).

    status: 'ok', 'error' (the pose node reported a failed frame) or
    'malformed'. Candidates are returned unvalidated (localize_frame checks
    each one).
    """
    try:
        msg = json.loads(text)
    except (TypeError, ValueError):
        return 'malformed', [], None
    if not isinstance(msg, dict):
        return 'malformed', [], None
    stamp = msg.get('image_stamp')
    try:
        stamp = _finite(stamp) if stamp is not None else None
    except (TypeError, ValueError):
        stamp = None
    if msg.get('status') != 'ok':
        return 'error', [], stamp
    cands = msg.get('candidates')
    if not isinstance(cands, list):
        return 'malformed', [], stamp
    return 'ok', cands, stamp


def parse_detections(text):
    """Parse /victim_detections. Returns (candidates or None, image_stamp or None, wall stamp)."""
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None, None, None
    if not isinstance(data, list):
        return None, None, None
    stamp = wall = None
    for d in data:
        if isinstance(d, dict):
            for key in ('image_stamp', 'timestamp'):
                try:
                    v = _finite(d.get(key)) if d.get(key) is not None else None
                except (TypeError, ValueError):
                    v = None
                if key == 'image_stamp' and stamp is None:
                    stamp = v
                if key == 'timestamp' and wall is None:
                    wall = v
    return data, stamp, wall


GUARD_DEFAULTS = {
    'max_image_age_s': 1.0,          # the estimator's own detection_timeout
    'max_future_s': 0.5,             # tolerated clock skew of image stamps
    'center_tolerance_px': 0.5,      # |center.x - bbox mid| (detector rounds to 0.1 px)
    # optional close-pair check (default off = validated 2026-10-17 behaviour): forward a box
    # only if its single-box cluster (what the estimator measures) equals its multi-box
    # exclusive cluster; otherwise withhold it as 'ambiguous_close_pair'
    'exclusive_cluster_check': False,
}


def _frame_stamp(detections):
    """Return the frame's image_stamp as the guard node reads it: first box that has one."""
    for d in detections:
        if isinstance(d, dict) and d.get('image_stamp') is not None:
            try:
                return float(d['image_stamp'])
            except (TypeError, ValueError):
                return None
    return None


def estimator_contract(det, frame_stamp, tol_px):
    """
    Return None if the frozen estimator would measure exactly what the guard checks.

    victim_distance_estimator reads victim_id, confidence, center.x and the bbox
    dict; a missing/unreadable field makes it drop the WHOLE frame (valid boxes
    too), and it measures around center.x, while the guard validates the bbox.
    Every box must also carry the frame's image_stamp (the one age-checked).
    Returns (reason, detail) otherwise.
    """
    if not isinstance(det, dict):
        return None                  # localize_frame reports invalid_candidate
    try:
        st = det.get('image_stamp')
        if st is None or frame_stamp is None or float(st) != frame_stamp:
            return 'image_stamp_mismatch', f'{st!r} != frame {frame_stamp!r}'
    except (TypeError, ValueError):
        return 'image_stamp_mismatch', f'unreadable {det.get("image_stamp")!r}'
    try:
        if 'victim_id' not in det:
            raise KeyError('victim_id')
        conf = _finite(det['confidence'])
        if not 0.0 <= conf <= 1.0:
            raise ValueError('confidence outside [0, 1]')
        b = det['bbox']
        if not isinstance(b, dict):
            raise TypeError('bbox is not {x1,y1,x2,y2}')
        mid = (_finite(b['x1']) + _finite(b['x2'])) / 2.0
        cx = _finite(det['center']['x'])
    except (KeyError, TypeError, ValueError) as exc:
        return 'estimator_contract', f'{type(exc).__name__}: {exc}'
    if abs(cx - mid) > tol_px:
        return 'estimator_contract', f'center.x {cx} vs bbox mid {mid:.1f}'
    return None


def guard_detections(detections, scan, image_age_s, cfg=None):
    """
    Navigation-facing gate for one /victim_detections frame.

    Each box is checked on its own exactly as the frozen
    victim_distance_estimator associates a single box (localize_frame with
    that box alone), with the RVC checks enabled, after estimator_contract
    (fields the estimator reads; center.x consistent with the bbox; the
    frame's image_stamp). Returns
    (forwarded, decisions): forwarded = the input box dicts that passed,
    unchanged and in input order; decisions = one entry per input box (or one
    frame-level entry) with 'status' and, when withheld, 'reason'.
    image_age_s = now - image_stamp (None when the frame has no stamp);
    scan = the fresh LaserScan(-like) or None.
    """
    g = dict(GUARD_DEFAULTS, **(cfg or {}))
    if not isinstance(detections, list):
        return [], [{'index': None, 'status': 'withheld', 'reason': 'malformed_message'}]
    if not detections:
        return [], []
    frame_reason = None
    if image_age_s is None:
        frame_reason = 'no_image_stamp'
    elif not math.isfinite(image_age_s) or image_age_s > g['max_image_age_s'] \
            or image_age_s < -g['max_future_s']:
        frame_reason = 'stale_image'
    elif scan is None:
        frame_reason = 'no_fresh_scan'
    if frame_reason:
        return [], [{'index': i, 'status': 'withheld', 'reason': frame_reason}
                    for i in range(len(detections))]
    loc_cfg = {k: v for k, v in (cfg or {}).items() if k in DEFAULTS}
    loc_cfg['rvc_enabled'] = True
    stamp = _frame_stamp(detections)
    multi = localize_frame(detections, scan, loc_cfg) if g['exclusive_cluster_check'] else None
    forwarded, decisions = [], []
    for i, det in enumerate(detections):
        r = localize_frame([det], scan, loc_cfg)[0]
        bad = estimator_contract(det, stamp, g['center_tolerance_px']) \
            if r['status'] == LOCALIZED else None
        if not bad and multi is not None and r['status'] == LOCALIZED:
            m = multi[i]
            if m['status'] != LOCALIZED:
                bad = 'ambiguous_close_pair', f'multi-box status {m["status"]}'
            elif set(m['cluster_ray_indices']) != set(r['cluster_ray_indices']):
                bad = 'ambiguous_close_pair', 'single-box cluster differs from exclusive cluster'
        if bad:
            decisions.append({'index': i, 'status': 'withheld', 'reason': bad[0],
                              'detail': bad[1]})
        elif r['status'] == LOCALIZED:
            forwarded.append(det)
            decisions.append({'index': i, 'status': 'forwarded', 'distance_m': r['distance_m']})
        else:
            d = {'index': i, 'status': 'withheld', 'reason': r['status']}
            if 'rvc' in r:
                d['rvc'] = r['rvc']
            decisions.append(d)
    return forwarded, decisions


def select_scan(scans, image_stamp, max_dt):
    """Return (scan, dt) of the scan closest to image_stamp within max_dt, else (None, None)."""
    best = None
    for t, s in scans:
        dt = t - image_stamp
        if abs(dt) <= max_dt and (best is None or abs(dt) < abs(best[1])):
            best = (s, dt)
    return best if best is not None else (None, None)


class PersonTracker:
    """
    Greedy one-to-one track assignment of usable localizations (advisory IDs).

    Positions must be in one fixed frame (odom when available). A record
    joins the nearest live track within gate_m; otherwise a new track
    starts. Tracks expire after timeout_s without a hit. Two records of
    one frame never share a track.
    """

    def __init__(self, gate_m=0.5, timeout_s=3.0):
        self.gate_m, self.timeout_s = gate_m, timeout_s
        self.tracks = {}
        self.next_id = 1

    def update(self, t, points):
        """points: list of (x, y); returns a track id per point."""
        self.tracks = {k: v for k, v in self.tracks.items() if t - v['t'] <= self.timeout_s}
        pairs = sorted((math.hypot(x - tr['x'], y - tr['y']), pi, tid)
                       for pi, (x, y) in enumerate(points) for tid, tr in self.tracks.items())
        ids = [None] * len(points)
        used = set()
        for d, pi, tid in pairs:
            if d > self.gate_m or ids[pi] is not None or tid in used:
                continue
            ids[pi] = tid
            used.add(tid)
        for pi, (x, y) in enumerate(points):
            if ids[pi] is None:
                ids[pi] = self.next_id
                self.next_id += 1
            tr = self.tracks.setdefault(ids[pi], {'hits': 0})
            tr.update(x=x, y=y, t=t, hits=tr['hits'] + 1)
        return ids
