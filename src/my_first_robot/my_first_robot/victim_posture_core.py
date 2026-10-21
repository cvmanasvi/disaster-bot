"""
Pure helpers for the optional victim posture node (no ROS, no Ultralytics).

The posture model is YOLO11m-pose (COCO): ONE class, 'person', with a box,
a box confidence and 17 COCO keypoints (x, y, confidence). It does NOT
classify postures. The only posture information published here is:

  * torso_angle_deg: angle between the image vertical and the line from the
    hip midpoint to the shoulder midpoint (0 = vertical, 90 = horizontal),
    measured in the distorted 320x240 fisheye image, and
  * orientation_hint: a coarse heuristic derived from that angle
    ('upright', 'horizontal', 'indeterminate' or 'unknown').

'upright' covers standing, sitting and kneeling alike; no
sitting / crouching / kneeling / prone / supine label is produced.

Pose boxes are associated with the validated YOLO11n /victim_detections
boxes by IoU. Distance is never derived here: metric distance stays with
victim_distance_estimator (camera bearing + LiDAR).
"""

import json
import math

SCHEMA = 'victim_posture/1'

COCO_KEYPOINTS = (
    'nose', 'left_eye', 'right_eye', 'left_ear', 'right_ear',
    'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow',
    'left_wrist', 'right_wrist', 'left_hip', 'right_hip',
    'left_knee', 'right_knee', 'left_ankle', 'right_ankle',
)
SHOULDERS = (5, 6)
HIPS = (11, 12)

HINT_UPRIGHT = 'upright'
HINT_HORIZONTAL = 'horizontal'
HINT_INDETERMINATE = 'indeterminate'
HINT_UNKNOWN = 'unknown'

ASSOC_MATCHED = 'matched'
ASSOC_UNMATCHED = 'unmatched'
ASSOC_AMBIGUOUS = 'ambiguous'
ASSOC_NO_DETECTIONS = 'no_detections'
ASSOC_DUPLICATE = 'duplicate'

# Set the four *_containment / dedup parameters to None to restore the
# IoU-only behaviour of the first integration (2026-10-14).


def _as_list(value):
    """Return value as nested Python lists (accepts numpy / torch arrays)."""
    if value is None:
        return []
    for attr in ('cpu', 'numpy'):
        if hasattr(value, attr) and not isinstance(value, list):
            value = getattr(value, attr)()
    if hasattr(value, 'tolist'):
        return value.tolist()
    return list(value)


def extract_pose_outputs(result):
    """
    Convert one Ultralytics pose result into plain lists.

    Returns a list of dicts {'bbox': [x1, y1, x2, y2], 'confidence': c,
    'keypoints': [[x, y, kc] * 17]}. A missing keypoint tensor gives
    keypoints = [] for every box.
    """
    boxes = getattr(result, 'boxes', None)
    if boxes is None:
        return []
    xyxy = _as_list(getattr(boxes, 'xyxy', None))
    conf = _as_list(getattr(boxes, 'conf', None))
    kp = getattr(result, 'keypoints', None)
    kxy = _as_list(getattr(kp, 'xy', None)) if kp is not None else []
    kconf = _as_list(getattr(kp, 'conf', None)) if kp is not None else []
    out = []
    for i, box in enumerate(xyxy):
        keypoints = []
        if i < len(kxy) and i < len(kconf):
            keypoints = [[float(x), float(y), float(c)]
                         for (x, y), c in zip(kxy[i], kconf[i])]
        out.append({
            'bbox': [float(v) for v in box[:4]],
            'confidence': float(conf[i]) if i < len(conf) else 0.0,
            'keypoints': keypoints,
        })
    return out


def confident_keypoints(keypoints, keypoint_confidence):
    """Return the number of keypoints whose confidence exceeds the bar."""
    return sum(1 for k in keypoints if len(k) >= 3 and k[2] > keypoint_confidence)


def filter_candidates(raw, pose_confidence, keypoint_confidence, min_keypoints,
                      dedup_iou=None, dedup_containment=None):
    """
    Split raw pose boxes into (accepted, rejected).

    A box is accepted only if its confidence >= pose_confidence and at least
    min_keypoints keypoints exceed keypoint_confidence. The keypoint rule is
    the one validated on 2026-10-12: it removed every fisheye 'sky' box.

    With dedup_iou / dedup_containment set, an otherwise accepted box is
    rejected as 'duplicate_pose_box' when it overlaps a higher-confidence
    accepted box with IoU >= dedup_iou, or lies >= dedup_containment of its
    own area inside it (a partial second box of the same person: 3 frames in
    the 2026-10-14 saved-image evaluation).
    """
    accepted, rejected = [], []
    order = sorted(range(len(raw)), key=lambda i: -float(raw[i].get('confidence', 0.0)))
    for i in order:
        item = raw[i]
        n = confident_keypoints(item['keypoints'], keypoint_confidence)
        rec = dict(item, confident_keypoints=n)
        if item['confidence'] < pose_confidence:
            rec['reject_reason'] = 'low_confidence'
            rejected.append(rec)
        elif n < min_keypoints:
            rec['reject_reason'] = 'too_few_keypoints'
            rejected.append(rec)
        elif any(_is_duplicate(rec['bbox'], a['bbox'], dedup_iou, dedup_containment)
                 for a in accepted):
            rec['reject_reason'] = 'duplicate_pose_box'
            rejected.append(rec)
        else:
            accepted.append(rec)
    return accepted, rejected


def _area(a):
    return max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])


def _inter(a, b):
    return (max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
            * max(0.0, min(a[3], b[3]) - max(a[1], b[1])))


def containment(inner, outer):
    """Return the fraction of box `inner` that lies inside box `outer`."""
    a = _area(inner)
    return _inter(inner, outer) / a if a > 0.0 else 0.0


def _is_duplicate(box, kept, dedup_iou, dedup_containment):
    if dedup_iou is not None and iou(box, kept) >= dedup_iou:
        return True
    return dedup_containment is not None and containment(box, kept) >= dedup_containment


def keypoints_inside(keypoints, box, keypoint_confidence):
    """Count confident keypoints that fall inside box [x1, y1, x2, y2]."""
    return sum(1 for k in keypoints if len(k) >= 3 and k[2] > keypoint_confidence
               and box[0] <= k[0] <= box[2] and box[1] <= k[1] <= box[3])


def _midpoint(keypoints, indices, keypoint_confidence):
    pts = [keypoints[i] for i in indices
           if i < len(keypoints) and keypoints[i][2] > keypoint_confidence]
    if not pts:
        return None
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))


def torso_angle_deg(keypoints, keypoint_confidence):
    """
    Return the torso angle from the image vertical in degrees [0, 90].

    Uses the mean of the confident shoulders and the mean of the confident
    hips. Returns None when no shoulder or no hip is confident, or when the
    two points coincide.
    """
    shoulder = _midpoint(keypoints, SHOULDERS, keypoint_confidence)
    hip = _midpoint(keypoints, HIPS, keypoint_confidence)
    if shoulder is None or hip is None:
        return None
    dx, dy = shoulder[0] - hip[0], shoulder[1] - hip[1]
    if math.hypot(dx, dy) < 1e-6:
        return None
    return math.degrees(math.atan2(abs(dx), abs(dy)))


def orientation_hint(angle_deg, upright_max_deg, horizontal_min_deg):
    """Map a torso angle to a coarse heuristic hint (never a posture class)."""
    if angle_deg is None:
        return HINT_UNKNOWN
    if angle_deg <= upright_max_deg:
        return HINT_UPRIGHT
    if angle_deg >= horizontal_min_deg:
        return HINT_HORIZONTAL
    return HINT_INDETERMINATE


def iou(a, b):
    """Intersection over union of two [x1, y1, x2, y2] boxes."""
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter)
    return inter / union if union > 0.0 else 0.0


def parse_detections(text):
    """
    Parse one /victim_detections payload (the unchanged yolo_detector format).

    Returns a list of {'bbox': [x1, y1, x2, y2], 'confidence', 'victim_id',
    'timestamp'} or None if the payload is not a well-formed list.
    """
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(data, list):
        return None
    out = []
    for d in data:
        try:
            b = d['bbox']
            out.append({
                'bbox': [float(b['x1']), float(b['y1']), float(b['x2']), float(b['y2'])],
                'confidence': float(d['confidence']),
                'victim_id': d.get('victim_id'),
                'timestamp': float(d.get('timestamp', float('nan'))),
                'image_stamp': _optional_float(d.get('image_stamp')),
            })
        except (KeyError, TypeError, ValueError, AttributeError):
            return None
    return out


def _optional_float(value):
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def select_detection_set(history, t_ref, max_dt, image_stamp=None):
    """
    Pick the detection message of the same camera frame, else the closest in wall time.

    history: iterable of (wall_time, detections). If image_stamp is given and
    a detection set carries the same 'image_stamp' (yolo_detector adds it
    since 2026-10-15), that set is chosen. Otherwise the set closest in wall
    time to t_ref within max_dt is chosen (the original rule). Returns
    (detections, dt) with dt = wall_time - t_ref, or (None, None).
    """
    if image_stamp is not None:
        for t, dets in history:
            if dets and any(d.get('image_stamp') is not None
                            and abs(d['image_stamp'] - image_stamp) < 5e-4 for d in dets):
                return dets, (t - t_ref if math.isfinite(t) else 0.0)
    best = None
    for t, dets in history:
        if not math.isfinite(t):
            continue
        dt = t - t_ref
        if abs(dt) <= max_dt and (best is None or abs(dt) < abs(best[1])):
            best = (dets, dt)
    return best if best is not None else (None, None)


def associate(poses, detections, iou_min, ambiguity_margin, containment_min=None,
              containment_min_keypoints=2, keypoint_confidence=0.5):
    """
    One-to-one association of pose boxes with detection boxes.

    A pose box links to a detection when IoU >= iou_min ('iou' link), or,
    if containment_min is set, when >= containment_min of the DETECTION box
    lies inside the pose box AND >= containment_min_keypoints confident pose
    keypoints fall inside the detection box ('containment' link). The second
    rule covers YOLO11n boxing only part of a person (head-only boxes of a
    face-down figure: IoU ~0.1 with the whole-body pose box).

    Returns one dict per pose: {'status', 'detection_index', 'iou', 'rule'}.
      * no_detections: there was no detection set to compare with,
      * unmatched: no link,
      * ambiguous: several links whose IoU differs by < ambiguity_margin, or
        several links of which any is a containment link, or two pose boxes
        claim one detection about equally,
      * duplicate: a weaker pose box on a detection already matched,
      * matched: a unique link.
    """
    if detections is None:
        return [{'status': ASSOC_NO_DETECTIONS, 'detection_index': None, 'iou': None,
                 'rule': None} for _ in poses]
    m = [[iou(p['bbox'], d['bbox']) for d in detections] for p in poses]
    result = []
    for pi, row in enumerate(m):
        links = {j: 'iou' for j, v in enumerate(row) if v >= iou_min}
        if containment_min is not None:
            for j, d in enumerate(detections):
                if (j not in links and containment(d['bbox'], poses[pi]['bbox']) >= containment_min
                        and keypoints_inside(poses[pi]['keypoints'], d['bbox'],
                                             keypoint_confidence) >= containment_min_keypoints):
                    links[j] = 'containment'
        best = round(max(row), 4) if row else None
        if not links:
            result.append({'status': ASSOC_UNMATCHED, 'detection_index': None, 'iou': best,
                           'rule': None})
            continue
        ranked = sorted(links, key=lambda j: row[j], reverse=True)
        j = ranked[0]
        if len(ranked) > 1 and ('containment' in links.values()
                                or row[j] - row[ranked[1]] < ambiguity_margin):
            # The pose box links two detections about equally (or by containment).
            result.append({'status': ASSOC_AMBIGUOUS, 'detection_index': None, 'iou': best,
                           'rule': None})
        else:
            result.append({'status': ASSOC_MATCHED, 'detection_index': j,
                           'iou': round(row[j], 4), 'rule': links[j]})
    # One detection, several pose boxes: a clearly stronger claimant keeps it
    # and the others are duplicates (e.g. a partial second box of the same
    # person); near-equal claimants are all ambiguous.
    for j in range(len(detections)):
        claim = sorted((pi for pi, r in enumerate(result)
                        if r['status'] == ASSOC_MATCHED and r['detection_index'] == j),
                       key=lambda pi: m[pi][j], reverse=True)
        if len(claim) < 2:
            continue
        tie = m[claim[0]][j] - m[claim[1]][j] < ambiguity_margin
        for k, pi in enumerate(claim):
            if tie:
                result[pi] = {'status': ASSOC_AMBIGUOUS, 'detection_index': None,
                              'iou': result[pi]['iou'], 'rule': None}
            elif k > 0:
                result[pi] = {'status': ASSOC_DUPLICATE, 'detection_index': None,
                              'iou': result[pi]['iou'], 'rule': None}
    return result


def _union(a, b):
    return [min(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]), max(a[3], b[3])]


def fuse_candidates(accepted, detections, assoc):
    """
    Return the de-duplicated person candidates of one frame (advisory only).

    * 'yolo11n+pose': a detection with its matched pose box (one person;
      bbox = union of both boxes, so a head-only detection gains the body),
    * 'yolo11n': a detection without a matched pose box (always kept),
    * 'pose_only': an accepted pose box with no link to any detection,
    * 'pose_unassociated': an accepted pose box while no detection set was
      available in the time window (it may duplicate a YOLO11n person).
    Ambiguous and duplicate pose boxes never create candidates; the
    detections they touch stay as 'yolo11n' candidates. No distance is given.
    """
    out = []
    matched = {a['detection_index']: pi for pi, a in enumerate(assoc)
               if a['status'] == ASSOC_MATCHED}
    for j, d in enumerate(detections or []):
        if j in matched:
            p = accepted[matched[j]]
            box = _union(d['bbox'], p['bbox'])
            out.append({'source': 'yolo11n+pose', 'bbox': [round(v, 1) for v in box],
                        'detection_index': j, 'detection_confidence': round(d['confidence'], 3),
                        'pose_index': matched[j], 'pose_confidence': round(p['confidence'], 3),
                        'association_rule': assoc[matched[j]]['rule']})
        else:
            out.append({'source': 'yolo11n', 'bbox': [round(v, 1) for v in d['bbox']],
                        'detection_index': j, 'detection_confidence': round(d['confidence'], 3),
                        'pose_index': None, 'pose_confidence': None, 'association_rule': None})
    for pi, (p, a) in enumerate(zip(accepted, assoc)):
        if a['status'] in (ASSOC_UNMATCHED, ASSOC_NO_DETECTIONS):
            src = 'pose_only' if a['status'] == ASSOC_UNMATCHED else 'pose_unassociated'
            out.append({'source': src,
                        'bbox': [round(v, 1) for v in p['bbox']], 'detection_index': None,
                        'detection_confidence': None, 'pose_index': pi,
                        'pose_confidence': round(p['confidence'], 3), 'association_rule': None})
    return out


def build_message(accepted, rejected, detections, det_dt, assoc, cfg, meta):
    """
    Build the /victim_posture JSON dict.

    cfg: keypoint_confidence, upright_max_deg, horizontal_min_deg.
    meta: extra frame fields (image_stamp, inference_s, model, ...).
    No distance field is ever produced.
    """
    persons = []
    for p, a in zip(accepted, assoc):
        angle = torso_angle_deg(p['keypoints'], cfg['keypoint_confidence'])
        assoc_out = dict(a)
        if a['status'] == ASSOC_MATCHED:
            d = detections[a['detection_index']]
            assoc_out.update(detection_bbox=[round(v, 1) for v in d['bbox']],
                             detection_confidence=round(d['confidence'], 3),
                             victim_id=d.get('victim_id'))
        persons.append({
            'pose_bbox': [round(v, 1) for v in p['bbox']],
            'pose_confidence': round(p['confidence'], 3),
            'confident_keypoints': p['confident_keypoints'],
            'keypoints': [[round(k[0], 1), round(k[1], 1), round(k[2], 3)]
                          for k in p['keypoints']],
            'torso_angle_deg': None if angle is None else round(angle, 1),
            'orientation_hint': orientation_hint(
                angle, cfg['upright_max_deg'], cfg['horizontal_min_deg']),
            'association': assoc_out,
        })
    msg = {
        'schema': SCHEMA,
        'posture_classes': 'none (orientation_hint is a torso-angle heuristic)',
        'persons': persons,
        'candidates': fuse_candidates(accepted, detections, assoc),
        'candidates_note': 'advisory; no distance here. Navigation localizes only the '
                           'YOLO11n max-confidence box (victim_distance_estimator); '
                           'per-candidate LiDAR localization: victim_localizer '
                           '(/victim_localizations, advisory)',
        'ambiguous_pose_boxes': sum(1 for a in assoc if a['status'] == ASSOC_AMBIGUOUS),
        'rejected_boxes': len(rejected),
        'duplicate_pose_boxes': sum(1 for r in rejected
                                    if r.get('reject_reason') == 'duplicate_pose_box'),
        'detections_considered': None if detections is None else len(detections),
        'detection_dt_s': None if det_dt is None else round(det_dt, 3),
        'status': 'ok',
    }
    msg.update(meta)
    return msg


def error_message(reason, meta):
    """Return a /victim_posture payload for a frame whose inference failed."""
    msg = {'schema': SCHEMA, 'status': 'error', 'error': str(reason)[:200], 'persons': [],
           'candidates': []}
    msg.update(meta)
    return msg


def process_frame(infer, image, history, t_ref, cfg, meta, image_stamp=None):
    """
    Run one frame end to end; never raises.

    infer(image) must return a list of raw pose dicts (extract_pose_outputs
    format). Any exception yields an error_message instead.
    """
    try:
        raw = infer(image)
        accepted, rejected = filter_candidates(
            raw, cfg['pose_confidence'], cfg['keypoint_confidence'], cfg['min_keypoints'],
            cfg.get('pose_dedup_iou'), cfg.get('pose_dedup_containment'))
        detections, det_dt = select_detection_set(history, t_ref, cfg['association_max_dt_s'],
                                                  image_stamp)
        assoc = associate(accepted, detections, cfg['association_iou_min'],
                          cfg['association_ambiguity_margin'],
                          cfg.get('association_containment_min'),
                          int(cfg.get('association_containment_min_keypoints', 2)),
                          cfg['keypoint_confidence'])
        return build_message(accepted, rejected, detections, det_dt, assoc, cfg, meta)
    except Exception as exc:  # noqa: B902 - posture failure must never propagate
        return error_message(f'{type(exc).__name__}: {exc}', meta)
