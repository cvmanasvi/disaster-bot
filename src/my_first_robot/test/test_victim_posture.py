"""Tests for the optional victim posture integration (no ROS runtime, no Ultralytics)."""

import ast
import hashlib
import json
import math
import os

from my_first_robot import victim_posture_core as core
import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NODE_SRC = os.path.join(PKG, 'my_first_robot', 'victim_posture_estimator.py')
POSTURE_LAUNCH = os.path.join(PKG, 'launch', 'disaster_bot_posture.launch.py')
FULL_LAUNCH = os.path.join(PKG, 'launch', 'disaster_bot_full.launch.py')

CFG = {
    'pose_confidence': 0.50, 'keypoint_confidence': 0.5, 'min_keypoints': 5,
    'association_iou_min': 0.30, 'association_ambiguity_margin': 0.10,
    'association_max_dt_s': 0.5, 'upright_max_deg': 30.0, 'horizontal_min_deg': 60.0,
}


def kpts(shoulder=(100.0, 50.0), hip=(100.0, 100.0), conf=0.9, others=0.9):
    """17 COCO keypoints with the given shoulder / hip centres."""
    k = [[0.0, 0.0, others] for _ in range(17)]
    for i, dx in ((5, -5.0), (6, 5.0)):
        k[i] = [shoulder[0] + dx, shoulder[1], conf]
    for i, dx in ((11, -5.0), (12, 5.0)):
        k[i] = [hip[0] + dx, hip[1], conf]
    return k


def pose(bbox, conf=0.8, keypoints=None):
    return {'bbox': list(bbox), 'confidence': conf,
            'keypoints': kpts() if keypoints is None else keypoints}


def det_payload(*boxes, ts=1000.0):
    """Build a /victim_detections payload exactly in the yolo_detector format."""
    out = []
    for (x1, y1, x2, y2), c in boxes:
        out.append({'victim_id': 1, 'class': 'person', 'confidence': c,
                    'bbox': {'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2},
                    'center': {'x': (x1 + x2) / 2, 'y': (y1 + y2) / 2}, 'timestamp': ts})
    return json.dumps(out)


# ---------------------------------------------------------------- model outputs

class _Arr:

    def __init__(self, v):
        self.v = v

    def cpu(self):
        return self

    def numpy(self):
        return self

    def tolist(self):
        return self.v


class _Res:

    def __init__(self, boxes, confs, kxy, kconf, with_kpts=True):
        self.boxes = type('B', (), {'xyxy': _Arr(boxes), 'conf': _Arr(confs)})()
        self.keypoints = (type('K', (), {'xy': _Arr(kxy), 'conf': _Arr(kconf)})()
                          if with_kpts else None)


def test_extract_pose_outputs_plain_lists():
    k = kpts()
    r = _Res([[10, 20, 30, 80]], [0.77], [[[x, y] for x, y, _ in k]], [[c for _, _, c in k]])
    out = core.extract_pose_outputs(r)
    assert len(out) == 1
    assert out[0]['bbox'] == [10.0, 20.0, 30.0, 80.0]
    assert out[0]['confidence'] == pytest.approx(0.77)
    assert len(out[0]['keypoints']) == 17 and len(out[0]['keypoints'][0]) == 3


def test_extract_pose_outputs_missing_keypoints_and_empty():
    r = _Res([[10, 20, 30, 80]], [0.6], [], [], with_kpts=False)
    assert core.extract_pose_outputs(r)[0]['keypoints'] == []
    assert core.extract_pose_outputs(_Res([], [], [], [])) == []
    assert core.extract_pose_outputs(object()) == []


# ---------------------------------------------------------------- filtering

def test_filter_rejects_sky_box_low_conf_and_keeps_person():
    sky = pose((4, 0, 317, 118), conf=0.63, keypoints=[[0, 0, 0.1]] * 16 + [[5, 5, 0.9]])
    low = pose((0, 0, 10, 10), conf=0.45)
    good = pose((50, 40, 90, 200), conf=0.81)
    acc, rej = core.filter_candidates([sky, low, good], 0.50, 0.5, 5)
    assert [a['bbox'] for a in acc] == [[50, 40, 90, 200]]
    assert sorted(r['reject_reason'] for r in rej) == ['low_confidence', 'too_few_keypoints']
    assert acc[0]['confident_keypoints'] == 17


def test_filter_boundary_exactly_min_keypoints():
    k = [[0, 0, 0.9]] * 5 + [[0, 0, 0.5]] * 12      # 0.5 is NOT > 0.5
    acc, _ = core.filter_candidates([pose((0, 0, 5, 5), keypoints=k)], 0.5, 0.5, 5)
    assert len(acc) == 1
    k2 = [[0, 0, 0.9]] * 4 + [[0, 0, 0.5]] * 13
    acc2, rej2 = core.filter_candidates([pose((0, 0, 5, 5), keypoints=k2)], 0.5, 0.5, 5)
    assert not acc2 and rej2[0]['reject_reason'] == 'too_few_keypoints'


# ---------------------------------------------------------------- torso / hint

@pytest.mark.parametrize('shoulder,hip,expected,hint', [
    ((100, 50), (100, 100), 0.0, 'upright'),
    ((150, 100), (100, 100), 90.0, 'horizontal'),
    ((100, 100), (150, 100), 90.0, 'horizontal'),          # head on the other side
    ((150, 50), (100, 100), 45.0, 'indeterminate'),
    ((100, 150), (100, 100), 0.0, 'upright'),              # upside down still 'vertical'
])
def test_torso_angle_and_hint(shoulder, hip, expected, hint):
    a = core.torso_angle_deg(kpts(shoulder, hip), 0.5)
    assert a == pytest.approx(expected, abs=1e-6)
    assert core.orientation_hint(a, 30.0, 60.0) == hint


def test_torso_unknown_without_shoulders_or_hips():
    k = kpts(conf=0.2)
    assert core.torso_angle_deg(k, 0.5) is None
    assert core.orientation_hint(None, 30.0, 60.0) == 'unknown'
    k2 = kpts()
    k2[11][2] = k2[12][2] = 0.1
    assert core.torso_angle_deg(k2, 0.5) is None
    assert core.torso_angle_deg([], 0.5) is None


def test_torso_uses_single_visible_side():
    k = kpts((100, 50), (100, 100))
    k[6][2] = k[12][2] = 0.0                     # only left shoulder + left hip
    assert core.torso_angle_deg(k, 0.5) == pytest.approx(0.0)


def test_hint_boundaries_inclusive():
    assert core.orientation_hint(30.0, 30.0, 60.0) == 'upright'
    assert core.orientation_hint(60.0, 30.0, 60.0) == 'horizontal'
    assert core.orientation_hint(30.01, 30.0, 60.0) == 'indeterminate'


# ---------------------------------------------------------------- detections contract

def test_parse_detections_production_format():
    d = core.parse_detections(det_payload(((10.0, 20.0, 30.0, 80.0), 0.9), ts=12.5))
    assert d == [{'bbox': [10.0, 20.0, 30.0, 80.0], 'confidence': 0.9, 'victim_id': 1,
                  'timestamp': 12.5, 'image_stamp': None}]


@pytest.mark.parametrize('payload', ['not json', '{}', '[{"bbox": 3}]', '[{"confidence": 1}]',
                                     None, '[{"bbox": {"x1": 1}, "confidence": 0.9}]'])
def test_parse_detections_malformed(payload):
    assert core.parse_detections(payload) is None


def test_parse_detections_empty_list():
    assert core.parse_detections('[]') == []


def test_select_detection_set_closest_within_window():
    h = [(9.0, ['a']), (10.2, ['b']), (10.45, ['c']), (float('nan'), ['x'])]
    dets, dt = core.select_detection_set(h, 10.3, 0.5)
    assert dets == ['b'] and dt == pytest.approx(-0.1)
    assert core.select_detection_set(h, 20.0, 0.5) == (None, None)
    assert core.select_detection_set([], 1.0, 0.5) == (None, None)


# ---------------------------------------------------------------- association

def D(*boxes):
    return [{'bbox': list(b), 'confidence': 0.9, 'victim_id': 1} for b in boxes]


def test_iou():
    assert core.iou([0, 0, 10, 10], [0, 0, 10, 10]) == pytest.approx(1.0)
    assert core.iou([0, 0, 10, 10], [20, 20, 30, 30]) == 0.0
    assert core.iou([0, 0, 10, 10], [5, 0, 15, 10]) == pytest.approx(50 / 150)
    assert core.iou([0, 0, 0, 0], [0, 0, 0, 0]) == 0.0


def test_associate_single_match():
    r = core.associate([pose((10, 10, 50, 200))], D((12, 11, 49, 198)), 0.3, 0.1)
    assert r[0]['status'] == 'matched' and r[0]['detection_index'] == 0 and r[0]['iou'] > 0.9


def test_associate_missing_detections_and_empty_set():
    assert core.associate([pose((0, 0, 9, 9))], None, 0.3, 0.1)[0]['status'] == 'no_detections'
    r = core.associate([pose((0, 0, 9, 9))], [], 0.3, 0.1)[0]
    assert r['status'] == 'unmatched' and r['iou'] is None
    assert core.associate([], D((0, 0, 9, 9)), 0.3, 0.1) == []


def test_associate_unmatched_lying_person_without_detection():
    # YOLO11n boxed only the upright person; the pose model also sees a lying one.
    poses = [pose((180, 60, 210, 200)), pose((60, 150, 160, 190))]
    r = core.associate(poses, D((182, 62, 209, 199)), 0.3, 0.1)
    assert [x['status'] for x in r] == ['matched', 'unmatched']


def test_associate_two_people_one_to_one():
    poses = [pose((200, 50, 240, 220)), pose((20, 60, 70, 210))]
    r = core.associate(poses, D((22, 58, 71, 212), (198, 52, 241, 219)), 0.3, 0.1)
    assert [x['detection_index'] for x in r] == [1, 0]
    assert all(x['status'] == 'matched' for x in r)


def test_associate_low_iou_is_unmatched():
    r = core.associate([pose((0, 0, 10, 100))], D((8, 0, 18, 100)), 0.3, 0.1)
    assert r[0]['status'] == 'unmatched' and 0.0 < r[0]['iou'] < 0.3


def test_associate_pose_spanning_two_detections_is_ambiguous():
    # one wide pose box straddling two side-by-side people equally
    r = core.associate([pose((0, 0, 100, 100))], D((0, 0, 50, 100), (50, 0, 100, 100)), 0.3, 0.1)
    assert r[0]['status'] == 'ambiguous' and r[0]['detection_index'] is None


def test_associate_duplicate_partial_box():
    full, partial = pose((10, 10, 60, 210)), pose((10, 10, 60, 90))
    r = core.associate([partial, full], D((11, 11, 59, 209)), 0.3, 0.1)
    assert r[1]['status'] == 'matched' and r[0]['status'] in ('duplicate', 'unmatched')


def test_associate_two_equal_claimants_are_ambiguous():
    a, b = pose((0, 0, 50, 100)), pose((2, 0, 52, 100))
    r = core.associate([a, b], D((1, 0, 51, 100)), 0.3, 0.1)
    assert [x['status'] for x in r] == ['ambiguous', 'ambiguous']


# ---------------------------------------------------------------- message + failure

def _frame(raw, history, t_ref=1000.0):
    return core.process_frame(lambda img: raw, None, history, t_ref, CFG,
                              {'image_stamp': 1.0, 'model': 'test'})


def test_message_contract_and_no_distance():
    raw = [pose((10, 10, 50, 200), 0.83), pose((4, 0, 317, 118), 0.6, [[0, 0, 0.0]] * 17)]
    hist = [(1000.1, core.parse_detections(det_payload(((11, 9, 49, 201), 0.91))))]
    msg = _frame(raw, hist)

    def keys(o):
        if isinstance(o, dict):
            for k, v in o.items():
                yield k
                yield from keys(v)
        elif isinstance(o, list):
            for v in o:
                yield from keys(v)
    assert not any('distance' in k for k in keys(msg))
    json.dumps(msg)
    assert msg['schema'] == 'victim_posture/1' and msg['status'] == 'ok'
    assert msg['rejected_boxes'] == 1 and msg['detections_considered'] == 1
    assert msg['detection_dt_s'] == pytest.approx(0.1)
    p = msg['persons'][0]
    assert set(p) == {'pose_bbox', 'pose_confidence', 'confident_keypoints', 'keypoints',
                      'torso_angle_deg', 'orientation_hint', 'association'}
    assert p['orientation_hint'] == 'upright' and len(p['keypoints']) == 17
    assert p['association']['status'] == 'matched'
    assert p['association']['detection_confidence'] == pytest.approx(0.91)
    assert p['association']['victim_id'] == 1
    assert msg['image_stamp'] == 1.0 and msg['model'] == 'test'


def test_message_no_person_and_stale_detections():
    msg = _frame([], [(1.0, [])], t_ref=1000.0)
    assert msg['status'] == 'ok' and msg['persons'] == []
    assert msg['detections_considered'] is None and msg['detection_dt_s'] is None
    msg2 = _frame([pose((10, 10, 50, 200))], [(1.0, D((10, 10, 50, 200)))], t_ref=1000.0)
    assert msg2['persons'][0]['association']['status'] == 'no_detections'


def test_low_confidence_only_yields_no_persons():
    msg = _frame([pose((10, 10, 50, 200), conf=0.42)], [])
    assert msg['persons'] == [] and msg['rejected_boxes'] == 1


def test_inference_exception_is_contained():
    def boom(img):
        raise RuntimeError('model exploded')
    msg = core.process_frame(boom, None, [], 0.0, CFG, {'image_stamp': 2.0})
    assert msg['status'] == 'error' and 'model exploded' in msg['error']
    assert msg['persons'] == [] and msg['candidates'] == [] and msg['image_stamp'] == 2.0
    json.dumps(msg)


def test_malformed_model_output_is_contained():
    msg = core.process_frame(lambda img: [{'bbox': None}], None, [], 0.0, CFG, {})
    assert msg['status'] == 'error'


def test_nan_keypoints_do_not_crash():
    k = kpts()
    k[5] = [math.nan, math.nan, 0.9]
    msg = _frame([pose((10, 10, 50, 200), keypoints=k)], [])
    assert msg['status'] in ('ok', 'error')
    json.dumps(msg)


# ---------------------------------------------------------------- node + launch (static)

def _calls(path, attr):
    tree = ast.parse(open(path).read())
    return [n for n in ast.walk(tree) if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute) and n.func.attr == attr]


def test_node_publishes_only_victim_posture():
    pubs = _calls(NODE_SRC, 'create_publisher')
    topics = [c.args[1].value for c in pubs]
    assert topics == ['/victim_posture']
    subs = sorted(c.args[1].value for c in _calls(NODE_SRC, 'create_subscription'))
    assert subs == ['/camera/image_raw', '/victim_detections']
    src = open(NODE_SRC).read()
    for forbidden in ('cmd_vel', '/victim_distance', 'victim_found', '/scan'):
        assert forbidden not in src.replace('/victim_distance or', '')


def test_node_imports_ultralytics_lazily():
    tree = ast.parse(open(NODE_SRC).read())
    top = [n for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom))]
    names = [a.name for n in top for a in n.names] + [getattr(n, 'module', '') or '' for n in top]
    assert not any('ultralytics' in x for x in names)


def test_frozen_master_launch_unchanged():
    frozen = os.path.join(PKG, '..', '..', 'evidence', 'SHA256SUMS.frozen')
    if not os.path.isfile(frozen):
        pytest.skip('frozen manifest not available')
    want = [ln.split()[0] for ln in open(frozen)
            if ln.strip().endswith('disaster_bot_full.launch.py')]
    assert want and hashlib.sha256(open(FULL_LAUNCH, 'rb').read()).hexdigest() == want[0]


def test_posture_launch_structure():
    pytest.importorskip('launch')
    pytest.importorskip('ament_index_python')
    import importlib.util
    from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
    from launch.conditions import IfCondition
    try:
        spec = importlib.util.spec_from_file_location('posture_launch', POSTURE_LAUNCH)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        ld = mod.generate_launch_description()
    except Exception as exc:  # noqa: B902
        pytest.skip(f'my_first_robot share not resolvable here: {exc}')
    ents = ld.entities
    args = {e.name: e for e in ents if isinstance(e, DeclareLaunchArgument)}
    for a in ('posture_enabled', 'posture_model', 'posture_max_rate_hz', 'localizer_enabled',
              'gui', 'x_pose', 'y_pose', 'yaw', 'explore_max_steps', 'record_dir'):
        assert a in args
    inc = [e for e in ents if isinstance(e, IncludeLaunchDescription)]
    assert len(inc) == 1
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    src = inc[0].launch_description_source
    loc = perform_substitutions(LaunchContext(), src._LaunchDescriptionSource__location)
    assert loc.endswith('disaster_bot_full.launch.py')
    proc = [e for e in ents if isinstance(e, ExecuteProcess)]
    # posture node, localizer, navigation guard (2026-10-17): all optional
    assert len(proc) == 3 and all(isinstance(p.condition, IfCondition) for p in proc)
    ctx = LaunchContext()
    ctx.launch_configurations.update(
        {'posture_model': '/m/yolo11m-pose.pt', 'posture_max_rate_hz': '0.5'})
    cmd = [perform_substitutions(ctx, part) for part in proc[0].cmd]
    assert cmd[0].endswith('yolo_env/bin/python')
    assert cmd[1].endswith('victim_posture_estimator.py')
    assert 'model_path:=/m/yolo11m-pose.pt' in cmd and 'max_rate_hz:=0.5' in cmd
    assert 'use_sim_time:=true' in cmd
    assert not any('autonomous_search' in c for c in cmd)
    ctx.launch_configurations['posture_enabled'] = 'false'
    assert proc[0].condition.evaluate(ctx) is False
    ctx.launch_configurations['posture_enabled'] = 'true'
    assert proc[0].condition.evaluate(ctx) is True
    assert args['posture_enabled'].default_value[0].describe().strip("'") == 'true'


# ---------------------------------------------------------------- 2026-10-14 fusion revision
# Real example from the saved-image evaluation (isolated_validation yard_prone f00012):
# YOLO11n boxed only the head of a face-down figure; YOLO11m-pose boxed the whole body.
HEAD_DET = [196.6, 108.5, 210.5, 122.2]
BODY_POSE = [106.3, 103.8, 210.4, 123.3]
BODY_KPTS = [[198.5, 115.3, 0.797], [200.7, 114.7, 0.739], [200.3, 113.6, 0.627],
             [204.1, 115.0, 0.655], [203.0, 113.3, 0.417], [197.2, 115.3, 0.932],
             [194.6, 114.9, 0.923], [177.7, 115.8, 0.759], [176.8, 116.0, 0.704],
             [158.0, 115.3, 0.665], [159.8, 116.6, 0.563], [163.9, 114.8, 0.937],
             [162.5, 111.0, 0.937], [134.7, 115.5, 0.883], [135.4, 113.8, 0.875],
             [114.4, 115.0, 0.815], [117.2, 116.1, 0.8]]
FCFG = dict(CFG, association_containment_min=0.70, association_containment_min_keypoints=2,
            pose_dedup_iou=0.50, pose_dedup_containment=0.70)


def test_head_only_detection_iou_rule_fails_containment_matches():
    poses = [pose(BODY_POSE, 0.738, BODY_KPTS)]
    dets = D(HEAD_DET)
    old = core.associate(poses, dets, 0.3, 0.1)
    assert old[0]['status'] == 'unmatched' and old[0]['iou'] < 0.3
    new = core.associate(poses, dets, 0.3, 0.1, 0.70, 2, 0.5)
    assert new[0]['status'] == 'matched' and new[0]['rule'] == 'containment'
    assert core.keypoints_inside(BODY_KPTS, HEAD_DET, 0.5) >= 3


def test_head_only_end_to_end_one_candidate_with_whole_body_extent():
    hist = [(1000.0, core.parse_detections(det_payload((tuple(HEAD_DET), 0.587))))]
    msg = _frame_cfg([pose(BODY_POSE, 0.738, BODY_KPTS)], hist, FCFG)
    c = msg['candidates']
    assert len(c) == 1 and c[0]['source'] == 'yolo11n+pose'
    assert c[0]['bbox'] == [106.3, 103.8, 210.5, 123.3]
    assert c[0]['association_rule'] == 'containment'
    old = _frame_cfg([pose(BODY_POSE, 0.738, BODY_KPTS)], hist, CFG)
    assert sorted(x['source'] for x in old['candidates']) == ['pose_only', 'yolo11n']


def _frame_cfg(raw, history, cfg, t_ref=1000.0):
    return core.process_frame(lambda img: raw, None, history, t_ref, cfg, {'image_stamp': 1.0})


def test_containment_requires_keypoints_inside():
    # A small box of ANOTHER person inside a big pose box, with no pose keypoint in it:
    # must not be merged (that would hide a person).
    k = kpts((150, 60), (150, 120))
    big = pose((100, 20, 220, 220), keypoints=k)
    r = core.associate([big], D((190, 30, 215, 80)), 0.3, 0.1, 0.70, 2, 0.5)
    assert r[0]['status'] == 'unmatched'
    msg = _frame_cfg([big], [(1000.0, D((190, 30, 215, 80)))], FCFG)
    assert sorted(x['source'] for x in msg['candidates']) == ['pose_only', 'yolo11n']


def test_two_detections_inside_one_pose_box_is_ambiguous_and_loses_nobody():
    k = [[60, 60, 0.9]] * 8 + [[160, 60, 0.9]] * 9
    wide = pose((40, 20, 200, 220), keypoints=k)
    dets = D((50, 40, 80, 100), (150, 40, 180, 100))
    r = core.associate([wide], dets, 0.3, 0.1, 0.70, 2, 0.5)
    assert r[0]['status'] == 'ambiguous' and r[0]['detection_index'] is None
    msg = _frame_cfg([wide], [(1000.0, dets)], FCFG)
    assert [x['source'] for x in msg['candidates']] == ['yolo11n', 'yolo11n']
    assert msg['ambiguous_pose_boxes'] == 1


def test_iou_link_plus_containment_link_is_ambiguous():
    k = [[60, 100, 0.9]] * 10 + [[95, 50, 0.9]] * 7
    p = pose((40, 20, 110, 220), keypoints=k)
    dets = D((42, 22, 108, 218), (90, 40, 105, 60))   # the same person + a small box inside
    assert core.associate([p], dets, 0.3, 0.1, 0.70, 2, 0.5)[0]['status'] == 'ambiguous'


def test_pose_duplicate_box_suppressed_and_disable_switch():
    full = pose((10, 10, 60, 210), conf=0.70)
    partial = pose((12, 100, 58, 208), conf=0.55)
    acc, rej = core.filter_candidates([partial, full], 0.5, 0.5, 5, 0.5, 0.7)
    assert [a['bbox'] for a in acc] == [[10, 10, 60, 210]]
    assert rej[0]['reject_reason'] == 'duplicate_pose_box'
    acc2, _ = core.filter_candidates([partial, full], 0.5, 0.5, 5)
    assert len(acc2) == 2


def test_two_separate_lying_people_are_not_deduplicated():
    a = pose((10, 150, 120, 190), conf=0.7)
    b = pose((180, 150, 300, 190), conf=0.6)
    acc, rej = core.filter_candidates([a, b], 0.5, 0.5, 5, 0.5, 0.7)
    assert len(acc) == 2 and not rej


def test_mixed_scene_candidates_are_unique_people():
    upright = pose((200, 40, 230, 200), conf=0.88)
    lying = pose((40, 150, 170, 190), conf=0.67, keypoints=kpts((160, 170), (100, 170)))
    lying_partial = pose((120, 150, 170, 190), conf=0.52, keypoints=kpts((160, 170), (140, 170)))
    hist = [(1000.0, D((201, 41, 229, 199), (290, 50, 310, 120)))]
    msg = _frame_cfg([upright, lying, lying_partial], hist, FCFG)
    src = sorted(x['source'] for x in msg['candidates'])
    assert src == ['pose_only', 'yolo11n', 'yolo11n+pose']
    assert msg['duplicate_pose_boxes'] == 1
    lone = [p for p in msg['persons'] if p['association']['status'] == 'unmatched']
    assert lone and lone[0]['orientation_hint'] == 'horizontal'


def test_no_detection_set_gives_pose_unassociated():
    msg = _frame_cfg([pose((10, 10, 50, 200))], [], FCFG)
    assert [x['source'] for x in msg['candidates']] == ['pose_unassociated']


def test_yolo_detections_always_survive_fusion():
    dets = D((10, 10, 50, 200), (100, 10, 140, 200), (200, 10, 240, 200))
    wide_kpts = [[30, 50, 0.9]] * 17
    raws = [[], [pose((12, 12, 48, 198))], [pose((0, 0, 300, 220), keypoints=wide_kpts)]]
    for raw in raws:
        msg = _frame_cfg(raw, [(1000.0, dets)], FCFG)
        assert sorted(c['detection_index'] for c in msg['candidates']
                      if c['detection_index'] is not None) == [0, 1, 2]


def test_node_declares_fusion_parameters():
    src = open(NODE_SRC).read()
    for name in ('association_containment_min', 'association_containment_min_keypoints',
                 'pose_dedup_iou', 'pose_dedup_containment'):
        assert f"'{name}'" in src
