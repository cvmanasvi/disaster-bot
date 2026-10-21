"""In-process tests of the localizer node's pose -> YOLO11n fallback switching (no simulation)."""

import json
import math

from builtin_interfaces.msg import Time
from my_first_robot import victim_localizer as node_mod
import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

N_RAYS = 360


def _scan(stamp):
    m = LaserScan()
    m.header.stamp = Time(sec=int(stamp), nanosec=int(round((stamp % 1) * 1e9)))
    m.angle_min, m.angle_increment = 0.0, 2 * math.pi / N_RAYS
    m.range_min, m.range_max = 0.12, 3.5
    r = [math.inf] * N_RAYS
    for i in (357, 358, 359, 0, 1, 2, 3):        # a small surface 2 m ahead
        r[i] = 2.0
    m.ranges = r
    return m


def _box():
    def col(b):
        return 160.0 + 160.0 * math.tan(-b / 4.0) / math.tan(3.183 / 8.0)
    return [round(col(0.12), 1), 40.0, round(col(-0.12), 1), 128.0]


def _posture(stamp, status='ok'):
    b = _box()
    return String(data=json.dumps({'status': status, 'image_stamp': stamp, 'candidates': [
        {'source': 'yolo11n+pose', 'bbox': b, 'detection_confidence': 0.9,
         'pose_confidence': 0.8}]}))


def _detections(stamp):
    b = _box()
    return String(data=json.dumps([{'victim_id': 1, 'class': 'person', 'confidence': 0.9,
                                    'bbox': {'x1': b[0], 'y1': b[1], 'x2': b[2], 'y2': b[3]},
                                    'timestamp': 1000.0, 'image_stamp': stamp}]))


@pytest.fixture
def node():
    rclpy.init()
    n = node_mod.VictimLocalizer()
    n.clock_now = 100.0
    n._now = lambda: n.clock_now
    n.sent = []
    n.pub.publish = lambda msg: n.sent.append(json.loads(msg.data))
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _tick(n, t):
    n.clock_now = t
    n._scan(_scan(t))


def test_yolo_frames_are_used_from_the_start_without_a_pose_stream(node):
    _tick(node, 100.0)
    node._detections(_detections(100.0))
    assert [m['input_stream'] for m in node.sent] == ['detections']
    m = node.sent[0]
    assert m['image_stamp'] == 100.0 and m['scan_rule'] == 'image_stamp'
    assert m['rvc_enabled'] is False
    assert m['localizations'][0]['usable']


def test_yolo_ignored_while_pose_healthy_then_fallback_after_timeout(node):
    _tick(node, 100.0)
    node._posture(_posture(100.0))
    # within posture_timeout_s = 3.0 of the last ok frame
    for t in (100.5, 101.5, 102.9):
        _tick(node, t)
        node._detections(_detections(t))
    assert [m['input_stream'] for m in node.sent] == ['posture']
    _tick(node, 103.2)                              # 3.2 s after the last ok frame
    node._detections(_detections(103.2))
    assert [m['input_stream'] for m in node.sent] == ['posture', 'detections']
    # the YOLO frame's own stamp, not the old pose frame's
    assert node.sent[-1]['image_stamp'] == 103.2


def test_pose_error_frames_do_not_keep_the_stream_healthy(node):
    _tick(node, 100.0)
    node._posture(_posture(100.0))
    for t in (101.0, 102.0, 103.0):
        _tick(node, t)
        node._posture(_posture(t, status='error'))
    _tick(node, 103.5)
    node._detections(_detections(103.5))
    assert [m['input_stream'] for m in node.sent] == ['posture', 'detections']


def test_returns_to_pose_stream_when_ok_frames_resume(node):
    _tick(node, 100.0)
    node._detections(_detections(100.0))
    _tick(node, 100.4)
    node._posture(_posture(100.4))
    _tick(node, 100.6)
    node._detections(_detections(100.6))            # ignored again: pose healthy
    assert [m['input_stream'] for m in node.sent] == ['detections', 'posture']


def test_rvc_enabled_ros_parameter_reaches_the_core():
    rclpy.init(args=['--ros-args', '-p', 'rvc_enabled:=true'])
    n = node_mod.VictimLocalizer()
    try:
        assert n.cfg['rvc_enabled'] is True
        n.sent = []
        n._now = lambda: 100.0
        n.pub.publish = lambda msg: n.sent.append(json.loads(msg.data))
        n._scan(_scan(100.0))
        n._detections(_detections(100.0))
        assert n.sent[0]['rvc_enabled'] is True and 'rvc' in n.sent[0]['localizations'][0]
    finally:
        n.destroy_node()
        rclpy.shutdown()
