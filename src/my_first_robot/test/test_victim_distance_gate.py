"""
Synthetic tests for the victim_distance_estimator size gate.

Each case builds a 360-ray scan (same geometry as the burger_cam LDS:
1 deg rays, 0.12-3.5 m) containing the victim's two legs plus optional
occluders, feeds a YOLO-format detection through process_detection(), and
checks what is published on /victim_distance. No Gazebo is needed.
"""

import json
import math
from types import SimpleNamespace

from my_first_robot.victim_distance_estimator import (
    IMAGE_CENTER_X,
    IMAGE_HALF_WIDTH,
    LENS_HFOV_RAD,
    VictimDistanceEstimator,
)
import pytest
import rclpy

RAY_COUNT = 360
ANGLE_INCREMENT = 6.28 / (RAY_COUNT - 1)
RANGE_MIN = 0.12
RANGE_MAX = 3.5

PERSON_WIDTH = 0.55     # shoulder/arm width seen by the camera
LEG_OFFSET = 0.13       # lateral offset of each leg from the body centre
LEG_WIDTH = 0.13        # leg width at LiDAR height


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    estimator = VictimDistanceEstimator()
    estimator.published = []
    estimator.distance_publisher = SimpleNamespace(
        publish=lambda msg: estimator.published.append(json.loads(msg.data))
    )
    yield estimator
    estimator.destroy_node()


def bearing_to_pixel(bearing):
    """Inverse of pixel_to_bearing() (+ bearing = left = smaller pixel x)."""
    normalized = math.tan(-bearing / 4.0) / math.tan(LENS_HFOV_RAD / 8.0)
    return IMAGE_CENTER_X + normalized * IMAGE_HALF_WIDTH


def blob(center_bearing, distance, width):
    half = math.atan((width / 2.0) / distance)
    return (center_bearing - half, center_bearing + half, distance)


def person_legs(center_bearing, distance):
    lateral = math.atan(LEG_OFFSET / distance)
    return [
        blob(center_bearing + lateral, distance, LEG_WIDTH),
        blob(center_bearing - lateral, distance, LEG_WIDTH),
    ]


def make_scan(objects):
    """objects: (bearing_min, bearing_max, distance); nearest one wins."""
    ranges = []
    for i in range(RAY_COUNT):
        angle = math.atan2(math.sin(i * ANGLE_INCREMENT), math.cos(i * ANGLE_INCREMENT))
        hits = [d for lo, hi, d in objects if lo <= angle <= hi]
        ranges.append(min(hits) if hits else math.inf)
    return SimpleNamespace(
        ranges=ranges,
        angle_min=0.0,
        angle_increment=ANGLE_INCREMENT,
        range_min=RANGE_MIN,
        range_max=RANGE_MAX,
    )


def detect(node, scan, bearing_left, bearing_right):
    """Run one YOLO detection whose bbox spans [bearing_right, bearing_left]."""
    x1 = max(0.0, bearing_to_pixel(bearing_left))
    x2 = min(IMAGE_CENTER_X + IMAGE_HALF_WIDTH, bearing_to_pixel(bearing_right))
    detection = {
        'victim_id': 1,
        'class': 'person',
        'confidence': 0.8,
        'bbox': {'x1': round(x1, 1), 'y1': 20.0, 'x2': round(x2, 1), 'y2': 200.0},
        'center': {'x': round((x1 + x2) / 2.0, 1), 'y': 110.0},
        'timestamp': 0.0,
    }
    node.latest_scan = scan
    node.latest_scan_time = node._now_sec()
    node.previous_distance = None
    node.published.clear()
    node.detection_callback(SimpleNamespace(data=json.dumps([detection])))
    return node.published[-1] if node.published else None


def person_span(center_bearing, distance):
    half = math.atan((PERSON_WIDTH / 2.0) / distance)
    return center_bearing + half, center_bearing - half


def test_a_door_jamb_rejected(node):
    # Victim 3.4 m ahead; a jamb 1.0 m away covers the left part of the
    # view, so the bbox is the visible part of the person only.
    jamb_edge = math.radians(3.5)
    jamb = (jamb_edge, math.radians(25.0), 1.0)
    scan = make_scan(person_legs(0.0, 3.4) + [jamb])
    _, right = person_span(0.0, 3.4)

    result = detect(node, scan, jamb_edge, right)

    assert result is not None
    assert result['distance_m'] == pytest.approx(3.4, abs=0.15)


def test_b_table_legs_rejected(node):
    # Victim 2.44 m ahead; a 4 cm table leg at 0.8 m sits exactly on the
    # bbox centre bearing, and a second leg at 0.95 m is also in the span.
    # Neither may be chosen: the gate applies to every candidate cluster.
    scan = make_scan(
        person_legs(0.0, 2.44)
        + [blob(0.0, 0.8, 0.04), blob(math.radians(-5.5), 0.95, 0.04)]
    )
    left, right = person_span(0.0, 2.44)

    result = detect(node, scan, left, right)

    assert result is not None
    assert result['distance_m'] == pytest.approx(2.44, abs=0.15)


@pytest.mark.parametrize('distance', [0.76, 0.80])
def test_c_clean_close_accepted(node, distance):
    scan = make_scan(person_legs(0.0, distance))
    left, right = person_span(0.0, distance)

    result = detect(node, scan, left, right)

    assert result is not None
    assert result['distance_m'] == pytest.approx(distance, abs=0.1)


@pytest.mark.parametrize('distance', [2.70, 2.75])
def test_d_clean_mid_accepted(node, distance):
    scan = make_scan(person_legs(math.radians(2.0), distance))
    left, right = person_span(math.radians(2.0), distance)

    result = detect(node, scan, left, right)

    assert result is not None
    assert result['distance_m'] == pytest.approx(distance, abs=0.15)


def test_e_edge_cut_bbox_publishes_nothing(node):
    # Person at the far right of the ~182 deg image, partly outside it:
    # the bbox is clipped at x2 = 320, its width understates the person,
    # so no distance is published rather than a possibly wrong one.
    center = math.radians(-88.0)
    scan = make_scan(person_legs(center, 1.5) + [blob(math.radians(-84.0), 0.6, 0.04)])
    left, right = person_span(center, 1.5)

    result = detect(node, scan, left, right)

    assert result is None


def test_e_bbox_near_but_not_at_edge_still_measured(node):
    # Same side of the image, fully inside it: normal gated behaviour.
    center = math.radians(-75.0)
    scan = make_scan(person_legs(center, 1.5))
    left, right = person_span(center, 1.5)
    assert bearing_to_pixel(right) < IMAGE_CENTER_X + IMAGE_HALF_WIDTH - 2.0

    result = detect(node, scan, left, right)

    assert result is not None
    assert result['distance_m'] == pytest.approx(1.5, abs=0.15)


def test_output_format_unchanged(node):
    scan = make_scan(person_legs(0.0, 2.0))
    left, right = person_span(0.0, 2.0)

    result = detect(node, scan, left, right)

    assert set(result) == {
        'victim_id', 'class', 'confidence', 'distance_m',
        'camera_x', 'angle_deg', 'detection_age_s',
    }
