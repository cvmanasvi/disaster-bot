import json
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


# =================================================================
# burger_cam wide-angle lens model
# =================================================================
#
# The Gazebo burger_cam is a 320x240 "wideanglecamera" with a custom
# lens (r ~ tan(theta / 4)) scaled to a 3.183 rad horizontal FOV.
# CameraInfo reports a pinhole fx=277, which does NOT describe this
# lens, so it must not be used for bearing.
#
# Empirically validated against LiDAR in simulation.
# Positive angle = left, matching the LiDAR / ROS convention.

IMAGE_CENTER_X = 160.0
IMAGE_HALF_WIDTH = 160.0
LENS_HFOV_RAD = 3.183


def pixel_to_bearing(pixel_x):
    normalized = (pixel_x - IMAGE_CENTER_X) / IMAGE_HALF_WIDTH
    return -4.0 * math.atan(normalized * math.tan(LENS_HFOV_RAD / 8.0))


class VictimDistanceEstimator(Node):

    def __init__(self):
        super().__init__('victim_distance_estimator')

        self.latest_scan = None
        self.latest_scan_time = None

        # True while a detection is considered current; used only so the
        # "stale" warning is logged once per loss, not every timer tick.
        self.detection_active = False
        self.last_detection_time = None

        # ---------------------------------------------------------
        # Freshness (ROS time: simulation time when use_sim_time=true)
        # ---------------------------------------------------------
        self.detection_timeout = 1.0
        self.scan_timeout = 0.5

        # ---------------------------------------------------------
        # LiDAR association
        # ---------------------------------------------------------

        # Extra margin added on each side of the bbox angular span
        # (YOLO / LiDAR are not captured at the same instant).
        self.span_padding = math.radians(2.0)

        # Never search a window narrower than this half-width.
        self.minimum_half_span = math.radians(2.0)

        # Ignore extremely close readings that are unlikely
        # to correspond to the detected victim.
        self.minimum_victim_distance = 0.20

        # Outlier protection: the chosen range must be supported by at
        # least this many rays within cluster_tolerance of it.
        self.minimum_support = 2
        self.cluster_tolerance = 0.15

        # Size gate: a person at distance d spans about width / d rad in
        # the image, so a cluster nearer than
        # person_min_width_m / bbox_angular_width cannot be the person
        # (e.g. a door jamb or table leg inside the bbox span).
        self.declare_parameter('person_min_width_m', 0.25)
        self.person_min_width_m = float(
            self.get_parameter('person_min_width_m').value
        )

        # A bbox touching the image edge may be truncated, so its width
        # understates the person and the size gate cannot be trusted:
        # no distance is published for such detections.
        self.image_edge_margin_px = 2.0

        # ---------------------------------------------------------
        # Distance smoothing
        # ---------------------------------------------------------
        self.previous_distance = None
        self.smoothing_factor = 0.35

        # A jump larger than this means a different target: restart
        # the filter instead of blending two objects together.
        self.smoothing_reset_jump = 0.5

        # ---------------------------------------------------------
        # ROS subscriptions
        # ---------------------------------------------------------
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.detection_subscription = self.create_subscription(
            String,
            '/victim_detections',
            self.detection_callback,
            10
        )

        # ---------------------------------------------------------
        # ROS publisher
        # ---------------------------------------------------------
        self.distance_publisher = self.create_publisher(
            String,
            '/victim_distance',
            10
        )

        # Estimates are published once per YOLO detection message.
        # This timer only watches for detections going stale.
        self.timer = self.create_timer(
            0.2,
            self.check_detection_timeout
        )

        self.get_logger().info(
            '📏 Victim Distance Estimator Started'
        )

        self.get_logger().info(
            '📡 Listening to /scan and /victim_detections'
        )

        if not self.get_parameter('use_sim_time').value:
            self.get_logger().warning(
                '⚠️ use_sim_time is false: freshness timeouts will use '
                'wall-clock time. Run with -p use_sim_time:=true in simulation.'
            )

    # =============================================================
    # Time helpers
    # =============================================================

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    # =============================================================
    # LiDAR callback
    # =============================================================

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.latest_scan_time = self._now_sec()

    # =============================================================
    # Staleness watchdog
    # =============================================================

    def check_detection_timeout(self):

        if not self.detection_active:
            return

        detection_age = self._now_sec() - self.last_detection_time

        if detection_age > self.detection_timeout:

            self.detection_active = False
            self.previous_distance = None

            self.get_logger().warning(
                f'⚠️ YOLO detection stale ({detection_age:.2f} s) '
                f'— no current victim'
            )

    # =============================================================
    # YOLO detection callback
    # =============================================================

    def detection_callback(self, msg):

        try:
            detections = json.loads(msg.data)

        except Exception as e:

            self.get_logger().error(
                f'❌ Detection parsing error: {e}'
            )

            return

        if not isinstance(detections, list) or not detections:
            return

        self.last_detection_time = self._now_sec()
        self.detection_active = True

        try:
            self.process_detection(detections)

        except Exception as e:

            self.get_logger().error(
                f'❌ Distance estimation error: {e}'
            )

    # =============================================================
    # Main processing
    # =============================================================

    def process_detection(self, detections):

        # MVP: only the most confident person is tracked.
        detection = max(
            detections,
            key=lambda d: float(d.get('confidence', 0.0))
        )

        victim_id = detection['victim_id']
        confidence = float(detection['confidence'])
        center_x = float(detection['center']['x'])

        bbox = detection.get('bbox')

        if bbox is not None:
            left_x = float(bbox['x1'])
            right_x = float(bbox['x2'])
        else:
            left_x = right_x = center_x

        # ---------------------------------------------------------
        # Pixels → LiDAR bearings (wide-angle model, + = left)
        # ---------------------------------------------------------

        center_angle = pixel_to_bearing(center_x)

        # Left image edge maps to the larger (more positive) angle.
        span_max = pixel_to_bearing(left_x)
        span_min = pixel_to_bearing(right_x)

        # ---------------------------------------------------------
        # Size gate (uses the unpadded bbox angular width)
        # ---------------------------------------------------------

        image_right_edge = IMAGE_CENTER_X + IMAGE_HALF_WIDTH

        if bbox is not None and (
            left_x <= self.image_edge_margin_px
            or right_x >= image_right_edge - self.image_edge_margin_px
        ):

            self.get_logger().warning(
                f'⚠️ Victim {victim_id}: bbox touches image edge '
                f'(x1={left_x:.0f}, x2={right_x:.0f}) — no distance',
                throttle_duration_sec=2.0
            )

            return

        angular_width = span_max - span_min

        minimum_distance = self.minimum_victim_distance

        if angular_width > 0.0:
            minimum_distance = max(
                minimum_distance,
                self.person_min_width_m / angular_width
            )

        span_max = max(span_max, center_angle + self.minimum_half_span)
        span_min = min(span_min, center_angle - self.minimum_half_span)

        span_max += self.span_padding
        span_min -= self.span_padding

        # ---------------------------------------------------------
        # LiDAR freshness
        # ---------------------------------------------------------

        now = self._now_sec()

        if (
            self.latest_scan is None
            or now - self.latest_scan_time > self.scan_timeout
        ):

            self.get_logger().warning(
                f'⚠️ Victim {victim_id}: no fresh LiDAR scan',
                throttle_duration_sec=2.0
            )

            return

        # ---------------------------------------------------------
        # Find LiDAR distance
        # ---------------------------------------------------------

        distance, support = self.get_lidar_distance(
            self.latest_scan,
            span_min,
            span_max,
            minimum_distance
        )

        if distance is None:

            self.get_logger().warning(
                f'⚠️ Victim {victim_id}: no usable LiDAR points in '
                f'[{math.degrees(span_min):.1f}°, '
                f'{math.degrees(span_max):.1f}°] '
                f'beyond {minimum_distance:.2f} m',
                throttle_duration_sec=2.0
            )

            return

        # ---------------------------------------------------------
        # Smooth distance
        # ---------------------------------------------------------

        if (
            self.previous_distance is None
            or abs(distance - self.previous_distance)
            > self.smoothing_reset_jump
        ):

            filtered_distance = distance

        else:

            filtered_distance = (
                self.smoothing_factor * distance
                +
                (1.0 - self.smoothing_factor)
                * self.previous_distance
            )

        self.previous_distance = filtered_distance

        # ---------------------------------------------------------
        # Create output
        # ---------------------------------------------------------

        detection_age = now - self.last_detection_time

        result = {

            'victim_id': victim_id,

            'class': 'person',

            'confidence': round(
                confidence,
                3
            ),

            'distance_m': round(
                filtered_distance,
                3
            ),

            'camera_x': round(
                center_x,
                1
            ),

            'angle_deg': round(
                math.degrees(center_angle),
                2
            ),

            'detection_age_s': round(
                detection_age,
                3
            )
        }

        output = String()

        output.data = json.dumps(result)

        self.distance_publisher.publish(
            output
        )

        self.get_logger().info(

            f'👤 VICTIM {victim_id} | '

            f'Confidence: '
            f'{confidence:.2f} | '

            f'Angle: '
            f'{math.degrees(center_angle):+.1f}° | '

            f'Distance: '
            f'{filtered_distance:.2f} m '
            f'(raw {distance:.2f} m, {support} rays)'
        )

    # =============================================================
    # LiDAR association
    # =============================================================

    def get_lidar_distance(
        self,
        scan,
        span_min,
        span_max,
        minimum_distance=None
    ):
        """Nearest well-supported range inside [span_min, span_max].

        Ranges nearer than minimum_distance (the size gate) are ignored
        for every candidate cluster, not just the nearest one.

        Returns (distance, support) or (None, 0).
        """

        if minimum_distance is None:
            minimum_distance = self.minimum_victim_distance

        span_center = (span_min + span_max) / 2.0
        half_width = (span_max - span_min) / 2.0

        valid_ranges = []

        for i, distance in enumerate(
            scan.ranges
        ):

            # Ignore invalid LiDAR values
            if not math.isfinite(distance):
                continue

            if distance < scan.range_min:
                continue

            if distance > scan.range_max:
                continue

            if distance < minimum_distance:
                continue

            # Calculate angle of this LiDAR ray
            angle = (
                scan.angle_min
                +
                i * scan.angle_increment
            )

            # Normalize difference to [-pi, pi]
            difference = math.atan2(
                math.sin(
                    angle - span_center
                ),
                math.cos(
                    angle - span_center
                )
            )

            # Keep only rays inside the victim's angular span
            if abs(difference) <= half_width:

                valid_ranges.append(
                    distance
                )

        valid_ranges.sort()

        # ---------------------------------------------------------
        # Nearest supported cluster:
        # walk outward from the closest reading and accept the first
        # range that has enough neighbours within cluster_tolerance.
        # A lone floor/wall ray cannot win on its own.
        # ---------------------------------------------------------

        for i, nearest in enumerate(valid_ranges):

            cluster = [
                r for r in valid_ranges[i:]
                if r - nearest <= self.cluster_tolerance
            ]

            if len(cluster) >= self.minimum_support:

                return cluster[len(cluster) // 2], len(cluster)

        return None, 0

    # =============================================================
    # Shutdown
    # =============================================================

    def destroy_node(self):

        self.get_logger().info(
            '🛑 Victim Distance Estimator stopped'
        )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = VictimDistanceEstimator()

    try:

        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
