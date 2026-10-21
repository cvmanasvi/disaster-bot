"""
Navigation-facing detection guard (optional; off unless launched).

    /victim_detections --+                          +--> /victim_detections_nav
    /scan  --------------+--> victim_detection_guard +    (boxes that passed, unchanged)
                                                     +--> /victim_detection_guard
                                                          (per-frame decisions, advisory)

victim_distance_estimator (frozen) is the only node that turns detections
into the /victim_distance readings autonomous_search approaches. It
associates the max-confidence box with the nearest supported LiDAR cluster
and has no RVC checks. disaster_bot_posture.launch.py nav_guard:=true remaps
the estimator's input to /victim_detections_nav and starts this node, so
every box the estimator sees has passed victim_localizer_core
.guard_detections: the same single-box association with RVC (no LiDAR hit
below the box bottom; support in the box centre), a fresh scan and a
non-stale image stamp. Each box must also be readable by the estimator
exactly as checked (victim_id, confidence, bbox dict, center.x at the bbox
midpoint, the frame's image_stamp), so one bad box cannot make the
estimator drop the frame or measure other bearings. Boxes are forwarded
verbatim; a frame where no box passes is not forwarded at all (the
estimator publishes nothing, as for any frame it cannot measure). If this
node is not running the estimator receives nothing, so no distance can be
produced.

This node publishes no velocity and no distance.

    python3 src/my_first_robot/my_first_robot/victim_detection_guard.py \
        --ros-args -p use_sim_time:=true
"""

import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

try:  # run as a script from the source tree (like the localizer)
    import victim_localizer_core as core
except ImportError:  # imported as part of the installed package
    from my_first_robot import victim_localizer_core as core  # type: ignore[no-redef]


class VictimDetectionGuard(Node):

    def __init__(self):
        super().__init__('victim_detection_guard')
        defaults = dict(core.GUARD_DEFAULTS, scan_timeout_s=0.5)
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.g = {k: (bool if isinstance(v, bool) else float)(self.get_parameter(k).value)
                  for k, v in defaults.items()}
        self.latest_scan = None
        self.latest_scan_time = None
        self.stats = {'frames': 0, 'frames_forwarded': 0, 'boxes': 0, 'boxes_forwarded': 0}
        self.pub = self.create_publisher(String, '/victim_detections_nav', 10)
        self.audit = self.create_publisher(String, '/victim_detection_guard', 10)
        # same QoS as victim_distance_estimator
        self.create_subscription(LaserScan, '/scan', self._scan, 10)
        self.create_subscription(String, '/victim_detections', self._detections, 10)
        self.get_logger().info(
            f'Victim detection guard started: /victim_detections -> /victim_detections_nav '
            f'(RVC single-box checks, max image age {self.g["max_image_age_s"]} s, '
            f'scan timeout {self.g["scan_timeout_s"]} s, exclusive_cluster_check '
            f'{self.g["exclusive_cluster_check"]})')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _scan(self, msg):
        self.latest_scan = msg
        self.latest_scan_time = self._now()

    def _detections(self, msg):
        now = self._now()
        try:
            dets = json.loads(msg.data)
        except (TypeError, ValueError):
            dets = None
        stamp = core._frame_stamp(dets) if isinstance(dets, list) else None
        image_age = None if stamp is None or not math.isfinite(stamp) else now - stamp
        scan_age = None if self.latest_scan_time is None else now - self.latest_scan_time
        fresh = scan_age is not None and scan_age <= self.g['scan_timeout_s']
        scan = self.latest_scan if fresh else None
        try:
            forwarded, decisions = core.guard_detections(dets, scan, image_age, self.g)
        except Exception as exc:  # noqa: B902 - a bad frame must never pass unchecked
            forwarded, decisions = [], [{'index': None, 'status': 'withheld',
                                         'reason': f'guard_error: {type(exc).__name__}'}]
            self.get_logger().error(f'guard failed: {exc}', throttle_duration_sec=5.0)
        n = len(dets) if isinstance(dets, list) else 0
        self.stats['frames'] += 1
        self.stats['boxes'] += n
        self.stats['boxes_forwarded'] += len(forwarded)
        try:
            if forwarded:
                self.stats['frames_forwarded'] += 1
                self.pub.publish(String(data=json.dumps(forwarded)))
            self.audit.publish(String(data=json.dumps({
                'image_stamp': stamp,
                'image_age_s': None if image_age is None else round(image_age, 3),
                'scan_age_s': None if scan_age is None else round(scan_age, 3),
                'boxes': n, 'forwarded': len(forwarded), 'withheld': n - len(forwarded),
                'decisions': decisions,
                'note': 'advisory; /victim_detections_nav feeds the estimator'})))
        except Exception as exc:  # noqa: B902 - e.g. during shutdown
            self.get_logger().warning(f'publish failed: {exc}')

    def destroy_node(self):
        self.get_logger().info(f'Victim detection guard stopped: {self.stats}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VictimDetectionGuard()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: B902
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
