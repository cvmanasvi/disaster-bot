"""
Optional per-person victim localizer (advisory; nothing in the mission consumes it).

    /victim_posture     --+  (fused candidates: yolo11n+pose, yolo11n, pose_only,
    /victim_detections  --+   pose_unassociated; or YOLO11n boxes as fallback)
    /scan               --+--> victim_localizer --> /victim_localizations
    /odom (optional)    --+                         (std_msgs/String JSON)

Runs with the system Python (no Ultralytics):

    python3 src/my_first_robot/my_first_robot/victim_localizer.py \
        --ros-args -p use_sim_time:=true [-p rvc_enabled:=true]

rvc_enabled (default false) turns on the RVC withhold checks of
victim_localizer_core (step 6); each output message carries 'rvc_enabled'.

Input selection (input_mode):
  * 'auto' (default): the fused /victim_posture candidates while that stream
    is healthy (a status 'ok' message within posture_timeout_s); otherwise the
    YOLO11n /victim_detections boxes, so a failed or disabled pose model does
    not stop localization. Only one stream is localized at a time, so one
    person never gets two records from two streams.
  * 'posture' / 'detections': that stream only.

Each output message covers one camera frame and lists every candidate with
'status', 'usable' and the LiDAR evidence (victim_localizer_core). The scan
used is the one whose header stamp is closest to the camera image stamp
(within scan_match_max_s); without an image stamp the newest scan younger than
scan_timeout_s is used, as in victim_distance_estimator, and image_age_s is
null. Positions are given in the robot frame (base_footprint, x forward) at
the image time, and in odom when an odom pose within odom_match_max_s exists.
track_id is an advisory identity from PersonTracker (odom frame only).

This node publishes ONLY /victim_localizations. It never publishes
/victim_distance, /victim_detections or velocity topics, so it cannot change
robot motion. The validated victim_distance_estimator keeps feeding
autonomous_search unchanged.
"""

import collections
import json
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

try:  # run as a script from the source tree (like the posture node)
    import victim_localizer_core as core
except ImportError:  # imported as part of the installed package
    from my_first_robot import victim_localizer_core as core  # type: ignore[no-redef]


def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class VictimLocalizer(Node):

    def __init__(self):
        super().__init__('victim_localizer')
        params = {
            'input_mode': 'auto',
            'posture_timeout_s': 3.0,
            'scan_match_max_s': 0.15,
            'scan_timeout_s': 0.5,
            'max_image_age_s': 4.0,
            'odom_match_max_s': 0.1,
            'track_gate_m': 0.5,
            'track_timeout_s': 3.0,
        }
        params.update(core.DEFAULTS)
        for k, v in params.items():
            self.declare_parameter(k, v)
        g = {k: self.get_parameter(k).value for k in params}
        self.mode = str(g['input_mode'])
        if self.mode not in ('auto', 'posture', 'detections'):
            raise RuntimeError(
                f'input_mode must be auto, posture or detections, not {self.mode!r}')
        self.cfg = {k: g[k] for k in core.DEFAULTS}
        self.g = g
        self.scans = collections.deque(maxlen=40)       # (header stamp, scan, arrival)
        self.odoms = collections.deque(maxlen=400)      # (header stamp, x, y, yaw)
        self.last_posture_ok = None
        self.tracker = core.PersonTracker(g['track_gate_m'], g['track_timeout_s'])
        self.stats = collections.Counter()

        self.pub = self.create_publisher(String, '/victim_localizations', 10)
        self.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.create_subscription(String, '/victim_posture', self._posture, 10)
        self.create_subscription(String, '/victim_detections', self._detections, 10)
        self.get_logger().info(
            f'Victim localizer started: input_mode {self.mode}, camera offset '
            f'{self.cfg["camera_offset_x_m"]} m, rvc_enabled {bool(self.cfg["rvc_enabled"])} '
            f'-> /victim_localizations (advisory)')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _scan(self, msg):
        self.scans.append((_stamp(msg.header), msg, self._now()))

    def _odom(self, msg):
        p = msg.pose.pose
        self.odoms.append((_stamp(msg.header), p.position.x, p.position.y, _yaw(p.orientation)))

    def _posture_healthy(self, now):
        return (self.last_posture_ok is not None
                and now - self.last_posture_ok <= self.g['posture_timeout_s'])

    def _posture(self, msg):
        now = self._now()
        status, cands, stamp = core.parse_posture(msg.data)
        self.stats['posture_' + status] += 1
        if status != 'ok':
            return
        self.last_posture_ok = now
        if self.mode in ('auto', 'posture'):
            self._process('posture', cands, stamp, None, now)

    def _detections(self, msg):
        now = self._now()
        if self.mode == 'posture' or (self.mode == 'auto' and self._posture_healthy(now)):
            return
        cands, stamp, wall = core.parse_detections(msg.data)
        if cands is None:
            self.stats['detections_malformed'] += 1
            return
        self._process('detections', cands, stamp, wall, now)

    def _pick_scan(self, image_stamp, now):
        scans = [(t, s) for t, s, _ in self.scans]
        if image_stamp is not None:
            scan, dt = core.select_scan(scans, image_stamp, self.g['scan_match_max_s'])
            return scan, dt, 'image_stamp'
        if self.scans:
            t, s, arrival = self.scans[-1]
            if now - arrival <= self.g['scan_timeout_s']:
                return s, None, 'newest_fresh'
        return None, None, 'newest_fresh'

    def _odom_at(self, t):
        best = min(self.odoms, key=lambda o: abs(o[0] - t), default=None)
        if best is None or abs(best[0] - t) > self.g['odom_match_max_s']:
            return None
        return best

    def _process(self, stream, cands, image_stamp, wall, now):
        image_age = None if image_stamp is None else now - image_stamp
        too_old = image_age is not None and image_age > self.g['max_image_age_s']
        scan, scan_dt, scan_rule = (None, None, 'image_too_old') if too_old else \
            self._pick_scan(image_stamp, now)
        try:
            results = core.localize_frame(cands, scan, self.cfg)
        except Exception as exc:  # noqa: B902 - a bad frame must never stop the node
            self.stats['frame_errors'] += 1
            self.get_logger().error(f'localization failed: {exc}', throttle_duration_sec=5.0)
            return
        odom = self._odom_at(image_stamp) if image_stamp is not None else None
        usable = [r for r in results if r['usable']]
        if odom is not None:
            _, ox, oy, oyaw = odom
            pts = []
            for r in usable:
                c, s = math.cos(oyaw), math.sin(oyaw)
                r['odom_x_m'] = round(ox + c * r['x_m'] - s * r['y_m'], 3)
                r['odom_y_m'] = round(oy + s * r['x_m'] + c * r['y_m'], 3)
                pts.append((r['odom_x_m'], r['odom_y_m']))
            for r, tid in zip(usable, self.tracker.update(image_stamp, pts)):
                r['track_id'] = tid
        out = {
            'schema': core.SCHEMA,
            'input_stream': stream,
            'image_stamp': None if image_stamp is None else round(image_stamp, 3),
            'image_age_s': None if image_age is None else round(image_age, 3),
            'detector_wall_stamp': wall,
            'scan_stamp': None if scan is None else round(_stamp(scan.header), 3),
            'scan_image_dt_s': None if scan_dt is None else round(scan_dt, 3),
            'scan_rule': scan_rule,
            'odom_available': odom is not None,
            'frame': 'base_footprint (x forward, y left) at image time',
            'candidates': len(results),
            'usable': len(usable),
            'rvc_enabled': bool(self.cfg['rvc_enabled']),
            'localizations': results,
            'note': 'advisory; not used by navigation',
        }
        self.stats[f'{stream}_frames'] += 1
        self.stats['usable_records'] += len(usable)
        try:
            self.pub.publish(String(data=json.dumps(out)))
        except Exception as exc:  # noqa: B902 - e.g. during shutdown
            self.get_logger().warning(f'publish failed: {exc}')

    def destroy_node(self):
        self.get_logger().info(f'Victim localizer stopped: {dict(self.stats)}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VictimLocalizer()
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
