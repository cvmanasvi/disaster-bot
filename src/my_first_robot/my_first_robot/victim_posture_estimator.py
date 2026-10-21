"""
Optional victim posture node (YOLO11m-pose), parallel to the validated detector.

    /camera/image_raw  --+
                         +--> victim_posture_estimator --> /victim_posture
    /victim_detections --+                                 (std_msgs/String JSON)

Runs with the YOLO venv like yolo_detector (it imports Ultralytics):

    ~/ros2_ws/yolo_env/bin/python \
        src/my_first_robot/my_first_robot/victim_posture_estimator.py \
        --ros-args -p use_sim_time:=true -p model_path:=/path/to/yolo11m-pose.pt

It publishes ONLY /victim_posture. It does not publish /victim_detections,
/victim_distance or any velocity topic, and nothing in the validated mission
subscribes to /victim_posture, so it cannot change robot motion. Distance is
never derived from posture. Inference runs on a worker thread on the newest
frame only; a failed frame produces a status 'error' payload and the node
keeps running. See victim_posture_core for the message fields.
"""

import collections
import json
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:  # run as a script from the source tree (like yolo_detector)
    import victim_posture_core as core
except ImportError:  # imported as part of the installed package
    from my_first_robot import victim_posture_core as core  # type: ignore[no-redef]


def image_to_bgr(msg):
    """Convert an rgb8 / bgr8 sensor_msgs/Image to a BGR numpy array."""
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'unsupported encoding {msg.encoding}')
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    a = a[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return a[:, :, ::-1].copy() if msg.encoding == 'rgb8' else a.copy()


class VictimPostureEstimator(Node):

    def __init__(self):
        super().__init__('victim_posture_estimator')
        p = {
            'model_path': '',
            'pose_confidence': 0.50,
            'keypoint_confidence': 0.5,
            'min_keypoints': 5,
            'association_iou_min': 0.30,
            'association_ambiguity_margin': 0.10,
            'association_max_dt_s': 0.5,
            # 2026-10-14 saved-image evaluation: containment + keypoint guard
            # fixes head-only YOLO11n boxes; values <= 0 disable a rule.
            'association_containment_min': 0.70,
            'association_containment_min_keypoints': 2,
            'pose_dedup_iou': 0.50,
            'pose_dedup_containment': 0.70,
            'upright_max_deg': 30.0,
            'horizontal_min_deg': 60.0,
            'max_rate_hz': 1.0,
            'imgsz': 640,
        }
        for k, v in p.items():
            self.declare_parameter(k, v)
        g = {k: self.get_parameter(k).value for k in p}
        self.cfg = {k: g[k] for k in (
            'pose_confidence', 'keypoint_confidence', 'min_keypoints', 'association_iou_min',
            'association_ambiguity_margin', 'association_max_dt_s', 'upright_max_deg',
            'horizontal_min_deg', 'association_containment_min_keypoints')}
        for k in ('association_containment_min', 'pose_dedup_iou', 'pose_dedup_containment'):
            self.cfg[k] = g[k] if g[k] > 0 else None
        self.min_period = 1.0 / g['max_rate_hz'] if g['max_rate_hz'] > 0 else 0.0
        self.imgsz = int(g['imgsz'])
        self.model_path = os.path.expanduser(str(g['model_path']))
        if not os.path.isfile(self.model_path):
            raise RuntimeError(f'model_path not found: {self.model_path!r}')
        from ultralytics import YOLO
        self.model = YOLO(self.model_path)
        self.model_name = os.path.basename(self.model_path)

        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.latest = None
        self.history = collections.deque(maxlen=50)
        self.stats = collections.Counter()
        self.running = True

        self.pub = self.create_publisher(String, '/victim_posture', 10)
        self.create_subscription(Image, '/camera/image_raw', self._image, qos_profile_sensor_data)
        self.create_subscription(String, '/victim_detections', self._detections, 10)
        self.worker = threading.Thread(target=self._work, daemon=True)
        self.worker.start()
        self.get_logger().info(
            f'Victim posture estimator started: {self.model_name}, '
            f'conf {self.cfg["pose_confidence"]}, '
            f'>= {self.cfg["min_keypoints"]} keypoints > {self.cfg["keypoint_confidence"]}, '
            f'max {g["max_rate_hz"]} Hz -> /victim_posture (no posture classes; torso-angle hint)')

    def _image(self, msg):
        with self.lock:
            self.latest = (msg, time.time())
        self.wake.set()

    def _detections(self, msg):
        dets = core.parse_detections(msg.data)
        if dets is None:
            self.stats['bad_detection_msgs'] += 1
            return
        stamps = [d['timestamp'] for d in dets if np.isfinite(d['timestamp'])]
        t = stamps[0] if stamps else time.time()
        with self.lock:
            self.history.append((t, dets))

    def _infer(self, image):
        r = self.model.predict(image, classes=[0], conf=self.cfg['pose_confidence'],
                               imgsz=self.imgsz, verbose=False)[0]
        return core.extract_pose_outputs(r)

    def _work(self):
        last = 0.0
        while self.running:
            if not self.wake.wait(timeout=0.5):
                continue
            wait = self.min_period - (time.monotonic() - last)
            if wait > 0.0:
                time.sleep(wait)
            with self.lock:
                item, self.latest = self.latest, None
                history = list(self.history)
            self.wake.clear()
            if item is None:
                continue
            msg, t_recv = item
            last = time.monotonic()
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            meta = {'image_stamp': round(stamp, 3),
                    'image_recv_wall': round(t_recv, 3), 'model': self.model_name}
            t0 = time.monotonic()
            try:
                image = image_to_bgr(msg)
            except Exception as exc:  # noqa: B902
                out = core.error_message(f'{type(exc).__name__}: {exc}', meta)
            else:
                out = core.process_frame(self._infer, image, history, t_recv, self.cfg, meta,
                                         round(stamp, 3))
            out['inference_s'] = round(time.monotonic() - t0, 3)
            self.stats[out['status']] += 1
            if out['status'] != 'ok':
                self.get_logger().error(f'posture frame failed: {out.get("error")}',
                                        throttle_duration_sec=5.0)
            try:
                self.pub.publish(String(data=json.dumps(out)))
            except Exception as exc:  # noqa: B902 - e.g. during shutdown
                self.get_logger().warning(f'publish failed: {exc}')

    def destroy_node(self):
        self.running = False
        self.wake.set()
        self.get_logger().info(f'Victim posture estimator stopped: {dict(self.stats)}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VictimPostureEstimator()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: B902
        # SIGINT can shut the context down while spin() builds a wait set
        # (rclpy RCLError); that is a normal stop, anything else is not.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
