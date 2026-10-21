import collections
import json
import statistics
import threading
import time

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
from ultralytics import YOLO

try:  # run as a script from the source tree (as the launch files do)
    from frame_buffer import LatestFrameBuffer
except ImportError:  # imported as part of the installed package
    from my_first_robot.frame_buffer import LatestFrameBuffer  # type: ignore[no-redef]


class YOLODetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        self.get_logger().info(
            '🚨 Disaster Bot YOLO Detector Started'
        )

        # --------------------------------------------------
        # YOLO
        # --------------------------------------------------

        # Defaults = the validated production detector: YOLO11n (path relative
        # to the launch cwd ~/ros2_ws) at confidence 0.50. model_path /
        # confidence_threshold select an optional alternative with the same
        # call and output schema, e.g. YOLO-World v2 (open vocabulary, built-in
        # 80-class vocabulary, class 0 = person; 2026-10-18 evaluation in
        # ~/disaster_bot_test_logs/detector_eval_20261018_000055).
        self.declare_parameter('model_path', 'yolo11n.pt')
        self.declare_parameter('confidence_threshold', 0.50)
        model_path = str(self.get_parameter('model_path').value)

        self.model = YOLO(model_path)
        if self.model.names.get(0) != 'person':
            raise RuntimeError(f'{model_path}: class 0 is not person')

        # Only consider detections above this confidence
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value)
        if not 0.0 < self.confidence_threshold <= 1.0:
            raise ValueError('confidence_threshold must be in (0, 1]')

        # Inference resolution (pixels on the longer side).  Default 640 matches
        # the validated production baseline.  Set to 480 for worldv2m to get
        # 2.2x faster inference with better lying-person recall (2026-10-18
        # offline evaluation in posture_improvement_20261018_152105/).
        self.declare_parameter('inference_imgsz', 640)
        self.imgsz = int(self.get_parameter('inference_imgsz').value)

        self.get_logger().info(
            f'Detector model {model_path}, confidence >= '
            f'{self.confidence_threshold}, imgsz={self.imgsz}')

        # --------------------------------------------------
        # ROS image conversion
        # --------------------------------------------------

        self.bridge = CvBridge()

        # --------------------------------------------------
        # Frame policy
        # --------------------------------------------------
        # latest_frame_only (default false = the original synchronous
        # callback, validated in the standing regressions). true: the image
        # callback only stores the newest frame and a worker thread runs YOLO
        # on it; frames that arrive while inference is running replace the
        # stored one and are dropped (counted), so detections are never made
        # on a backlog of old frames (image age 0.44 -> 0.165 s sim, about
        # 11 % fewer detections; 2026-10-17 ABBA test). Opt-in until the
        # 2.5 m 0 deg standing-regression deviation is resolved.

        self.declare_parameter('latest_frame_only', False)
        self.latest_frame_only = bool(
            self.get_parameter('latest_frame_only').value)
        self.frames = LatestFrameBuffer()
        self.timing = {'inference_s': collections.deque(maxlen=2000),
                       'image_age_s': collections.deque(maxlen=2000)}
        self.timed = 0
        self.timing_lock = threading.Lock()
        self.display_frame = None
        self.running = True

        # --------------------------------------------------
        # Camera subscriber
        # --------------------------------------------------

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # --------------------------------------------------
        # Detection publisher
        # --------------------------------------------------

        self.detection_publisher = self.create_publisher(
            String,
            '/victim_detections',
            10
        )

        self.get_logger().info(
            '📷 Listening to /camera/image_raw'
        )

        self.get_logger().info(
            '📡 Publishing detections on /victim_detections'
        )

        # For now we are testing with one simulated human.
        # Proper multi-person tracking will be added later.
        self.victim_id = 1

        self.get_logger().info(
            f'🖼️ Frame policy: '
            f'{"newest frame only" if self.latest_frame_only else "every queued frame (legacy)"}'
        )

        if self.latest_frame_only:
            self.worker = threading.Thread(target=self._work, daemon=True)
            self.worker.start()
            self.create_timer(0.05, self._show)

    def image_callback(self, msg):

        if self.latest_frame_only:
            self.frames.put(msg)
            return

        self.frames.received += 1
        self.frames.taken += 1
        annotated_frame = self._process(msg)

        if annotated_frame is not None:
            cv2.imshow(
                'Disaster Bot - YOLO Detection',
                annotated_frame
            )
            cv2.waitKey(1)

    def _work(self):

        while self.running and rclpy.ok():
            try:
                msg = self.frames.take(timeout=0.5)
                if msg is None:
                    continue
                annotated_frame = self._process(msg)
                if annotated_frame is not None:
                    self.display_frame = annotated_frame
            except Exception as e:  # noqa: B902 - the worker must keep running
                self.get_logger().error(f'❌ YOLO worker error: {e}')

    def _show(self):

        frame, self.display_frame = self.display_frame, None
        if frame is not None:
            cv2.imshow(
                'Disaster Bot - YOLO Detection',
                frame
            )
            cv2.waitKey(1)

    def _process(self, msg):
        """Run YOLO on one frame, publish detections, return the annotated frame."""
        try:

            # --------------------------------------------------
            # Convert ROS Image → OpenCV
            # --------------------------------------------------

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            # --------------------------------------------------
            # Run YOLO
            # --------------------------------------------------

            t_inference = time.monotonic()

            results = self.model(
                frame,
                verbose=False,
                classes=[0],  # COCO class 0 = person
                conf=self.confidence_threshold,
                imgsz=self.imgsz
            )

            inference_s = time.monotonic() - t_inference

            result = results[0]

            # Camera capture time (header stamp, sim time in simulation).
            # 'timestamp' below stays the wall time of publication.
            image_stamp = round(
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                3
            )

            detections = []

            # --------------------------------------------------
            # Process detected people
            # --------------------------------------------------

            for box in result.boxes:

                # Bounding box
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                # Confidence
                confidence = float(box.conf[0])

                # Center of bounding box
                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0

                detection = {
                    'victim_id': self.victim_id,

                    'class': 'person',

                    'confidence': round(
                        confidence,
                        3
                    ),

                    'bbox': {
                        'x1': round(x1, 1),
                        'y1': round(y1, 1),
                        'x2': round(x2, 1),
                        'y2': round(y2, 1)
                    },

                    'center': {
                        'x': round(center_x, 1),
                        'y': round(center_y, 1)
                    },

                    'timestamp': time.time(),

                    'image_stamp': image_stamp
                }

                detections.append(detection)

            # --------------------------------------------------
            # Publish detections
            # --------------------------------------------------

            if detections:

                message = String()

                message.data = json.dumps(
                    detections
                )

                self.detection_publisher.publish(
                    message
                )

                self.get_logger().info(
                    f'👤 PERSON DETECTED | '
                    f'Count: {len(detections)} | '
                    f'Confidence: '
                    f'{detections[0]["confidence"]:.2f}'
                )

            self._record_timing(inference_s, image_stamp)

            # --------------------------------------------------
            # YOLO results for display
            # --------------------------------------------------

            return result.plot()

        except Exception as e:

            self.get_logger().error(
                f'❌ YOLO error: {e}'
            )

            return None

    def _record_timing(self, inference_s, image_stamp):

        age = self.get_clock().now().nanoseconds * 1e-9 - image_stamp
        with self.timing_lock:
            self.timing['inference_s'].append(inference_s)
            self.timing['image_age_s'].append(age)
            self.timed += 1
            n = self.timed
        if n % 200 == 0:
            self.get_logger().info(f'⏱️ {self.timing_summary()}')

    def timing_summary(self):

        with self.timing_lock:
            inf = sorted(self.timing['inference_s'])
            age = sorted(self.timing['image_age_s'])

        def med(v):
            return round(statistics.median(v), 3) if v else None

        return (f'frames {self.frames.stats()} | inference_s median {med(inf)} | '
                f'image age at publication (node clock) median {med(age)} '
                f'(last {len(inf)} frames)')

    def destroy_node(self):

        self.running = False
        self.frames.wake()
        if self.latest_frame_only:
            self.worker.join(timeout=5.0)

        self.get_logger().info(f'⏱️ final {self.timing_summary()}')

        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = YOLODetector()

    try:

        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):

        node.get_logger().info(
            '🛑 YOLO Detector stopped'
        )

    except Exception:  # noqa: B902
        # SIGINT can shut the context down while spin() builds a wait set
        # (rclpy RCLError); that is a normal stop, anything else is not.
        if rclpy.ok():
            raise

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


