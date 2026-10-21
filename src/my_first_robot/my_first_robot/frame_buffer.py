"""
Newest-frame hand-off between a ROS image callback and an inference worker.

The camera publishes faster than YOLO11n can process under Gazebo. Running
inference inside the subscription callback lets the subscription queue fill,
so the detector keeps working through old frames. With this buffer the
callback only stores the newest frame; a frame that is replaced before the
worker takes it is dropped and counted. A taken frame is never handed out
again. No ROS or Ultralytics dependency, so it is unit-testable.
"""

import threading


class LatestFrameBuffer:
    """Single-slot, thread-safe buffer that keeps only the newest item."""

    def __init__(self):
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._item = None
        self.received = 0
        self.taken = 0
        self.dropped = 0

    def put(self, item):
        """Store item, replacing (and counting as dropped) an untaken older one."""
        with self._lock:
            if self._item is not None:
                self.dropped += 1
            self._item = item
            self.received += 1
        self._event.set()

    def take(self, timeout=None):
        """Return the newest untaken item, waiting up to timeout s; None if there is none."""
        if not self._event.wait(timeout):
            return None
        with self._lock:
            item, self._item = self._item, None
            self._event.clear()
            if item is not None:
                self.taken += 1
        return item

    def wake(self):
        """Release a waiting take() (used at shutdown)."""
        self._event.set()

    def stats(self):
        with self._lock:
            return {'received': self.received, 'processed': self.taken, 'dropped': self.dropped,
                    'pending': int(self._item is not None)}
