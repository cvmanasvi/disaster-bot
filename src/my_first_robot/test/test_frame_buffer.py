"""Tests for the detector's newest-frame hand-off (frame_buffer) and static detector checks."""

import ast
import os
import threading
import time

from my_first_robot.frame_buffer import LatestFrameBuffer

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
YOLO = os.path.join(PKG, 'my_first_robot', 'yolo_detector.py')


def test_take_returns_newest_and_counts_drops():
    b = LatestFrameBuffer()
    for i in range(5):
        b.put(i)
    assert b.take(0.0) == 4
    assert b.stats() == {'received': 5, 'processed': 1, 'dropped': 4, 'pending': 0}


def test_taken_frame_is_never_handed_out_again():
    b = LatestFrameBuffer()
    b.put('a')
    assert b.take(0.0) == 'a'
    assert b.take(0.05) is None
    assert b.stats()['processed'] == 1


def test_take_times_out_without_frames_and_wake_releases():
    b = LatestFrameBuffer()
    t0 = time.monotonic()
    assert b.take(0.1) is None and time.monotonic() - t0 >= 0.09
    threading.Timer(0.05, b.wake).start()
    assert b.take(2.0) is None                                   # woken, nothing pending


def test_slow_consumer_skips_but_never_reorders_or_loses_count():
    b = LatestFrameBuffer()
    got = []
    stop = threading.Event()

    def consume():
        while not stop.is_set():
            x = b.take(0.05)
            if x is not None:
                got.append(x)
                time.sleep(0.01)                                  # slower than the producer

    t = threading.Thread(target=consume)
    t.start()
    for i in range(300):
        b.put(i)
        time.sleep(0.001)
    time.sleep(0.1)
    stop.set()
    t.join()
    s = b.stats()
    assert got == sorted(got) and len(set(got)) == len(got)
    assert got[-1] == 299                                         # recovers to the newest frame
    assert s['received'] == 300 and s['processed'] + s['dropped'] + s['pending'] == 300
    assert s['dropped'] > 0


def test_fast_consumer_processes_every_frame_at_low_rate():
    b = LatestFrameBuffer()
    got = []

    def consume():
        for _ in range(20):
            x = b.take(1.0)
            if x is not None:
                got.append(x)

    t = threading.Thread(target=consume)
    t.start()
    for i in range(20):
        b.put(i)
        time.sleep(0.02)
    t.join()
    assert got == list(range(20)) and b.stats()['dropped'] == 0


# ------------------------------------------------------------------ static detector checks
def _calls(path, name):
    tree = ast.parse(open(path).read())
    return [n for n in ast.walk(tree)
            if isinstance(n, ast.Call) and getattr(n.func, 'attr', None) == name]


def test_detector_topics_and_schema_unchanged():
    subs = [c.args[1].value for c in _calls(YOLO, 'create_subscription')]
    pubs = [c.args[1].value for c in _calls(YOLO, 'create_publisher')]
    assert subs == ['/camera/image_raw'] and pubs == ['/victim_detections']
    src = open(YOLO).read()
    for key in ("'victim_id'", "'class'", "'confidence'", "'bbox'", "'center'", "'timestamp'",
                "'image_stamp'"):
        assert key in src
    assert "'timestamp': time.time()" in src


def test_detector_model_and_threshold_default_to_production():
    decl = {c.args[0].value: c.args[1].value for c in _calls(YOLO, 'declare_parameter')}
    assert decl['model_path'] == 'yolo11n.pt' and decl['confidence_threshold'] == 0.50
    tree = ast.parse(open(YOLO).read())
    loads = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and getattr(n.func, 'id', None) == 'YOLO']
    # one model, loaded from the parameter (no hard-coded second detector)
    assert len(loads) == 1 and isinstance(loads[0].args[0], ast.Name)
    infer = _calls(YOLO, 'model')
    assert len(infer) == 1
    kw = {k.arg: k.value for k in infer[0].keywords}
    assert ast.unparse(kw['conf']) == 'self.confidence_threshold'
    assert ast.unparse(kw['classes']) == '[0]'
    src = open(YOLO).read()
    assert "self.model.names.get(0) != 'person'" in src


def test_detector_declares_frame_policy_default_legacy():
    decl = [c for c in _calls(YOLO, 'declare_parameter') if c.args[0].value == 'latest_frame_only']
    assert len(decl) == 1 and decl[0].args[1].value is False      # opt-in (2026-10-17 decision)
    src = open(YOLO).read()
    assert 'LatestFrameBuffer' in src and 'self.frames.put(msg)' in src
    # inference happens only in _process; the callback never calls the model in latest mode
    tree = ast.parse(src)
    cb = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == 'image_callback')
    first = cb.body[0]
    assert isinstance(first, ast.If) and isinstance(first.body[-1], ast.Return)
