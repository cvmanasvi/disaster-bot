"""Timing of identical consecutive victim_distance_estimator /rosout lines (analysis only).
True duplicate processes publish the same detection ~simultaneously (same stamp); one process
repeating identical text when nothing moves is spaced by the detection period. usage: dupe_timing.py <run_dir>"""
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rcl_interfaces.msg import Log
r = sys.argv[1]
rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=f'{r}/bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
rd.set_filter(rosbag2_py.StorageFilter(topics=['/rosout']))
est, yolo = [], []
while rd.has_next():
    _, d, _ = rd.read_next()
    m = deserialize_message(d, Log)
    t = m.stamp.sec + m.stamp.nanosec * 1e-9
    if m.name == 'victim_distance_estimator' and 'Distance' in m.msg:
        est.append((t, m.msg))
    elif m.name == 'yolo_detector':
        yolo.append(t)
est.sort()
dts = [b[0] - a[0] for a, b in zip(est, est[1:]) if a[1] == b[1] and b[0] - a[0] < 0.5]
allg = [b[0] - a[0] for a, b in zip(est, est[1:])]
same_stamp = sum(1 for a, b in zip(est, est[1:]) if abs(b[0] - a[0]) < 1e-3)
print(f'estimator lines {len(est)}, yolo lines {len(yolo)}, ratio {len(est)/max(1,len(yolo)):.3f}')
print(f'identical-text consecutive pairs <0.5 s: {len(dts)}, dt min {min(dts):.3f} median {sorted(dts)[len(dts)//2]:.3f} s' if dts else 'no identical pairs')
print(f'any consecutive estimator lines with |dt|<1 ms (same detection twice): {same_stamp}')
print(f'all consecutive dt: min {min(allg):.3f} median {sorted(allg)[len(allg)//2]:.3f} s')
ts = [b[0] for a, b in zip(est, est[1:]) if a[1] == b[1] and b[0] - a[0] < 0.5]
import collections
print('identical pairs by 20 s sim bin:', dict(sorted(collections.Counter(int(t // 20) * 20 for t in ts).items())))
