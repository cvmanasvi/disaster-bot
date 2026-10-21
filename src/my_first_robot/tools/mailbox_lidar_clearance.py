"""Mailbox and door-jamb clearance during the house entry (analysis only).

usage: mailbox_lidar_clearance.py <run_dir> [<t0> <t1>]
Without t0/t1 the window comes from search.log: ENTRANCE_APPROACH -> VERIFY_INSIDE (whole
entry); the crossing (ENTRANCE_CROSS -> VERIFY_INSIDE) is reported separately.

Authoritative value: the burger_cam collision shapes (model.sdf: 0.14 m base box at x -0.032,
wheel cylinders r 0.033 x 0.018 at y +-0.08, LiDAR cylinder r 0.0508 at x -0.020, caster sphere
r 0.005 at x -0.081) swept along the Gazebo ground-truth pose (gt.csv), against the mailbox
COLLISION MESH (mailbox.dae sliced over the robot's height 0..0.19 m: a square post 0.044 m
wide, x 0.862..0.905, east face x ~0.905) and the door jambs (wall y -0.25..-0.10, gap
x 0.662..1.562). The 0.105 m disc around the robot origin is printed only as a comparison with
the earlier method. LiDAR returns (bag /scan + GT pose) are supporting evidence only.
"""
import bisect
import math
import re
import sys

import numpy as np

DAE = ('/opt/ros/lyrical/share/turtlebot3_gazebo/models/turtlebot3_house/'
       'mailbox/meshes/mailbox.dae')
MAILBOX_POSE = (0.883462, -0.576236, -3.14159)   # turtlebot3_house/model.sdf <pose> x y yaw
ROBOT_TOP = 0.19                                 # LiDAR cylinder top 0.161 + 0.0275
DOOR_X = (0.662, 1.562)
WALL_Y = (-0.25, -0.10)
JAMBS = {
    'west': (DOOR_X[0] - 0.60, DOOR_X[0], *WALL_Y),
    'east': (DOOR_X[1], DOOR_X[1] + 0.60, *WALL_Y),
}
# Robot frame (base_footprint origin): rectangles (x0, x1, y0, y1) and circles (cx, cy, r).
RECTS = [
    (-0.102, 0.038, -0.070, 0.070),     # base_collision box
    (-0.033, 0.033, 0.071, 0.089),      # wheel_left_collision
    (-0.033, 0.033, -0.089, -0.071),    # wheel_right_collision
]
CIRCLES = [(-0.020, 0.0, 0.0508), (-0.081, 0.0, 0.005)]   # LiDAR cylinder, caster sphere
LIDAR_X = -0.032
DISC_R = 0.105


def mailbox_slice_polygon():
    """Union of the mesh cross-sections over the robot's height -> convex hull (world x, y)."""
    s = open(DAE).read()
    arr = re.search(r'positions-array" count="\d+">([^<]+)<', s).group(1)
    pos = np.array(list(map(float, arr.split()))).reshape(-1, 3)
    tri = re.search(r'<triangles[^>]*>.*?<p>([^<]+)</p>', s, re.S).group(1)
    idx = np.array(list(map(int, tri.split()))).reshape(-1, 3, 3)[:, :, 0]
    m = np.eye(4)
    m[:3, 3] = [0, 979.6222, -3.708471]              # inner <node><matrix>
    ax = np.array([5.33851e-8, 0.7071068, 0.7071068])
    ax /= np.linalg.norm(ax)
    th = math.radians(-180)
    k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    rot = np.eye(3) + math.sin(th) * k + (1 - math.cos(th)) * k @ k
    scale = np.diag([0.0367934, 0.0367934, 0.06453677])
    p = (np.c_[pos, np.ones(len(pos))] @ m.T)[:, :3]
    p = (rot @ scale @ p.T).T * 0.0254               # <unit meter="0.0254"/>
    mx, my, myaw = MAILBOX_POSE
    c, sn = math.cos(myaw), math.sin(myaw)
    w = np.c_[mx + c * p[:, 0] - sn * p[:, 1], my + sn * p[:, 0] + c * p[:, 1], p[:, 2]]
    pts, extents = [], []
    for z in np.linspace(0.0005, ROBOT_TOP, 40):
        sl = []
        for t in idx:
            v = w[t]
            for a, b in ((0, 1), (1, 2), (2, 0)):
                za, zb = v[a, 2], v[b, 2]
                if (za - z) * (zb - z) < 0:
                    f = (z - za) / (zb - za)
                    sl.append(v[a, :2] + f * (v[b, :2] - v[a, :2]))
        sl = np.array(sl)
        extents.append((sl[:, 0].min(), sl[:, 0].max(), sl[:, 1].min(), sl[:, 1].max()))
        pts.extend(sl)
    return convex_hull(np.array(pts)), np.array(extents)


def convex_hull(p):
    p = sorted(set(map(tuple, np.round(p, 6))))

    def half(seq):
        h = []
        for q in seq:
            while len(h) >= 2 and ((h[-1][0] - h[-2][0]) * (q[1] - h[-2][1]) -
                                   (h[-1][1] - h[-2][1]) * (q[0] - h[-2][0])) <= 0:
                h.pop()
            h.append(q)
        return h
    lo, up = half(p), half(reversed(p))
    return lo[:-1] + up[:-1]


def sample_polygon(poly, step=0.001):
    out = []
    for a, b in zip(poly, poly[1:] + poly[:1]):
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
        out += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n)]
    return np.array(out)


def sample_rect(r, step=0.002):
    x0, x1, y0, y1 = r
    return sample_polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], step)


def body_clearance(x, y, yaw, pts):
    """Return min distance from the robot shapes at (x, y, yaw) to points (N x 2); 0 = contact."""
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = pts[:, 0] - x, pts[:, 1] - y
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    zero = np.zeros_like(lx)
    d = np.full(len(pts), np.inf)
    for x0, x1, y0, y1 in RECTS:
        d = np.minimum(d, np.hypot(np.maximum.reduce([x0 - lx, zero, lx - x1]),
                                   np.maximum.reduce([y0 - ly, zero, ly - y1])))
    for cx, cy, r in CIRCLES:
        d = np.minimum(d, np.maximum(np.hypot(lx - cx, ly - cy) - r, 0.0))
    i = int(np.argmin(d))
    return float(d[i]), pts[i]


def load_gt(run):
    gt = []
    for line in open(f'{run}/gt.csv').readlines()[1:]:
        try:
            gt.append(tuple(map(float, line.split(','))))
        except ValueError:
            pass
    return gt


def entry_windows(run):
    log = open(f'{run}/search.log').read()
    st = [(float(t), b) for t, b in re.findall(r'\[STATE\] t=([\d.]+) \w+ -> (\w+)', log)]

    def first(name):
        return next((t for t, b in st if b == name), None)
    return first('ENTRANCE_APPROACH'), first('ENTRANCE_CROSS'), first('VERIFY_INSIDE')


def sweep(rows, pts):
    best = (math.inf, None, None)
    for t, x, y, yaw in rows:
        d, p = body_clearance(x, y, yaw, pts)
        if d < best[0]:
            best = (d, (t, x, y, yaw), p)
    return best


def lidar_support(run, t0, t1, gt, post):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    gts = [g[0] for g in gt]
    cx, cy = post[:, 0].mean(), post[:, 1].mean()
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=f'{run}/bag', storage_id='mcap'),
            rosbag2_py.ConverterOptions('', ''))
    rd.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
    resid, headon, ret = [], [], []
    while rd.has_next():
        _, data, _ = rd.read_next()
        m = deserialize_message(data, LaserScan)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not t0 <= t <= t1:
            continue
        _, x, y, yaw = gt[min(bisect.bisect_left(gts, t), len(gt) - 1)][:4]
        sx, sy = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
        for i, r in enumerate(m.ranges):
            if not (m.range_min < r < m.range_max):
                continue
            a = yaw + m.angle_min + i * m.angle_increment
            px, py = sx + r * math.cos(a), sy + r * math.sin(a)
            if abs(px - cx) > 0.09 or abs(py - cy) > 0.09:
                continue
            dist = float(np.hypot(post[:, 0] - px, post[:, 1] - py).min())
            if dist < 0.06:
                resid.append(dist)
                if px > cx and abs(py - cy) < 0.006:
                    headon.append(px)
                ret.append((t, x, y, yaw, px, py))
    return resid, headon, ret


def main():
    run = sys.argv[1]
    gt = load_gt(run)
    t_app, t_cross, t_verify = entry_windows(run)
    if len(sys.argv) >= 4:
        t0, t1 = float(sys.argv[2]), float(sys.argv[3])
    else:
        t0, t1 = t_app, t_verify + 1.0
    poly, ext = mailbox_slice_polygon()
    post = sample_polygon(poly)
    constant = bool(np.ptp(ext, axis=0).max() < 1e-4)
    print(f'== {run}')
    print(f'mailbox mesh over z 0..{ROBOT_TOP:.2f} m: {len(poly)}-gon, '
          f'x {post[:, 0].min():.4f}..{post[:, 0].max():.4f} (east face) '
          f'y {post[:, 1].min():.4f}..{post[:, 1].max():.4f}; '
          f'cross-section constant over height: {constant}')
    windows = [('entry (approach..verify)', t0, t1)]
    if t_cross is not None and len(sys.argv) < 4:
        windows.append(('crossing only', t_cross, t_verify))
    for label, a, b in windows:
        rows = [g for g in gt if a <= g[0] <= b]
        d, (t, x, y, yaw), p = sweep(rows, post)
        disc = min(float(np.hypot(post[:, 0] - g[1], post[:, 1] - g[2]).min())
                   for g in rows) - DISC_R
        print(f'[{label}] t {a:.1f}..{b:.1f}')
        print(f'  MAILBOX true body clearance (collision shapes vs mesh post): {d:.3f} m '
              f'at t={t:.1f} robot GT ({x:.3f},{y:.3f},{math.degrees(yaw):+.1f}deg) '
              f'post point ({p[0]:.3f},{p[1]:.3f})')
        print(f'  mailbox 0.105 m disc vs mesh post (comparison only): {disc:.3f} m')
        near = [g for g in rows if -0.75 < g[2] < 0.40]
        for name, r in JAMBS.items():
            jd, (jt, jx, jy, _), _ = sweep(near, sample_rect(r))
            print(f'  {name.upper()} JAMB true body clearance: {jd:.3f} m at t={jt:.1f} '
                  f'robot GT ({jx:.3f},{jy:.3f})')
    rows = [g for g in gt if t0 <= g[0] <= t1]
    centre = sum(DOOR_X) / 2
    for yy, name in ((post[:, 1].mean(), 'mailbox row'), (-0.175, 'door line')):
        for g0, g1 in zip(rows, rows[1:]):
            if (g0[2] - yy) * (g1[2] - yy) <= 0 and g1[2] != g0[2]:
                xx = g0[1] + (yy - g0[2]) / (g1[2] - g0[2]) * (g1[1] - g0[1])
                print(f'  GT x at {name} (y={yy:+.3f}): {xx:.3f} (door centre {centre:.3f}, '
                      f'offset {xx - centre:+.3f}; + = east)')
                break
    try:
        resid, headon, ret = lidar_support(run, t0, t1, gt, post)
    except Exception as e:      # no bag: the geometry result stands on its own
        print(f'LiDAR support unavailable: {e}')
        return
    if ret:
        dmin = min(body_clearance(x, y, yaw, np.array([[px, py]]))[0]
                   for _, x, y, yaw, px, py in ret)
        hm = np.mean(headon) if headon else float('nan')
        print(f'LiDAR support (not authoritative): {len(ret)} post returns, distance to mesh '
              f'boundary mean {np.mean(resid):.4f} sd {np.std(resid):.4f} m (sigma 0.01); '
              f'head-on east-face x mean {hm:.4f} (n={len(headon)}) vs mesh '
              f'{post[:, 0].max():.4f}; max return x {max(r[4] for r in ret):.3f}; '
              f'min body-to-return {dmin:.3f} m')


if __name__ == '__main__':
    main()
