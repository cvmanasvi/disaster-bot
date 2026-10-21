"""Geometry validation of the disaster-site layout (2 cm raster, analysis only).

usage: python3 check_geometry.py [out_dir]
Writes geometry_report.txt and layout_map.png into out_dir (default ../evidence).

Robot envelope (turtlebot3_burger_cam SDF): chassis x -0.102..+0.038 (base_footprint
frame), width 0.178 m incl. wheels, circumscribed turning radius ~0.125 m.
"""
import math
import os
import sys
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), '..', 'evidence')
os.makedirs(OUT, exist_ok=True)
RES = 0.02
X0, X1, Y0, Y1 = -3.0, 10.0, -3.5, 8.5
NX, NY = int((X1 - X0) / RES), int((Y1 - Y0) / RES)
R_TURN = 0.125      # circumscribed radius: can rotate in place anywhere in the free set
R_HALF = 0.089      # half the robot width: absolute minimum to drive straight through

xs = X0 + (np.arange(NX) + 0.5) * RES
ys = Y0 + (np.arange(NY) + 0.5) * RES
GX, GY = np.meshgrid(xs, ys)          # [iy, ix]


def poly_mask(pts):
    m = np.ones_like(GX, dtype=bool)
    n = len(pts)
    for k in range(n):
        (ax, ay), (bx, by) = pts[k], pts[(k + 1) % n]
        m &= ((bx - ax) * (GY - ay) - (by - ay) * (GX - ax)) >= 0
    return m


occ = np.zeros_like(GX, dtype=bool)
for b in L.all_boxes():
    occ |= poly_mask(L.corners(b))
vx, vy, _ = L.VICTIM
victim_plate = poly_mask([(vx + 0.25, vy - 0.275), (vx + 0.25, vy + 0.075),
                          (vx - 0.25, vy + 0.075), (vx - 0.25, vy - 0.275)])
victim_body = (GX - vx) ** 2 + (GY - vy) ** 2 <= 0.20 ** 2   # legs/feet footprint (approx.)
occ_all = occ | victim_plate | victim_body


def edt(mask):
    """Exact-ish Euclidean distance (m) from every cell to the nearest True cell (two-pass chamfer 3-4-5 refined)."""
    big = 1e9
    d = np.where(mask, 0.0, big)
    # 8-neighbour chamfer with true diagonal weights (two passes) -- error < 8 %
    s2 = math.sqrt(2.0)
    for iy in range(NY):
        row = d[iy]
        if iy > 0:
            up = d[iy - 1]
            row[:] = np.minimum(row, up + 1)
            row[1:] = np.minimum(row[1:], up[:-1] + s2)
            row[:-1] = np.minimum(row[:-1], up[1:] + s2)
        for ix in range(1, NX):
            if row[ix - 1] + 1 < row[ix]:
                row[ix] = row[ix - 1] + 1
    for iy in range(NY - 1, -1, -1):
        row = d[iy]
        if iy < NY - 1:
            dn = d[iy + 1]
            row[:] = np.minimum(row, dn + 1)
            row[1:] = np.minimum(row[1:], dn[:-1] + s2)
            row[:-1] = np.minimum(row[:-1], dn[1:] + s2)
        for ix in range(NX - 2, -1, -1):
            if row[ix + 1] + 1 < row[ix]:
                row[ix] = row[ix + 1] + 1
    return d * RES


def cell(x, y):
    return int((y - Y0) / RES), int((x - X0) / RES)


def flood(free, start):
    seen = np.zeros_like(free)
    iy, ix = cell(*start)
    if not free[iy, ix]:
        return seen
    q = deque([(iy, ix)])
    seen[iy, ix] = True
    while q:
        a, b = q.popleft()
        for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            na, nb = a + da, b + db
            if 0 <= na < NY and 0 <= nb < NX and free[na, nb] and not seen[na, nb]:
                seen[na, nb] = True
                q.append((na, nb))
    return seen


def bottleneck(dist, p, q, block=None):
    """Largest clearance radius r such that p and q are connected in {dist > r} (binary search)."""
    lo, hi = 0.0, 1.0
    for _ in range(18):
        r = 0.5 * (lo + hi)
        free = dist > r
        if block is not None:
            free &= ~block
        if flood(free, p)[cell(*q)]:
            lo = r
        else:
            hi = r
    return lo


lines = []


def say(s=''):
    print(s)
    lines.append(s)


dist = edt(occ_all)
dist_static = edt(occ)
say('== Disaster site geometry check (2 cm raster; obstacle boxes from layout.py)')
say(f'boxes: {len(L.all_boxes())} ({", ".join(f"{k} {len(v)}" for k, v in L.MODELS.items())})')
small = [b for b in L.all_boxes() if min(b['sx'], b['sy']) < 0.25 and not b['name'].startswith(('facade', 'north', 'west', 'east', 'lobby_n', 'corr_w', 'corr_e', 'mid_n', 'cd'))]
say(f'non-wall boxes thinner than 0.25 m (narrow-obstacle risk): {[b["name"] for b in small] or "none"}')
low = [b['name'] for b in L.all_boxes() if b['h'] < 0.35]
say(f'boxes lower than 0.35 m: {low or "none"} (LiDAR plane 0.171 m; robot top 0.189 m)')

sx, sy, _ = L.SPAWN
say()
say('-- Spawn')
say(f'spawn {L.SPAWN[:2]} yaw {L.SPAWN[2]}: nearest obstacle surface {dist[cell(sx, sy)]:.3f} m from base_footprint origin '
    f'(robot circumscribed radius {R_TURN})')

reach = flood(dist > R_TURN, (sx, sy))
reach_min = flood(dist > R_HALF, (sx, sy))
key = {
    'inside door (2.45,0.8)': (2.45, 0.8),
    'lobby east (7.0,1.7)': (7.0, 1.7),
    'B west room (1.5,3.5)': (1.5, 3.5),
    'C collapsed room (1.6,5.7)': (1.6, 5.7),
    'E south pocket (5.0,3.0)': (5.0, 3.0),
    'E north lane (6.25,4.3)': (6.25, 4.3),
    'D west (4.5,5.6)': (4.5, 5.6),
    'D NE / victim approach (6.45,6.65)': (6.45, 6.65),
    'corridor north of choke (3.75,4.4)': (3.75, 4.4),
    'corridor south of choke (3.75,2.8)': (3.75, 2.8),
}
say()
say(f'-- Reachability from spawn (free = clearance > {R_TURN} m, robot can rotate everywhere; '
    f'[min] = clearance > {R_HALF} m straight-drive minimum)')
for n, p in key.items():
    iy, ix = cell(*p)
    say(f'  {n:36s} reachable={bool(reach[iy, ix])!s:5s} [min {bool(reach_min[iy, ix])!s:5s}] clearance {dist[iy, ix]:.3f} m')

say()
say('-- Passage bottlenecks (clear width = 2 x max clearance radius along the best path)')
passages = {
    'facade doorway (yard -> lobby)': ((2.45, -0.8), (2.45, 0.8), None),
    'lobby -> B opening': ((1.0, 2.0), (1.0, 3.2), None),
    'B -> C doorway': ((1.65, 4.3), (1.65, 5.3), None),
    'narrow passage lobby -> E': ((4.95, 2.1), (4.95, 3.0), None),
    'wide opening lobby -> E': ((7.1, 2.1), (7.1, 3.0), None),
    'E: around beam east end (pocket -> north lane)': ((6.5, 3.15), (6.25, 4.3), None),
    'E -> D doorway': ((6.25, 4.3), (5.95, 5.4), None),
    'D: west of pile to victim approach': ((5.6, 5.4), (6.45, 6.65), None),
    'BLOCKED corridor (lobby -> D)': ((3.75, 2.8), (3.75, 4.4), None),
    'BLOCKED facade doorway (yard -> lobby)': ((6.05, -1.0), (6.05, 1.0), None),
    'BLOCKED beam west end (pocket -> north lane, west)': ((4.45, 3.1), (4.45, 4.2), 'west'),
}
results = {}
for n, (p, q, opt) in passages.items():
    block = None
    if opt == 'west':           # only allow the west end: forbid going round the east end
        block = GX > 4.9
    elif n.startswith('BLOCKED corridor'):
        block = ~((GX > 3.3) & (GX < 4.2) & (GY > 2.5) & (GY < 4.9))   # stay inside the corridor
    elif n.startswith('BLOCKED facade'):
        block = ~((GX > 5.4) & (GX < 6.7) & (GY > -1.1) & (GY < 1.1))
    r = bottleneck(dist, p, q, block)
    results[n] = 2 * r
    verdict = ('BLOCKED (< robot width 0.178)' if 2 * r < 0.178 else
               'traversable (rotate-anywhere)' if r > R_TURN else 'traversable straight only')
    say(f'  {n:50s} clear width {2 * r:.3f} m -> {verdict}')

say()
say('-- Dead ends')
no_bc = (GX > 1.15) & (GX < 2.15) & (GY > 4.72) & (GY < 4.88)
r_c = flood((dist > R_HALF) & ~no_bc, (sx, sy))
say(f'  C collapsed room reachable when the B->C doorway is closed: {bool(r_c[cell(1.6, 5.7)])} '
    f'(False = C (and B beyond it) is a dead-end branch, single entrance)')
no_bd = ((GX > 1.15) & (GX < 2.15) & (GY > 4.72) & (GY < 4.88)) | ((GX > 0.45) & (GX < 1.55) & (GY > 2.52) & (GY < 2.68))
say(f'  B/C reachable with the lobby->B opening closed: {bool(flood((dist > R_HALF) & ~no_bd, (sx, sy))[cell(1.5, 3.5)])}')
CORR = (GX > 3.3) & (GX < 4.2) & (GY > 2.6) & (GY < 4.8)
cor_s = flood((dist > R_HALF) & CORR, (3.75, 2.8))
ys_reach = GY[cor_s & (GX > 3.375) & (GX < 4.125) & (GY > 2.6)]
say(f'  corridor from the lobby: deepest reachable y {ys_reach.max():.2f} m (corridor mouth y 2.675): dead-end stub')
cor_n = flood((dist > R_HALF) & CORR, (3.75, 4.4))
say(f'  corridor alcove from D side: reachable from spawn {bool(reach_min[cell(3.75, 4.4)])}; deepest south y '
    f'{GY[cor_n & (GX > 3.375) & (GX < 4.125) & (GY < 4.8)].min():.2f} m (dead-end alcove)')

say()
say('-- Victim')
say(f'victim origin {L.VICTIM[:2]} (static). plate x {vx - 0.25:.2f}..{vx + 0.25:.2f}, y {vy - 0.275:.3f}..{vy + 0.075:.3f}')
plate_d = edt(victim_plate)
body_d = edt(victim_body)
for b in L.all_boxes():
    m = poly_mask(L.corners(b))
    b['d_plate'] = float(plate_d[m].min()) if m.any() else 9
    b['d_body'] = float(math.hypot(b['cx'] - vx, b['cy'] - vy))
near = sorted(L.all_boxes(), key=lambda b: b['d_plate'])[:6]
for b in near:
    say(f'  plate -> {b["name"]:16s} {b["d_plate"]:.3f} m')
say(f'  person origin -> north wall face {7.2 - L.O - vy:.3f} m, east wall face {8.0 - L.O - vx:.3f} m')
beam = L.FALLEN_BEAM[0]
say(f'  person origin -> fallen beam surface {float(edt(poly_mask(L.corners(beam)))[cell(vx, vy)]):.3f} m; '
    f'-> pile (nearest surface) {min(float(edt(poly_mask(L.corners(b)))[cell(vx, vy)]) for b in L.DEBRIS if b["name"].startswith("pile")):.3f} m')
appr = flood(dist > R_TURN, (sx, sy))
dv = np.hypot(GX - vx, GY - vy)
ok = appr & (dv >= 0.75) & (dv <= 1.05)
say(f'  free robot positions 0.75-1.05 m from the person (rotate-anywhere, reachable): {int(ok.sum())} cells '
    f'({ok.sum() * RES * RES:.2f} m^2); nearest reachable centre {dv[appr].min():.3f} m')

# camera line-of-sight (3-D: camera z 0.093 -> person torso z 1.0 / head z 1.6, boxes with heights)
CAM_Z = 0.093


box_polys = [(L.corners(b), b['h']) for b in L.all_boxes()]


def seg_box_hit(px, py, tz):
    n = 60
    for k in range(1, n):
        t = k / n
        x = px + t * (vx - px)
        y = py + t * (vy - py)
        z = CAM_Z + t * (tz - CAM_Z)
        for pts, h in box_polys:
            if z > h:
                continue
            inside = True
            for i in range(4):
                (ax, ay), (bx, by) = pts[i], pts[(i + 1) % 4]
                if (bx - ax) * (y - ay) - (by - ay) * (x - ax) < 0:
                    inside = False
                    break
            if inside:
                return True
    return False


vis = np.zeros_like(GX, dtype=bool)
step = 5   # 10 cm sampling for the visibility map
for iy in range(0, NY, step):
    for ix in range(0, NX, step):
        if not reach_min[iy, ix]:
            continue
        x, y = GX[iy, ix], GY[iy, ix]
        if math.hypot(x - vx, y - vy) > 6.0:
            continue
        if not seg_box_hit(x, y, 1.0) or not seg_box_hit(x, y, 1.6):
            vis[iy:iy + step, ix:ix + step] = True
say()
say('-- Victim visibility from the camera (z 0.093 m -> torso 1.0 m or head 1.6 m, through/over box heights)')
for n, p in [('spawn', (sx, sy)), ('first inside scan (2.45,0.75)', (2.45, 0.75)), ('lobby centre (4.0,1.9)', (4.0, 1.9)),
             ('lobby east (7.1,1.8)', (7.1, 1.8)), ('E north lane (6.25,4.3)', (6.25, 4.3)), ('E east gap (7.4,4.0)', (7.4, 4.0)),
             ('D door (6.25,5.0)', (6.25, 5.0)), ('D inside door (6.0,5.4)', (6.0, 5.4)), ('D west (4.5,5.6)', (4.5, 5.6)),
             ('D (5.8,6.2)', (5.8, 6.2))]:
    v = not seg_box_hit(p[0], p[1], 1.0) or not seg_box_hit(p[0], p[1], 1.6)
    say(f'  {n:32s} victim visible: {v}  range {math.hypot(p[0] - vx, p[1] - vy):.2f} m')
vy_cells = GY[vis]
say(f'  visibility region: {vis.sum() * RES * RES:.2f} m^2, y range {vy_cells.min():.2f}..{vy_cells.max():.2f}, '
    f'x range {GX[vis].min():.2f}..{GX[vis].max():.2f} (all inside room D: {bool((GY[vis] > 4.8).all())})')
outside = vis & (GY <= 4.8)
if outside.any():
    say(f'  visible cells OUTSIDE room D: {outside.sum() * RES * RES:.2f} m^2, e.g. {[(round(float(a), 2), round(float(b), 2)) for a, b in zip(GX[outside][::max(1, outside.sum() // 6)], GY[outside][::max(1, outside.sum() // 6)])]}')

say()
say('-- Route clearances')
say(f'  min clearance of any reachable (rotate-anywhere) cell inside the building: {dist[reach & (GX > 0) & (GX < 8) & (GY > 0) & (GY < 7.2)].min():.3f} m (= {R_TURN} by construction)')
say(f'  free floor area reachable inside: {(reach & (GX > 0) & (GX < 8) & (GY > 0) & (GY < 7.2)).sum() * RES * RES:.1f} m^2')

open(os.path.join(OUT, 'geometry_report.txt'), 'w').write('\n'.join(lines) + '\n')

# ---------------------------------------------------------------- map
import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
fig, ax = plt.subplots(figsize=(13, 12))
img = np.ones((NY, NX, 3))
img[reach] = (0.85, 0.95, 0.85)
img[reach_min & ~reach] = (0.95, 0.95, 0.75)
img[vis & reach_min] = (1.0, 0.82, 0.82)
ax.imshow(img, origin='lower', extent=(X0, X1, Y0, Y1))
for b in L.all_boxes():
    pts = L.corners(b)
    ax.add_patch(plt.Polygon(pts, closed=True, color=b['color'], ec='k', lw=0.4))
ax.add_patch(plt.Rectangle((vx - 0.25, vy - 0.275), 0.5, 0.35, color='red', alpha=0.5))
ax.plot(vx, vy, 'r*', ms=16)
ax.text(vx - 0.3, vy + 0.25, 'VICTIM', color='red', fontsize=10, weight='bold')
ax.plot(sx, sy, 'b^', ms=12)
ax.text(sx - 0.4, sy - 0.4, 'SPAWN', color='b')
for t, (x, y) in {'A lobby': (1.0, 1.6), 'B west room': (1.0, 3.9), 'C collapsed room\n(dead end)': (1.0, 6.0),
                  'blocked\ncorridor': (3.42, 2.95), 'E east hall': (7.0, 4.35), 'D victim room': (4.2, 6.1),
                  'narrow\n0.70 m': (4.65, 2.2), 'wide\n1.0 m': (6.75, 2.2), 'fallen beam': (5.0, 3.85),
                  'blocked door': (5.6, -0.75), 'doorway': (2.1, -0.45), 'pile': (6.6, 5.0), 'yard': (0.5, -2.5)}.items():
    ax.text(x, y, t, fontsize=8)
ax.set_xlim(-2.0, 9.0)
ax.set_ylim(-3.0, 8.0)
ax.set_aspect('equal')
ax.grid(True, lw=0.3)
ax.set_title('Earthquake disaster site (world frame). green: reachable (rotate-anywhere), yellow: straight-drive only, '
             'pink: victim visible from camera', fontsize=9)
plt.savefig(os.path.join(OUT, 'layout_map.png'), dpi=80, bbox_inches='tight')
print('wrote', OUT)


# ---------------------------------------------------------------- exact gaps
def _seg_pt(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def poly_gap(p, q):
    """Exact minimum distance between two convex polygons (0 if they overlap/touch)."""
    def inside(pt, poly):
        return all((poly[(i + 1) % 4][0] - poly[i][0]) * (pt[1] - poly[i][1]) -
                   (poly[(i + 1) % 4][1] - poly[i][1]) * (pt[0] - poly[i][0]) >= -1e-9 for i in range(4))
    if any(inside(v, q) for v in p) or any(inside(v, p) for v in q):
        return 0.0
    d = min(_seg_pt(*v, *e0, *e1) for v in p for e0, e1 in zip(q, q[1:] + q[:1]))
    return min(d, min(_seg_pt(*v, *e0, *e1) for v in q for e0, e1 in zip(p, p[1:] + p[:1])))


B = {b['name']: L.corners(b) for b in L.all_boxes()}
gl = ['', '-- Exact polygon gaps (design values; robot width 0.178, circumscribed radius 0.125)']
for n, a, b in [('facade doorway', 'facade_w', 'facade_mid'), ('narrow passage', 'lobby_n_2', 'lobby_n_3'),
                ('wide opening', 'lobby_n_3', 'lobby_n_4'), ('lobby->B opening', 'lobby_n_stub_1', 'lobby_n_b_0'),
                ('B->C doorway', 'mid_n_1', 'mid_n_2'), ('E->D doorway', 'mid_n_3', 'mid_n_4'),
                ('corridor clear width', 'corr_w', 'corr_e'),
                ('beam east end <-> east wall', 'beam', 'east'), ('beam <-> y=4.8 wall west of D door', 'beam', 'mid_n_3'),
                ('beam <-> y=4.8 wall east of D door', 'beam', 'mid_n_4'), ('beam <-> lobby north wall (narrow-passage side)', 'beam', 'lobby_n_3'),
                ('beam <-> corridor east wall (west end)', 'beam', 'corr_e'),
                ('lobby island <-> lobby north wall', 'lobby_island', 'lobby_n_2'), ('lobby island <-> facade', 'lobby_island', 'facade_mid'),
                ('pile_1 <-> y=4.8 wall (D door side)', 'pile_1', 'mid_n_4'), ('pile_2 <-> east wall', 'pile_2', 'east'),
                ('r_lobby_mid <-> lobby north wall', 'r_lobby_mid', 'lobby_n_3')]:
    gl.append(f'  {n:50s} {poly_gap(B[a], B[b]):.3f} m')
gl.append('  corridor choke (blocked passage), pairwise gaps between corridor walls and fill pieces:')
cw = ['corr_w', 'corr_e', 'corr_fill_1', 'corr_fill_2', 'corr_fill_3', 'corr_fill_4']
for i in range(len(cw)):
    for j in range(i + 1, len(cw)):
        g = poly_gap(B[cw[i]], B[cw[j]])
        if g < 0.6:
            gl.append(f'    {cw[i]:12s} <-> {cw[j]:12s} {g:.3f} m')
gl.append('  blocked facade doorway, pairwise gaps (jambs + fill):')
dw = ['facade_mid', 'door_fill_1', 'door_fill_2', 'door_fill_3', 'facade_e']
for a, b in zip(dw, dw[1:]):
    gl.append(f'    {a:12s} <-> {b:12s} {poly_gap(B[a], B[b]):.3f} m')
for s_ in gl:
    print(s_)
with open(os.path.join(OUT, 'geometry_report.txt'), 'a') as f:
    f.write('\n'.join(gl) + '\n')
