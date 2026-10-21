"""Coarse persistent 2-D exploration grid (not SLAM).

The grid lives in the node's local navigation frame (odometry distance along
the IMU heading). Each completed 360-degree STOP_AND_SCAN is integrated from
its merged polar profile (every 1-degree bin = median of every LiDAR ray that
fell in it during the turn): cells along the ray become FREE, the endpoint of
a real return becomes OCCUPIED, nothing beyond a return is touched, and a
no-return (max-range) bin frees cells only up to the sensor maximum range.

Cell states:
    UNKNOWN   never observed
    FREE      observed free, robot has not been there
    OCCUPIED  obstacle return
    VISITED   robot footprint has covered it (implies free)

Evidence is one vote per cell per scan (hit wins inside a scan; across scans
a cell is OCCUPIED while hits >= misses), so a wall cell is not erased by one
later grazing ray. VISITED is a separate flag and always wins: the robot body
was physically there.

No ROS imports: everything here is testable offline.
"""

import json
import math
from collections import deque

UNKNOWN, FREE, OCCUPIED, VISITED = 0, 1, 2, 3
STATE_NAMES = {UNKNOWN: 'UNKNOWN', FREE: 'FREE', OCCUPIED: 'OCCUPIED', VISITED: 'VISITED'}
STATE_CHARS = {UNKNOWN: ' ', FREE: '.', OCCUPIED: '#', VISITED: 'v'}

N4 = ((1, 0), (-1, 0), (0, 1), (0, -1))
N8 = N4 + ((1, 1), (1, -1), (-1, 1), (-1, -1))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ExplorationGrid:

    def __init__(self, resolution=0.20, robot_radius=0.11):
        self.res = resolution
        self.robot_radius = robot_radius
        # (i, j) -> [hits, misses, visited]; absent = UNKNOWN.
        self.cells = {}
        self.scans_integrated = 0

    # ------------------------------------------------------------------
    # cells
    # ------------------------------------------------------------------

    def cell_of(self, x, y):
        return int(math.floor(x / self.res)), int(math.floor(y / self.res))

    def center(self, cell):
        return (cell[0] + 0.5) * self.res, (cell[1] + 0.5) * self.res

    def state(self, cell):
        c = self.cells.get(cell)
        if c is None:
            return UNKNOWN
        hits, misses, visited = c
        if visited:
            return VISITED
        if hits > 0 and hits >= misses:
            return OCCUPIED
        if misses > 0:
            return FREE
        return UNKNOWN

    def is_free(self, cell):
        return self.state(cell) in (FREE, VISITED)

    def counts(self):
        out = dict.fromkeys(STATE_NAMES.values(), 0)
        for cell in self.cells:
            out[STATE_NAMES[self.state(cell)]] += 1
        i0, i1, j0, j1 = self.bounds()
        out['UNKNOWN'] = (i1 - i0 + 1) * (j1 - j0 + 1) - len(self.cells) + out['UNKNOWN']
        return out

    def bounds(self, margin=1):
        if not self.cells:
            return 0, 0, 0, 0
        i = [c[0] for c in self.cells]
        j = [c[1] for c in self.cells]
        return min(i) - margin, max(i) + margin, min(j) - margin, max(j) + margin

    # ------------------------------------------------------------------
    # scan integration
    # ------------------------------------------------------------------

    def _ray_cells(self, x, y, bearing, length):
        """Cells crossed from (x, y) along bearing for `length` metres, in
        order, sampled finely enough not to skip a 0.2 m cell."""
        step = self.res / 4.0
        n = int(length / step)
        c, s = math.cos(bearing), math.sin(bearing)
        out = []
        last = None
        for k in range(n + 1):
            d = min(k * step, length)
            cell = self.cell_of(x + d * c, y + d * s)
            if cell != last:
                out.append(cell)
                last = cell
        return out

    def integrate_profile(self, x, y, profile, rays_per_bin, maxrange_per_bin, range_max,
                          no_return_min_fraction=0.5):
        """Integrate one merged 360-degree profile taken at (x, y).

        profile[k]: robot-centred range of bin k (world bearing (k+0.5) deg),
        inf when the bin has no reliable return. A bin with no return counts
        as free out to range_max only if at least no_return_min_fraction of
        its rays were clean max-range readings; otherwise it adds nothing.
        """
        num_bins = len(profile)
        free, hit = set(), set()
        returns = no_returns = skipped = 0
        for k, r in enumerate(profile):
            bearing = math.radians((k + 0.5) * 360.0 / num_bins)
            if math.isfinite(r):
                end = self.cell_of(x + r * math.cos(bearing), y + r * math.sin(bearing))
                for cell in self._ray_cells(x, y, bearing, r):
                    if cell != end:
                        free.add(cell)
                hit.add(end)
                returns += 1
            elif (rays_per_bin[k] and
                  maxrange_per_bin[k] >= no_return_min_fraction * rays_per_bin[k]):
                free.update(self._ray_cells(x, y, bearing, range_max))
                no_returns += 1
            else:
                skipped += 1

        for cell in hit:
            self.cells.setdefault(cell, [0, 0, False])[0] += 1
        for cell in free - hit:
            self.cells.setdefault(cell, [0, 0, False])[1] += 1
        self.scans_integrated += 1
        return {'return_bins': returns, 'no_return_bins': no_returns, 'skipped_bins': skipped,
                'free_cells_touched': len(free - hit), 'hit_cells_touched': len(hit)}

    def mark_visited(self, x, y):
        """Mark every cell the robot footprint (circle) overlaps as VISITED."""
        r = self.robot_radius
        i0, j0 = self.cell_of(x - r, y - r)
        i1, j1 = self.cell_of(x + r, y + r)
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                # closest point of the cell to the circle centre
                cx = min(max(x, i * self.res), (i + 1) * self.res)
                cy = min(max(y, j * self.res), (j + 1) * self.res)
                if math.hypot(cx - x, cy - y) <= r:
                    self.cells.setdefault((i, j), [0, 0, False])[2] = True

    def mark_path_visited(self, x0, y0, x1, y1):
        d = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(d / (self.res / 2.0)))
        for k in range(n + 1):
            self.mark_visited(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n)

    # ------------------------------------------------------------------
    # frontiers
    # ------------------------------------------------------------------

    def frontier_cells(self):
        """FREE, not VISITED, with at least one 4-neighbour UNKNOWN."""
        out = []
        for cell in self.cells:
            if self.state(cell) != FREE:
                continue
            i, j = cell
            if any(self.state((i + di, j + dj)) == UNKNOWN for di, dj in N4):
                out.append(cell)
        return out

    @staticmethod
    def cluster(cells):
        """8-connected groups of cells, largest first."""
        remaining = set(cells)
        clusters = []
        while remaining:
            seed = remaining.pop()
            group, queue = [seed], deque([seed])
            while queue:
                i, j = queue.popleft()
                for di, dj in N8:
                    n = (i + di, j + dj)
                    if n in remaining:
                        remaining.remove(n)
                        group.append(n)
                        queue.append(n)
            clusters.append(sorted(group))
        clusters.sort(key=lambda g: (-len(g), g[0]))
        return clusters

    def clearance(self, cell, max_m=1.0):
        """Distance (cell centre to cell centre) to the nearest OCCUPIED cell,
        capped at max_m."""
        k = int(math.ceil(max_m / self.res))
        best = max_m
        i, j = cell
        for di in range(-k, k + 1):
            for dj in range(-k, k + 1):
                d = self.res * math.hypot(di, dj)
                if d < best and self.state((i + di, j + dj)) == OCCUPIED:
                    best = d
        return best

    def traversable(self, cell, min_clear_m):
        return self.is_free(cell) and self.clearance(cell, min_clear_m) >= min_clear_m

    def reachable(self, start, min_clear_m, blocked=None, escape_m=0.4):
        """Breadth-first flood fill over free cells with clearance >= min_clear_m
        (cells within escape_m of the start may be closer to obstacles, so the
        robot can leave a tight spot). Returns (dist_m, parent).

        This is only a reachability test plus a grid distance, not a planner.
        """
        dist = {start: 0.0}
        parent = {start: None}
        queue = deque([start])
        sx, sy = self.center(start)
        while queue:
            cur = queue.popleft()
            for di, dj in N8:
                n = (cur[0] + di, cur[1] + dj)
                if n in dist:
                    continue
                if blocked is not None and blocked(n):
                    continue
                if not self.is_free(n):
                    continue
                if di and dj and not (self.is_free((cur[0] + di, cur[1])) and
                                      self.is_free((cur[0], cur[1] + dj))):
                    continue
                cx, cy = self.center(n)
                near_start = math.hypot(cx - sx, cy - sy) <= escape_m
                if not near_start and self.clearance(n, min_clear_m) < min_clear_m:
                    continue
                dist[n] = dist[cur] + self.res * math.hypot(di, dj)
                parent[n] = cur
                queue.append(n)
        return dist, parent

    def line_clear(self, x0, y0, x1, y1, ok):
        d = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(d / (self.res / 4.0)))
        for k in range(n + 1):
            cell = self.cell_of(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n)
            if not ok(cell):
                return False
        return True

    # ------------------------------------------------------------------
    # dump
    # ------------------------------------------------------------------

    def to_dict(self, robot=None, frontier=(), selected=(), excluded=()):
        """Inspectable dump: rows top (max y) to bottom, one char per cell:
        ' ' unknown, '.' free, '#' occupied, 'v' visited, 'F' frontier,
        'E' frontier excluded (entrance / outside), 'S' selected frontier
        cluster, 'R' robot cell."""
        i0, i1, j0, j1 = self.bounds()
        frontier, selected, excluded = set(frontier), set(selected), set(excluded)
        rcell = self.cell_of(*robot) if robot else None
        rows = []
        for j in range(j1, j0 - 1, -1):
            row = []
            for i in range(i0, i1 + 1):
                c = (i, j)
                if c == rcell:
                    row.append('R')
                elif c in selected:
                    row.append('S')
                elif c in frontier:
                    row.append('F')
                elif c in excluded:
                    row.append('E')
                else:
                    row.append(STATE_CHARS[self.state(c)])
            rows.append(''.join(row))
        return {
            'resolution_m': self.res,
            'frame': 'nav (odometry distance along IMU heading; origin = spawn)',
            'origin_cell': [i0, j1],
            'origin_xy_of_top_left_cell_corner': [i0 * self.res, (j1 + 1) * self.res],
            'width': i1 - i0 + 1,
            'height': j1 - j0 + 1,
            'legend': {' ': 'UNKNOWN', '.': 'FREE', '#': 'OCCUPIED', 'v': 'VISITED',
                       'F': 'FRONTIER (free, not visited, next to unknown)',
                       'E': 'frontier cell excluded (entrance / outside)',
                       'S': 'SELECTED frontier cluster', 'R': 'robot cell'},
            'rows': rows,
            'cells': [[i, j, h, m, int(v), self.state((i, j))]
                      for (i, j), (h, m, v) in sorted(self.cells.items())],
            'cells_columns': ['i', 'j', 'hits', 'misses', 'visited', 'state'],
        }

    def dump(self, path, **kwargs):
        with open(path, 'w') as f:
            json.dump(self.to_dict(**kwargs), f)


# ----------------------------------------------------------------------
# entrance portal / interior region
# ----------------------------------------------------------------------

def door_frame(entrance, x, y):
    """(a, lat) of a point: a along the entrance normal (0 = outer wall face,
    positive = into the house), lat across the doorway."""
    nx, ny = math.cos(entrance['normal']), math.sin(entrance['normal'])
    dx, dy = x - entrance['x'], y - entrance['y']
    return dx * nx + dy * ny, -dx * ny + dy * nx


def in_portal(entrance, x, y, depth_out_m=0.30, depth_in_m=0.40, lateral_margin_m=0.30):
    """True inside the doorway band: the entrance gap plus a margin into
    both jambs, from depth_out_m outside the outer wall face to depth_in_m
    inside it. The interior flood fill never crosses this band."""
    a, lat = door_frame(entrance, x, y)
    return (-depth_out_m <= a <= depth_in_m and
            abs(lat) <= 0.5 * entrance['width'] + lateral_margin_m)


def interior_region(grid, seeds, blocked):
    """Free cells (FREE or VISITED) 8-connected to any seed cell without
    entering a blocked (portal) cell and without cutting a corner between
    two non-free cells. Pure connectivity: no clearance requirement."""
    region = set()
    queue = deque()
    for seed in seeds:
        if seed not in region and not blocked(seed):
            region.add(seed)
            queue.append(seed)
    while queue:
        cur = queue.popleft()
        for di, dj in N8:
            n = (cur[0] + di, cur[1] + dj)
            if n in region or blocked(n) or not grid.is_free(n):
                continue
            if di and dj and not (grid.is_free((cur[0] + di, cur[1])) or
                                  grid.is_free((cur[0], cur[1] + dj))):
                continue
            region.add(n)
            queue.append(n)
    return region


def move_target(step_m, free_travel_m, aim_distance_m, aim_stop_margin_m):
    """Bounded move length toward the frontier aim point: never more than the
    normal step, the measured free travel, or the aim distance minus a small
    stopping margin; never negative. Returns (target_m, limiting_term)."""
    terms = {
        'step': step_m,
        'free_travel': free_travel_m,
        'aim': aim_distance_m - aim_stop_margin_m,
    }
    order = ('step', 'free_travel', 'aim')          # tie -> the first listed
    limit = min(order, key=lambda k: (terms[k], order.index(k)))
    return max(0.0, terms[limit]), limit


# ----------------------------------------------------------------------
# frontier selection
# ----------------------------------------------------------------------

DEFAULT_WEIGHTS = {
    'size': 1.0,         # min(size, size_cap) / size_cap
    'distance': 0.5,     # grid distance / distance_scale
    'clearance': 0.5,    # min(clearance, 1 m)
    'turn': 0.3,         # |turn| / pi
    'size_cap': 15,
    'distance_scale_m': 4.0,
}


def evaluate_frontiers(grid, x, y, yaw, min_clear_m=0.3, min_cluster_cells=3,
                       excluded=None, blocked=None, weights=None):
    """Find, cluster and score frontiers from robot pose (x, y, yaw).

    excluded(cell) -> True drops a frontier cell (entrance / outside).
    blocked(cell) -> True keeps the reachability flood fill out of a cell.
    Returns a dict with every cluster (scored or with the reason it was not)
    sorted best first, plus counts.
    """
    w = dict(DEFAULT_WEIGHTS, **(weights or {}))
    start = grid.cell_of(x, y)
    frontier = grid.frontier_cells()
    kept = [c for c in frontier if not (excluded and excluded(c))]
    dropped = [c for c in frontier if excluded and excluded(c)]
    dist, parent = grid.reachable(start, min_clear_m, blocked=blocked)

    def ok(cell):
        return cell in dist

    clusters = []
    for cells in ExplorationGrid.cluster(kept):
        cx = sum(grid.center(c)[0] for c in cells) / len(cells)
        cy = sum(grid.center(c)[1] for c in cells) / len(cells)
        info = {
            'size': len(cells),
            'centroid': [round(cx, 2), round(cy, 2)],
            'straight_distance_m': round(math.hypot(cx - x, cy - y), 2),
            'bearing_deg': round(math.degrees(math.atan2(cy - y, cx - x)), 1),
            'cells': cells,
        }
        reach = [c for c in cells if c in dist]
        # Frontier cells sit next to unknown space, often against a wall, so
        # the cluster may be reached through a neighbouring reachable cell.
        if not reach:
            near = set()
            for i, j in cells:
                for di, dj in N8:
                    if (i + di, j + dj) in dist:
                        near.add((i + di, j + dj))
            reach = sorted(near)
        if len(cells) < min_cluster_cells:
            info['status'] = 'too_small'
        elif not reach:
            info['status'] = 'unreachable'
        else:
            target = min(reach, key=lambda c: (math.hypot(grid.center(c)[0] - cx,
                                                          grid.center(c)[1] - cy), c))
            # Line-of-sight lookahead along the flood-fill tree: the furthest
            # tree cell toward the target still in straight view of the robot.
            chain = []
            c = target
            while c is not None:
                chain.append(c)
                c = parent[c]
            aim = start
            for c in chain:                       # target ... start
                ax, ay = grid.center(c)
                if grid.line_clear(x, y, ax, ay, ok):
                    aim = c
                    break
            ax, ay = grid.center(aim)
            heading = math.atan2(ay - y, ax - x) if aim != start else math.atan2(cy - y, cx - x)
            tx, ty = grid.center(target)
            clearance = grid.clearance(target)
            turn = abs(_wrap(heading - yaw))
            distance = dist[target]
            terms = {
                'size': w['size'] * min(len(cells), w['size_cap']) / w['size_cap'],
                'distance': -w['distance'] * distance / w['distance_scale_m'],
                'clearance': w['clearance'] * min(clearance, 1.0),
                'turn': -w['turn'] * turn / math.pi,
            }
            info.update({
                'status': 'candidate',
                'target_cell_xy': [round(tx, 2), round(ty, 2)],
                'distance_m': round(distance, 2),
                'aim_xy': [round(ax, 2), round(ay, 2)],
                'aim_distance_m': round(math.hypot(ax - x, ay - y), 2),
                'heading_deg': round(math.degrees(heading), 1),
                'turn_deg': round(math.degrees(_wrap(heading - yaw)), 1),
                'clearance_m': round(clearance, 2),
                'score_terms': {k: round(v, 3) for k, v in terms.items()},
                'score': round(sum(terms.values()), 3),
                '_heading': heading,
                '_aim': (ax, ay),
            })
        clusters.append(info)

    clusters.sort(key=lambda c: (c['status'] != 'candidate', -c.get('score', -1e9),
                                 c['straight_distance_m']))
    return {
        'robot_cell': list(start),
        'frontier_cells': len(frontier),
        'frontier_cells_excluded': len(dropped),
        'excluded_cells': dropped,
        'reachable_cells': len(dist),
        'clusters': clusters,
    }
