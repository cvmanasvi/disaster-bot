import json
import math
import os
from collections import deque, namedtuple
from types import SimpleNamespace
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile

from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from my_first_robot.exploration_grid import (
    DEFAULT_WEIGHTS as GRID_DEFAULT_WEIGHTS, ExplorationGrid, OCCUPIED, evaluate_frontiers,
    in_portal, interior_region, move_target)
from my_first_robot.victim_prioritizer import (
    choose as victim_choose, rank as victim_rank, risk_score as victim_risk, VictimCandidate)


class State(Enum):
    STARTUP = auto()
    EXPLORING = auto()
    TURNING = auto()
    FOLLOWING_BOUNDARY = auto()
    CROSSING_OPENING = auto()
    RECOVERY = auto()
    FAULT_HOLD = auto()

    # House-entrance entry (IMU heading based).
    ENTRANCE_APPROACH = auto()
    ENTRANCE_ALIGN = auto()
    ENTRANCE_CROSS = auto()
    VERIFY_INSIDE = auto()
    STOP_AND_SCAN = auto()
    EXPLORE_TURN = auto()
    EXPLORE_MOVE = auto()
    INSIDE_HOLD = auto()

    # Victim reached (victim_approach_enabled): terminal for the MVP.
    VICTIM_HOLD = auto()


ENTRY_STATES = (
    State.ENTRANCE_APPROACH,
    State.ENTRANCE_ALIGN,
    State.ENTRANCE_CROSS,
    State.VERIFY_INSIDE,
    State.STOP_AND_SCAN,
    State.EXPLORE_TURN,
    State.EXPLORE_MOVE,
    State.INSIDE_HOLD,
)


Pose2D = namedtuple('Pose2D', ['x', 'y', 'yaw'])


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_in_sector(angle_deg, min_deg, max_deg):
    if min_deg <= max_deg:
        return min_deg <= angle_deg <= max_deg
    return angle_deg >= min_deg or angle_deg <= max_deg


def clamp(value, low, high):
    return max(low, min(high, value))


def _flank_extent(samples, edge_index, step, edge_jump_m, max_angle_gap_rad):
    """Physical length (m) of the contiguous surface a flank edge belongs to.

    Walks outward from samples[edge_index] (step = -1 or +1) while range
    stays continuous and no rays are missing, then returns the straight-line
    distance between the first and last points of that surface.
    """

    j = edge_index

    while 0 <= j + step < len(samples):
        a = samples[j]
        b = samples[j + step]
        if abs(b[1] - a[1]) >= edge_jump_m:
            break
        if abs(normalize_angle(b[0] - a[0])) > max_angle_gap_rad:
            break
        j += step

    p = samples[edge_index]
    q = samples[j]

    return math.hypot(
        p[1] * math.cos(p[0]) - q[1] * math.cos(q[0]),
        p[1] * math.sin(p[0]) - q[1] * math.sin(q[0]),
    )


def detect_opening(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    center_deg,
    half_width_deg,
    edge_jump_m,
    min_passable_width_m,
    max_passable_width_m,
    min_depth_m,
    min_flank_extent_m=0.0,
):
    """Find a doorway-like gap (near / far / near) inside a scan window.

    Returns None, or a dict with 'bearing_rad', 'width_m', 'depth_m' for the
    candidate whose bearing is closest to center_deg. Pure function of raw
    scan data so it can be unit tested without ROS.
    """

    samples = []

    for i, distance in enumerate(ranges):

        if not math.isfinite(distance):
            continue

        if distance < range_min or distance > range_max:
            continue

        angle = normalize_angle(angle_min + i * angle_increment)

        angular_offset_deg = math.degrees(
            normalize_angle(angle - math.radians(center_deg))
        )

        if abs(angular_offset_deg) > half_width_deg:
            continue

        samples.append((angle, distance))

    if len(samples) < 5:
        return None

    samples.sort(key=lambda s: s[0])

    # Segment the angularly-sorted samples wherever range jumps sharply.
    boundaries = [0]

    for i in range(1, len(samples)):
        if abs(samples[i][1] - samples[i - 1][1]) >= edge_jump_m:
            boundaries.append(i)

    boundaries.append(len(samples))

    best_candidate = None
    best_score = None

    for seg in range(1, len(boundaries) - 2):

        start = boundaries[seg]
        end = boundaries[seg + 1]
        run = samples[start:end]

        if not run:
            continue

        run_depth = min(d for _, d in run)

        if run_depth < min_depth_m:
            continue

        if start - 1 < 0 or end >= len(samples):
            continue

        near_edge = samples[start - 1]
        far_edge = samples[end]

        # Both flanks must look like nearer jambs, not more far-run.
        if near_edge[1] >= run_depth or far_edge[1] >= run_depth:
            continue

        ax = near_edge[1] * math.cos(near_edge[0])
        ay = near_edge[1] * math.sin(near_edge[0])
        bx = far_edge[1] * math.cos(far_edge[0])
        by = far_edge[1] * math.sin(far_edge[0])

        width = math.hypot(bx - ax, by - ay)

        if width < min_passable_width_m or width > max_passable_width_m:
            continue

        # Doorway jambs belong to walls that continue sideways; isolated
        # furniture legs are only a few cm wide. Reject thin flanks.
        max_angle_gap = 1.5 * abs(angle_increment)
        near_extent = _flank_extent(samples, start - 1, -1, edge_jump_m, max_angle_gap)
        far_extent = _flank_extent(samples, end, 1, edge_jump_m, max_angle_gap)

        if near_extent < min_flank_extent_m or far_extent < min_flank_extent_m:
            continue

        bearing = normalize_angle((near_edge[0] + far_edge[0]) / 2.0)
        score = abs(math.degrees(bearing) - center_deg)

        if best_score is None or score < best_score:
            best_score = score
            best_candidate = {
                'bearing_rad': bearing,
                'width_m': width,
                'depth_m': run_depth,
            }

    return best_candidate


# =========================================================================
# ENTRANCE DETECTION (pure functions, no ROS)
# =========================================================================

def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max, x_offset=0.0):
    """Valid returns as (ray_index, angle, range, x, y) in the robot base
    frame (LiDAR origin at (x_offset, 0)), in ray order."""

    points = []

    for i, distance in enumerate(ranges):

        if not math.isfinite(distance):
            continue

        if distance < range_min or distance > range_max:
            continue

        angle = normalize_angle(angle_min + i * angle_increment)
        points.append((
            i, angle, distance,
            x_offset + distance * math.cos(angle),
            distance * math.sin(angle),
        ))

    return points


def _fit_line(xy):
    """Total-least-squares line: (mean_x, mean_y, direction_angle, rms)."""

    n = len(xy)
    mx = sum(p[0] for p in xy) / n
    my = sum(p[1] for p in xy) / n
    sxx = sum((p[0] - mx) ** 2 for p in xy)
    syy = sum((p[1] - my) ** 2 for p in xy)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in xy)
    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    nx, ny = -math.sin(theta), math.cos(theta)
    rms = math.sqrt(sum(((p[0] - mx) * nx + (p[1] - my) * ny) ** 2 for p in xy) / n)
    return mx, my, theta, rms


def _split_polyline(xy, start, end, tol, out):
    """Douglas-Peucker split of xy[start..end] into near-straight pieces."""

    ax, ay = xy[start]
    bx, by = xy[end]
    length = math.hypot(bx - ax, by - ay)
    worst = -1.0
    worst_k = None

    for k in range(start + 1, end):
        px, py = xy[k]
        if length < 1e-6:
            d = math.hypot(px - ax, py - ay)
        else:
            d = abs((bx - ax) * (ay - py) - (ax - px) * (by - ay)) / length
        if d > worst:
            worst = d
            worst_k = k

    if worst_k is not None and worst > tol:
        _split_polyline(xy, start, worst_k, tol, out)
        _split_polyline(xy, worst_k, end, tol, out)
    else:
        out.append((start, end))


def extract_wall_segments(
    points,
    num_rays,
    max_point_gap_m=0.35,
    max_index_gap=2,
    split_tol_m=0.05,
    min_points=4,
    min_length_m=0.30,
):
    """Straight wall pieces from scan_to_points() output.

    Returns dicts with the fitted line ('cx', 'cy', 'theta'), 'length', and
    the two ends ('end0'/'end1': (x, y) projected on the line, 'idx0'/'idx1':
    ray index of the first/last point, in ray order).
    """

    if len(points) < min_points:
        return []

    clusters = [[points[0]]]

    for prev, cur in zip(points, points[1:]):
        gap = math.hypot(cur[3] - prev[3], cur[4] - prev[4])
        if cur[0] - prev[0] <= max_index_gap and gap <= max_point_gap_m:
            clusters[-1].append(cur)
        else:
            clusters.append([cur])

    # Full 360-degree scan: the last and first clusters may be one surface.
    if len(clusters) > 1:
        first, last = clusters[0], clusters[-1]
        wrap_idx = first[0][0] + num_rays - last[-1][0]
        wrap_gap = math.hypot(first[0][3] - last[-1][3], first[0][4] - last[-1][4])
        if wrap_idx <= max_index_gap and wrap_gap <= max_point_gap_m:
            clusters[0] = last + first
            clusters.pop()

    segments = []

    for cluster in clusters:

        if len(cluster) < min_points:
            continue

        xy = [(p[3], p[4]) for p in cluster]
        pieces = []
        _split_polyline(xy, 0, len(xy) - 1, split_tol_m, pieces)

        for s, e in pieces:

            # Drop a shared split vertex that belongs to the neighbouring
            # piece (e.g. the jump point between a wall and an occluder).
            idx = list(range(s, e + 1))
            if len(idx) < min_points:
                continue

            mx, my, theta, _ = _fit_line([xy[k] for k in idx])
            nx, ny = -math.sin(theta), math.cos(theta)
            idx = [
                k for k in idx
                if abs((xy[k][0] - mx) * nx + (xy[k][1] - my) * ny) <= split_tol_m
            ]
            if len(idx) < min_points:
                continue

            mx, my, theta, rms = _fit_line([xy[k] for k in idx])
            ux, uy = math.cos(theta), math.sin(theta)

            def project(k):
                t = (xy[k][0] - mx) * ux + (xy[k][1] - my) * uy
                return (mx + t * ux, my + t * uy)

            end0 = project(idx[0])
            end1 = project(idx[-1])
            length = math.hypot(end1[0] - end0[0], end1[1] - end0[1])

            if length < min_length_m:
                continue

            segments.append({
                'cx': mx, 'cy': my, 'theta': theta, 'rms': rms,
                'length': length,
                'end0': end0, 'end1': end1,
                'idx0': cluster[idx[0]][0], 'idx1': cluster[idx[-1]][0],
            })

    return segments


def _ray_index_for_angle(angle, angle_min, angle_increment, num_rays):
    return int(round(normalize_angle(angle - angle_min) / angle_increment)) % num_rays


def detect_entrances(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    x_offset=0.0,
    min_width_m=0.70,
    max_width_m=1.30,
    collinear_tol_deg=8.0,
    collinear_offset_m=0.10,
    max_range_m=3.3,
    min_line_distance_m=0.15,
    min_through_rays=2,
    max_blocked_ratio=0.2,
    segment_kwargs=None,
):
    """Doorways in a straight wall: two collinear wall segments separated by
    a gap of door width, with the scan passing THROUGH the gap.

    Works on the full 360-degree scan, so the doorway can be anywhere around
    the robot. Returns candidates in the robot base frame:
      center (x, y) on the wall face line, 'normal' (rad, pointing from the
      robot's side of the wall to the far side), 'width', 'jambs' (two
      (x, y) points), ray statistics through the gap, and 'occluded' True
      when a jamb end is hidden behind a nearer object (width unreliable).
    """

    num_rays = len(ranges)
    points = scan_to_points(ranges, angle_min, angle_increment, range_min, range_max, x_offset)
    segments = extract_wall_segments(points, num_rays, **(segment_kwargs or {}))

    candidates = []

    for ia in range(len(segments)):
        for ib in range(ia + 1, len(segments)):

            sa = segments[ia]
            sb = segments[ib]

            dtheta = abs(normalize_angle(sa['theta'] - sb['theta']))
            dtheta = min(dtheta, math.pi - dtheta)
            if math.degrees(dtheta) > collinear_tol_deg:
                continue

            ref = sa if sa['length'] >= sb['length'] else sb
            ux, uy = math.cos(ref['theta']), math.sin(ref['theta'])
            vx, vy = -uy, ux

            def along(p):
                return (p[0] - ref['cx']) * ux + (p[1] - ref['cy']) * uy

            def across(p):
                return (p[0] - ref['cx']) * vx + (p[1] - ref['cy']) * vy

            if any(abs(across(p)) > collinear_offset_m
                   for s in (sa, sb) for p in (s['end0'], s['end1'])):
                continue

            ends_a = sorted([(along(sa['end0']), sa['idx0']), (along(sa['end1']), sa['idx1'])])
            ends_b = sorted([(along(sb['end0']), sb['idx0']), (along(sb['end1']), sb['idx1'])])

            if ends_a[1][0] < ends_b[0][0]:
                (ta, ray_a), (tb, ray_b) = ends_a[1], ends_b[0]
            elif ends_b[1][0] < ends_a[0][0]:
                (ta, ray_a), (tb, ray_b) = ends_a[0], ends_b[1]
            else:
                continue

            width = abs(tb - ta)
            if width < min_width_m or width > max_width_m:
                continue

            # Jamb points and centre on the wall-face line.
            offset = sum(across(p) for s in (sa, sb) for p in (s['end0'], s['end1'])) / 4.0
            jamb_a = (ref['cx'] + ta * ux + offset * vx, ref['cy'] + ta * uy + offset * vy)
            jamb_b = (ref['cx'] + tb * ux + offset * vx, ref['cy'] + tb * uy + offset * vy)
            cx = 0.5 * (jamb_a[0] + jamb_b[0])
            cy = 0.5 * (jamb_a[1] + jamb_b[1])

            if math.hypot(cx, cy) > max_range_m:
                continue

            line_distance = -(ref['cx'] * vx + ref['cy'] * vy) - offset
            nx, ny = (vx, vy) if line_distance < 0.0 else (-vx, -vy)
            line_distance = abs(line_distance)
            if line_distance < min_line_distance_m:
                continue

            # Classify every ray aimed between the jambs.
            ang_a = math.atan2(jamb_a[1], jamb_a[0] - x_offset)
            ang_b = math.atan2(jamb_b[1], jamb_b[0] - x_offset)
            span = normalize_angle(ang_b - ang_a)
            steps = int(abs(span) / abs(angle_increment))
            direction = 1.0 if span > 0 else -1.0

            through = occluded = blocked = 0
            for k in range(1, steps):
                ang = ang_a + direction * k * abs(angle_increment)
                wx, wy = math.cos(ang), math.sin(ang)
                denom = wx * nx + wy * ny
                if denom < 0.05:
                    continue
                t_line = ((cx - x_offset) * nx + cy * ny) / denom
                r = ranges[_ray_index_for_angle(ang, angle_min, angle_increment, num_rays)]
                if not math.isfinite(r) or r > range_max or r > t_line + 0.25:
                    through += 1
                elif r < range_min:
                    continue
                elif r < t_line - 0.12:
                    occluded += 1
                else:
                    blocked += 1

            total = through + occluded + blocked
            if through < min_through_rays:
                continue
            if total and blocked > max(1, max_blocked_ratio * total):
                continue

            # A jamb end is only trustworthy if the ray just beyond it (on
            # the gap side) is not a nearer object hiding the real jamb.
            jamb_occluded = False
            for ray, other in ((ray_a, ray_b), (ray_b, ray_a)):
                step = 1 if (other - ray) % num_rays < num_rays // 2 else -1
                r_end = ranges[ray]
                r_next = ranges[(ray + step) % num_rays]
                if (math.isfinite(r_next) and range_min <= r_next <= range_max and
                        math.isfinite(r_end) and r_next < r_end - 0.10):
                    jamb_occluded = True

            candidates.append({
                'center': (cx, cy),
                'normal': math.atan2(ny, nx),
                'width': width,
                'jambs': (jamb_a, jamb_b),
                'line_distance': line_distance,
                'through': through,
                'occluded_rays': occluded,
                'blocked': blocked,
                'occluded': jamb_occluded,
            })

    # The same gap can be found from several segment pairs; keep the one
    # with the most through-rays per location.
    unique = []
    for c in sorted(candidates, key=lambda c: -c['through']):
        if all(math.hypot(c['center'][0] - u['center'][0], c['center'][1] - u['center'][1]) > 0.25
               for u in unique):
            unique.append(c)

    return unique


def corridor_bounds(points_xy, x_min, x_max, center_y, half_window):
    """Nearest obstacle on each side of the line y = center_y, among points
    with x in [x_min, x_max] and |y - center_y| <= half_window.

    Returns (left_y, right_y); either may be None.
    """

    left = None
    right = None

    for x, y in points_xy:
        if x < x_min or x > x_max:
            continue
        if abs(y - center_y) > half_window:
            continue
        if y >= center_y:
            left = y if left is None else min(left, y)
        else:
            right = y if right is None else max(right, y)

    return left, right


def straight_path_blocker(points_xy, travel_m, min_lateral_m, end_margin_m):
    """First point that would come within min_lateral_m of the robot's
    straight forward path over travel_m (plus end_margin_m), or None."""

    worst = None

    for x, y in points_xy:
        if x <= 0.0 or x > travel_m + end_margin_m:
            continue
        if abs(y) < min_lateral_m and (worst is None or x < worst[0]):
            worst = (x, y)

    return worst


def cone_min_range(
    ranges, angle_min, angle_increment, range_min, range_max, center_deg, half_width_deg
):
    """Minimum raw LiDAR range inside a cone (same test as safety_controller)."""

    best = float('inf')

    for i, distance in enumerate(ranges):
        if not math.isfinite(distance) or distance < range_min or distance > range_max:
            continue
        angle_deg = math.degrees(normalize_angle(angle_min + i * angle_increment))
        offset_deg = math.degrees(normalize_angle(math.radians(angle_deg - center_deg)))
        if abs(offset_deg) <= half_width_deg:
            best = min(best, distance)

    return best


def scan_polar_from_robot(ranges, angle_min, angle_increment, range_min, range_max, x_offset=0.0):
    """Every ray in ray order as (bearing, range) from the ROBOT CENTRE
    (LiDAR at (x_offset, 0)); invalid / no-return rays have range inf."""

    out = []
    for i, distance in enumerate(ranges):
        angle = angle_min + i * angle_increment
        if not math.isfinite(distance) or distance < range_min or distance > range_max:
            out.append((normalize_angle(angle), math.inf))
            continue
        x = x_offset + distance * math.cos(angle)
        y = distance * math.sin(angle)
        out.append((math.atan2(y, x), math.hypot(x, y)))
    return out


# Release of a remembered narrow obstacle on contradicting evidence: the
# contradicting surface must pass this close to the remembered centre, and
# contradictions only count from scan poses at least this far apart (a
# rescan from the same spot is not an independent view).
NARROW_RELEASE_NEAR_M = 0.15
NARROW_RELEASE_MIN_VIEW_SEPARATION_M = 0.10


def contiguous_clusters(polar, jump_m=0.10):
    """
    Split one circular sweep into contiguous runs of returns.

    polar: (bearing, range) in angular order (inf = no return). Neighbouring
    rays belong to the same run when both are returns whose ranges differ by
    no more than max(jump_m, 5 % of the nearer range); a no-return ray ends a
    run. Returns lists of indices in angular order, or None when the whole
    sweep is a single closed contour (no break anywhere).
    """
    n = len(polar)

    def cont(a, b):
        ra, rb = polar[a][1], polar[b][1]
        if not (math.isfinite(ra) and math.isfinite(rb)):
            return False
        return abs(ra - rb) <= max(jump_m, 0.05 * min(ra, rb))

    start = next((i for i in range(n) if not cont((i - 1) % n, i)), None)
    if start is None:
        return None

    clusters, cur = [], []
    for k in range(n):
        i = (start + k) % n
        if not math.isfinite(polar[i][1]):
            if cur:
                clusters.append(cur)
            cur = []
            continue
        if cur and not cont(cur[-1], i):
            clusters.append(cur)
            cur = []
        cur.append(i)
    if cur:
        clusters.append(cur)
    return clusters


def contiguous_surface_length(polar, px, py, near_m=0.15, jump_m=0.10):
    """
    Return the length of the longest contiguous surface passing near a point.

    Surfaces are the runs of contiguous_clusters (the continuity rule of
    detect_narrow_obstacles); a run counts when one of its returns lies
    within near_m of (px, py), and its length is measured along its returns.
    The point is in the frame of the sweep (centred on the sensor); 0.0 when
    no surface passes that close. Used as contradicting evidence for a
    remembered narrow obstacle: an isolated post is a short surface on its
    own; a wall end / cabinet edge seen from another side is part of a long
    one.
    """
    if len(polar) < 3:
        return 0.0
    clusters = contiguous_clusters(polar, jump_m)
    if clusters is None:
        clusters = [[i for i in range(len(polar)) if math.isfinite(polar[i][1])]]
    best = 0.0
    for c in clusters:
        pts = [(polar[i][1] * math.cos(polar[i][0]), polar[i][1] * math.sin(polar[i][0]))
               for i in c]
        if min(math.hypot(x - px, y - py) for x, y in pts) > near_m:
            continue
        length = sum(math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1])
                     for k in range(len(pts) - 1))
        best = max(best, length)
    return best


def detect_narrow_obstacles(polar, max_width_m=0.15, max_range_m=3.0, jump_m=0.10,
                            background_m=0.20):
    """Isolated narrow obstacles (table columns, legs) in one circular sweep.

    polar: (bearing, range) in angular order around the robot (inf = no
    return). A cluster is a run of consecutive returns whose range changes by
    no more than max(jump_m, 5 % of range) between neighbours. It is a narrow
    obstacle when its chord (first to last point) is <= max_width_m, it is
    within max_range_m, and it stands in front of its surroundings: on both
    sides the next ray is a no-return or at least background_m farther. A
    cluster with a nearer neighbour is partly hidden and is left to the
    normal obstacle handling, as are wider clusters (walls, furniture faces).
    The reported position is the estimated object centre (face centroid moved
    back by half the chord).

    Returns [{'x', 'y', 'range', 'bearing', 'width', 'rays'}] in the frame of
    the bearings (centred on the robot).
    """

    n = len(polar)
    if n < 3:
        return []
    clusters = contiguous_clusters(polar, jump_m)
    if clusters is None:
        return []                                 # one closed contour, no isolated object

    out = []
    for c in clusters:
        rmin = min(polar[i][1] for i in c)
        if rmin > max_range_m:
            continue
        pts = [(polar[i][1] * math.cos(polar[i][0]), polar[i][1] * math.sin(polar[i][0]))
               for i in c]
        width = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
        if width > max_width_m:
            continue
        isolated = True
        for flank in ((c[0] - 1) % n, (c[-1] + 1) % n):
            r = polar[flank][1]
            if math.isfinite(r) and r < rmin + background_m:
                isolated = False
        if not isolated:
            continue
        # The returns are the object's near face: its centre lies about half
        # its width further along the bearing.
        x = sum(p[0] for p in pts) / len(pts)
        y = sum(p[1] for p in pts) / len(pts)
        face = math.hypot(x, y)
        if face > 1e-6:
            k = (face + 0.5 * width) / face
            x, y = x * k, y * k
        out.append({'x': x, 'y': y, 'range': math.hypot(x, y), 'bearing': math.atan2(y, x),
                    'width': width, 'rays': len(c)})
    return out


def narrow_keepout_travel(obstacles, heading, max_travel_m, radius_m, step_m=0.02,
                          tolerance_m=0.05):
    """Longest straight travel from the origin along heading (<= max_travel_m)
    that never brings the robot centre within radius_m of a narrow obstacle.
    A robot that already stands inside radius_m (by at most tolerance_m, e.g.
    after odometry/centroid jitter) may move as long as it stays outside
    radius_m - tolerance_m; deeper inside, only moves that do not get closer
    are allowed. Returns (travel_m, limiting obstacle or None)."""

    if not obstacles:
        return max_travel_m, None
    c, s = math.cos(heading), math.sin(heading)
    prev = [math.hypot(o['x'], o['y']) for o in obstacles]
    limit = [radius_m if d0 >= radius_m else radius_m - tolerance_m for d0 in prev]
    travel = 0.0
    k = 1
    while True:
        d_s = min(k * step_m, max_travel_m)
        px, py = d_s * c, d_s * s
        for j, o in enumerate(obstacles):
            d = math.hypot(o['x'] - px, o['y'] - py)
            if d < limit[j] and d < prev[j] - 1e-9:
                return travel, o
            prev[j] = d
        travel = d_s
        if d_s >= max_travel_m:
            return travel, None
        k += 1


def tilt_exceeded(roll_deg, pitch_deg, limit_deg):
    """True when the body is tilted to or beyond the limit."""
    return abs(roll_deg) >= limit_deg or abs(pitch_deg) >= limit_deg


def _bearing_deg_of(point):
    return math.degrees(math.atan2(point[1], point[0])) % 360.0


def _circular_runs(flags):
    """(start, length) runs of True in a circular list."""

    n = len(flags)
    if all(flags):
        return [(0, n)]
    start = next(i for i in range(n) if not flags[i]) + 1
    runs = []
    run_start = None
    for k in range(n):
        i = (start + k) % n
        if flags[i] and run_start is None:
            run_start = i
            length = 0
        if flags[i]:
            length += 1
        elif run_start is not None:
            runs.append((run_start, length))
            run_start = None
    if run_start is not None:
        runs.append((run_start, length))
    return runs


def analyze_rotation_scan(
    samples,
    x_offset=0.0,
    free_range_m=1.5,
    min_free_sector_deg=15.0,
    min_wall_length_m=0.5,
    min_valid_fraction=0.3,
    num_bins=360,
):
    """Merge LiDAR scans taken while rotating in place into one world-frame
    polar profile around the robot and describe it.

    samples: list of (yaw, ranges, angle_min, angle_increment, range_min,
    range_max) with yaw the world heading at the scan time. Bearings in the
    result are world (IMU) bearings in degrees, 0..360.
    """

    bin_width = 2.0 * math.pi / num_bins
    bins = [[] for _ in range(num_bins)]
    rays_per_bin = [0] * num_bins
    maxrange_per_bin = [0] * num_bins
    valid_rays = 0
    total_rays = 0

    for yaw, ranges, angle_min, angle_increment, range_min, range_max in samples:
        c, s = math.cos(yaw), math.sin(yaw)
        lx, ly = x_offset * c, x_offset * s
        for i, r in enumerate(ranges):
            total_rays += 1
            ray_angle = yaw + angle_min + i * angle_increment
            valid = math.isfinite(r) and range_min <= r <= range_max
            if valid:
                px = lx + r * math.cos(ray_angle)
                py = ly + r * math.sin(ray_angle)
                bearing = math.atan2(py, px)
                distance = math.hypot(px, py)
            else:
                bearing = normalize_angle(ray_angle)
            k = int((bearing % (2.0 * math.pi)) / bin_width) % num_bins
            rays_per_bin[k] += 1
            if valid:
                valid_rays += 1
                bins[k].append(distance)
            elif not math.isfinite(r) or r > range_max:
                maxrange_per_bin[k] += 1

    profile = []
    for k in range(num_bins):
        values = sorted(bins[k])
        if values and len(values) >= min_valid_fraction * rays_per_bin[k]:
            profile.append(values[len(values) // 2])
        else:
            profile.append(float('inf'))

    def bearing_deg(k):
        return (k + 0.5) * 360.0 / num_bins

    # Walls: straight segments of the merged profile.
    points = scan_to_points(profile, 0.5 * bin_width, bin_width, 0.0, 1e9, 0.0)
    walls = []
    for seg in extract_wall_segments(points, num_bins, min_length_m=min_wall_length_m):
        nx, ny = -math.sin(seg['theta']), math.cos(seg['theta'])
        distance = seg['cx'] * nx + seg['cy'] * ny
        if distance < 0.0:
            nx, ny, distance = -nx, -ny, -distance
        walls.append({
            'distance_m': round(distance, 3),
            'normal_bearing_deg': round(math.degrees(math.atan2(ny, nx)) % 360.0, 1),
            'length_m': round(seg['length'], 2),
            'from_bearing_deg': round(_bearing_deg_of(seg['end0']), 1),
            'to_bearing_deg': round(_bearing_deg_of(seg['end1']), 1),
        })
    walls.sort(key=lambda w: w['distance_m'])

    # Free / open sectors.
    free = [r >= free_range_m for r in profile]
    free_sectors = []
    for start, length in _circular_runs(free):
        width = length * 360.0 / num_bins
        if width < min_free_sector_deg:
            continue
        idx = [(start + j) % num_bins for j in range(length)]
        finite = [profile[i] for i in idx if math.isfinite(profile[i])]
        free_sectors.append({
            'from_deg': round(bearing_deg(idx[0]) - 0.5, 1),
            'to_deg': round((bearing_deg(idx[-1]) + 0.5) % 360.0, 1),
            'width_deg': round(width, 1),
            'centre_deg': round((bearing_deg(idx[0]) + 0.5 * width - 0.5) % 360.0, 1),
            'min_range_m': round(min(finite), 2) if finite else None,
            'no_return_fraction': round(1.0 - len(finite) / length, 2),
        })
    free_sectors.sort(key=lambda f: -f['width_deg'])

    # Nearest obstacles, at least 30 degrees apart.
    nearest = []
    for k in sorted(range(num_bins), key=lambda k: profile[k]):
        if not math.isfinite(profile[k]) or len(nearest) >= 5:
            break
        separation = [
            abs((bearing_deg(k) - n['bearing_deg'] + 180.0) % 360.0 - 180.0) for n in nearest
        ]
        if all(d >= 30.0 for d in separation):
            nearest.append({
                'bearing_deg': round(bearing_deg(k), 1), 'range_m': round(profile[k], 3)
            })

    return {
        'profile': profile,
        'scans': len(samples),
        'total_rays': total_rays,
        'valid_rays': valid_rays,
        'valid_per_bin': [len(b) for b in bins],
        'rays_per_bin': rays_per_bin,
        'maxrange_per_bin': maxrange_per_bin,
        'bins_with_rays': sum(1 for n in rays_per_bin if n),
        'bins_with_returns': sum(1 for r in profile if math.isfinite(r)),
        'num_bins': num_bins,
        'walls': walls,
        'free_sectors': free_sectors,
        'nearest': nearest,
    }


# ---------------------------------------------------------------------------
# Victim approach (victim_approach_enabled). /victim_distance comes from
# victim_distance_estimator: LiDAR range to the person (from the LiDAR origin)
# along the camera bearing of the YOLO box centre. The detector stamps its
# messages with wall-clock time, so a reading cannot be matched to the pose at
# image time: readings are only used while the robot is not rotating, and the
# position is placed with the CURRENT nav pose (odometry distance along the
# IMU heading, origin = spawn). That pose drifts (4-11 cm over 60-step runs)
# and the bearing is good to about +-3 deg (+-0.15 m sideways at 3 m), so a
# victim position is a local nav-frame estimate good to roughly 0.2 m, not a
# ground-truth or map-corrected coordinate.

VICTIM_MIN_CONFIDENCE = 0.60         # same bar as victim_rescue.py confirmation
VICTIM_MAX_RANGE_M = 3.5             # LiDAR range limit
VICTIM_MIN_RANGE_M = 0.20            # estimator's own minimum
VICTIM_MAX_BEARING_DEG = 100.0       # camera half-FOV is ~91 deg
VICTIM_MAX_AGE_S = 0.5               # estimator-reported detection age
VICTIM_STILL_RATE_DEG_S = 3.0        # IMU yaw rate (1 s window) counted as not rotating
VICTIM_CLOSE_RANGE_M = 1.2           # camera may lose the person closer than this
VICTIM_REACHED_TOL_M = 0.05          # within this of the stop distance = reached
VICTIM_MAX_APPROACH_MOVES = 8        # never chase one target for ever
VICTIM_TRACK_LEN = 5                 # position = mean of the last N agreeing readings
VICTIM_FACE_TIMEOUT_S = 15.0         # VICTIM_HOLD facing turn gives up after this
VICTIM_REPUBLISH_S = 5.0             # /victim_found repeated while holding
# Approach headings tried in this order (deg from the victim bearing, + = left /
# counter-clockwise): the direct bearing first, then the frontier planner's
# +-10 / +-20 deg heading offsets. Each is only a candidate: it must pass the
# same checks as the direct approach before anything moves.
VICTIM_HEADING_OFFSETS_DEG = (0.0, 10.0, -10.0, 20.0, -20.0)


def victim_measurement(data):
    """
    Parse one /victim_distance JSON dict.

    Returns (distance_m, angle_rad, confidence), or None when anything is
    missing, non-finite, stale or out of range.
    """
    try:
        distance = float(data['distance_m'])
        angle_deg = float(data['angle_deg'])
        confidence = float(data['confidence'])
        age = float(data.get('detection_age_s', 0.0))
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in (distance, angle_deg, confidence, age)):
        return None
    if not VICTIM_MIN_RANGE_M < distance <= VICTIM_MAX_RANGE_M:
        return None
    if abs(angle_deg) > VICTIM_MAX_BEARING_DEG:
        return None
    if not VICTIM_MIN_CONFIDENCE <= confidence <= 1.0:
        return None
    if not 0.0 <= age <= VICTIM_MAX_AGE_S:
        return None
    return distance, math.radians(angle_deg), confidence


def victim_nav_position(pose, distance, angle, lidar_offset_x):
    """
    Place a reading in the nav frame.

    The point lies `distance` along robot bearing `angle` from the LiDAR
    origin (lidar_offset_x ahead of the robot centre).
    """
    lx = pose.x + lidar_offset_x * math.cos(pose.yaw)
    ly = pose.y + lidar_offset_x * math.sin(pose.yaw)
    return lx + distance * math.cos(pose.yaw + angle), ly + distance * math.sin(pose.yaw + angle)


class VictimTrack:
    """
    Agreeing readings of one victim candidate (nav frame).

    Two confidence figures are kept on purpose: `confidence` is the MAX over
    all readings (reported in /victim_found and the confirmation log, as
    before), mean_confidence() is the MEAN over all readings (used only by
    the prioritizer, so one lucky frame cannot dominate the ranking).
    """

    def __init__(self, t, x, y, confidence, distance):
        self.obs = deque(maxlen=VICTIM_TRACK_LEN)
        self.count = 0
        self.confidence = 0.0
        self.conf_sum = 0.0
        self.id = 0                       # set by VictimMemory (YOLO victim_id is always 1)
        self.add(t, x, y, confidence, distance)

    def add(self, t, x, y, confidence, distance):
        self.obs.append((x, y))
        self.count += 1
        self.confidence = max(self.confidence, confidence)
        self.conf_sum += confidence
        self.distance = distance
        self.last_time = t
        self.x = sum(p[0] for p in self.obs) / len(self.obs)
        self.y = sum(p[1] for p in self.obs) / len(self.obs)

    def spread(self):
        return max(math.hypot(px - self.x, py - self.y) for px, py in self.obs)

    def mean_confidence(self):
        return self.conf_sum / self.count

    def as_dict(self):
        return {'x': round(self.x, 3), 'y': round(self.y, 3), 'count': self.count,
                'confidence': round(self.confidence, 3), 'distance_m': round(self.distance, 3),
                'last_time': round(self.last_time, 2)}


class VictimMemory:
    """
    Unconfirmed victim candidates.

    A reading joins the nearest track within radius_m, otherwise starts a new
    one; tracks not seen for timeout_s are dropped. A track is confirmed
    after `count` readings whose positions all lie within radius_m of their
    mean.
    """

    def __init__(self, radius_m, count, timeout_s):
        self.radius_m = radius_m
        self.count = count
        self.timeout_s = timeout_s
        self.tracks = []
        self.next_id = 1                  # candidate ids: creation order, never reused

    def observe(self, t, x, y, confidence, distance):
        self.prune(t)                     # never extend an expired track
        near = [tr for tr in self.tracks if math.hypot(tr.x - x, tr.y - y) <= self.radius_m]
        if near:
            tr = min(near, key=lambda tr: math.hypot(tr.x - x, tr.y - y))
            tr.add(t, x, y, confidence, distance)
            return tr
        tr = VictimTrack(t, x, y, confidence, distance)
        tr.id = self.next_id
        self.next_id += 1
        self.tracks.append(tr)
        return tr

    def prune(self, now):
        self.tracks = [tr for tr in self.tracks if now - tr.last_time <= self.timeout_s]

    def confirmed_all(self, now, excluded=()):
        """Return every confirmed, non-excluded track (creation order)."""
        self.prune(now)
        return [tr for tr in self.tracks
                if tr.count >= self.count and tr.spread() <= self.radius_m
                and all(math.hypot(tr.x - ex, tr.y - ey) > 2.0 * self.radius_m
                        for ex, ey in excluded)]

    def confirmed(self, now, excluded=()):
        ok = self.confirmed_all(now, excluded)
        return max(ok, key=lambda tr: tr.count) if ok else None

    def clear(self):
        self.tracks = []


def victim_offset_wanted(range_m, offset_rad, stop_m):
    """
    Return the useful travel along a heading offset_rad off the victim bearing.

    Longest straight move that keeps the victim (range_m away) at least stop_m
    away; for offset 0 it is exactly range_m - stop_m. When the line passes
    outside the stop circle, the move ends at the closest point of approach.
    """
    along = range_m * math.cos(offset_rad)
    across = range_m * math.sin(offset_rad)
    if abs(across) >= stop_m:
        return max(0.0, along)
    return max(0.0, along - math.sqrt(stop_m * stop_m - across * across))


def victim_approach_move(range_m, stop_m, free_m, keep_m, min_step_m):
    """
    Size one approach step toward a victim `range_m` ahead (after turning).

    Returns (move_m, outcome): 'reached' (within VICTIM_REACHED_TOL_M of the
    stop distance, no move), 'move' (0 < move_m <= range_m - stop_m, inside
    the free travel and the keep-out) or 'blocked' (a useful move is not
    safe: the target is abandoned, never forced).
    """
    wanted = range_m - stop_m
    if wanted <= VICTIM_REACHED_TOL_M:
        return 0.0, 'reached'
    move = min(wanted, free_m, keep_m)
    if move < min(min_step_m, wanted):
        return 0.0, 'blocked'
    return move, 'move'


class AutonomousSearch(Node):

    def __init__(self):
        super().__init__('autonomous_search')

        scalar_defaults = {
            'control_period_sec': 0.10,

            # Sector boundaries (degrees, robot-relative, 0 = forward).
            'front_min_deg': -20.0,
            'front_max_deg': 20.0,
            'front_left_min_deg': 20.0,
            'front_left_max_deg': 65.0,
            'front_right_min_deg': -65.0,
            'front_right_max_deg': -20.0,
            'left_min_deg': 65.0,
            'left_max_deg': 115.0,
            'right_min_deg': -115.0,
            'right_max_deg': -65.0,
            'rear_min_deg': 160.0,
            'rear_max_deg': -160.0,
            'sector_outlier_trim_ratio': 0.20,

            # Speeds / steering.
            'forward_speed_max': 0.18,
            'slow_speed': 0.09,
            'min_creep_speed': 0.04,
            'wall_follow_speed': 0.13,
            'turn_speed': 0.35,
            'opening_align_turn_speed': 0.35,
            'opening_cross_speed': 0.12,
            'angular_slew_rate': 1.0,
            'explore_steer_gain': 0.15,
            'explore_max_steer': 0.25,
            'explore_steer_deadband': 0.10,
            'wall_kp': 0.9,
            'wall_max_angular': 0.35,
            'wall_corner_turn_rate': 0.30,

            # Distances.
            'stop_distance': 0.45,
            'slow_distance': 0.85,
            'full_speed_distance': 1.3,
            'clear_distance': 1.1,
            'wall_target': 0.6,
            'wall_min': 0.30,
            'wall_max': 1.05,
            'wall_lost_grace_sec': 1.5,
            'max_follow_duration_sec': 45.0,
            'max_follow_distance_m': 6.0,

            # Turning.
            'turn_min_deg': 30.0,
            'turn_max_deg': 150.0,
            'turn_progress_eps_deg': 2.0,
            'turn_stall_timeout_sec': 4.0,
            'turn_backstop_timeout_sec': 12.0,

            # Opening / doorway detection.
            'opening_search_half_width_deg': 55.0,
            'opening_edge_jump_m': 0.35,
            'opening_min_depth_m': 1.0,
            'opening_min_passable_width_m': 0.45,
            'opening_max_passable_width_m': 2.2,
            'opening_min_flank_extent_m': 0.20,
            'opening_confirmation_scans': 3,
            'opening_bearing_match_tolerance_deg': 15.0,
            'opening_align_tolerance_deg': 8.0,
            'opening_align_timeout_sec': 8.0,
            'crossing_align_stall_timeout_sec': 2.5,
            'crossing_align_progress_epsilon_deg': 1.0,
            'opening_margin_m': 0.30,
            'opening_min_cross_distance_m': 0.6,
            'opening_cross_backstop_sec': 20.0,

            # Stuck detection.
            'stuck_linear_eps_m': 0.03,
            'stuck_angular_eps_deg': 3.0,
            'stuck_cmd_linear_eps': 0.02,
            'stuck_cmd_angular_eps': 0.05,
            'stuck_time_threshold_sec': 4.0,

            # Oscillation / direction memory.
            'oscillation_radius_m': 0.5,
            'oscillation_window_sec': 20.0,
            'oscillation_min_repeats': 3,
            'direction_history_window_sec': 20.0,
            'direction_history_radius_m': 0.5,

            # Recovery.
            'recovery_backup_speed': 0.06,
            'recovery_turn_speed': 0.30,
            'recovery_backup_backstop_sec': 8.0,
            'recovery_rotate_backstop_sec': 10.0,
            'recovery_rear_min_clearance': 0.35,
            'recovery_rear_emergency_clearance': 0.20,
            'recovery_success_linear_eps_m': 0.08,
            'recovery_success_angular_eps_deg': 15.0,

            # Freshness.
            'scan_stale_timeout_sec': 2.0,
            'odom_stale_timeout_sec': 1.0,
            'sensor_fresh_confirm_sec': 2.0,

            # ------------------------------------------------------------
            # Exploration Intelligence v2
            # ------------------------------------------------------------

            # Exploration memory.
            'memory_sample_min_dist_m': 0.25,
            'visited_memory_maxlen': 300,
            'visited_influence_radius_m': 0.6,

            # Opening memory.
            'opening_memory_maxlen': 20,
            'opening_memory_radius_m': 1.0,
            'opening_memory_bearing_tolerance_deg': 20.0,
            'opening_memory_min_travel_m': 3.0,

            # Decision scoring.
            'score_clearance_norm_m': 1.2,
            'score_lookahead_m': 1.0,
            'score_w_clearance': 1.0,
            'score_w_opening': 1.0,
            'score_w_unexplored': 0.6,
            'score_w_failed': 0.8,
            'score_w_repeat': 0.5,
            'score_tie_epsilon': 0.05,

            # Local structure classification.
            'corridor_side_max_m': 1.0,
            'corridor_balance_tolerance_m': 0.35,
            'corridor_speed_cap': 0.13,

            # Anti-loop (direction alternation pattern).
            'alternation_min_length': 4,

            # Exploration steering nudge -- kept weak so obstacle-clearance
            # steering always dominates it.
            'explore_memory_steer_gain': 0.05,

            # ------------------------------------------------------------
            # Exploration Boundary / Enclosure Detection v3
            # ------------------------------------------------------------

            'enclosure_forward_half_width_deg': 70.0,
            'enclosure_lateral_half_width_deg': 45.0,
            'enclosure_low_threshold': 0.25,
            'enclosure_high_threshold': 0.45,
            'enclosure_confirm_scans': 3,
            'global_void_ratio_threshold': 0.05,
            'score_w_enclosure': 0.7,

            # ------------------------------------------------------------
            # House-entrance entry (heading from /imu)
            # ------------------------------------------------------------

            # Start in OUTSIDE_SEARCH and enter the house before anything
            # else. After VERIFY_INSIDE: hold (true) or resume EXPLORING.
            'entry_mission_enabled': True,
            'entry_hold_after_inside': True,
            'imu_stale_timeout_sec': 0.5,
            'lidar_offset_x_m': -0.032,

            # Detection (see detect_entrances).
            'entry_min_width_m': 0.70,
            'entry_max_width_m': 1.30,
            'entry_max_detect_range_m': 3.3,
            'entry_confirm_hits': 3,
            'entry_confirm_window_sec': 4.0,
            'entry_match_radius_m': 0.35,
            'entry_match_angle_deg': 20.0,
            'entry_refine_alpha': 0.3,

            # Approach to a standoff point on the doorway normal.
            'entry_standoff_m': 1.0,
            'entry_approach_speed': 0.12,
            'entry_goal_tolerance_m': 0.08,
            'entry_heading_kp': 1.5,
            'entry_max_angular': 0.45,
            'entry_min_angular': 0.10,
            'entry_turn_tolerance_deg': 1.5,
            'entry_drive_heading_gate_deg': 20.0,
            'entry_block_cone_m': 0.40,
            'entry_progress_window_sec': 3.0,
            'entry_progress_min_m': 0.04,
            'entry_max_attempts': 3,
            'entry_retry_cooldown_sec': 20.0,

            # Alignment / centring. The safety_controller zeroes forward
            # speed when anything within +-30 deg is <= 0.30 m. A point at
            # lateral offset L beside a straight path reaches the cone edge
            # at range 2 L, so every obstacle must stay >= 0.15 m (+margin)
            # to the side of the path.
            'entry_safety_half_angle_deg': 30.0,
            'entry_safety_emergency_m': 0.30,
            'entry_safety_margin_m': 0.04,
            'entry_min_lateral_clearance_m': 0.17,
            'entry_center_tolerance_m': 0.02,
            'entry_center_margin_m': 0.20,
            'entry_corridor_half_window_m': 0.65,
            'entry_max_sidesteps': 3,
            'entry_sidestep_speed': 0.06,
            'entry_settle_sec': 0.6,
            'entry_align_timeout_sec': 60.0,

            # Crossing: stop just past the doorway, not at the far wall.
            'entry_cross_speed': 0.10,
            'entry_inside_target_m': 0.70,
            'entry_inside_min_m': 0.45,
            'entry_front_stop_m': 0.40,
            'entry_cross_lateral_gain': 0.6,
            'entry_cross_max_bias_deg': 6.0,
            'entry_cross_max_heading_err_deg': 12.0,
            'entry_cross_timeout_sec': 30.0,
            'entry_backout_speed': 0.06,
            'entry_backout_to_m': 0.70,

            # Inside verification.
            'entry_verify_min_inside_m': 0.40,
            'entry_verify_min_wall_points': 3,
            'entry_verify_max_scans': 5,

            # ------------------------------------------------------------
            # STOP_AND_SCAN: stationary 360-degree rotation after entry
            # (rotation measured with the IMU heading, never odometry).
            # ------------------------------------------------------------

            'scan_after_inside': True,
            'scan_settle_sec': 1.0,
            'scan_still_rate_deg_s': 0.5,
            'scan_target_deg': 360.0,
            'scan_done_tolerance_deg': 1.0,
            'scan_turn_speed': 0.35,
            'scan_min_turn_speed': 0.10,
            'scan_slowdown_deg': 30.0,
            'scan_stall_sec': 4.0,
            'scan_timeout_sec': 90.0,
            'scan_free_range_m': 1.5,
            'scan_min_free_sector_deg': 15.0,
            'scan_min_wall_length_m': 0.5,
            'scan_camera_fov_deg': 182.0,
            # Directory for per-scan JSON lines + summary ('' = off).
            'scan_record_dir': '',

            # ------------------------------------------------------------
            # Local exploration prototype: STOP_AND_SCAN -> choose sector
            # -> EXPLORE_TURN (IMU) -> EXPLORE_MOVE (short, bounded)
            # -> STOP_AND_SCAN ... for at most explore_max_steps moves.
            # ------------------------------------------------------------

            'explore_enabled': True,
            'explore_max_steps': 8,
            'explore_sector_count': 12,
            'explore_step_m': 1.0,
            'explore_min_step_m': 0.5,
            'explore_speed': 0.10,
            # Sector status (robust min = 3rd-nearest 1-degree bin, each bin
            # itself the median of every ray that fell in it during the turn).
            'explore_blocked_m': 0.70,
            # Unknown = too few rays actually measured (return or clean
            # max-range reading); a max-range ray means nothing within range.
            'explore_min_measured_fraction': 0.50,
            # Straight-path check: nothing within this lateral half-width
            # (>= 0.15 m keeps obstacles out of the safety cone) up to the
            # travel distance plus the stop margin.
            'explore_path_half_width_m': 0.20,
            'explore_stop_margin_m': 0.40,
            'explore_heading_offsets_deg': 10.0,
            'explore_visited_radius_m': 0.6,
            'explore_entrance_guard_m': 0.35,
            'explore_turn_timeout_sec': 25.0,
            'explore_move_heading_tol_deg': 10.0,

            # ------------------------------------------------------------
            # Persistent exploration grid (exploration_grid.py, not SLAM):
            # every complete scan updates a coarse UNKNOWN/FREE/OCCUPIED/
            # VISITED grid in the nav frame; the next move heads toward the
            # best reachable frontier cluster. 'sector' = old 12-sector table.
            # ------------------------------------------------------------

            'explore_planner': 'grid',
            'grid_resolution_m': 0.20,
            'grid_robot_radius_m': 0.11,
            # Flood-fill (reachability) cells need this much clearance to the
            # nearest OCCUPIED cell centre (0.30 = one-cell inflation).
            'grid_min_clear_m': 0.30,
            'grid_min_cluster_cells': 3,
            # A no-return bin frees cells (to range_max) only if at least this
            # fraction of its rays were clean max-range readings.
            'grid_no_return_min_fraction': 0.5,
            'grid_min_step_m': 0.30,
            # A move stopped by a LiDAR safety check after less than this
            # ends exploration (movement unsafe).
            'grid_unsafe_min_progress_m': 0.15,
            # The grid planner checks its straight corridor on the merged
            # 1-degree MEDIAN profile; the live check during the move uses the
            # minimum over every raw ray (range noise sigma 0.01 m). Recorded
            # moves: live minimum lateral 0.013..0.041 m (median 0.027) below
            # the planner's, so an obstacle the planner sees at 0.20..0.24 m is
            # likely to trip the live 0.20 m check right after the start. The
            # planner therefore requires half-width + this margin; the live
            # check keeps explore_path_half_width_m unchanged.
            'grid_plan_path_margin_m': 0.04,
            # The known entrance is a portal: a band over the doorway gap
            # (plus a margin into both jambs) from depth_out outside the outer
            # wall face to depth_in inside it. The interior region is every
            # free cell connected to the first inside scan pose without
            # crossing the band; only interior frontiers are eligible.
            # depth_in stays below explore_entrance_guard_m (0.35).
            'grid_portal_depth_out_m': 0.30,
            'grid_portal_depth_in_m': 0.30,
            'grid_portal_lateral_margin_m': 0.30,
            # Move length = min(explore_step_m, free travel, aim distance -
            # this margin); below grid_min_move_m the robot does not move and
            # scans again (twice in a row ends exploration).
            'grid_aim_stop_margin_m': 0.10,
            'grid_min_move_m': 0.10,

            # Narrow-obstacle keep-out (exploration only): an isolated LiDAR
            # cluster no wider than the threshold (table column / leg) may
            # stand on geometry below the scan plane (e.g. a 4 cm base plate).
            # The robot's rotation-swept body (radius robot_swept_radius_m)
            # must stay keepout_m away from it, i.e. the robot centre stays
            # keepout_m + robot_swept_radius_m away -- on the planned path,
            # at the stopping point and before every in-place 360 scan.
            'narrow_obstacle_width_threshold_m': 0.15,
            'narrow_obstacle_keepout_m': 0.50,
            'robot_swept_radius_m': 0.13,
            'narrow_obstacle_max_range_m': 2.0,
            'narrow_obstacle_background_m': 0.20,
            'narrow_obstacle_max_escapes': 2,
            # Jitter tolerance: the hard floor is keep-out radius minus this
            # (rotation check and moves that start just inside the keep-out).
            'narrow_obstacle_tolerance_m': 0.05,
            # A narrow cluster sitting on a connected OCCUPIED grid structure
            # of at least this many cells (within 1 m) is a wall end / door
            # jamb, not an isolated post: no keep-out.
            'narrow_obstacle_structure_cells': 5,
            # Release on contradicting evidence: a remembered narrow obstacle
            # whose EVERY observed width (first detection and all later ones
            # merged into it) exceeds narrow_release_min_width_m (thin columns,
            # and anything ever measured thin, are never released) that later
            # 360 scans, taken within
            # narrow_release_range_m, show to lie on one contiguous LiDAR
            # surface longer than narrow_release_min_surface_m is not a post
            # (e.g. a wall end first seen edge-on). After
            # narrow_release_contradictions such scans it leaves the keep-out
            # memory, and nothing within narrow_release_suppress_radius_m of
            # it is remembered again. Live and pre-rotation checks still act
            # on whatever the current scan shows.
            'narrow_release_range_m': 3.0,
            'narrow_release_min_width_m': 0.06,
            'narrow_release_min_surface_m': 0.30,
            'narrow_release_contradictions': 2,
            'narrow_release_suppress_radius_m': 0.20,
            # IMU tilt guard: |roll| or |pitch| at or above this -> zero
            # command, exploration over, terminal FAULT_HOLD.
            'tilt_stop_deg': 10.0,
            'grid_score_size': GRID_DEFAULT_WEIGHTS['size'],
            'grid_score_distance': GRID_DEFAULT_WEIGHTS['distance'],
            'grid_score_clearance': GRID_DEFAULT_WEIGHTS['clearance'],
            'grid_score_turn': GRID_DEFAULT_WEIGHTS['turn'],

            # Victim approach (inside the house, grid planner only). Off by
            # default: with False nothing below is created or consulted.
            # A /victim_distance person seen in victim_confirm_count readings
            # agreeing within victim_confirm_radius_m takes priority over the
            # next frontier; it is approached with ordinary EXPLORE_TURN /
            # EXPLORE_MOVE steps (scan after each) to victim_stop_distance_m
            # (LiDAR range), then VICTIM_HOLD. Not seen in
            # victim_lost_scan_limit approach scans -> abandoned. Unconfirmed
            # candidates expire after victim_memory_timeout_sec.
            'victim_approach_enabled': False,
            'victim_confirm_count': 3,
            'victim_confirm_radius_m': 0.50,
            'victim_stop_distance_m': 0.80,
            'victim_lost_scan_limit': 2,
            'victim_memory_timeout_sec': 30.0,
            # Victim prioritizer (needs victim_approach_enabled; off by
            # default). Confirmed candidates are ranked by confidence,
            # distance, approach accessibility and narrow-obstacle risk
            # (victim_prioritizer.py) at the post-scan decision point; the
            # target locks once its first approach move starts.
            'victim_prioritization_enabled': False,
        }

        for name, default in scalar_defaults.items():
            self.declare_parameter(name, default)

        self.declare_parameter('recovery_backup_distance_stage', [0.15, 0.30, 0.45])
        self.declare_parameter('recovery_rotate_deg_stage', [90.0, 135.0, 180.0])

        self.p = SimpleNamespace(**{
            name: self.get_parameter(name).value for name in scalar_defaults
        })
        self.p.recovery_backup_distance_stage = list(
            self.get_parameter('recovery_backup_distance_stage').value
        )
        self.p.recovery_rotate_deg_stage = list(
            self.get_parameter('recovery_rotate_deg_stage').value
        )

        # Parameters used as counts/lengths must be real ints regardless of
        # how rclpy hands them back.
        self.p.alternation_min_length = int(self.p.alternation_min_length)
        self.p.visited_memory_maxlen = int(self.p.visited_memory_maxlen)
        self.p.opening_memory_maxlen = int(self.p.opening_memory_maxlen)
        self.p.enclosure_confirm_scans = int(self.p.enclosure_confirm_scans)
        self.p.entry_confirm_hits = int(self.p.entry_confirm_hits)
        self.p.entry_max_attempts = int(self.p.entry_max_attempts)
        self.p.entry_max_sidesteps = int(self.p.entry_max_sidesteps)
        self.p.entry_verify_min_wall_points = int(self.p.entry_verify_min_wall_points)
        self.p.entry_verify_max_scans = int(self.p.entry_verify_max_scans)
        self.p.explore_max_steps = int(self.p.explore_max_steps)
        self.p.grid_min_cluster_cells = int(self.p.grid_min_cluster_cells)
        self.p.narrow_obstacle_max_escapes = int(self.p.narrow_obstacle_max_escapes)
        self.p.narrow_obstacle_structure_cells = int(self.p.narrow_obstacle_structure_cells)
        self.p.narrow_release_contradictions = int(self.p.narrow_release_contradictions)
        self.p.victim_confirm_count = int(self.p.victim_confirm_count)
        self.p.victim_lost_scan_limit = int(self.p.victim_lost_scan_limit)

        # ---------------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------------

        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.imu_subscriber = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10
        )
        self.cmd_publisher = self.create_publisher(
            TwistStamped, '/cmd_vel_raw', 10
        )
        # Victim approach: only when enabled (flag off = no extra interfaces).
        self.victim_found_publisher = None
        if self.p.victim_approach_enabled:
            self.create_subscription(String, '/victim_distance', self.victim_callback, 10)
            self.victim_found_publisher = self.create_publisher(
                String, '/victim_found',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )

        # ---------------------------------------------------------------
        # Sensor state
        # ---------------------------------------------------------------

        self.latest_scan = None
        self.latest_scan_stamp = None
        self.latest_pose = None
        self.latest_odom_stamp = None

        # IMU heading and the entry navigation pose: odometry supplies only
        # the forward distance increments, integrated along the IMU yaw.
        self.imu_yaw = None
        self.latest_imu_stamp = None
        self.nav_pose = None
        self._nav_prev_odom = None
        # (stamp_sec, yaw) for the heading at each LiDAR scan's timestamp.
        self.imu_history = deque(maxlen=800)

        # ---------------------------------------------------------------
        # State machine
        # ---------------------------------------------------------------

        self.state = State.STARTUP

        self.last_angular_cmd = 0.0
        self._last_cmd_linear = 0.0
        self._last_cmd_angular = 0.0

        self.last_meaningful_pose = None
        self.last_meaningful_time = 0.0

        self.total_path_length = 0.0
        self.prev_path_pose = None

        # Direction memory / oscillation detection: (time, x, y, direction)
        self.turn_entry_history = deque()
        self.last_turn_direction = 0

        self.opening_confirm_count = 0
        self.opening_last_bearing_deg = None

        self.follow_direction = 1
        self.wall_lost_since = None
        self.follow_entry_time = 0.0
        self.follow_start_path_length = 0.0
        self.last_wall_follow_angular = 0.0

        self.crossing_phase = None
        self.crossing_fallback_state = State.EXPLORING
        self.crossing_target_yaw = 0.0
        self.crossing_target_distance = 0.0
        self.crossing_align_start_time = 0.0
        self.crossing_align_best_error_deg = None
        self.crossing_align_best_error_time = 0.0
        self.crossing_start_pose = None
        self.crossing_drive_start_time = 0.0

        self.recovery_stage = 0
        self.recovery_substate = None
        self.recovery_stage_start_pose = None
        self.recovery_substate_start_time = 0.0
        self.recovery_backup_target_distance = 0.0
        self.recovery_rotate_start_yaw = 0.0
        self.recovery_rotate_direction = 1
        self.recovery_rotate_target_deg = 90.0

        self.fault_reason = ''
        self.fault_terminal = False
        self.fault_fresh_since = None

        # ---------------------------------------------------------------
        # Exploration Intelligence v2 state
        # ---------------------------------------------------------------

        self.visited_memory = deque(maxlen=self.p.visited_memory_maxlen)
        self.opening_memory = deque(maxlen=self.p.opening_memory_maxlen)
        self.last_structure_label = None

        # ---------------------------------------------------------------
        # Exploration Boundary / Enclosure Detection v3 state
        # ---------------------------------------------------------------

        self.turn_search_mode = False
        self.enclosure_low_streak = 0
        self._enclosure_last_scan_key = None
        self._enclosure_last_forward_ratio = 1.0
        self._enclosure_last_global_ratio = 1.0

        # ---------------------------------------------------------------
        # House-entrance entry state
        # ---------------------------------------------------------------

        # OUTSIDE_SEARCH -> ENTERING -> INSIDE (or DISABLED).
        self.mission_phase = 'OUTSIDE_SEARCH' if self.p.entry_mission_enabled else 'DISABLED'
        self._entry_last_scan_key = None
        self.entry_tracks = []
        # Persisted entrance, nav frame: {'x', 'y', 'normal', 'width'}.
        # Survives every state transition until the house is entered.
        self.entrance = None
        self.entry_retry_after = 0.0
        self.entry_attempts = 0
        self.entry_phase = None
        self.entry_phase_start = 0.0
        self.entry_waypoints = []
        self.entry_progress_best = None
        self.entry_progress_time = 0.0
        self.entry_align_start = None
        self.entry_sidesteps = 0
        self.entry_sidestep_start = None
        self.entry_sidestep_distance = 0.0
        self.entry_sidestep_yaw = 0.0
        self.entry_settle_scan_key = None
        self.entry_stop_a = 0.0
        self.entry_lateral_target = 0.0
        self.entry_cross_start = None
        self.entry_cross_safety_stops = 0
        self.entry_verify_scans = 0
        self.entry_inside_verified = False
        self.entry_backoff_start_pose = None
        self._nav_log_time = 0.0
        self._cross_log_time = 0.0
        self.scan_data = None
        self.scan_count = 0
        self.explore_step = 0
        self.explore_visited = []
        self.grid = None
        self.grid_last_eval = None
        self.grid_interior_seed = None
        self.grid_interior = set()
        self.grid_no_move_count = 0
        self.narrow_plan = []
        # Narrow obstacles the live checks acted on (nav frame), so planning
        # never proposes a move the live keep-out would refuse.
        self.narrow_memory = []
        # Positions (nav frame) of remembered narrow obstacles released on
        # contradicting evidence; never remembered again.
        self.narrow_released = []
        # Grid cells inside a narrow-obstacle keep-out (reachability only).
        self.grid_keepout_cells = set()
        self._grid_pose = None
        self.keepout_escapes = 0
        self.keepout_stats = {
            'scans_with_narrow': 0, 'narrow_detections': 0, 'headings_shortened': 0,
            'clusters_rejected': 0, 'live_shortened': 0, 'live_stops': 0,
            'escape_moves': 0, 'escape_failed': 0, 'structure_filtered': 0,
            'remembered': 0, 'contradictions': 0, 'released': 0, 'suppressed_released': 0,
        }
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.tilt_max = {'roll_deg': 0.0, 'pitch_deg': 0.0}
        self.tilt_latched = None
        self.explore_plan = None
        self.explore_move = None

        # Victim approach state (used only when victim_approach_enabled).
        self.victim_memory = VictimMemory(
            self.p.victim_confirm_radius_m, self.p.victim_confirm_count,
            self.p.victim_memory_timeout_sec)
        self.victim_target = None        # dict: track, missed, moves, checked
        self.victim_abandoned = []       # nav (x, y) of abandoned targets
        self.victim_report = None        # /victim_found payload once reached
        self.victim_hold = None          # facing / republish bookkeeping
        # Bearing cues: readings rejected only because the robot was turning.
        # Same agreement rule as victims (count / radius / timeout), but a cue
        # group can only trigger one turn-only look, never a confirmation.
        self.victim_cues = VictimMemory(
            self.p.victim_confirm_radius_m, self.p.victim_confirm_count,
            self.p.victim_memory_timeout_sec)
        self.victim_looked = []          # nav (x, y) of cue groups already looked at
        self.victim_stats = {'accepted': 0, 'rejected_invalid': 0,
                             'rejected_state': 0, 'rejected_rotating': 0,
                             'cues': 0, 'looks': 0}
        # Victim prioritizer (only with victim_approach_enabled as well).
        self.victim_prioritizing = bool(
            self.p.victim_approach_enabled and self.p.victim_prioritization_enabled)
        self.victim_priority = None      # last ranking of the current target

        self._handlers = {
            State.EXPLORING: self.handle_exploring,
            State.TURNING: self.handle_turning,
            State.FOLLOWING_BOUNDARY: self.handle_following_boundary,
            State.CROSSING_OPENING: self.handle_crossing_opening,
            State.RECOVERY: self.handle_recovery,
            State.ENTRANCE_APPROACH: self.handle_entrance_approach,
            State.ENTRANCE_ALIGN: self.handle_entrance_align,
            State.ENTRANCE_CROSS: self.handle_entrance_cross,
            State.VERIFY_INSIDE: self.handle_verify_inside,
            State.STOP_AND_SCAN: self.handle_stop_and_scan,
            State.EXPLORE_TURN: self.handle_explore_turn,
            State.EXPLORE_MOVE: self.handle_explore_move,
            State.INSIDE_HOLD: self.handle_inside_hold,
            State.VICTIM_HOLD: self.handle_victim_hold,
        }

        self.timer = self.create_timer(
            self.p.control_period_sec, self.control_loop
        )

        self.get_logger().info(
            'Disaster Bot autonomous search started '
            '(exploration intelligence v3: boundary/enclosure aware)'
        )

    # =====================================================================
    # CALLBACKS
    # =====================================================================

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.latest_scan_stamp = msg.header.stamp

    def odom_callback(self, msg):
        self.latest_odom_stamp = msg.header.stamp
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.latest_pose = Pose2D(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )
        self._update_nav_pose(self.latest_pose)

    def imu_callback(self, msg):
        q = msg.orientation
        self.imu_roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        self.imu_pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        roll_deg, pitch_deg = math.degrees(self.imu_roll), math.degrees(self.imu_pitch)
        if abs(roll_deg) > abs(self.tilt_max['roll_deg']):
            self.tilt_max['roll_deg'] = roll_deg
        if abs(pitch_deg) > abs(self.tilt_max['pitch_deg']):
            self.tilt_max['pitch_deg'] = pitch_deg
        if self.tilt_latched is None and tilt_exceeded(roll_deg, pitch_deg, self.p.tilt_stop_deg):
            # Immediate zero command from the IMU callback; control_loop then
            # latches the terminal FAULT_HOLD.
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.tilt_latched = {
                't': stamp, 'roll_deg': roll_deg, 'pitch_deg': pitch_deg,
                'state': self.state.name, 'nav': (
                    [self.nav_pose.x, self.nav_pose.y] if self.nav_pose is not None else None),
            }
            self.publish_velocity(0.0, 0.0)
            self.get_logger().error(
                f'[TILT_GUARD] roll={roll_deg:+.1f}deg pitch={pitch_deg:+.1f}deg reaches '
                f'{self.p.tilt_stop_deg:.1f}deg in {self.state.name} -- zero command, '
                f'exploration stopped (terminal FAULT_HOLD)'
            )
            self._scan_record('tilt_events.jsonl', self.tilt_latched)
        self.imu_yaw = quaternion_to_yaw(msg.orientation)
        self.latest_imu_stamp = msg.header.stamp
        self.imu_history.append((
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, self.imu_yaw
        ))

    def _update_nav_pose(self, odom_pose):
        """Advance the nav pose by the odometry forward step, applied along
        the IMU heading (odometry heading drifts; its distance does not)."""

        if self.imu_yaw is None:
            return

        if self.nav_pose is None or self._nav_prev_odom is None:
            self.nav_pose = Pose2D(0.0, 0.0, self.imu_yaw)
            self._nav_prev_odom = odom_pose
            return

        prev = self._nav_prev_odom
        mid_yaw = prev.yaw + 0.5 * normalize_angle(odom_pose.yaw - prev.yaw)
        step = (
            (odom_pose.x - prev.x) * math.cos(mid_yaw)
            + (odom_pose.y - prev.y) * math.sin(mid_yaw)
        )
        self._nav_prev_odom = odom_pose

        heading = self.imu_yaw
        self.nav_pose = Pose2D(
            self.nav_pose.x + step * math.cos(heading),
            self.nav_pose.y + step * math.sin(heading),
            heading,
        )

    # =====================================================================
    # TIME / FRESHNESS
    # =====================================================================

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _age_seconds(self, stamp, now_ns):
        msg_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        return max(0.0, (now_ns - msg_ns) / 1e9)

    def _is_scan_fresh(self, now_ns):
        if self.latest_scan is None or self.latest_scan_stamp is None:
            return False
        return self._age_seconds(self.latest_scan_stamp, now_ns) <= self.p.scan_stale_timeout_sec

    def _is_odom_fresh(self, now_ns):
        if self.latest_pose is None or self.latest_odom_stamp is None:
            return False
        return self._age_seconds(self.latest_odom_stamp, now_ns) <= self.p.odom_stale_timeout_sec

    # =====================================================================
    # SECTORS
    # =====================================================================

    def _sector_distance(self, scan, min_deg, max_deg):

        values = []

        for i, distance in enumerate(scan.ranges):

            if not math.isfinite(distance):
                continue

            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle_deg = math.degrees(
                normalize_angle(scan.angle_min + i * scan.angle_increment)
            )

            if angle_in_sector(angle_deg, min_deg, max_deg):
                values.append(distance)

        if not values:
            return float('inf')

        values.sort()

        # Ignore the closest outlier readings (LiDAR noise spikes).
        index = min(
            len(values) - 1,
            int(len(values) * self.p.sector_outlier_trim_ratio),
        )

        return values[index]

    def _compute_sectors(self):
        scan = self.latest_scan
        return {
            'front': self._sector_distance(scan, self.p.front_min_deg, self.p.front_max_deg),
            'front_left': self._sector_distance(scan, self.p.front_left_min_deg, self.p.front_left_max_deg),
            'front_right': self._sector_distance(scan, self.p.front_right_min_deg, self.p.front_right_max_deg),
            'left': self._sector_distance(scan, self.p.left_min_deg, self.p.left_max_deg),
            'right': self._sector_distance(scan, self.p.right_min_deg, self.p.right_max_deg),
            'rear': self._sector_distance(scan, self.p.rear_min_deg, self.p.rear_max_deg),
        }

    def _detect_opening_from_scan(self, center_deg):
        scan = self.latest_scan
        return detect_opening(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            center_deg,
            self.p.opening_search_half_width_deg,
            self.p.opening_edge_jump_m,
            self.p.opening_min_passable_width_m,
            self.p.opening_max_passable_width_m,
            self.p.opening_min_depth_m,
            self.p.opening_min_flank_extent_m,
        )

    # =====================================================================
    # ENCLOSURE / STRUCTURE-DENSITY MEASUREMENT (v3)
    # =====================================================================

    def _scan_enclosure_ratio(self, scan, center_deg, half_width_deg):
        """Fraction of beams in an angular window that return *any* finite
        value. 1.0 = fully enclosed by nearby structure in that window,
        0.0 = nothing within sensor range anywhere in that window."""

        min_deg = ((center_deg - half_width_deg + 180.0) % 360.0) - 180.0
        max_deg = ((center_deg + half_width_deg + 180.0) % 360.0) - 180.0

        total = 0
        finite = 0

        for i, distance in enumerate(scan.ranges):

            angle_deg = math.degrees(
                normalize_angle(scan.angle_min + i * scan.angle_increment)
            )

            if not angle_in_sector(angle_deg, min_deg, max_deg):
                continue

            total += 1

            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                finite += 1

        if total == 0:
            return 1.0

        return finite / total

    def _global_enclosure_ratio(self, scan):
        """Fraction of the FULL 360-degree scan that returns any finite
        value -- used only for the extreme near-total-void case."""

        total = len(scan.ranges)

        if total == 0:
            return 1.0

        finite = sum(
            1 for d in scan.ranges
            if math.isfinite(d) and scan.range_min <= d <= scan.range_max
        )

        return finite / total

    def _update_enclosure_tracking(self):
        """Scan-identity-based confirmation: only a NEW distinct /scan
        message can move the low-enclosure streak, so evaluating the same
        cached scan across several control ticks can never fake a
        confirmation, and a single noisy scan can't cause or cancel one."""

        scan = self.latest_scan
        stamp_key = (self.latest_scan_stamp.sec, self.latest_scan_stamp.nanosec)

        if stamp_key != self._enclosure_last_scan_key:
            self._enclosure_last_scan_key = stamp_key

            forward_ratio = self._scan_enclosure_ratio(
                scan, 0.0, self.p.enclosure_forward_half_width_deg
            )
            self._enclosure_last_forward_ratio = forward_ratio
            self._enclosure_last_global_ratio = self._global_enclosure_ratio(scan)

            if forward_ratio < self.p.enclosure_low_threshold:
                self.enclosure_low_streak += 1
            elif forward_ratio >= self.p.enclosure_high_threshold:
                self.enclosure_low_streak = 0
            # else: hysteresis dead-band -- hold the streak as-is.

        if self.enclosure_low_streak < self.p.enclosure_confirm_scans:
            return False, False

        search_mode = self._enclosure_last_global_ratio <= self.p.global_void_ratio_threshold

        # Debounce: require a fresh confirmation streak before acting again.
        self.enclosure_low_streak = 0

        return True, search_mode

    # =====================================================================
    # OUTPUT
    # =====================================================================

    def publish_velocity(self, linear_x, angular_z):
        self._last_cmd_linear = linear_x
        self._last_cmd_angular = angular_z

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = linear_x
        cmd.twist.angular.z = angular_z

        self.cmd_publisher.publish(cmd)

    def _slew_limit(self, target_angular):
        max_delta = self.p.angular_slew_rate * self.p.control_period_sec
        delta = clamp(target_angular - self.last_angular_cmd, -max_delta, max_delta)
        self.last_angular_cmd += delta
        return self.last_angular_cmd

    def _speed_ramp(self, front_distance):
        p = self.p

        if front_distance <= p.stop_distance:
            return 0.0

        if front_distance >= p.full_speed_distance:
            return p.forward_speed_max

        if front_distance <= p.slow_distance:
            span = max(1e-6, p.slow_distance - p.stop_distance)
            t = (front_distance - p.stop_distance) / span
            return p.min_creep_speed + t * (p.slow_speed - p.min_creep_speed)

        span = max(1e-6, p.full_speed_distance - p.slow_distance)
        t = (front_distance - p.slow_distance) / span
        return p.slow_speed + t * (p.forward_speed_max - p.slow_speed)

    # =====================================================================
    # PATH LENGTH TRACKING (for bounded boundary-following)
    # =====================================================================

    def _update_path_length(self):
        if self.prev_path_pose is not None:
            self.total_path_length += math.hypot(
                self.latest_pose.x - self.prev_path_pose.x,
                self.latest_pose.y - self.prev_path_pose.y,
            )
        self.prev_path_pose = self.latest_pose

    # =====================================================================
    # STATE TRANSITION BOOKKEEPING
    # =====================================================================

    def _enter_state(self, new_state, reason):
        now_sec = self._now_sec()
        self.get_logger().info(
            f'[STATE] t={now_sec:.2f} {self.state.name} -> {new_state.name} ({reason})'
        )
        self.state = new_state

        if self.latest_pose is not None:
            self.last_meaningful_pose = self.latest_pose
            self.last_meaningful_time = now_sec

        if new_state in (State.EXPLORING, State.FOLLOWING_BOUNDARY, State.STARTUP):
            self.opening_confirm_count = 0
            self.opening_last_bearing_deg = None

        if new_state == State.FOLLOWING_BOUNDARY:
            self.wall_lost_since = None
            self.follow_entry_time = now_sec
            self.follow_start_path_length = self.total_path_length
            self.last_wall_follow_angular = 0.0

        if new_state == State.FAULT_HOLD:
            self.last_angular_cmd = 0.0

    def _enter_fault_hold(self, reason, terminal):
        self.fault_reason = reason
        self.fault_terminal = terminal
        self.fault_fresh_since = None
        self._enter_state(State.FAULT_HOLD, reason)

    # =====================================================================
    # DIRECTION MEMORY / OSCILLATION DETECTION
    # =====================================================================

    def _prune_turn_history(self, now_sec):
        horizon = max(self.p.oscillation_window_sec, self.p.direction_history_window_sec)
        while self.turn_entry_history and (now_sec - self.turn_entry_history[0][0]) > horizon:
            self.turn_entry_history.popleft()

    def _check_oscillation(self, now_sec):
        count = 0
        for (t, x, y, _direction) in self.turn_entry_history:
            if now_sec - t > self.p.oscillation_window_sec:
                continue
            if math.hypot(self.latest_pose.x - x, self.latest_pose.y - y) <= self.p.oscillation_radius_m:
                count += 1
        return count >= self.p.oscillation_min_repeats

    def _check_alternation(self, now_sec):
        recent = [
            d for (t, _x, _y, d) in self.turn_entry_history
            if now_sec - t <= self.p.oscillation_window_sec
        ]
        if len(recent) < self.p.alternation_min_length:
            return False
        tail = recent[-self.p.alternation_min_length:]
        return all(tail[i] != tail[i - 1] for i in range(1, len(tail)))

    def _direction_recently_failed(self, direction, now_sec):
        for (t, x, y, d) in self.turn_entry_history:
            if d != direction:
                continue
            if now_sec - t > self.p.direction_history_window_sec:
                continue
            if math.hypot(self.latest_pose.x - x, self.latest_pose.y - y) <= self.p.direction_history_radius_m:
                return True
        return False

    def _direction_repeat_ratio(self, direction, now_sec):
        recent = [
            d for (t, _x, _y, d) in self.turn_entry_history
            if now_sec - t <= self.p.oscillation_window_sec
        ]
        if not recent:
            return 0.0
        matches = sum(1 for d in recent if d == direction)
        return matches / len(recent)

    # =====================================================================
    # EXPLORATION MEMORY (v2)
    # =====================================================================

    def _maybe_record_visited(self):
        if self.visited_memory:
            last_x, last_y = self.visited_memory[-1]
            if math.hypot(self.latest_pose.x - last_x, self.latest_pose.y - last_y) < self.p.memory_sample_min_dist_m:
                return
        self.visited_memory.append((self.latest_pose.x, self.latest_pose.y))

    def _visited_freshness(self, x, y):
        """1.0 = far from anywhere recently visited, 0.0 = right on top of it."""
        if not self.visited_memory:
            return 1.0
        nearest = min(math.hypot(x - vx, y - vy) for (vx, vy) in self.visited_memory)
        return clamp(nearest / self.p.visited_influence_radius_m, 0.0, 1.0)

    def _lateral_freshness(self, direction):
        angle = self.latest_pose.yaw + direction * math.radians(90.0)
        x = self.latest_pose.x + self.p.score_lookahead_m * math.cos(angle)
        y = self.latest_pose.y + self.p.score_lookahead_m * math.sin(angle)
        return self._visited_freshness(x, y)

    def _opening_recently_investigated(self, world_bearing, x, y):
        for entry in self.opening_memory:
            if math.hypot(x - entry['x'], y - entry['y']) > self.p.opening_memory_radius_m:
                continue
            bearing_diff = abs(math.degrees(normalize_angle(world_bearing - entry['world_bearing'])))
            if bearing_diff > self.p.opening_memory_bearing_tolerance_deg:
                continue
            if self.total_path_length - entry['path_length_at_detection'] < self.p.opening_memory_min_travel_m:
                return True
        return False

    # =====================================================================
    # DECISION SCORING (v2 + v3 enclosure term)
    # =====================================================================

    def _score_direction(self, sectors, direction, now_sec):
        """Deterministic weighted score for turning LEFT (+1) or RIGHT (-1).

        Combines clearance, opening quality, unexplored-space preference,
        nearby-structure density, and direction memory (recent failures /
        recent repetition) into a single explainable number -- see
        _log_decision for the breakdown. This is NOT "prefer the most open
        direction": raw distance is only one of several weighted terms, and
        enclosure_score explicitly rewards structure over emptiness.
        """

        if direction > 0:
            clearance = min(sectors['front_left'], sectors['left'])
            side_center_deg = 90.0
        else:
            clearance = min(sectors['front_right'], sectors['right'])
            side_center_deg = -90.0

        clearance_score = clamp(clearance / self.p.score_clearance_norm_m, 0.0, 1.0)

        opening_candidate = self._detect_opening_from_scan(side_center_deg)
        opening_score = 0.0
        if opening_candidate is not None:
            world_bearing = normalize_angle(self.latest_pose.yaw + opening_candidate['bearing_rad'])
            if not self._opening_recently_investigated(world_bearing, self.latest_pose.x, self.latest_pose.y):
                opening_score = 1.0

        enclosure_score = self._scan_enclosure_ratio(
            self.latest_scan, side_center_deg, self.p.enclosure_lateral_half_width_deg
        )

        freshness_score = self._lateral_freshness(direction)
        revisit_penalty = 1.0 - freshness_score

        failed_penalty = 1.0 if self._direction_recently_failed(direction, now_sec) else 0.0
        repeat_penalty = self._direction_repeat_ratio(direction, now_sec)

        total = (
            self.p.score_w_clearance * clearance_score
            + self.p.score_w_opening * opening_score
            + self.p.score_w_unexplored * freshness_score
            + self.p.score_w_enclosure * enclosure_score
            - self.p.score_w_failed * failed_penalty
            - self.p.score_w_repeat * repeat_penalty
        )

        info = {
            'clearance': clearance,
            'opening_score': opening_score,
            'enclosure_score': enclosure_score,
            'freshness_score': freshness_score,
            'revisit_penalty': revisit_penalty,
            'failed_penalty': failed_penalty,
            'repeat_penalty': repeat_penalty,
            'total': total,
        }

        return total, info

    def _log_decision(self, sectors, left_info, right_info, chosen, tag):
        chosen_label = 'LEFT' if chosen > 0 else 'RIGHT'

        if left_info['opening_score'] > 0 and right_info['opening_score'] > 0:
            opening_label = 'BOTH'
        elif left_info['opening_score'] > 0:
            opening_label = 'LEFT'
        elif right_info['opening_score'] > 0:
            opening_label = 'RIGHT'
        else:
            opening_label = 'NONE'

        self.get_logger().info(
            f'[{tag}] forward={sectors["front"]:.2f} '
            f'left={left_info["clearance"]:.2f}(score={left_info["total"]:.2f}) '
            f'right={right_info["clearance"]:.2f}(score={right_info["total"]:.2f}) '
            f'opening={opening_label} '
            f'enclosure=L{left_info["enclosure_score"]:.2f}/R{right_info["enclosure_score"]:.2f} '
            f'memory_penalty=L{left_info["revisit_penalty"]:.2f}/R{right_info["revisit_penalty"]:.2f} '
            f'selected={chosen_label}'
        )

    def _choose_turn_direction(self, sectors, now_sec):
        left_score, left_info = self._score_direction(sectors, 1, now_sec)
        right_score, right_info = self._score_direction(sectors, -1, now_sec)

        if abs(left_score - right_score) < self.p.score_tie_epsilon:
            chosen = -self.last_turn_direction if self.last_turn_direction != 0 else 1
        else:
            chosen = 1 if left_score > right_score else -1

        self._log_decision(sectors, left_info, right_info, chosen, 'DECISION')

        return chosen

    # =====================================================================
    # LOCAL STRUCTURE CLASSIFICATION (v2)
    # =====================================================================

    def _classify_structure(self, sectors, opening_candidate):
        """Cheap, memoryless local read of the immediate surroundings."""

        if opening_candidate is not None:
            return 'DOORWAY'

        left = sectors['left']
        right = sectors['right']

        if (left < self.p.corridor_side_max_m and right < self.p.corridor_side_max_m and
                abs(left - right) <= self.p.corridor_balance_tolerance_m):
            return 'CORRIDOR'

        return 'OPEN'

    def _maybe_enter_turning(self, sectors, now_sec, reason, search_mode=False):
        self._prune_turn_history(now_sec)

        oscillating = self._check_oscillation(now_sec)
        alternating = self._check_alternation(now_sec)

        if oscillating or alternating:
            self._enter_recovery('local_oscillation' if oscillating else 'direction_alternation')
            return

        direction = self._choose_turn_direction(sectors, now_sec)
        self._enter_turning(direction, reason, search_mode=search_mode)

    def _enter_turning(self, direction, reason, search_mode=False):
        now_sec = self._now_sec()

        self.turn_direction = direction
        self.last_turn_direction = direction
        self.turn_start_yaw = self.latest_pose.yaw
        self.turn_start_time = now_sec
        self.turn_last_yaw_progress_time = now_sec
        self.turn_last_yaw_delta = 0.0
        self.turn_search_mode = search_mode

        self.turn_entry_history.append(
            (now_sec, self.latest_pose.x, self.latest_pose.y, direction)
        )

        self._enter_state(State.TURNING, reason)

    # =====================================================================
    # STUCK DETECTION
    # =====================================================================

    def _update_stuck_tracking(self, now_sec):
        if self.state not in (State.EXPLORING, State.FOLLOWING_BOUNDARY, State.CROSSING_OPENING):
            self.last_meaningful_pose = self.latest_pose
            self.last_meaningful_time = now_sec
            return

        commanding_motion = (
            abs(self._last_cmd_linear) > self.p.stuck_cmd_linear_eps
            or abs(self._last_cmd_angular) > self.p.stuck_cmd_angular_eps
        )

        if not commanding_motion:
            self.last_meaningful_pose = self.latest_pose
            self.last_meaningful_time = now_sec
            return

        dx = self.latest_pose.x - self.last_meaningful_pose.x
        dy = self.latest_pose.y - self.last_meaningful_pose.y
        dyaw = abs(normalize_angle(self.latest_pose.yaw - self.last_meaningful_pose.yaw))

        if math.hypot(dx, dy) >= self.p.stuck_linear_eps_m or math.degrees(dyaw) >= self.p.stuck_angular_eps_deg:
            self.last_meaningful_pose = self.latest_pose
            self.last_meaningful_time = now_sec
            return

        if now_sec - self.last_meaningful_time >= self.p.stuck_time_threshold_sec:
            self._enter_recovery('stuck_no_displacement')

    # =====================================================================
    # OPENING CONFIRMATION (temporal debounce)
    # =====================================================================

    def _update_opening_confirmation(self, candidate):
        if candidate is None:
            self.opening_confirm_count = 0
            self.opening_last_bearing_deg = None
            return None

        bearing_deg = math.degrees(candidate['bearing_rad'])

        if (self.opening_last_bearing_deg is not None and
                abs(bearing_deg - self.opening_last_bearing_deg) <= self.p.opening_bearing_match_tolerance_deg):
            self.opening_confirm_count += 1
        else:
            self.opening_confirm_count = 1

        self.opening_last_bearing_deg = bearing_deg

        if self.opening_confirm_count >= self.p.opening_confirmation_scans:
            self.opening_confirm_count = 0
            self.opening_last_bearing_deg = None
            return candidate

        return None

    # =====================================================================
    # STATE: EXPLORING
    # =====================================================================

    def handle_exploring(self, sectors, now_sec):
        front = sectors['front']

        if front <= self.p.stop_distance:
            self._maybe_enter_turning(sectors, now_sec, 'exploring_obstacle')
            return

        boundary_event, search_mode = self._update_enclosure_tracking()
        if boundary_event:
            reason = 'exploration_void' if search_mode else 'exploration_boundary'
            self.get_logger().info(
                f'[BOUNDARY] forward_enclosure={self._enclosure_last_forward_ratio:.2f} '
                f'global_enclosure={self._enclosure_last_global_ratio:.2f} '
                f'search_mode={search_mode} -> {reason}'
            )
            self._maybe_enter_turning(sectors, now_sec, reason, search_mode=search_mode)
            return

        candidate = self._update_opening_confirmation(self._detect_opening_from_scan(0.0))

        structure = self._classify_structure(sectors, candidate)
        if structure != self.last_structure_label:
            self.get_logger().info(f'[STRUCTURE] {self.last_structure_label} -> {structure}')
            self.last_structure_label = structure

        if candidate is not None and self._generic_crossing_allowed():
            world_bearing = normalize_angle(self.latest_pose.yaw + candidate['bearing_rad'])
            if not self._opening_recently_investigated(world_bearing, self.latest_pose.x, self.latest_pose.y):
                self._begin_crossing(candidate, State.EXPLORING)
                return

        linear = self._speed_ramp(front)
        if structure == 'CORRIDOR':
            linear = min(linear, self.p.corridor_speed_cap)

        diff = sectors['left'] - sectors['right']
        steer = 0.0
        if abs(diff) > self.p.explore_steer_deadband:
            steer = clamp(diff * self.p.explore_steer_gain, -self.p.explore_max_steer, self.p.explore_max_steer)

        # Weak nudge toward less-visited territory. explore_memory_steer_gain
        # is small relative to explore_max_steer so obstacle-clearance
        # steering above always dominates this term.
        memory_term = (
            self._lateral_freshness(1) - self._lateral_freshness(-1)
        ) * self.p.explore_memory_steer_gain
        steer = clamp(steer + memory_term, -self.p.explore_max_steer, self.p.explore_max_steer)

        steer = self._slew_limit(steer)

        self.publish_velocity(linear, steer)

    # =====================================================================
    # STATE: TURNING
    # =====================================================================

    def handle_turning(self, sectors, now_sec):
        front = sectors['front']

        yaw_delta = abs(normalize_angle(self.latest_pose.yaw - self.turn_start_yaw))
        yaw_delta_deg = math.degrees(yaw_delta)

        if yaw_delta_deg - math.degrees(self.turn_last_yaw_delta) >= self.p.turn_progress_eps_deg:
            self.turn_last_yaw_progress_time = now_sec
            self.turn_last_yaw_delta = yaw_delta

        if now_sec - self.turn_last_yaw_progress_time >= self.p.turn_stall_timeout_sec:
            self._enter_recovery('turn_no_angular_progress')
            return

        if now_sec - self.turn_start_time >= self.p.turn_backstop_timeout_sec:
            self._enter_recovery('turn_backstop_timeout')
            return

        reached_min = yaw_delta_deg >= self.p.turn_min_deg
        reached_max = yaw_delta_deg >= self.p.turn_max_deg

        if self.turn_search_mode:
            # Searching for ANY nearby structure: "front is clear" is
            # meaningless here (everything is clear -- that's the problem),
            # so completion instead requires forward enclosure to recover.
            forward_ratio = self._scan_enclosure_ratio(
                self.latest_scan, 0.0, self.p.enclosure_forward_half_width_deg
            )
            structure_found = forward_ratio >= self.p.enclosure_high_threshold

            if (reached_min and structure_found) or reached_max:
                side = sectors['left'] if self.turn_direction > 0 else sectors['right']
                if self.p.wall_min <= side <= self.p.wall_max:
                    self.follow_direction = self.turn_direction
                    self._enter_state(State.FOLLOWING_BOUNDARY, 'search_found_wall')
                else:
                    self._enter_state(
                        State.EXPLORING,
                        'search_found_structure' if structure_found else 'search_exhausted',
                    )
                return

            self.publish_velocity(0.0, self.turn_direction * self.p.turn_speed)
            return

        if (reached_min and front >= self.p.clear_distance) or reached_max:
            side = sectors['left'] if self.turn_direction > 0 else sectors['right']
            if self.p.wall_min <= side <= self.p.wall_max:
                self.follow_direction = self.turn_direction
                self._enter_state(State.FOLLOWING_BOUNDARY, 'turn_complete_wall_found')
            else:
                self._enter_state(State.EXPLORING, 'turn_complete_open')
            return

        self.publish_velocity(0.0, self.turn_direction * self.p.turn_speed)

    # =====================================================================
    # STATE: FOLLOWING_BOUNDARY
    # =====================================================================

    def handle_following_boundary(self, sectors, now_sec):
        front = sectors['front']

        if front <= self.p.stop_distance:
            self._maybe_enter_turning(sectors, now_sec, 'boundary_obstacle')
            return

        side_center_deg = 90.0 if self.follow_direction > 0 else -90.0
        candidate = self._update_opening_confirmation(self._detect_opening_from_scan(side_center_deg))
        if candidate is not None and self._generic_crossing_allowed():
            world_bearing = normalize_angle(self.latest_pose.yaw + candidate['bearing_rad'])
            if not self._opening_recently_investigated(world_bearing, self.latest_pose.x, self.latest_pose.y):
                self._begin_crossing(candidate, State.FOLLOWING_BOUNDARY)
                return

        side = sectors['left'] if self.follow_direction > 0 else sectors['right']

        if side > self.p.wall_max:

            if self.wall_lost_since is None:
                self.wall_lost_since = now_sec

            if now_sec - self.wall_lost_since < self.p.wall_lost_grace_sec:
                # Wall may have just dipped out of range briefly (noise, a
                # narrow gap in the wall itself) -- coast on the last known
                # curvature instead of reacting immediately.
                angular = self._slew_limit(self.last_wall_follow_angular * 0.5)
                self.publish_velocity(self.p.wall_follow_speed, angular)
                self._check_follow_bounds(now_sec)
                return

            # Grace period elapsed with no confirmed opening: treat as the
            # wall curving away (a corner) and keep tracking it.
            angular = self._slew_limit(self.follow_direction * self.p.wall_corner_turn_rate)
            self.publish_velocity(self.p.wall_follow_speed, angular)
            self._check_follow_bounds(now_sec)
            return

        self.wall_lost_since = None

        error = side - self.p.wall_target
        raw_angular = -error * self.p.wall_kp if self.follow_direction > 0 else error * self.p.wall_kp
        raw_angular = clamp(raw_angular, -self.p.wall_max_angular, self.p.wall_max_angular)
        angular = self._slew_limit(raw_angular)
        self.last_wall_follow_angular = angular

        self.publish_velocity(self.p.wall_follow_speed, angular)
        self._check_follow_bounds(now_sec)

    def _check_follow_bounds(self, now_sec):
        elapsed = now_sec - self.follow_entry_time
        traveled = self.total_path_length - self.follow_start_path_length

        if elapsed >= self.p.max_follow_duration_sec or traveled >= self.p.max_follow_distance_m:
            self._enter_state(State.EXPLORING, 'boundary_follow_bounded_exit')

    # =====================================================================
    # STATE: CROSSING_OPENING
    # =====================================================================

    def _begin_crossing(self, candidate, fallback_state):
        world_bearing = normalize_angle(self.latest_pose.yaw + candidate['bearing_rad'])

        self.opening_memory.append({
            'x': self.latest_pose.x,
            'y': self.latest_pose.y,
            'world_bearing': world_bearing,
            'path_length_at_detection': self.total_path_length,
        })

        self.get_logger().info(
            f'[DECISION] opening confirmed bearing={math.degrees(candidate["bearing_rad"]):.1f}deg '
            f'width={candidate["width_m"]:.2f}m depth={candidate["depth_m"]:.2f}m -> CROSSING_OPENING'
        )

        self.crossing_fallback_state = fallback_state
        self.crossing_phase = 'ALIGN'
        self.crossing_target_yaw = world_bearing
        self.crossing_target_distance = max(
            self.p.opening_min_cross_distance_m,
            candidate['depth_m'] + self.p.opening_margin_m,
        )
        self.crossing_align_start_time = self._now_sec()
        self.crossing_align_best_error_deg = None
        self.crossing_align_best_error_time = self.crossing_align_start_time
        self._enter_state(State.CROSSING_OPENING, 'opening_confirmed')

    def _crossing_begin_drive(self, now_sec):
        self.crossing_phase = 'DRIVE'
        self.crossing_start_pose = self.latest_pose
        self.crossing_drive_start_time = now_sec

    def handle_crossing_opening(self, sectors, now_sec):

        if self.crossing_phase == 'ALIGN':
            error = normalize_angle(self.crossing_target_yaw - self.latest_pose.yaw)
            error_deg = abs(math.degrees(error))

            if error_deg <= self.p.opening_align_tolerance_deg:
                self._crossing_begin_drive(now_sec)
                return

            # Preserve the original full-timeout fallback as a backstop.
            if (
                now_sec - self.crossing_align_start_time
                >= self.p.opening_align_timeout_sec
            ):
                self._enter_state(
                    self.crossing_fallback_state,
                    'opening_align_timeout',
                )
                return

            # Track meaningful improvement in heading error.
            epsilon = self.p.crossing_align_progress_epsilon_deg
            best_error = self.crossing_align_best_error_deg

            if best_error is None:
                self.crossing_align_best_error_deg = error_deg
                self.crossing_align_best_error_time = now_sec
            elif error_deg < best_error - epsilon:
                self.crossing_align_best_error_deg = error_deg
                self.crossing_align_best_error_time = now_sec

            # A brief improvement must not indefinitely postpone abandonment.
            # Require sustained lack of progress, while retaining the original
            # full-timeout fallback above.
            if (
                now_sec - self.crossing_align_best_error_time
                >= self.p.crossing_align_stall_timeout_sec
            ):
                self._enter_state(
                    self.crossing_fallback_state,
                    'opening_align_stalled',
                )
                return

            turn_dir = 1.0 if error > 0 else -1.0
            self.publish_velocity(0.0, turn_dir * self.p.opening_align_turn_speed)
            return

        # DRIVE phase: committed straight-through crossing.
        front = sectors['front']

        if front <= self.p.stop_distance:
            self._enter_recovery('opening_crossing_blocked')
            return

        traveled = math.hypot(
            self.latest_pose.x - self.crossing_start_pose.x,
            self.latest_pose.y - self.crossing_start_pose.y,
        )

        if traveled >= self.crossing_target_distance:
            self._enter_state(State.EXPLORING, 'opening_crossing_complete')
            return

        if now_sec - self.crossing_drive_start_time >= self.p.opening_cross_backstop_sec:
            self._enter_recovery('opening_crossing_timeout')
            return

        self.publish_velocity(self.p.opening_cross_speed, 0.0)

    # =====================================================================
    # STATE: RECOVERY
    # =====================================================================

    def _enter_recovery(self, reason):
        self.recovery_stage = 0
        self._enter_state(State.RECOVERY, reason)
        self._recovery_begin_backup()

    def _recovery_begin_backup(self):
        now_sec = self._now_sec()
        self.recovery_substate = 'BACKUP'
        self.recovery_stage_start_pose = self.latest_pose
        self.recovery_substate_start_time = now_sec

        stages = self.p.recovery_backup_distance_stage
        idx = min(self.recovery_stage, len(stages) - 1)
        self.recovery_backup_target_distance = stages[idx]

    def _choose_recovery_direction(self, sectors, now_sec):
        left_score, left_info = self._score_direction(sectors, 1, now_sec)
        right_score, right_info = self._score_direction(sectors, -1, now_sec)

        chosen = 1 if left_score >= right_score else -1

        self._log_decision(sectors, left_info, right_info, chosen, 'RECOVERY_DECISION')

        return chosen

    def _recovery_begin_rotate(self, sectors):
        now_sec = self._now_sec()
        self.recovery_substate = 'ROTATE'
        self.recovery_substate_start_time = now_sec
        self.recovery_rotate_start_yaw = self.latest_pose.yaw

        direction = self._choose_recovery_direction(sectors, now_sec)
        self.recovery_rotate_direction = direction
        self.turn_entry_history.append(
            (now_sec, self.latest_pose.x, self.latest_pose.y, direction)
        )

        stages = self.p.recovery_rotate_deg_stage
        idx = min(self.recovery_stage, len(stages) - 1)
        self.recovery_rotate_target_deg = stages[idx]

    def _recovery_evaluate_stage(self, sectors, now_sec):
        dx = self.latest_pose.x - self.recovery_stage_start_pose.x
        dy = self.latest_pose.y - self.recovery_stage_start_pose.y
        dyaw = abs(normalize_angle(self.latest_pose.yaw - self.recovery_stage_start_pose.yaw))

        made_progress = (
            math.hypot(dx, dy) >= self.p.recovery_success_linear_eps_m
            or math.degrees(dyaw) >= self.p.recovery_success_angular_eps_deg
        )

        front_clear = sectors['front'] > self.p.stop_distance

        if made_progress and front_clear:
            self._enter_state(State.EXPLORING, 'recovery_succeeded')
            return

        self.recovery_stage += 1
        max_stages = max(
            len(self.p.recovery_backup_distance_stage),
            len(self.p.recovery_rotate_deg_stage),
        )

        if self.recovery_stage >= max_stages:
            self._enter_fault_hold('recovery_exhausted', terminal=True)
            return

        self.get_logger().warning(f'RECOVERY escalating to stage {self.recovery_stage}')
        self._recovery_begin_backup()

    def handle_recovery(self, sectors, now_sec):

        if self.recovery_substate == 'BACKUP':
            rear = sectors['rear']

            if rear <= self.p.recovery_rear_emergency_clearance:
                self._recovery_begin_rotate(sectors)
                return

            if rear <= self.p.recovery_rear_min_clearance:
                self._recovery_begin_rotate(sectors)
                return

            elapsed = now_sec - self.recovery_substate_start_time
            traveled = math.hypot(
                self.latest_pose.x - self.recovery_stage_start_pose.x,
                self.latest_pose.y - self.recovery_stage_start_pose.y,
            )

            if traveled >= self.recovery_backup_target_distance or elapsed >= self.p.recovery_backup_backstop_sec:
                self._recovery_begin_rotate(sectors)
                return

            self.publish_velocity(-self.p.recovery_backup_speed, 0.0)
            return

        if self.recovery_substate == 'ROTATE':
            yaw_delta = abs(normalize_angle(self.latest_pose.yaw - self.recovery_rotate_start_yaw))
            elapsed = now_sec - self.recovery_substate_start_time

            if (math.degrees(yaw_delta) >= self.recovery_rotate_target_deg or
                    elapsed >= self.p.recovery_rotate_backstop_sec):
                self._recovery_evaluate_stage(sectors, now_sec)
                return

            self.publish_velocity(0.0, self.recovery_rotate_direction * self.p.recovery_turn_speed)
            return

    # =====================================================================
    # HOUSE ENTRANCE ENTRY -- helpers
    # =====================================================================
    #
    # OUTSIDE_SEARCH (mission phase: the exploration states above run, the
    # 360-degree entrance detector watches every new scan) -> ENTRANCE_APPROACH
    # (standoff point on the doorway normal) -> ENTRANCE_ALIGN (face the
    # doorway, centre in the free passage, verify safety-compatible path)
    # -> ENTRANCE_CROSS (straight, progress-checked, stops just past the
    # doorway) -> VERIFY_INSIDE -> INSIDE_HOLD.
    #
    # All entry states use the IMU heading. Position is the nav pose (odom
    # forward steps along the IMU heading). The entrance is persisted in that
    # frame once confirmed, so it survives every state transition.

    def _entry_log(self, msg, level='info'):
        text = f't={self._now_sec():.2f} {msg}'
        if level == 'warn':
            self.get_logger().warning(text)
        elif level == 'error':
            self.get_logger().error(text)
        else:
            self.get_logger().info(text)

    def _set_mission_phase(self, phase, reason):
        if phase != self.mission_phase:
            self._entry_log(f'[MISSION] {self.mission_phase} -> {phase} ({reason})')
            self.mission_phase = phase

    def _generic_crossing_allowed(self):
        # While the house entrance is being searched for or entered, doorways
        # are handled only by the entry states, never by CROSSING_OPENING.
        return self.mission_phase not in ('OUTSIDE_SEARCH', 'ENTERING')

    def _is_imu_fresh(self, now_ns):
        if self.imu_yaw is None or self.latest_imu_stamp is None:
            return False
        return self._age_seconds(self.latest_imu_stamp, now_ns) <= self.p.imu_stale_timeout_sec

    def _entry_pose(self):
        return Pose2D(self.nav_pose.x, self.nav_pose.y, self.imu_yaw)

    def _scan_key(self):
        return (self.latest_scan_stamp.sec, self.latest_scan_stamp.nanosec)

    def _scan_points_xy(self):
        scan = self.latest_scan
        return [
            (p[3], p[4]) for p in scan_to_points(
                scan.ranges, scan.angle_min, scan.angle_increment,
                scan.range_min, scan.range_max, self.p.lidar_offset_x_m,
            )
        ]

    def _safety_cone_min(self, center_deg=0.0):
        scan = self.latest_scan
        return cone_min_range(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max,
            center_deg, self.p.entry_safety_half_angle_deg,
        )

    def _detect_entrances_now(self):
        scan = self.latest_scan
        return detect_entrances(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max,
            x_offset=self.p.lidar_offset_x_m,
            min_width_m=self.p.entry_min_width_m,
            max_width_m=self.p.entry_max_width_m,
            max_range_m=self.p.entry_max_detect_range_m,
        )

    @staticmethod
    def _to_world(pose, x, y):
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        return pose.x + c * x - s * y, pose.y + s * x + c * y

    @staticmethod
    def _to_robot(pose, wx, wy):
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        dx, dy = wx - pose.x, wy - pose.y
        return c * dx + s * dy, -s * dx + c * dy

    def _door_frame(self, pose):
        """(a, lat): a along the doorway normal (negative = outside, 0 = the
        detected outer wall face), lat lateral (positive = left when facing
        into the house)."""

        e = self.entrance
        nx, ny = math.cos(e['normal']), math.sin(e['normal'])
        dx, dy = pose.x - e['x'], pose.y - e['y']
        return dx * nx + dy * ny, -dx * ny + dy * nx

    def _door_point(self, a, lat):
        e = self.entrance
        nx, ny = math.cos(e['normal']), math.sin(e['normal'])
        return e['x'] + a * nx - lat * ny, e['y'] + a * ny + lat * nx

    def _entry_rotate_to(self, target_yaw):
        error = normalize_angle(target_yaw - self.imu_yaw)
        limit = self.p.entry_max_angular
        angular = clamp(self.p.entry_heading_kp * error, -limit, limit)
        if abs(angular) < self.p.entry_min_angular:
            angular = math.copysign(self.p.entry_min_angular, error)
        self.publish_velocity(0.0, angular)
        return error

    def _entry_progress_reset(self, now_sec, value):
        self.entry_progress_best = value
        self.entry_progress_time = now_sec

    def _entry_progress_stalled(self, now_sec, value):
        """value must keep increasing by entry_progress_min_m per window."""
        if value >= self.entry_progress_best + self.p.entry_progress_min_m:
            self.entry_progress_best = value
            self.entry_progress_time = now_sec
            return False
        return now_sec - self.entry_progress_time >= self.p.entry_progress_window_sec

    def _entry_set_phase(self, phase, now_sec):
        self.entry_phase = phase
        self.entry_phase_start = now_sec

    def _entry_match(self, wx, wy, normal):
        e = self.entrance
        return (
            math.hypot(wx - e['x'], wy - e['y']) <= self.p.entry_match_radius_m
            and abs(math.degrees(normalize_angle(normal - e['normal'])))
            <= self.p.entry_match_angle_deg
        )

    def _entry_refine(self, pose, now_sec, tag):
        """Blend a fresh unoccluded detection of the SAME entrance into the
        persisted estimate (once per scan)."""

        key = self._scan_key()
        if key == self._entry_last_scan_key:
            return
        self._entry_last_scan_key = key

        for c in self._detect_entrances_now():
            if c['occluded']:
                continue
            wx, wy = self._to_world(pose, *c['center'])
            wn = normalize_angle(pose.yaw + c['normal'])
            if not self._entry_match(wx, wy, wn):
                continue
            e = self.entrance
            k = self.p.entry_refine_alpha
            e['x'] += k * (wx - e['x'])
            e['y'] += k * (wy - e['y'])
            e['normal'] = normalize_angle(e['normal'] + k * normalize_angle(wn - e['normal']))
            e['width'] += k * (c['width'] - e['width'])
            e['width_min'] = min(e.get('width_min', c['width']), c['width'])
            self._entry_log(
                f'[ENTRY_REFINE] {tag} seen=({wx:.2f},{wy:.2f}) normal={math.degrees(wn):+.1f}deg '
                f'width={c["width"]:.2f} -> entrance=({e["x"]:.2f},{e["y"]:.2f}) '
                f'normal={math.degrees(e["normal"]):+.1f}deg width={e["width"]:.2f} '
                f'width_min={e["width_min"]:.2f}'
            )
            return

    def _log_nav(self, now_sec):
        if now_sec - self._nav_log_time < 1.0:
            return
        self._nav_log_time = now_sec
        if self.nav_pose is None or self.imu_yaw is None or self.latest_pose is None:
            return
        door = ''
        if self.entrance is not None:
            a, lat = self._door_frame(self._entry_pose())
            door = f' door_a={a:+.2f} door_l={lat:+.2f}'
        self.get_logger().info(
            f'[NAV] t={now_sec:.2f} state={self.state.name} phase={self.mission_phase} '
            f'nav=({self.nav_pose.x:.2f},{self.nav_pose.y:.2f}) '
            f'imu_yaw={math.degrees(self.imu_yaw):+.1f} '
            f'odom=({self.latest_pose.x:.2f},{self.latest_pose.y:.2f},'
            f'{math.degrees(self.latest_pose.yaw):+.1f})'
            f'{door}'
        )

    # =====================================================================
    # OUTSIDE_SEARCH: entrance detection / confirmation
    # =====================================================================

    def _entry_outside_search_tick(self, now_sec):
        """Runs before the exploration handler while OUTSIDE_SEARCH. Returns
        True when the entry states have taken over."""

        if self.entrance is not None:
            if now_sec >= self.entry_retry_after:
                self._entry_start_approach(now_sec, 'persisted_entrance_retry')
                return True
            return False

        key = self._scan_key()
        if key == self._entry_last_scan_key:
            return False
        self._entry_last_scan_key = key

        pose = self._entry_pose()
        self.entry_tracks = [
            t for t in self.entry_tracks
            if now_sec - t['last'] <= self.p.entry_confirm_window_sec
        ]

        for c in self._detect_entrances_now():

            wx, wy = self._to_world(pose, *c['center'])
            wn = normalize_angle(pose.yaw + c['normal'])
            bearing = math.degrees(math.atan2(c['center'][1], c['center'][0]))

            self._entry_log(
                f'[ENTRY_DETECT] bearing={bearing:+.1f}deg range={math.hypot(*c["center"]):.2f}m '
                f'width={c["width"]:.2f}m normal={math.degrees(wn):+.1f}deg '
                f'through={c["through"]} blocked={c["blocked"]} occluded={c["occluded"]} '
                f'world=({wx:.2f},{wy:.2f})'
            )

            if c['occluded']:
                continue

            track = None
            for t in self.entry_tracks:
                angle_diff = abs(math.degrees(normalize_angle(wn - t['normal'])))
                if (math.hypot(wx - t['x'], wy - t['y']) <= self.p.entry_match_radius_m and
                        angle_diff <= self.p.entry_match_angle_deg):
                    track = t
                    break

            if track is None:
                self.entry_tracks.append({
                    'x': wx, 'y': wy, 'normal': wn, 'width': c['width'],
                    'width_min': c['width'], 'hits': 1, 'last': now_sec,
                })
                continue

            k = self.p.entry_refine_alpha
            track['x'] += k * (wx - track['x'])
            track['y'] += k * (wy - track['y'])
            track['normal'] = normalize_angle(
                track['normal'] + k * normalize_angle(wn - track['normal'])
            )
            track['width'] += k * (c['width'] - track['width'])
            track['width_min'] = min(track['width_min'], c['width'])
            track['hits'] += 1
            track['last'] = now_sec

            if track['hits'] >= self.p.entry_confirm_hits:
                self.entrance = {
                    'x': track['x'], 'y': track['y'],
                    'normal': track['normal'], 'width': track['width'],
                    'width_min': track['width_min'],
                }
                self.entry_tracks = []
                self.entry_attempts = 0
                self._entry_log(
                    f'[ENTRY_SELECT] entrance confirmed after {track["hits"]} scans: '
                    f'centre=({track["x"]:.2f},{track["y"]:.2f}) '
                    f'normal={math.degrees(track["normal"]):+.1f}deg '
                    f'width={track["width"]:.2f}m width_min={track["width_min"]:.2f}m '
                    f'bearing={bearing:+.1f}deg '
                    f'robot_nav=({pose.x:.2f},{pose.y:.2f}) '
                    f'imu_yaw={math.degrees(pose.yaw):+.1f}deg (from {self.state.name})'
                )
                self._entry_start_approach(now_sec, 'entrance_confirmed')
                return True

        return False

    # =====================================================================
    # STATE: ENTRANCE_APPROACH
    # =====================================================================

    def _entry_start_approach(self, now_sec, reason):
        self.publish_velocity(0.0, 0.0)
        self.last_angular_cmd = 0.0
        self._set_mission_phase('ENTERING', reason)

        pose = self._entry_pose()
        a, lat = self._door_frame(pose)
        standoff = self.p.entry_standoff_m

        # Beside the wall and not in front of the doorway: first step
        # straight away from the wall to standoff depth, then travel parallel
        # to it, so the path cannot clip the wall or objects standing close
        # in front of it (e.g. the mailbox beside this entrance).
        self.entry_waypoints = []
        if a > -(standoff - 0.2) and abs(lat) > 0.3:
            self.entry_waypoints.append(('away', self._door_point(-standoff, lat)))
        self.entry_waypoints.append(('standoff', self._door_point(-standoff, 0.0)))

        self._entry_log(
            f'[ENTRY_APPROACH] start ({reason}) door_frame a={a:+.2f} lat={lat:+.2f} '
            f'waypoints=' + ' '.join(
                f'{k}({p[0]:.2f},{p[1]:.2f})' for k, p in self.entry_waypoints
            )
        )

        self._entry_set_phase('TURN', now_sec)
        self._enter_state(State.ENTRANCE_APPROACH, reason)

    def _entry_abort(self, now_sec, reason):
        self.publish_velocity(0.0, 0.0)
        self._entry_log(f'[ENTRY] abort: {reason} -- back to OUTSIDE_SEARCH, retry in '
                        f'{self.p.entry_retry_cooldown_sec:.0f}s', level='error')
        self.entry_retry_after = now_sec + self.p.entry_retry_cooldown_sec
        self.entry_attempts = 0
        self.entry_align_start = None
        self._set_mission_phase('OUTSIDE_SEARCH', reason)
        self._enter_state(State.EXPLORING, f'entry_abort_{reason}')

    def _entry_approach_blocked(self, now_sec, reason):
        self.publish_velocity(0.0, 0.0)
        self.entry_attempts += 1
        self._entry_log(
            f'[ENTRY_APPROACH] blocked ({reason}), attempt {self.entry_attempts}', level='warn'
        )

        if self.entry_attempts > self.p.entry_max_attempts:
            self._entry_abort(now_sec, 'approach_blocked')
            return

        # Back off a little, then detour further away from the wall.
        a, lat = self._door_frame(self._entry_pose())
        self.entry_waypoints = [
            ('detour', self._door_point(min(a, -self.p.entry_standoff_m) - 0.4, lat)),
            ('standoff', self._door_point(-self.p.entry_standoff_m, 0.0)),
        ]
        self.entry_backoff_start_pose = self._entry_pose()
        self._entry_set_phase('BACKOFF', now_sec)

    def handle_entrance_approach(self, sectors, now_sec):
        pose = self._entry_pose()
        self._entry_refine(pose, now_sec, 'approach')

        # The standoff point follows the refined entrance estimate.
        if self.entry_waypoints and self.entry_waypoints[-1][0] == 'standoff':
            self.entry_waypoints[-1] = (
                'standoff', self._door_point(-self.p.entry_standoff_m, 0.0)
            )

        if self.entry_phase == 'BACKOFF':
            start = self.entry_backoff_start_pose
            moved = math.hypot(pose.x - start.x, pose.y - start.y)
            rear = self._safety_cone_min(180.0)
            if (moved >= 0.15 or rear <= self.p.entry_safety_emergency_m + 0.05 or
                    now_sec - self.entry_phase_start >= 4.0):
                self.publish_velocity(0.0, 0.0)
                self._entry_set_phase('TURN', now_sec)
                return
            self.publish_velocity(-self.p.entry_backout_speed, 0.0)
            return

        kind, (gx, gy) = self.entry_waypoints[0]
        dx, dy = gx - pose.x, gy - pose.y
        dist = math.hypot(dx, dy)

        if dist <= self.p.entry_goal_tolerance_m:
            self.publish_velocity(0.0, 0.0)
            self.entry_waypoints.pop(0)
            a, lat = self._door_frame(pose)
            self._entry_log(
                f'[ENTRY_APPROACH] reached {kind} nav=({pose.x:.2f},{pose.y:.2f}) '
                f'a={a:+.2f} lat={lat:+.2f}'
            )
            if not self.entry_waypoints:
                self._entry_begin_align(now_sec, 'standoff_reached')
                return
            self._entry_set_phase('TURN', now_sec)
            return

        bearing = math.atan2(dy, dx)
        error = normalize_angle(bearing - pose.yaw)

        if self.entry_phase == 'TURN':
            if abs(math.degrees(error)) <= 8.0:
                self.publish_velocity(0.0, 0.0)
                self._entry_set_phase('DRIVE', now_sec)
                self._entry_progress_reset(now_sec, -dist)
                return
            self._entry_rotate_to(bearing)
            return

        # DRIVE
        if abs(math.degrees(error)) > self.p.entry_drive_heading_gate_deg:
            self._entry_set_phase('TURN', now_sec)
            self.publish_velocity(0.0, 0.0)
            return

        cone = self._safety_cone_min()
        if cone <= self.p.entry_block_cone_m:
            self._entry_approach_blocked(now_sec, f'front cone {cone:.2f}m')
            return

        if self._entry_progress_stalled(now_sec, -dist):
            self._entry_approach_blocked(now_sec, 'no_progress')
            return

        linear = min(self.p.entry_approach_speed, 0.5 * dist + 0.03)
        limit = self.p.entry_max_angular
        angular = clamp(self.p.entry_heading_kp * error, -limit, limit)
        self.publish_velocity(linear, angular)

    # =====================================================================
    # STATE: ENTRANCE_ALIGN
    # =====================================================================

    def _entry_begin_align(self, now_sec, reason, phase='FACE'):
        self.entry_sidesteps = 0
        if self.entry_align_start is None:
            self.entry_align_start = now_sec
        self._entry_set_phase(phase, now_sec)
        self._enter_state(State.ENTRANCE_ALIGN, reason)

    def handle_entrance_align(self, sectors, now_sec):
        pose = self._entry_pose()
        normal = self.entrance['normal']

        if now_sec - self.entry_align_start >= self.p.entry_align_timeout_sec:
            self._entry_abort(now_sec, 'align_timeout')
            return

        if self.entry_phase == 'BACKOUT':
            a, _ = self._door_frame(pose)
            rear = self._safety_cone_min(180.0)
            if (a <= -self.p.entry_backout_to_m or
                    rear <= self.p.entry_safety_emergency_m + 0.05 or
                    now_sec - self.entry_phase_start >= 15.0):
                self.publish_velocity(0.0, 0.0)
                self._entry_log(f'[ENTRY_ALIGN] backout done a={a:+.2f} rear={rear:.2f}')
                self._entry_set_phase('FACE', now_sec)
                return
            error = normalize_angle(normal - pose.yaw)
            angular = clamp(self.p.entry_heading_kp * error, -0.3, 0.3)
            self.publish_velocity(-self.p.entry_backout_speed, angular)
            return

        if self.entry_phase == 'FACE':
            error = normalize_angle(normal - pose.yaw)
            if abs(math.degrees(error)) <= self.p.entry_turn_tolerance_deg:
                self.publish_velocity(0.0, 0.0)
                self._entry_set_phase('SETTLE', now_sec)
                self.entry_settle_scan_key = self._scan_key()
                return
            self._entry_rotate_to(normal)
            return

        if self.entry_phase == 'SETTLE':
            self.publish_velocity(0.0, 0.0)
            if (now_sec - self.entry_phase_start >= self.p.entry_settle_sec and
                    self._scan_key() != self.entry_settle_scan_key):
                self._entry_measure_and_decide(pose, now_sec)
            return

        if self.entry_phase == 'SS_TURN':
            error = normalize_angle(self.entry_sidestep_yaw - pose.yaw)
            if abs(math.degrees(error)) <= self.p.entry_turn_tolerance_deg:
                self.publish_velocity(0.0, 0.0)
                self.entry_sidestep_start = pose
                self._entry_set_phase('SS_DRIVE', now_sec)
                return
            self._entry_rotate_to(self.entry_sidestep_yaw)
            return

        if self.entry_phase == 'SS_DRIVE':
            start = self.entry_sidestep_start
            moved = math.hypot(pose.x - start.x, pose.y - start.y)
            cone = self._safety_cone_min()
            cone_limit = self.p.entry_safety_emergency_m + self.p.entry_safety_margin_m + 0.05
            if (moved >= self.entry_sidestep_distance - 0.01 or cone <= cone_limit or
                    now_sec - self.entry_phase_start >= 12.0):
                self.publish_velocity(0.0, 0.0)
                self._entry_log(
                    f'[ENTRY_ALIGN] sidestep done moved={moved:.3f}m of '
                    f'{self.entry_sidestep_distance:.3f}m cone={cone:.2f}'
                )
                self._entry_set_phase('FACE', now_sec)
                return
            error = normalize_angle(self.entry_sidestep_yaw - pose.yaw)
            angular = clamp(self.p.entry_heading_kp * error, -0.3, 0.3)
            self.publish_velocity(self.p.entry_sidestep_speed, angular)
            return

    def _entry_measure_and_decide(self, pose, now_sec):
        """Facing the doorway, stopped, fresh scan: centre in the free
        passage, then verify the straight path is safety-compatible."""

        self._entry_refine(pose, now_sec, 'align')

        e = self.entrance
        a, lat = self._door_frame(pose)
        heading_err = math.degrees(normalize_angle(e['normal'] - pose.yaw))
        cx_r, cy_r = self._to_robot(pose, e['x'], e['y'])
        face_distance = -a
        points = self._scan_points_xy()

        left, right = corridor_bounds(
            points, 0.05, face_distance + 0.20, cy_r, self.p.entry_corridor_half_window_m
        )
        half = 0.5 * e['width']
        left_b = left if left is not None else cy_r + half
        right_b = right if right is not None else cy_r - half
        corridor_center = 0.5 * (left_b + right_b)
        corridor_width = left_b - right_b

        # Aim for the doorway midpoint (between the jambs). Shift only as far
        # as needed to keep entry_center_margin_m from anything beside the
        # path (e.g. an object standing in front of one side of the door).
        margin = self.p.entry_center_margin_m
        low, high = right_b + margin, left_b - margin
        target = clamp(cy_r, low, high) if low <= high else corridor_center

        self._entry_log(
            f'[ENTRY_ALIGN] measure: a={a:+.2f} lateral_from_door_centre={lat:+.3f}m '
            f'heading_err={heading_err:+.2f}deg '
            f'door_bearing={math.degrees(math.atan2(cy_r, cx_r)):+.1f}deg '
            f'door_width={e["width"]:.2f}m '
            f'corridor L={left_b:+.3f}{"" if left is not None else "(nominal)"} '
            f'R={right_b:+.3f}{"" if right is not None else "(nominal)"} '
            f'width={corridor_width:.2f}m centre={corridor_center:+.3f} '
            f'(door centre {cy_r:+.3f}, offset {corridor_center - cy_r:+.3f}) '
            f'target={target:+.3f} (offset from door centre {target - cy_r:+.3f}) '
            f'lateral_error={target:+.3f}m sidesteps={self.entry_sidesteps}'
        )

        if low > high or corridor_width < 2.0 * self.p.entry_min_lateral_clearance_m:
            self._entry_abort(now_sec, f'corridor_too_narrow_{corridor_width:.2f}')
            return

        if (abs(target) > self.p.entry_center_tolerance_m and
                self.entry_sidesteps < self.p.entry_max_sidesteps):
            self.entry_sidesteps += 1
            self.entry_sidestep_distance = abs(target)
            self.entry_sidestep_yaw = normalize_angle(
                e['normal'] + math.copysign(math.pi / 2.0, target)
            )
            self._entry_log(f'[ENTRY_ALIGN] sidestep {self.entry_sidesteps}: {target:+.3f}m')
            self._entry_set_phase('SS_TURN', now_sec)
            return

        # Stop target: just past the doorway, and short of whatever lies
        # straight ahead inside (never the far wall itself).
        ahead = [x for x, y in points if abs(y) < 0.20 and x > face_distance + 0.25]
        stop_a = self.p.entry_inside_target_m
        if ahead:
            stop_a = min(stop_a, a + min(ahead) - (self.p.entry_front_stop_m + 0.05))
        stop_a = max(stop_a, self.p.entry_inside_min_m)

        blocker = straight_path_blocker(
            points, stop_a - a, self.p.entry_min_lateral_clearance_m, 0.0
        )
        if blocker is not None:
            self.entry_attempts += 1
            self._entry_log(
                f'[ENTRY_ALIGN] path not safety-compatible: obstacle at '
                f'x={blocker[0]:.2f} y={blocker[1]:+.2f} '
                f'(needs |y| >= {self.p.entry_min_lateral_clearance_m:.2f}), '
                f'attempt {self.entry_attempts}',
                level='warn',
            )
            if self.entry_attempts > self.p.entry_max_attempts:
                self._entry_abort(now_sec, 'path_blocked')
                return
            self._entry_set_phase('FACE', now_sec)
            return

        self.entry_stop_a = stop_a
        # Door-frame lateral line the crossing holds (robot faces the normal,
        # so robot +y is door-frame +lateral).
        self.entry_lateral_target = lat + target
        self._entry_log(
            f'[ENTRY_ALIGN] aligned: heading_err={heading_err:+.2f}deg '
            f'lateral_error={target:+.3f}m '
            f'clearance L={left_b - target:+.3f} R={right_b - target:+.3f} '
            f'path clear to a={stop_a:+.2f} '
            f'align_time={now_sec - self.entry_align_start:.1f}s'
        )
        self._entry_begin_cross(now_sec, a)

    # =====================================================================
    # STATE: ENTRANCE_CROSS
    # =====================================================================

    def _entry_begin_cross(self, now_sec, a):
        self.entry_cross_start = now_sec
        self._entry_progress_reset(now_sec, a)
        self.last_angular_cmd = 0.0
        self._enter_state(State.ENTRANCE_CROSS, 'aligned_and_clear')

    def _entry_backout(self, now_sec, reason):
        self.publish_velocity(0.0, 0.0)
        self.entry_attempts += 1
        self._entry_log(
            f'[ENTRY_CROSS] stop: {reason} -> back out and realign '
            f'(attempt {self.entry_attempts})',
            level='warn',
        )
        if self.entry_attempts > self.p.entry_max_attempts:
            self._entry_abort(now_sec, f'cross_{reason}')
            return
        self._entry_begin_align(now_sec, f'cross_{reason}', phase='BACKOUT')

    def _entry_begin_verify(self, now_sec, reason):
        self.publish_velocity(0.0, 0.0)
        pose = self._entry_pose()
        a, lat = self._door_frame(pose)
        heading_err = math.degrees(normalize_angle(self.entrance['normal'] - pose.yaw))
        self._entry_log(
            f'[ENTRY_CROSS] done ({reason}): a={a:+.2f} lat={lat:+.3f} '
            f'heading_err={heading_err:+.1f}deg '
            f'crossing_time={now_sec - self.entry_cross_start:.1f}s'
        )
        self.entry_verify_scans = 0
        self.entry_settle_scan_key = self._scan_key()
        self._entry_set_phase('SETTLE', now_sec)
        self._enter_state(State.VERIFY_INSIDE, reason)

    def handle_entrance_cross(self, sectors, now_sec):
        pose = self._entry_pose()
        normal = self.entrance['normal']
        a, lat = self._door_frame(pose)
        cone = self._safety_cone_min()

        # Never command forward motion with anything in the safety
        # controller's emergency region (plus margin).
        if cone <= self.p.entry_safety_emergency_m + self.p.entry_safety_margin_m:
            self.publish_velocity(0.0, 0.0)
            if a >= self.p.entry_inside_min_m:
                self._entry_begin_verify(now_sec, f'safety_cone_{cone:.2f}_inside')
            else:
                self.entry_cross_safety_stops += 1
                self._entry_backout(now_sec, f'safety_cone_{cone:.2f}')
            return

        if a >= self.entry_stop_a:
            self._entry_begin_verify(now_sec, 'target_depth')
            return

        if a >= self.p.entry_inside_min_m and cone <= self.p.entry_front_stop_m:
            self._entry_begin_verify(now_sec, f'front_{cone:.2f}')
            return

        if now_sec - self.entry_cross_start >= self.p.entry_cross_timeout_sec:
            self._entry_backout(now_sec, 'timeout')
            return

        if self._entry_progress_stalled(now_sec, a):
            self._entry_backout(now_sec, 'no_progress')
            return

        heading_dev = math.degrees(normalize_angle(normal - pose.yaw))
        if abs(heading_dev) > self.p.entry_cross_max_heading_err_deg:
            self._entry_backout(now_sec, f'heading_{heading_dev:+.1f}')
            return

        # Hold the aligned line through the doorway; steer away only if
        # something beside the robot comes closer than the safe lateral gap.
        left, right = corridor_bounds(self._scan_points_xy(), -0.25, 0.45, 0.0, 0.6)
        max_bias = math.radians(self.p.entry_cross_max_bias_deg)
        min_gap = self.p.entry_min_lateral_clearance_m
        if left is not None and left < min_gap:
            bias = -max_bias
        elif right is not None and right > -min_gap:
            bias = max_bias
        else:
            lateral = self.entry_lateral_target - lat
            bias = clamp(self.p.entry_cross_lateral_gain * lateral, -max_bias, max_bias)
        error = normalize_angle(normal + bias - pose.yaw)
        angular = clamp(self.p.entry_heading_kp * error, -0.3, 0.3)

        self.publish_velocity(self.p.entry_cross_speed, angular)

        if now_sec - self._cross_log_time >= 0.5:
            self._cross_log_time = now_sec
            self._entry_log(
                f'[ENTRY_CROSS] a={a:+.2f} lat={lat:+.3f} '
                f'heading_err={heading_dev:+.1f}deg cone={cone:.2f} '
                f'side L={"-" if left is None else f"{left:+.2f}"} '
                f'R={"-" if right is None else f"{right:+.2f}"} '
                f'bias={math.degrees(bias):+.1f}deg'
            )

    # =====================================================================
    # STATE: VERIFY_INSIDE / INSIDE_HOLD
    # =====================================================================

    def handle_verify_inside(self, sectors, now_sec):
        self.publish_velocity(0.0, 0.0)

        if (now_sec - self.entry_phase_start < 1.0 or
                self._scan_key() == self.entry_settle_scan_key):
            return
        self.entry_settle_scan_key = self._scan_key()
        self.entry_verify_scans += 1

        pose = self._entry_pose()
        e = self.entrance
        a, lat = self._door_frame(pose)
        rel = normalize_angle(pose.yaw - e['normal'])
        c, s = math.cos(rel), math.sin(rel)
        verify_width = self._entry_verify_width()
        half = 0.5 * verify_width

        # The entrance wall (with its gap) must now be BEHIND the robot:
        # wall returns on both sides of the doorway, none inside the gap.
        wall_left = wall_right = gap = 0
        for x, y in self._scan_points_xy():
            pa = a + x * c - y * s
            pl = lat + x * s + y * c
            if pa > a - 0.10 or pa < -0.05 or pa > 0.35:
                continue
            if abs(pl) <= half - 0.10:
                gap += 1
            elif half - 0.05 <= abs(pl) <= 2.0:
                if pl > 0:
                    wall_left += 1
                else:
                    wall_right += 1

        depth_ok = a >= self.p.entry_verify_min_inside_m
        wall_ok = min(wall_left, wall_right) >= self.p.entry_verify_min_wall_points
        gap_ok = gap <= 1
        ok = depth_ok and wall_ok and gap_ok

        self._entry_log(
            f'[VERIFY_INSIDE] scan {self.entry_verify_scans}: '
            f'depth a={a:+.2f} ({"ok" if depth_ok else "FAIL"}) lateral={lat:+.3f} '
            f'wall_behind L={wall_left} R={wall_right} ({"ok" if wall_ok else "FAIL"}) '
            f'gap_returns={gap} ({"ok" if gap_ok else "FAIL"}) '
            f'width avg={e["width"]:.3f} min={e.get("width_min", float("nan")):.3f} '
            f'used={verify_width:.3f} '
            f'-> {"INSIDE" if ok else "not verified"}'
        )

        if ok:
            self.entry_inside_verified = True
            self._set_mission_phase('INSIDE', 'verified')
            if self.p.scan_after_inside:
                self._scan_begin(now_sec)
            else:
                self._enter_state(State.INSIDE_HOLD, 'inside_verified')
            return

        if self.entry_verify_scans >= self.p.entry_verify_max_scans:
            self.entry_inside_verified = False
            self._entry_log('[VERIFY_INSIDE] could not verify inside -- holding', level='error')
            self._enter_state(State.INSIDE_HOLD, 'inside_not_verified')

    def _entry_verify_width(self):
        """
        Width for the VERIFY_INSIDE gap window only.

        The detector tends to place a jamb end slightly beyond the real
        jamb, so the smoothed e['width'] can exceed the opening and pull a
        real jamb face into the gap window. The smallest matched, unoccluded
        single-scan width is closer to the real opening. It is floored at
        entry_min_width_m and never wider than e['width']; without it,
        e['width'] is used as before.
        """
        e = self.entrance
        width_min = e.get('width_min')
        if width_min is None or not math.isfinite(width_min):
            return e['width']
        return min(e['width'], max(self.p.entry_min_width_m, width_min))

    def handle_inside_hold(self, sectors, now_sec):
        if self.entry_inside_verified and not self.p.entry_hold_after_inside:
            self._enter_state(State.EXPLORING, 'inside_verified_resume_exploration')
            return
        self.publish_velocity(0.0, 0.0)

    # =====================================================================
    # STATE: STOP_AND_SCAN
    # =====================================================================
    #
    # SETTLE (zero command until the IMU heading is still and a fresh scan
    # has arrived) -> ROTATE (in place, one full turn measured by summing IMU
    # yaw increments, every new LiDAR scan stored with the IMU heading at its
    # timestamp) -> STOPPING (zero command until still) -> analysis ->
    # INSIDE_HOLD. Linear velocity is always zero in this state.

    def _imu_yaw_at(self, stamp_sec):
        """IMU heading interpolated at a message timestamp."""
        history = self.imu_history
        if not history:
            return self.imu_yaw
        if stamp_sec <= history[0][0]:
            return history[0][1]
        prev = history[0]
        for cur in history:
            if cur[0] >= stamp_sec:
                span = cur[0] - prev[0]
                f = (stamp_sec - prev[0]) / span if span > 0.0 else 0.0
                return normalize_angle(prev[1] + f * normalize_angle(cur[1] - prev[1]))
            prev = cur
        return history[-1][1]

    def _imu_rate_deg_s(self, window_sec=0.5):
        history = self.imu_history
        if len(history) < 2:
            return float('inf')
        t1, y1 = history[-1]
        for t0, y0 in reversed(history):
            if t1 - t0 >= window_sec:
                return abs(math.degrees(normalize_angle(y1 - y0))) / (t1 - t0)
        return float('inf')

    def _scan_record(self, name, payload, mode='a'):
        directory = self.p.scan_record_dir
        if not directory:
            return
        try:
            os.makedirs(directory, exist_ok=True)
            with open(os.path.join(directory, name), mode) as f:
                f.write(json.dumps(payload) + '\n')
        except OSError as exc:
            self.get_logger().warning(f'scan record failed: {exc}', throttle_duration_sec=10.0)

    def _scan_begin(self, now_sec):
        self.publish_velocity(0.0, 0.0)
        self.last_angular_cmd = 0.0
        self.scan_count += 1
        self.scan_data = {
            'start_time': now_sec,
            'rotate_start': None,
            'end_time': None,
            'start_yaw': None,
            'last_yaw': None,
            'rotation': 0.0,
            'progress_best': 0.0,
            'progress_time': now_sec,
            'next_log_deg': 45.0,
            'samples': [],
            'openings': [],
            'last_key': self._scan_key(),
            'result': None,
            'complete': False,
            'reason': '',
        }
        self._entry_set_phase('SETTLE', now_sec)
        self._enter_state(State.STOP_AND_SCAN, 'inside_verified_scan')

    def _scan_collect(self):
        """Store a new LiDAR scan with the IMU heading at its timestamp."""

        key = self._scan_key()
        data = self.scan_data
        if key == data['last_key']:
            return
        data['last_key'] = key

        scan = self.latest_scan
        stamp = key[0] + key[1] * 1e-9
        yaw = self._imu_yaw_at(stamp)
        ranges = [float(r) for r in scan.ranges]
        valid = sum(
            1 for r in ranges if math.isfinite(r) and scan.range_min <= r <= scan.range_max
        )

        data['samples'].append(
            (yaw, ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)
        )

        pose = self.nav_pose
        for c in self._detect_entrances_now():
            cx, cy = c['center']
            c_, s_ = math.cos(yaw), math.sin(yaw)
            data['openings'].append({
                'x': pose.x + c_ * cx - s_ * cy,
                'y': pose.y + s_ * cx + c_ * cy,
                'normal': normalize_angle(yaw + c['normal']),
                'width': c['width'],
                'occluded': c['occluded'],
            })

        self._scan_record('stop_and_scan_scans.jsonl', {
            'stamp': stamp, 'imu_yaw': yaw, 'rotation_deg': math.degrees(data['rotation']),
            'nav_x': pose.x, 'nav_y': pose.y, 'valid_rays': valid,
            'angle_min': scan.angle_min, 'angle_increment': scan.angle_increment,
            'range_min': scan.range_min, 'range_max': scan.range_max,
            'ranges': [r if math.isfinite(r) else None for r in ranges],
        })

    def _scan_track_rotation(self):
        data = self.scan_data
        yaw = self.imu_yaw
        data['rotation'] += normalize_angle(yaw - data['last_yaw'])
        data['last_yaw'] = yaw
        return math.degrees(data['rotation'])

    def handle_stop_and_scan(self, sectors, now_sec):
        data = self.scan_data

        if self.entry_phase == 'SETTLE':
            self.publish_velocity(0.0, 0.0)
            if (now_sec - self.entry_phase_start >= self.p.scan_settle_sec and
                    self._imu_rate_deg_s() <= self.p.scan_still_rate_deg_s and
                    self._scan_key() != data['last_key']):
                if self.p.explore_enabled and not self._keepout_before_rotation(now_sec):
                    self.scan_count -= 1          # this scan never happened
                    return
                data['start_yaw'] = data['last_yaw'] = self.imu_yaw
                data['rotate_start'] = now_sec
                data['progress_time'] = now_sec
                data['last_key'] = None
                self._scan_collect()
                self._entry_log(
                    f'[SCAN] #{self.scan_count} stopped '
                    f'(settled {now_sec - data["start_time"]:.1f}s) -> rotating: '
                    f'start imu_yaw={math.degrees(self.imu_yaw):+.1f}deg '
                    f'nav=({self.nav_pose.x:.2f},{self.nav_pose.y:.2f})'
                )
                self._entry_set_phase('ROTATE', now_sec)
            return

        if self.entry_phase == 'ROTATE':
            turned = self._scan_track_rotation()
            self._scan_collect()

            if turned >= data['progress_best'] + 2.0:
                data['progress_best'] = turned
                data['progress_time'] = now_sec

            if turned >= data['next_log_deg']:
                self._entry_log(
                    f'[SCAN] rotated {turned:.1f}deg imu_yaw={math.degrees(self.imu_yaw):+.1f} '
                    f'scans={len(data["samples"])} elapsed={now_sec - data["rotate_start"]:.1f}s'
                )
                data['next_log_deg'] += 45.0

            remaining = self.p.scan_target_deg - turned
            reason = None
            if remaining <= self.p.scan_done_tolerance_deg:
                reason = 'complete'
            elif now_sec - data['progress_time'] >= self.p.scan_stall_sec:
                reason = 'stalled'
            elif now_sec - data['rotate_start'] >= self.p.scan_timeout_sec:
                reason = 'timeout'

            if reason is not None:
                self.publish_velocity(0.0, 0.0)
                data['reason'] = reason
                self._entry_set_phase('STOPPING', now_sec)
                return

            speed = self.p.scan_turn_speed * min(1.0, remaining / self.p.scan_slowdown_deg)
            speed = max(self.p.scan_min_turn_speed, speed)
            self.publish_velocity(0.0, speed)
            return

        if self.entry_phase == 'STOPPING':
            self.publish_velocity(0.0, 0.0)
            self._scan_track_rotation()
            self._scan_collect()
            if (now_sec - self.entry_phase_start >= 0.5 and
                    self._imu_rate_deg_s() <= self.p.scan_still_rate_deg_s):
                data['end_time'] = now_sec
                self._scan_finish()
                self._explore_after_scan(now_sec)

    def _scan_finish(self):
        data = self.scan_data
        rotation = math.degrees(data['rotation'])
        duration = data['end_time'] - data['rotate_start']
        complete = data['reason'] == 'complete' and rotation >= self.p.scan_target_deg - 5.0
        data['complete'] = complete

        result = analyze_rotation_scan(
            data['samples'],
            x_offset=self.p.lidar_offset_x_m,
            free_range_m=self.p.scan_free_range_m,
            min_free_sector_deg=self.p.scan_min_free_sector_deg,
            min_wall_length_m=self.p.scan_min_wall_length_m,
        )
        data['analysis'] = result

        # Openings: cluster the per-scan detections in the nav frame.
        clusters = []
        for o in data['openings']:
            if o['occluded']:
                continue
            for c in clusters:
                if (math.hypot(o['x'] - c['x'], o['y'] - c['y']) <= self.p.entry_match_radius_m and
                        abs(normalize_angle(o['normal'] - c['normal'])) <= math.radians(20.0)):
                    k = 1.0 / (c['hits'] + 1)
                    c['x'] += k * (o['x'] - c['x'])
                    c['y'] += k * (o['y'] - c['y'])
                    c['normal'] = normalize_angle(
                        c['normal'] + k * normalize_angle(o['normal'] - c['normal'])
                    )
                    c['width'] += k * (o['width'] - c['width'])
                    c['hits'] += 1
                    break
            else:
                clusters.append(dict(o, hits=1))

        pose = self.nav_pose
        openings = []
        for c in sorted(clusters, key=lambda c: -c['hits']):
            if c['hits'] < 3:
                continue
            e = self.entrance
            is_entrance = (
                e is not None and math.hypot(c['x'] - e['x'], c['y'] - e['y']) <= 0.5 and
                abs(normalize_angle(c['normal'] + math.pi - e['normal'])) <= math.radians(25.0)
            )
            openings.append({
                'nav_x': round(c['x'], 2), 'nav_y': round(c['y'], 2),
                'bearing_deg': round(_bearing_deg_of((c['x'] - pose.x, c['y'] - pose.y)), 1),
                'range_m': round(math.hypot(c['x'] - pose.x, c['y'] - pose.y), 2),
                'width_m': round(c['width'], 2),
                'normal_deg': round(math.degrees(c['normal']) % 360.0, 1),
                'hits': c['hits'],
                'label': 'entrance (came through)' if is_entrance else 'opening',
            })

        # Camera coverage plan (FOV ~182 deg): first view faces into the
        # house along the entry direction, the rest evenly spaced.
        into_house = math.degrees(self.entrance['normal']) % 360.0 if self.entrance else 0.0
        fov = self.p.scan_camera_fov_deg
        n_min = int(math.ceil(360.0 / fov))
        views_min = [round((into_house + k * 360.0 / n_min) % 360.0, 1) for k in range(n_min)]
        views_margin = [round((into_house + k * 120.0) % 360.0, 1) for k in range(3)]

        summary = {
            'complete_360': complete,
            'end_reason': data['reason'],
            'rotation_deg': round(rotation, 1),
            'duration_s': round(duration, 1),
            'settle_s': round(data['rotate_start'] - data['start_time'], 1),
            'start_imu_yaw_deg': round(math.degrees(data['start_yaw']), 1),
            'end_imu_yaw_deg': round(math.degrees(self.imu_yaw), 1),
            'nav_position': (round(pose.x, 3), round(pose.y, 3)),
            'scans': result['scans'],
            'total_rays': result['total_rays'],
            'valid_rays': result['valid_rays'],
            'valid_rays_per_scan': round(result['valid_rays'] / max(1, result['scans']), 1),
            'bins_with_rays': f'{result["bins_with_rays"]}/{result["num_bins"]}',
            'bins_with_returns': f'{result["bins_with_returns"]}/{result["num_bins"]}',
            'walls': result['walls'],
            'openings': openings,
            'opening_detections_total': len(data['openings']),
            'free_sectors': result['free_sectors'],
            'nearest_obstacles': result['nearest'],
            'camera_fov_deg': fov,
            'camera_headings_min_deg': views_min,
            'camera_overlap_min_deg': round(n_min * fov - 360.0, 1),
            'camera_headings_3view_deg': views_margin,
        }
        data['result'] = summary

        self._entry_log(
            f'[SCAN_RESULT] #{self.scan_count} complete_360={complete} ({data["reason"]}) '
            f'rotation={rotation:.1f}deg '
            f'duration={duration:.1f}s settle={summary["settle_s"]}s scans={result["scans"]} '
            f'valid_rays={result["valid_rays"]}/{result["total_rays"]} '
            f'({summary["valid_rays_per_scan"]}/scan) bins_with_rays={summary["bins_with_rays"]} '
            f'bins_with_returns={summary["bins_with_returns"]}'
        )
        for w in result['walls']:
            self._entry_log(f'[SCAN_RESULT] wall {w}')
        for o in openings:
            self._entry_log(f'[SCAN_RESULT] opening {o}')
        for f in result['free_sectors']:
            self._entry_log(f'[SCAN_RESULT] free_sector {f}')
        self._entry_log(f'[SCAN_RESULT] nearest {result["nearest"]}')
        self._entry_log(
            f'[SCAN_RESULT] camera FOV {fov:.0f}deg: {n_min} headings {views_min} '
            f'(overlap {summary["camera_overlap_min_deg"]}deg total); '
            f'with margin 3 headings {views_margin}'
        )

        summary['profile_m'] = [
            round(r, 3) if math.isfinite(r) else None for r in result['profile']
        ]
        self._scan_record('stop_and_scan_summary.json', summary, mode='w')

    # =====================================================================
    # LOCAL EXPLORATION PROTOTYPE: scan -> choose sector -> turn -> move
    # =====================================================================
    #
    # Sectors are relative to the IMU heading at the end of the scan
    # (sector 0 = straight ahead, positive = left). Everything is decided from
    # the merged STOP_AND_SCAN profile: every 1-degree bin is the median of all
    # rays that fell in it during the full turn, and a sector's distance is
    # its 3rd-nearest bin, so no single ray can block or open a direction.

    def _explore_log(self, msg, level='info'):
        self._entry_log(f'[EXPLORE] {msg}', level)

    def _explore_record(self, payload):
        self._scan_record('explore_steps.jsonl', payload)

    def _explore_visit(self, pose):
        if self.grid is not None:
            self.grid.mark_visited(pose.x, pose.y)
        if not self.explore_visited or math.hypot(
                pose.x - self.explore_visited[-1][0], pose.y - self.explore_visited[-1][1]) >= 0.2:
            self.explore_visited.append((pose.x, pose.y))

    def _explore_novelty(self, x, y):
        if not self.explore_visited:
            return float('inf')
        return min(math.hypot(x - vx, y - vy) for vx, vy in self.explore_visited)

    def _explore_free_travel(self, points, heading, half_width=None):
        """Longest straight travel along world heading (from the scan
        position) keeping every point >= path half-width to the side and the
        stop margin ahead."""
        c, s = math.cos(heading), math.sin(heading)
        half = self.p.explore_path_half_width_m if half_width is None else half_width
        best = self.p.explore_step_m
        for x, y in points:
            ahead = x * c + y * s
            side = -x * s + y * c
            if ahead > 0.0 and abs(side) < half:
                best = min(best, ahead - self.p.explore_stop_margin_m)
        return max(0.0, best)

    def _explore_toward_entrance(self, pose, heading, distance):
        """True if moving `distance` along heading ends at (or beyond) the
        entrance plane within the doorway's lateral span."""
        if self.entrance is None:
            return False
        end = Pose2D(
            pose.x + distance * math.cos(heading), pose.y + distance * math.sin(heading), heading
        )
        a, lat = self._door_frame(end)
        near_doorway = abs(lat) < 0.5 * self.entrance['width'] + 0.5
        return a < self.p.explore_entrance_guard_m and near_doorway

    def _explore_sector_table(self, pose, analysis):
        n = self.p.explore_sector_count
        width = 360.0 / n
        profile = analysis['profile']
        valid_per_bin = analysis['valid_per_bin']
        rays_per_bin = analysis['rays_per_bin']
        maxrange_per_bin = analysis['maxrange_per_bin']
        num_bins = len(profile)
        points = [
            (r * math.cos(math.radians(k + 0.5)), r * math.sin(math.radians(k + 0.5)))
            for k, r in enumerate(profile) if math.isfinite(r)
        ]
        heading_deg = math.degrees(pose.yaw)
        table = []
        for i in range(n):
            rel = normalize_angle(math.radians(i * width))
            world = normalize_angle(pose.yaw + rel)
            first = int(math.floor(heading_deg + math.degrees(rel) - width / 2.0))
            bins = [(first + j) % num_bins for j in range(int(width))]
            # Bins without enough returns read as 'beyond range' (inf).
            values = sorted(profile[b] for b in bins)
            valid = sum(valid_per_bin[b] for b in bins)
            max_range = sum(maxrange_per_bin[b] for b in bins)
            rays = sum(rays_per_bin[b] for b in bins)
            fraction = (valid + max_range) / rays if rays else 0.0
            robust_min = values[min(2, len(values) - 1)] if values else float('inf')

            offsets = [0.0]
            step = self.p.explore_heading_offsets_deg
            k = step
            while k < width / 2.0:
                offsets += [k, -k]
                k += step
            best_heading, best_travel = world, -1.0
            for off in offsets:
                h = normalize_angle(world + math.radians(off))
                travel = self._explore_free_travel(points, h)
                if travel > best_travel + 1e-6:
                    best_heading, best_travel = h, travel

            end_x = pose.x + self.p.explore_step_m * math.cos(world)
            end_y = pose.y + self.p.explore_step_m * math.sin(world)
            novelty = self._explore_novelty(end_x, end_y)
            explored = novelty < self.p.explore_visited_radius_m
            entrance = self._explore_toward_entrance(pose, world, self.p.explore_step_m)

            if fraction < self.p.explore_min_measured_fraction:
                status = 'unknown'
            elif robust_min < self.p.explore_blocked_m:
                status = 'blocked'
            else:
                status = 'open'

            table.append({
                'sector': i,
                'rel_deg': round(math.degrees(rel), 1),
                'world_deg': round(math.degrees(world), 1),
                'min_m': round(robust_min, 3) if math.isfinite(robust_min) else None,
                'valid_rays': valid,
                'max_range_rays': max_range,
                'rays': rays,
                'measured_fraction': round(fraction, 2),
                'status': status,
                'toward_entrance': entrance,
                'explored': explored,
                'novelty_m': round(novelty, 2) if math.isfinite(novelty) else None,
                'free_travel_m': round(best_travel, 2),
                'best_heading_deg': round(math.degrees(best_heading), 1),
                '_heading': best_heading,
                '_novelty': novelty,
                '_min': robust_min,
            })
        return table

    def _explore_choose(self, pose, table):
        candidates = [
            s for s in table
            if s['status'] == 'open' and s['free_travel_m'] >= self.p.explore_min_step_m
            and not s['toward_entrance']
            and not self._explore_toward_entrance(pose, s['_heading'], s['free_travel_m'] + 0.1)
        ]
        fresh = [s for s in candidates if not s['explored']]
        pool = fresh if fresh else [s for s in candidates if s['_novelty'] >= 0.3]
        if not pool:
            return None, 'no open, reachable, unexplored sector'

        def score(s):
            novelty = min(s['_novelty'], 2.0)
            travel = min(s['free_travel_m'], self.p.explore_step_m) / self.p.explore_step_m
            clearance = min(s['_min'], 3.5) / 3.5  # inf (beyond range) -> 1.0
            turn = abs(normalize_angle(s['_heading'] - pose.yaw)) / math.pi
            return novelty + 0.5 * travel + 0.3 * clearance - 0.2 * turn

        best = max(pool, key=lambda s: (round(score(s), 6), -abs(s['rel_deg'])))
        why = (
            f'{"unexplored" if fresh else "least-explored (all explored)"}, open, '
            f'novelty={min(best["_novelty"], 99):.2f}m free_travel={best["free_travel_m"]:.2f}m '
            f'clearance={best["_min"]:.2f}m score={score(best):.2f} '
            f'(of {len(pool)} candidates: '
            + ', '.join(
                f'S{s["sector"]}={score(s):.2f}' for s in sorted(pool, key=score, reverse=True)
            )
            + ')'
        )
        return best, why

    def _explore_after_scan(self, now_sec):
        data = self.scan_data
        pose = self._entry_pose()
        self._explore_visit(pose)
        summary = data['result']

        last_move = self.explore_move
        if self.explore_move is not None:
            m = self.explore_move
            self._explore_log(
                f'step {self.explore_step} next scan #{self.scan_count}: '
                f'complete_360={summary["complete_360"]} rotation={summary["rotation_deg"]}deg '
                f'duration={summary["duration_s"]}s valid_rays={summary["valid_rays"]}/'
                f'{summary["total_rays"]} bins_with_returns={summary["bins_with_returns"]} '
                f'(after moving {m["achieved"]:.2f}m, stop reason {m["reason"]})'
            )
            self.explore_move = None

        if not self.p.explore_enabled:
            self._enter_state(State.INSIDE_HOLD, f'scan_{data["reason"]}')
            return
        if not data['complete']:
            self._explore_log(
                f'scan #{self.scan_count} incomplete ({data["reason"]}) -- holding', 'warn'
            )
            self._enter_state(State.INSIDE_HOLD, f'scan_{data["reason"]}')
            return
        grid_mode = self.p.explore_planner == 'grid'
        if grid_mode:
            self._grid_update(pose, data)
            stop = self._grid_stop_reason(pose, last_move)
            if stop is not None:
                self._explore_log(f'exploration stopped: {stop} -- holding', 'warn')
                self._grid_record(now_sec, pose, None, stop)
                self._enter_state(State.INSIDE_HOLD, 'explore_stopped')
                return
            # A confirmed victim takes priority over the next frontier.
            if self.p.victim_approach_enabled and self._victim_after_scan(now_sec, pose, data):
                return
        if self.explore_step >= self.p.explore_max_steps:
            self._explore_log(
                f'finished: {self.explore_step} moves, {self.scan_count} scans -- holding'
            )
            if grid_mode:
                self._grid_record(now_sec, pose, None, 'exploration_limit_reached')
            self._enter_state(State.INSIDE_HOLD, 'explore_steps_done')
            return

        self.explore_step += 1
        if grid_mode:
            self._grid_select(now_sec, pose, data)
            return
        a, lat = self._door_frame(pose)
        table = self._explore_sector_table(pose, data['analysis'])
        self._explore_log(
            f'===== EXPLORATION STEP {self.explore_step} ===== scan #{self.scan_count} '
            f'imu_heading={math.degrees(pose.yaw):+.1f}deg nav=({pose.x:.2f},{pose.y:.2f}) '
            f'from_entrance a={a:+.2f} lat={lat:+.2f}'
        )
        for s in table:
            self._explore_log(
                f'  S{s["sector"]:<2d} rel={s["rel_deg"]:+6.1f} world={s["world_deg"]:+6.1f} '
                f'min={">3.5m" if s["min_m"] is None else f"{s["min_m"]:.2f}m"} '
                f'valid={s["valid_rays"]} maxrange={s["max_range_rays"]} of {s["rays"]} '
                f'(measured {s["measured_fraction"]:.2f}) status={s["status"]:<7s} '
                f'explored={"yes" if s["explored"] else "no"} '
                f'entrance={"yes" if s["toward_entrance"] else "no"} '
                f'free_travel={s["free_travel_m"]:.2f}m'
            )

        choice, why = self._explore_choose(pose, table)
        record = {
            'step': self.explore_step, 'scan': self.scan_count, 't': now_sec,
            'imu_heading_deg': math.degrees(pose.yaw), 'nav': [pose.x, pose.y],
            'door_a': a, 'door_lat': lat,
            'sectors': [{k: v for k, v in s.items() if not k.startswith('_')} for s in table],
        }

        if choice is None:
            self._explore_log(f'no direction selected: {why} -- holding', 'warn')
            record['selected'] = None
            record['reason'] = why
            self._explore_record(record)
            self._enter_state(State.INSIDE_HOLD, 'explore_no_direction')
            return

        heading = choice['_heading']
        target = min(self.p.explore_step_m, choice['free_travel_m'])
        rel = math.degrees(normalize_angle(heading - pose.yaw))
        self.explore_plan = {
            'sector': choice['sector'], 'heading': heading, 'target': target,
            'start': pose, 'turn_start': now_sec,
        }
        self._explore_log(
            f'selected S{choice["sector"]} heading={math.degrees(heading):+.1f}deg '
            f'relative={rel:+.1f}deg clearance={choice["_min"]:.2f}m '
            f'free_travel={choice["free_travel_m"]:.2f}m target={target:.2f}m -- {why}'
        )
        record['selected'] = {
            'sector': choice['sector'], 'heading_deg': math.degrees(heading),
            'relative_deg': rel, 'clearance_m': choice['_min'],
            'free_travel_m': choice['free_travel_m'], 'target_m': target,
        }
        record['reason'] = why
        self._explore_record(record)
        self._enter_state(State.EXPLORE_TURN, f'explore_step_{self.explore_step}')

    # =====================================================================
    # PERSISTENT EXPLORATION GRID: scan -> update grid -> frontiers -> move
    # =====================================================================
    #
    # The grid (exploration_grid.py) is in the nav frame: odometry distance
    # along the IMU heading, origin = spawn. It is only as good as that
    # dead-reckoned position; no scan matching or correction is applied.

    # ---- narrow-obstacle keep-out -----------------------------------------

    def _keepout_radius(self):
        """Robot-centre distance a narrow obstacle must keep: keep-out to the
        rotation-swept body plus the swept radius."""
        return self.p.narrow_obstacle_keepout_m + self.p.robot_swept_radius_m

    def _narrow_on_structure(self, wx, wy):
        """True when the nav-frame point lies on a connected OCCUPIED grid
        structure (wall end, door jamb) rather than an isolated post."""
        grid = self.grid
        if grid is None:
            return False
        ci, cj = grid.cell_of(wx, wy)
        seeds = [(ci + di, cj + dj) for di in (-1, 0, 1) for dj in (-1, 0, 1)
                 if grid.state((ci + di, cj + dj)) == OCCUPIED]
        reach = int(math.ceil(1.0 / grid.res))
        seen, queue = set(seeds), list(seeds)
        while queue and len(seen) < self.p.narrow_obstacle_structure_cells:
            i, j = queue.pop()
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    n = (i + di, j + dj)
                    if (n not in seen and abs(n[0] - ci) <= reach and abs(n[1] - cj) <= reach and
                            grid.state(n) == OCCUPIED):
                        seen.add(n)
                        queue.append(n)
        return len(seen) >= self.p.narrow_obstacle_structure_cells

    def _narrow_remember(self, wx, wy, width):
        for r in self.narrow_released:
            if math.hypot(r['x'] - wx, r['y'] - wy) <= self.p.narrow_release_suppress_radius_m:
                self.keepout_stats['suppressed_released'] += 1
                r['suppressed'] += 1
                if r.get('last_logged_scan') != self.scan_count:
                    r['last_logged_scan'] = self.scan_count
                    self._explore_log(
                        f'[KEEPOUT] not remembering narrow obstacle at nav ({wx:.2f},{wy:.2f}) '
                        f'width {width:.3f} m: within '
                        f'{self.p.narrow_release_suppress_radius_m:.2f} m of released position '
                        f'({r["x"]:.2f},{r["y"]:.2f}) (suppressed {r["suppressed"]} times so far)')
                    self._scan_record('keepout_events.jsonl', {
                        'event': 'remember_suppressed', 'scan': self.scan_count,
                        'obstacle_nav': [wx, wy], 'width_m': width,
                        'released_nav': [r['x'], r['y']]})
                return
        for m in self.narrow_memory:
            if math.hypot(m['x'] - wx, m['y'] - wy) <= 0.15:
                m['x'], m['y'] = 0.5 * (m['x'] + wx), 0.5 * (m['y'] + wy)
                was_eligible = self._narrow_release_eligible(m)
                m['width_min'] = min(self._narrow_width_min(m), width)
                m['width_max'] = max(m.get('width_max', m['width']), width)
                m['width_n'] = m.get('width_n', 1) + 1
                if was_eligible and not self._narrow_release_eligible(m):
                    self._explore_log(
                        f'[KEEPOUT] narrow obstacle at nav ({m["x"]:.2f},{m["y"]:.2f}) measured '
                        f'{width:.3f} m wide (<= {self.p.narrow_release_min_width_m:.2f} m): '
                        f'never releasable (widths {m["width_min"]:.3f}..{m["width_max"]:.3f} m '
                        f'over {m["width_n"]} observations)')
                return
        self.narrow_memory.append({'x': wx, 'y': wy, 'width': width, 'width_min': width,
                                   'width_max': width, 'width_n': 1})
        self.keepout_stats['remembered'] += 1
        del self.narrow_memory[:-60]

    @staticmethod
    def _narrow_width_min(m):
        """Narrowest width observed for a remembered narrow obstacle."""
        return m.get('width_min', m['width'])

    def _narrow_release_eligible(self, m):
        """
        Return True when a remembered entry may ever be released.

        Every width observed for it (first detection and every later
        detection merged into it) must exceed narrow_release_min_width_m: a
        single narrower observation makes it permanently ineligible, so one
        early or outlier wide measurement can never make a thin object
        releasable.
        """
        return self._narrow_width_min(m) > self.p.narrow_release_min_width_m

    def _narrow_release_check(self, pose, profile):
        """
        Count contradicting evidence for remembered narrow obstacles.

        One merged 360 profile taken at pose (world-aligned, robot-centred).
        Must run after self.narrow_plan holds this scan's fresh detections:
        an entry re-detected as an isolated narrow obstacle here has its
        contradiction count reset instead. Entries released here leave
        narrow_memory (and with it the keep-out cells) and go to
        narrow_released.
        """
        if not self.narrow_memory:
            return
        polar = [(math.radians(k + 0.5), r) for k, r in enumerate(profile)]
        need = self.p.narrow_release_contradictions
        keep = []
        for m in self.narrow_memory:
            dx, dy = m['x'] - pose.x, m['y'] - pose.y
            dist = math.hypot(dx, dy)
            if dist > self.p.narrow_release_range_m or not self._narrow_release_eligible(m):
                keep.append(m)
                continue
            where = (f'nav ({m["x"]:.2f},{m["y"]:.2f}) widths '
                     f'{self._narrow_width_min(m):.3f}..{m.get("width_max", m["width"]):.3f} m '
                     f'({m.get("width_n", 1)} obs)')
            if any(math.hypot(o['x'] - dx, o['y'] - dy) <= NARROW_RELEASE_NEAR_M
                   for o in self.narrow_plan):
                if m.get('contradictions'):
                    self._explore_log(
                        f'[KEEPOUT] scan #{self.scan_count} narrow obstacle at {where} '
                        f're-detected as isolated -- contradiction count '
                        f'{m["contradictions"]} reset to 0')
                m['contradictions'] = 0
                m['contra_poses'] = []
                keep.append(m)
                continue
            length = contiguous_surface_length(polar, dx, dy, NARROW_RELEASE_NEAR_M)
            if length <= self.p.narrow_release_min_surface_m:
                keep.append(m)
                continue
            poses = m.setdefault('contra_poses', [])
            if any(math.hypot(pose.x - px, pose.y - py) < NARROW_RELEASE_MIN_VIEW_SEPARATION_M
                   for px, py in poses):
                self._explore_log(
                    f'[KEEPOUT] scan #{self.scan_count} narrow obstacle at {where}: surface '
                    f'{length:.2f} m again from the same view point -- not counted')
                keep.append(m)
                continue
            poses.append((pose.x, pose.y))
            m['contradictions'] = m.get('contradictions', 0) + 1
            self.keepout_stats['contradictions'] += 1
            self._explore_log(
                f'[KEEPOUT] scan #{self.scan_count} contradiction #{m["contradictions"]}/{need} '
                f'for narrow obstacle at {where}: lies on a contiguous surface {length:.2f} m '
                f'(> {self.p.narrow_release_min_surface_m:.2f} m) seen from '
                f'nav ({pose.x:.2f},{pose.y:.2f}) range {dist:.2f} m')
            self._scan_record('keepout_events.jsonl', {
                'event': 'contradiction', 'scan': self.scan_count, 'count': m['contradictions'],
                'obstacle_nav': [m['x'], m['y']], 'width_m': m['width'],
                'width_min_m': self._narrow_width_min(m), 'width_n': m.get('width_n', 1),
                'surface_m': length, 'range_m': dist, 'nav': [pose.x, pose.y]})
            if m['contradictions'] < need:
                keep.append(m)
                continue
            self.narrow_released.append({'x': m['x'], 'y': m['y'], 'width': m['width'],
                                         'width_min': self._narrow_width_min(m),
                                         'scan': self.scan_count, 'suppressed': 0})
            self.keepout_stats['released'] += 1
            self._explore_log(
                f'[KEEPOUT] scan #{self.scan_count} RELEASED narrow obstacle at {where} after '
                f'{m["contradictions"]} contradicting scans: removed from the keep-out memory; '
                f'nothing within {self.p.narrow_release_suppress_radius_m:.2f} m of it is '
                f'remembered again', 'warn')
            self._scan_record('keepout_events.jsonl', {
                'event': 'released', 'scan': self.scan_count,
                'obstacle_nav': [m['x'], m['y']], 'width_m': m['width'],
                'width_min_m': self._narrow_width_min(m), 'width_n': m.get('width_n', 1),
                'contradicting_views_nav': [list(p) for p in poses]})
        self.narrow_memory[:] = keep

    def _narrow_detect(self, polar, to_nav):
        """Isolated narrow obstacles; to_nav maps a detection to the nav frame
        for the grid structure test."""
        out = []
        for o in detect_narrow_obstacles(
                polar, max_width_m=self.p.narrow_obstacle_width_threshold_m,
                max_range_m=self.p.narrow_obstacle_max_range_m,
                background_m=self.p.narrow_obstacle_background_m):
            if self._narrow_on_structure(*to_nav(o['x'], o['y'])):
                self.keepout_stats['structure_filtered'] += 1
                continue
            out.append(o)
        return out

    def _narrow_from_profile(self, profile, pose):
        """Narrow obstacles in a merged scan profile (world-aligned axes,
        centred on the robot at pose)."""
        return self._narrow_detect(
            [(math.radians(k + 0.5), r) for k, r in enumerate(profile)],
            lambda x, y: (pose.x + x, pose.y + y))

    def _narrow_from_live_scan(self):
        """Narrow obstacles in the latest raw scan (robot frame)."""
        scan = self.latest_scan
        pose = self._entry_pose()
        return self._narrow_detect(scan_polar_from_robot(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max, self.p.lidar_offset_x_m,
        ), lambda x, y: self._to_world(pose, x, y))

    def _keepout_before_rotation(self, now_sec):
        """Called right before an in-place 360 scan. Returns True when the
        rotation may start; otherwise it has started an escape move away from
        the nearest narrow obstacle (or a terminal hold)."""
        radius = self._keepout_radius() - self.p.narrow_obstacle_tolerance_m
        near = [o for o in self._narrow_from_live_scan() if o['range'] < radius]
        if not near:
            self.keepout_escapes = 0
            return True
        o = min(near, key=lambda o: o['range'])
        pose = self._entry_pose()
        wx, wy = self._to_world(pose, o['x'], o['y'])
        self._narrow_remember(wx, wy, o['width'])
        where = (f'narrow obstacle at nav ({wx:.2f},{wy:.2f}) range {o["range"]:.2f} m '
                 f'width {o["width"]:.3f} m ({o["rays"]} rays) < keep-out {radius:.2f} m')
        # Escape heading: the straight move (free travel checked) whose end
        # point is farthest from every narrow obstacle around, not merely
        # away from the nearest one (two posts can bracket the robot).
        c, s_ = math.cos(pose.yaw), math.sin(pose.yaw)
        points = [(c * x - s_ * y, s_ * x + c * y) for x, y in self._scan_points_xy()]
        posts = [(c * q['x'] - s_ * q['y'], s_ * q['x'] + c * q['y'])
                 for q in self._narrow_from_live_scan()]
        need = self._keepout_radius() - o['range'] + 0.05
        best = (min(math.hypot(px, py) for px, py in posts), None, 0.0)
        for k in range(24):
            h = math.radians(15.0 * k)
            reach = min(need, self._explore_free_travel(points, h))
            if reach < 0.05:
                continue
            ex, ey = reach * math.cos(h), reach * math.sin(h)
            clear = min(math.hypot(px - ex, py - ey) for px, py in posts)
            if clear > best[0] + 0.01:
                best = (clear, h, reach)
        away = normalize_angle(best[1]) if best[1] is not None else 0.0
        target = best[2]
        if self.keepout_escapes >= self.p.narrow_obstacle_max_escapes or target < 0.05:
            self.keepout_stats['escape_failed'] += 1
            self.publish_velocity(0.0, 0.0)
            self._explore_log(
                f'[KEEPOUT] cannot rotate safely: {where}; escape not possible '
                f'(attempts {self.keepout_escapes}, free {target:.2f} m) -- holding', 'error')
            self._scan_record('keepout_events.jsonl', {
                't': now_sec, 'event': 'rotation_refused_hold', 'nav': [pose.x, pose.y],
                'obstacle_nav': [wx, wy], 'range_m': o['range'], 'width_m': o['width']})
            self._enter_state(State.INSIDE_HOLD, 'narrow_keepout_escape_failed')
            return False
        self.keepout_escapes += 1
        self.keepout_stats['escape_moves'] += 1
        self.explore_plan = {
            'sector': 'ESCAPE', 'heading': away, 'target': target,
            'start': pose, 'turn_start': now_sec,
        }
        self.explore_move = None
        self._explore_log(
            f'[KEEPOUT] scan rotation refused: {where} -> escape move '
            f'{target:.2f} m toward {math.degrees(away):+.1f}deg before scanning')
        self._scan_record('keepout_events.jsonl', {
            't': now_sec, 'event': 'rotation_refused_escape', 'nav': [pose.x, pose.y],
            'obstacle_nav': [wx, wy], 'range_m': o['range'], 'width_m': o['width'],
            'escape_heading_deg': math.degrees(away), 'escape_m': target})
        self._enter_state(State.EXPLORE_TURN, 'narrow_keepout_escape')
        return False

    def _grid_in_portal(self, x, y):
        if self.entrance is None:
            return False
        return in_portal(
            self.entrance, x, y, self.p.grid_portal_depth_out_m,
            self.p.grid_portal_depth_in_m, self.p.grid_portal_lateral_margin_m,
        )

    def _grid_blocked(self, cell):
        """The entrance portal: no flood fill (interior or reachability)
        crosses it, and none of its cells is an exploration target."""
        return self._grid_in_portal(*self.grid.center(cell))

    def _grid_reach_blocked(self, cell):
        """Reachability flood fill: the portal, plus cells inside a narrow-
        obstacle keep-out, so frontier aim points route around posts the
        path check would refuse (cells within 0.3 m of the robot stay open so
        it can leave its own spot)."""
        if self._grid_blocked(cell):
            return True
        if cell not in self.grid_keepout_cells:
            return False
        x, y = self.grid.center(cell)
        pose = self._grid_pose
        return pose is None or math.hypot(x - pose.x, y - pose.y) > 0.30

    def _grid_update_keepout_cells(self):
        grid = self.grid
        radius = self._keepout_radius() + 0.5 * grid.res
        k = int(math.ceil(radius / grid.res))
        cells = set()
        for m in self.narrow_memory:
            ci, cj = grid.cell_of(m['x'], m['y'])
            for di in range(-k, k + 1):
                for dj in range(-k, k + 1):
                    cx, cy = grid.center((ci + di, cj + dj))
                    if math.hypot(cx - m['x'], cy - m['y']) < radius:
                        cells.add((ci + di, cj + dj))
        self.grid_keepout_cells = cells

    def _grid_excluded(self, cell):
        """Frontier cells in the portal or outside the interior region
        (i.e. only seen through the entrance) are never targets."""
        return self._grid_blocked(cell) or cell not in self.grid_interior

    def _grid_update(self, pose, data):
        if self.grid is None:
            self.grid = ExplorationGrid(self.p.grid_resolution_m, self.p.grid_robot_radius_m)
        analysis = data['analysis']
        stats = self.grid.integrate_profile(
            pose.x, pose.y, analysis['profile'], analysis['rays_per_bin'],
            analysis['maxrange_per_bin'], self._scan_range_max(data),
            self.p.grid_no_return_min_fraction,
        )
        self.grid.mark_visited(pose.x, pose.y)
        self.narrow_plan = self._narrow_from_profile(analysis['profile'], pose)
        self._narrow_release_check(pose, analysis['profile'])
        for o in self.narrow_plan:
            self._narrow_remember(pose.x + o['x'], pose.y + o['y'], o['width'])
        for m in self.narrow_memory:
            dx, dy = m['x'] - pose.x, m['y'] - pose.y
            if math.hypot(dx, dy) > self.p.narrow_obstacle_max_range_m:
                continue
            if any(math.hypot(o['x'] - dx, o['y'] - dy) <= 0.15 for o in self.narrow_plan):
                continue
            self.narrow_plan.append({'x': dx, 'y': dy, 'range': math.hypot(dx, dy),
                                     'bearing': math.atan2(dy, dx), 'width': m['width'],
                                     'rays': 0})
        if self.narrow_plan:
            self.keepout_stats['scans_with_narrow'] += 1
            self.keepout_stats['narrow_detections'] += len(self.narrow_plan)
        for o in self.narrow_plan:
            self._explore_log(
                f'[KEEPOUT] scan #{self.scan_count} narrow obstacle at nav '
                f'({pose.x + o["x"]:.2f},{pose.y + o["y"]:.2f}) range {o["range"]:.2f} m '
                f'width {o["width"]:.3f} m rays {o["rays"] or "remembered"} '
                f'(keep-out radius '
                f'{self._keepout_radius():.2f} m)')
        if self.grid_interior_seed is None:
            # First grid scan happens right after VERIFY_INSIDE: this pose
            # is on the interior side of the portal.
            self.grid_interior_seed = self.grid.cell_of(pose.x, pose.y)
        self.grid_interior = interior_region(
            self.grid, [self.grid_interior_seed], self._grid_blocked
        )
        weights = {
            'size': self.p.grid_score_size, 'distance': self.p.grid_score_distance,
            'clearance': self.p.grid_score_clearance, 'turn': self.p.grid_score_turn,
        }
        self._grid_pose = pose
        self._grid_update_keepout_cells()
        ev = evaluate_frontiers(
            self.grid, pose.x, pose.y, pose.yaw,
            min_clear_m=self.p.grid_min_clear_m,
            min_cluster_cells=self.p.grid_min_cluster_cells,
            excluded=self._grid_excluded, blocked=self._grid_reach_blocked, weights=weights,
        )
        ev['integration'] = stats
        ev['counts'] = self.grid.counts()
        free = [c for c in self.grid.cells if self.grid.is_free(c)]
        robot_cell = self.grid.cell_of(pose.x, pose.y)
        ev['interior'] = {
            'seed_cell': list(self.grid_interior_seed),
            'interior_cells': len(self.grid_interior),
            'portal_free_cells': sum(1 for c in free if self._grid_blocked(c)),
            'exterior_free_cells': sum(
                1 for c in free if c not in self.grid_interior and not self._grid_blocked(c)
            ),
            'robot_cell_interior': robot_cell in self.grid_interior,
            'robot_in_portal': self._grid_in_portal(pose.x, pose.y),
        }
        self.grid_last_eval = ev
        c = ev['counts']
        a, lat = self._door_frame(pose) if self.entrance is not None else (float('nan'),) * 2
        self._explore_log(
            f'[GRID] scan #{self.scan_count} nav=({pose.x:.2f},{pose.y:.2f}) '
            f'imu_heading={math.degrees(pose.yaw):+.1f}deg from_entrance a={a:+.2f} '
            f'cells occupied={c["OCCUPIED"]} free={c["FREE"]} visited={c["VISITED"]} '
            f'unknown={c["UNKNOWN"]} | bins return={stats["return_bins"]} '
            f'no_return={stats["no_return_bins"]} skipped={stats["skipped_bins"]} | '
            f'frontier cells={ev["frontier_cells"]} (excluded {ev["frontier_cells_excluded"]}) '
            f'clusters={len(ev["clusters"])} reachable_cells={ev["reachable_cells"]} | '
            f'interior={ev["interior"]["interior_cells"]} '
            f'exterior_free={ev["interior"]["exterior_free_cells"]} '
            f'portal_free={ev["interior"]["portal_free_cells"]} '
            f'robot_interior={ev["interior"]["robot_cell_interior"]}'
        )
        for k, cl in enumerate(ev['clusters']):
            if cl['status'] == 'candidate':
                self._explore_log(
                    f'[GRID]   C{k} size={cl["size"]} centroid=({cl["centroid"][0]:.2f},'
                    f'{cl["centroid"][1]:.2f}) dist={cl["distance_m"]:.2f}m '
                    f'heading={cl["heading_deg"]:+.1f}deg turn={cl["turn_deg"]:+.1f}deg '
                    f'clearance={cl["clearance_m"]:.2f}m score={cl["score"]:+.3f} '
                    f'{cl["score_terms"]}'
                )
            else:
                self._explore_log(
                    f'[GRID]   C{k} size={cl["size"]} centroid=({cl["centroid"][0]:.2f},'
                    f'{cl["centroid"][1]:.2f}) {cl["status"]}'
                )

    @staticmethod
    def _scan_range_max(data):
        return max((s[5] for s in data['samples']), default=3.5)

    def _grid_stop_reason(self, pose, last_move):
        if self._grid_in_portal(pose.x, pose.y):
            return 'robot is in the entrance portal'
        robot_cell = self.grid.cell_of(pose.x, pose.y)
        if robot_cell not in self.grid_interior and not self._grid_blocked(robot_cell):
            return 'robot left the known interior region (not connected to the inside seed)'
        if last_move is not None:
            # Any cycle that ended without real progress (aim too close, move
            # refused by the live keep-out) counts; real progress resets.
            if last_move.get('reason') == 'target_close_no_move':
                pass                                  # counted when decided
            elif last_move.get('achieved', 0.0) < self.p.grid_min_move_m:
                self.grid_no_move_count += 1
            else:
                self.grid_no_move_count = 0
        if self.grid_no_move_count >= 2:
            return 'no progress: no usable move twice in a row (close aim point / keep-out)'
        if last_move is not None:
            reason = last_move.get('reason', '')
            if reason == 'left_interior':
                return 'robot left the known interior region during the move'
            if (reason.startswith(('front_cone', 'path_blocked')) and
                    last_move.get('achieved', 0.0) < self.p.grid_unsafe_min_progress_m):
                return f'movement unsafe ({reason}, achieved {last_move["achieved"]:.2f} m)'
        return None

    def _grid_record(self, now_sec, pose, selected, reason):
        ev = self.grid_last_eval or {'clusters': [], 'counts': {}, 'excluded_cells': []}
        clusters = [
            {k: v for k, v in cl.items() if k != 'cells' and not k.startswith('_')}
            for cl in ev['clusters']
        ]
        self._scan_record('grid_scans.jsonl', {
            'scan': self.scan_count, 'step': self.explore_step, 't': now_sec,
            'nav': [round(pose.x, 3), round(pose.y, 3)],
            'imu_heading_deg': round(math.degrees(pose.yaw), 2),
            'door_frame': [round(v, 3) for v in self._door_frame(pose)] if self.entrance else None,
            'resolution_m': self.p.grid_resolution_m,
            'counts': ev['counts'],
            'integration': ev.get('integration'),
            'frontier_cells': ev.get('frontier_cells'),
            'frontier_cells_excluded': ev.get('frontier_cells_excluded'),
            'reachable_cells': ev.get('reachable_cells'),
            'interior': ev.get('interior'),
            'narrow_obstacles': [
                {'nav': [round(pose.x + o['x'], 3), round(pose.y + o['y'], 3)],
                 'range_m': round(o['range'], 3), 'width_m': round(o['width'], 3),
                 'rays': o['rays']} for o in self.narrow_plan],
            'keepout_radius_m': self._keepout_radius(),
            'narrow_memory_nav': [
                [round(m['x'], 3), round(m['y'], 3)] for m in self.narrow_memory],
            'narrow_memory_width': [
                [round(self._narrow_width_min(m), 3), round(m.get('width_max', m['width']), 3),
                 m.get('width_n', 1)] for m in self.narrow_memory],
            'narrow_released_nav': [
                [round(r['x'], 3), round(r['y'], 3)] for r in self.narrow_released],
            'keepout_cells': len(self.grid_keepout_cells),
            'keepout_stats': dict(self.keepout_stats),
            'tilt_max': dict(self.tilt_max),
            'cluster_count': len(clusters),
            'clusters': clusters,
            'selected': selected,
            'reason': reason,
        })
        if self.p.scan_record_dir and self.grid is not None:
            sel_cells = []
            if selected is not None:
                sel_cells = ev['clusters'][selected['cluster']]['cells']
            try:
                self.grid.dump(
                    os.path.join(self.p.scan_record_dir, f'grid_scan_{self.scan_count:02d}.json'),
                    robot=(pose.x, pose.y),
                    frontier=[c for cl in ev['clusters'] for c in cl['cells']],
                    selected=sel_cells, excluded=ev['excluded_cells'],
                )
            except OSError as exc:
                self.get_logger().warning(f'grid dump failed: {exc}', throttle_duration_sec=10.0)

    def _grid_select(self, now_sec, pose, data):
        """Pick the best-scoring reachable frontier cluster that can actually
        be driven toward from here; turn + bounded move reuse EXPLORE_TURN /
        EXPLORE_MOVE."""
        profile = data['analysis']['profile']
        points = [
            (r * math.cos(math.radians(k + 0.5)), r * math.sin(math.radians(k + 0.5)))
            for k, r in enumerate(profile) if math.isfinite(r)
        ]
        ev = self.grid_last_eval
        rejected = []
        choice = None
        plan_half = self.p.explore_path_half_width_m + self.p.grid_plan_path_margin_m
        for k, cl in enumerate(ev['clusters']):
            if cl['status'] != 'candidate':
                continue
            best_heading, best_travel, best_free, best_keep = cl['_heading'], -1.0, 0.0, None
            max_free = 0.0
            for off in (0.0, 10.0, -10.0, 20.0, -20.0):
                h = normalize_angle(cl['_heading'] + math.radians(off))
                free = self._explore_free_travel(points, h, plan_half)
                keep, keep_obs = narrow_keepout_travel(
                    self.narrow_plan, h, self.p.explore_step_m, self._keepout_radius(),
                    tolerance_m=self.p.narrow_obstacle_tolerance_m)
                travel = min(free, keep)
                max_free = max(max_free, free)
                if travel > best_travel + 1e-6:
                    best_heading, best_travel, best_free = h, travel, free
                    best_keep = (keep, keep_obs)
                if off == 0.0 and travel >= self.p.grid_min_step_m:
                    break
            if best_travel < self.p.grid_min_step_m:
                if max_free >= self.p.grid_min_step_m:
                    self.keepout_stats['clusters_rejected'] += 1
                    o = best_keep[1] if best_keep and best_keep[1] else None
                    if o is None and self.narrow_plan:
                        o = min(self.narrow_plan, key=lambda o: o['range'])
                    geo = (f' (obstacle nav ({pose.x + o["x"]:.2f},{pose.y + o["y"]:.2f}) range '
                           f'{o["range"]:.2f} m width {o["width"]:.3f} m)' if o else '')
                    rejected.append(
                        f'C{k}: narrow keep-out {self._keepout_radius():.2f} m leaves '
                        f'{best_travel:.2f} m (free travel up to {max_free:.2f} m){geo}')
                else:
                    rejected.append(f'C{k}: free travel {best_travel:.2f} m')
                continue
            # Never drive past the reachable aim point (measured along the
            # chosen heading), never beyond the step or the free travel.
            ax, ay = cl['_aim']
            aim_along = (
                (ax - pose.x) * math.cos(best_heading) + (ay - pose.y) * math.sin(best_heading)
            )
            target, limit = move_target(
                self.p.explore_step_m, best_travel, aim_along, self.p.grid_aim_stop_margin_m
            )
            keep_limited = best_keep is not None and best_keep[0] < best_free - 1e-6
            if limit == 'free_travel' and keep_limited:
                limit = 'narrow_keepout'
            if self._explore_toward_entrance(pose, best_heading, target + 0.1):
                rejected.append(f'C{k}: toward entrance')
                continue
            choice = (k, cl, best_heading, target, best_travel, aim_along, limit)
            if limit == 'narrow_keepout':
                self.keepout_stats['headings_shortened'] += 1
            break

        if choice is None:
            why = 'no valid frontier' + (f' (rejected: {", ".join(rejected)})' if rejected else '')
            self._explore_log(f'no frontier selected: {why} -- holding', 'warn')
            self._grid_record(now_sec, pose, None, why)
            self._enter_state(State.INSIDE_HOLD, 'explore_no_frontier')
            return

        k, cl, heading, target, travel, aim_along, limit = choice
        rel = math.degrees(normalize_angle(heading - pose.yaw))
        no_move = target < self.p.grid_min_move_m
        self.explore_plan = {
            'sector': f'C{k}', 'heading': heading, 'target': target,
            'start': pose, 'turn_start': now_sec,
        }
        selected = {
            'cluster': k, 'size': cl['size'], 'centroid': cl['centroid'],
            'score': cl['score'], 'frontier_heading_deg': cl['heading_deg'],
            'heading_deg': round(math.degrees(heading), 1), 'relative_deg': round(rel, 1),
            'free_travel_m': round(travel, 2), 'aim_xy': cl['aim_xy'],
            'aim_along_heading_m': round(aim_along, 2), 'target_m': round(target, 2),
            'target_limited_by': limit, 'no_move': no_move,
            'rejected_before': rejected,
        }
        self._explore_log(
            f'===== EXPLORATION STEP {self.explore_step} ===== selected C{k} '
            f'size={cl["size"]} centroid=({cl["centroid"][0]:.2f},{cl["centroid"][1]:.2f}) '
            f'score={cl["score"]:+.3f} heading={math.degrees(heading):+.1f}deg '
            f'relative={rel:+.1f}deg free_travel={travel:.2f}m aim_along={aim_along:.2f}m '
            f'target={target:.2f}m (limited by {limit})'
            + (f' (skipped {", ".join(rejected)})' if rejected else '')
        )
        self._grid_record(now_sec, pose, selected, 'selected')
        if no_move:
            # Already at the aim point: no move, just look again from here.
            self.grid_no_move_count += 1
            self._explore_log(
                f'step {self.explore_step}: target {target:.2f} m < '
                f'{self.p.grid_min_move_m:.2f} m -- not moving, scanning again '
                f'(cycles without progress so far {self.grid_no_move_count})', 'warn'
            )
            self.explore_move = {'achieved': 0.0, 'reason': 'target_close_no_move'}
            self._scan_begin(now_sec)
            return
        self._enter_state(State.EXPLORE_TURN, f'explore_step_{self.explore_step}')

    def handle_explore_turn(self, sectors, now_sec):
        plan = self.explore_plan
        error = normalize_angle(plan['heading'] - self.imu_yaw)
        if abs(math.degrees(error)) <= self.p.entry_turn_tolerance_deg:
            self.publish_velocity(0.0, 0.0)
            self._explore_begin_move(now_sec)
            return
        if now_sec - plan['turn_start'] >= self.p.explore_turn_timeout_sec:
            self.publish_velocity(0.0, 0.0)
            self._explore_log('turn timeout -- scanning here instead', 'warn')
            self.explore_move = {'achieved': 0.0, 'reason': 'turn_timeout'}
            self._scan_begin(now_sec)
            return
        self._entry_rotate_to(plan['heading'])

    def _explore_begin_move(self, now_sec):
        pose = self._entry_pose()
        self.explore_move = {
            'start': pose, 'start_time': now_sec, 'achieved': 0.0, 'reason': '',
            'min_cone': float('inf'), 'last_key': None,
        }
        self._entry_progress_reset(now_sec, 0.0)
        self.last_angular_cmd = 0.0
        self._explore_log(
            f'step {self.explore_step}: turned to {math.degrees(self.imu_yaw):+.1f}deg in '
            f'{now_sec - self.explore_plan["turn_start"]:.1f}s -> moving target '
            f'{self.explore_plan["target"]:.2f}m'
        )
        self._enter_state(State.EXPLORE_MOVE, f'explore_step_{self.explore_step}_move')

    def _explore_end_move(self, now_sec, reason):
        self.publish_velocity(0.0, 0.0)
        m = self.explore_move
        m['reason'] = reason
        pose = self._entry_pose()
        a, lat = self._door_frame(pose)
        self._explore_log(
            f'step {self.explore_step} move done: target={self.explore_plan["target"]:.2f}m '
            f'achieved={m["achieved"]:.2f}m reason={reason} '
            f'duration={now_sec - m["start_time"]:.1f}s '
            f'min_front_cone={m["min_cone"]:.2f}m nav=({pose.x:.2f},{pose.y:.2f}) '
            f'from_entrance a={a:+.2f} lat={lat:+.2f}'
        )
        self._explore_record({
            'step': self.explore_step, 'move_done': True, 't': now_sec,
            'target_m': self.explore_plan['target'], 'achieved_m': m['achieved'],
            'reason': reason, 'duration_s': now_sec - m['start_time'],
            'min_front_cone_m': m['min_cone'], 'nav': [pose.x, pose.y],
        })
        self._scan_begin(now_sec)

    def handle_explore_move(self, sectors, now_sec):
        plan = self.explore_plan
        m = self.explore_move
        if plan.get('sector') == 'VICTIM_LOOK':
            # Turn-only victim look: never translate. Scan from here (its
            # stationary settle faces the cue); not a move, so it is not
            # counted as a progress cycle either way.
            self.publish_velocity(0.0, 0.0)
            self._victim_log(
                f'look turn done at {math.degrees(self.imu_yaw):+.1f}deg -- stationary scan '
                f'settle facing the cue')
            self.explore_move = None
            self._scan_begin(now_sec)
            return
        pose = self._entry_pose()
        start = m['start']
        travelled = (
            (pose.x - start.x) * math.cos(plan['heading'])
            + (pose.y - start.y) * math.sin(plan['heading'])
        )
        m['achieved'] = travelled
        self._explore_visit(pose)
        remaining = plan['target'] - travelled

        cone = self._safety_cone_min()
        m['min_cone'] = min(m['min_cone'], cone)

        if remaining <= 0.0:
            self._explore_end_move(
                now_sec, 'narrow_keepout' if m.get('keepout_cut') else 'target_reached')
            return
        if cone <= self.p.explore_stop_margin_m:
            self._explore_end_move(now_sec, f'front_cone_{cone:.2f}m')
            return

        # Re-check the remaining straight path on every new scan.
        key = self._scan_key()
        if key != m['last_key']:
            m['last_key'] = key
            # Narrow-obstacle keep-out on the raw scan: shorten the move so the
            # robot never gets within the keep-out (never extends it).
            allowed, o = narrow_keepout_travel(
                self._narrow_from_live_scan(), 0.0, remaining, self._keepout_radius(),
                tolerance_m=self.p.narrow_obstacle_tolerance_m)
            if allowed < remaining - 1e-6:
                wx, wy = self._to_world(pose, o['x'], o['y'])
                self._narrow_remember(wx, wy, o['width'])
                if not m.get('keepout_cut'):
                    self.keepout_stats['live_shortened'] += 1
                m['keepout_cut'] = True
                self._explore_log(
                    f'[KEEPOUT] live: narrow obstacle at nav ({wx:.2f},{wy:.2f}) range '
                    f'{o["range"]:.2f} m width {o["width"]:.3f} m -> move cut from '
                    f'{plan["target"]:.2f} m to {travelled + allowed:.2f} m')
                plan['target'] = travelled + allowed
                remaining = allowed
                if allowed <= 0.02:
                    self.keepout_stats['live_stops'] += 1
                    self._explore_end_move(now_sec, 'narrow_keepout')
                    return
            blocker = straight_path_blocker(
                self._scan_points_xy(), remaining,
                self.p.explore_path_half_width_m, self.p.explore_stop_margin_m,
            )
            if blocker is not None:
                self._explore_end_move(
                    now_sec, f'path_blocked_at_x{blocker[0]:.2f}_y{blocker[1]:+.2f}'
                )
                return

        if self._explore_toward_entrance(pose, plan['heading'], 0.0):
            self._explore_end_move(now_sec, 'entrance_guard')
            return
        if self.p.explore_planner == 'grid' and self._grid_in_portal(pose.x, pose.y):
            self._explore_end_move(now_sec, 'left_interior')
            return
        heading_err = normalize_angle(plan['heading'] - pose.yaw)
        if abs(math.degrees(heading_err)) > self.p.explore_move_heading_tol_deg:
            self._explore_end_move(now_sec, f'heading_dev_{math.degrees(heading_err):+.1f}')
            return
        if self._entry_progress_stalled(now_sec, travelled):
            self._explore_end_move(now_sec, 'no_progress')
            return
        timeout = 2.0 * plan['target'] / self.p.explore_speed + 5.0
        if now_sec - m['start_time'] >= timeout:
            self._explore_end_move(now_sec, 'timeout')
            return

        speed = min(self.p.explore_speed, 0.5 * remaining + 0.03)
        angular = clamp(self.p.entry_heading_kp * heading_err, -0.3, 0.3)
        self.publish_velocity(speed, angular)

    # =====================================================================
    # VICTIM APPROACH (victim_approach_enabled)
    # =====================================================================
    #
    # /victim_distance readings become nav-frame positions (current nav pose;
    # only inside the house, only in the exploration states, only while not
    # rotating) in VictimMemory. Nothing happens until the post-scan decision
    # point (_explore_after_scan): a confirmed victim then replaces the next
    # frontier and is approached with an ordinary EXPLORE_TURN / EXPLORE_MOVE
    # plan, so each approach move keeps the live narrow keep-out, the path
    # blocker / front-cone stops, the progress watchdog, the following scan's
    # pre-rotation check and escape, the tilt guard and the SafetyController
    # (/cmd_vel_raw). Reached -> VICTIM_HOLD (terminal, /victim_found).

    def _victim_log(self, msg, level='info'):
        self._entry_log(f'[VICTIM] {msg}', level)

    def _victim_record(self, payload):
        self._scan_record('victim_events.jsonl', payload)

    def _victim_observing(self):
        if self.mission_phase != 'INSIDE' or self.victim_report is not None:
            return False
        if self.nav_pose is None or self.imu_yaw is None:
            return False
        if self.state in (State.EXPLORE_TURN, State.EXPLORE_MOVE):
            return True
        return self.state == State.STOP_AND_SCAN and self.entry_phase in ('SETTLE', 'STOPPING')

    def victim_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            data = None
        reading = victim_measurement(data) if isinstance(data, dict) else None
        if reading is None:
            self.victim_stats['rejected_invalid'] += 1
            return
        if self._victim_cue_state() and (
                (self.state == State.STOP_AND_SCAN and self.entry_phase == 'ROTATE')
                or self._imu_rate_deg_s(1.0) > VICTIM_STILL_RATE_DEG_S):
            # Rejected only because the robot is turning: bearing cue only.
            self.victim_stats['rejected_rotating'] += 1
            self._victim_add_cue(reading)
            return
        if not self._victim_observing():
            self.victim_stats['rejected_state'] += 1
            return
        if self._imu_rate_deg_s(1.0) > VICTIM_STILL_RATE_DEG_S:
            self.victim_stats['rejected_rotating'] += 1
            return
        distance, angle, confidence = reading
        now_sec = self._now_sec()
        pose = self._entry_pose()
        x, y = victim_nav_position(pose, distance, angle, self.p.lidar_offset_x_m)
        self.victim_stats['accepted'] += 1
        target = self.victim_target
        if target is not None and math.hypot(
                x - target['track'].x, y - target['track'].y) <= self.p.victim_confirm_radius_m:
            target['track'].add(now_sec, x, y, confidence, distance)
            where, track = 'target', target['track']
        else:
            track = self.victim_memory.observe(now_sec, x, y, confidence, distance)
            where = 'candidate'
        self.get_logger().info(
            f't={now_sec:.2f} [VICTIM] reading {distance:.2f} m {math.degrees(angle):+.1f}deg '
            f'conf {confidence:.2f} -> nav ({x:.2f},{y:.2f}) {where} '
            f'({track.x:.2f},{track.y:.2f}) n={track.count} in {self.state.name}',
            throttle_duration_sec=2.0)
        self._victim_record({
            't': now_sec, 'event': 'reading', 'state': self.state.name,
            'distance_m': distance, 'angle_deg': math.degrees(angle), 'confidence': confidence,
            'nav': [pose.x, pose.y, math.degrees(pose.yaw)], 'victim_nav': [x, y],
            'assigned': where, 'track': track.as_dict()})

    def _victim_cue_state(self):
        """Return True in the interior exploration states (cue-eligible)."""
        return (self.mission_phase == 'INSIDE' and self.victim_report is None
                and self.nav_pose is not None and self.imu_yaw is not None
                and self.state in (State.STOP_AND_SCAN, State.EXPLORE_TURN,
                                   State.EXPLORE_MOVE))

    def _victim_add_cue(self, reading):
        """Store a turning reading as a direction (never a victim position)."""
        distance, angle, confidence = reading
        now_sec = self._now_sec()
        pose = self._entry_pose()
        x, y = victim_nav_position(pose, distance, angle, self.p.lidar_offset_x_m)
        cue = self.victim_cues.observe(now_sec, x, y, confidence, distance)
        self.victim_stats['cues'] += 1
        bearing = math.degrees(normalize_angle(pose.yaw + angle))
        self.get_logger().info(
            f't={now_sec:.2f} [VICTIM] cue (turning) {distance:.2f} m bearing {bearing:+.1f}deg '
            f'conf {confidence:.2f} -> group ({cue.x:.2f},{cue.y:.2f}) n={cue.count} '
            f'in {self.state.name}', throttle_duration_sec=2.0)
        self._victim_record({
            't': now_sec, 'event': 'cue', 'state': self.state.name, 'phase': self.entry_phase,
            'distance_m': distance, 'angle_deg': math.degrees(angle), 'bearing_deg': bearing,
            'confidence': confidence, 'nav': [pose.x, pose.y, math.degrees(pose.yaw)],
            'cue_nav': [x, y], 'group': cue.as_dict()})

    def _victim_try_look(self, now_sec, pose, group):
        """
        Start ONE turn-only look toward an agreeing cue group.

        Returns True when an action started (the look turn, or the existing
        keep-out escape / hold if an in-place turn here is not safe).
        """
        if group is None:
            return False
        self.victim_looked.append((group.x, group.y))
        self.victim_stats['looks'] += 1
        heading = math.atan2(group.y - pose.y, group.x - pose.x)
        rel = math.degrees(normalize_angle(heading - pose.yaw))
        self._victim_record({
            't': now_sec, 'event': 'look', 'nav': [pose.x, pose.y, math.degrees(pose.yaw)],
            'heading_deg': math.degrees(heading), 'relative_deg': rel, 'group': group.as_dict(),
            'spread_m': group.spread()})
        if not self._keepout_before_rotation(now_sec):
            self._victim_log(
                f'look toward cue ({group.x:.2f},{group.y:.2f}) refused: in-place turn not safe '
                f'here (keep-out escape / hold started)', 'warn')
            return True
        self.explore_plan = {
            'sector': 'VICTIM_LOOK', 'heading': heading, 'target': 0.0,
            'start': pose, 'turn_start': now_sec,
        }
        self.explore_move = None
        self._victim_log(
            f'look {self.victim_stats["looks"]}: {group.count} agreeing turning cues at nav '
            f'({group.x:.2f},{group.y:.2f}) (spread {group.spread():.2f} m) -> turn in place to '
            f'{math.degrees(heading):+.1f}deg ({rel:+.1f}deg), then a stationary scan settle')
        self._enter_state(State.EXPLORE_TURN, 'victim_look')
        return True

    def _victim_range(self, pose, track):
        """Return the LiDAR range to the victim estimate once facing it."""
        return math.hypot(track.x - pose.x, track.y - pose.y) - self.p.lidar_offset_x_m

    def _victim_abandon(self, now_sec, pose, why):
        track = self.victim_target['track'] if self.victim_target else None
        if track is not None:
            self.victim_abandoned.append((track.x, track.y))
        self._victim_log(f'target abandoned: {why} -- back to frontier exploration', 'warn')
        self._victim_record({'t': now_sec, 'event': 'abandoned', 'reason': why,
                             'nav': [pose.x, pose.y],
                             'track': track.as_dict() if track else None})
        self.victim_target = None
        self.victim_priority = None
        if not self.victim_prioritizing:
            self.victim_memory.clear()
        # Prioritizing: the other candidates stay (ranked at the next decision).

    def _victim_after_scan(self, now_sec, pose, data):
        """
        Run the victim decision after a complete scan.

        True = a victim action was started (approach move or VICTIM_HOLD);
        False = explore as usual.
        """
        if self.victim_report is not None:
            # Already reached (a sensor fault interrupted the hold and the
            # recovery scan landed here): never resume exploration.
            self._victim_enter_hold(now_sec, pose, 'victim_already_reached')
            return True
        # Cues live for one decision only: evaluated here, then cleared (no
        # stale or repeated looks); groups near a place already looked at or
        # abandoned are not eligible (one look per cue location).
        if self.victim_prioritizing:
            cue_group = self._victim_rank_cues(now_sec, pose)
        else:
            cue_group = self.victim_cues.confirmed(
                now_sec, self.victim_abandoned + self.victim_looked)
        self.victim_cues.clear()
        target = self.victim_target
        if target is None:
            if self.victim_prioritizing:
                track = self._victim_prioritize(now_sec, pose, data)
            else:
                track = self.victim_memory.confirmed(now_sec, self.victim_abandoned)
            if track is None:
                return self._victim_try_look(now_sec, pose, cue_group)
            self.victim_memory.tracks.remove(track)
            target = self.victim_target = {
                'track': track, 'missed': 0, 'moves': 0, 'checked': now_sec,
                'offsets_used': []}
            self._victim_log(
                f'CONFIRMED at nav ({track.x:.2f},{track.y:.2f}) after {track.count} readings '
                f'(spread {track.spread():.2f} m, conf {track.confidence:.2f}) -- takes '
                f'priority over the next frontier')
            self._victim_record({'t': now_sec, 'event': 'confirmed', 'nav': [pose.x, pose.y],
                                 'track': track.as_dict()})
        else:
            track = target['track']
            if track.last_time > target['checked']:
                target['missed'] = 0
            else:
                rng = self._victim_range(pose, track)
                if rng <= VICTIM_CLOSE_RANGE_M:
                    # The low wide-angle camera may lose the person this close.
                    self._victim_enter_hold(
                        now_sec, pose, f'not seen at close range {rng:.2f} m')
                    return True
                target['missed'] += 1
                self._victim_log(
                    f'not seen since the last decision ({target["missed"]}/'
                    f'{self.p.victim_lost_scan_limit})', 'warn')
                if target['missed'] >= self.p.victim_lost_scan_limit:
                    self._victim_abandon(
                        now_sec, pose, f'not seen in {target["missed"]} approach scans')
                    return False
            target['checked'] = now_sec
            if self.victim_prioritizing and target['moves'] == 0:
                # Safe decision point before the first approach move (e.g. the
                # previous turn was refused by the pre-rotation check): a
                # clearly better candidate (>= switch margin) may replace it.
                target = self._victim_maybe_switch(now_sec, pose, data, target)

        track = target['track']
        stop = self.p.victim_stop_distance_m
        rng = self._victim_range(pose, track)
        if rng - stop <= VICTIM_REACHED_TOL_M:
            self._victim_enter_hold(now_sec, pose, f'range {rng:.2f} m <= {stop:.2f} m')
            return True
        if target['moves'] >= VICTIM_MAX_APPROACH_MOVES:
            self._victim_abandon(now_sec, pose, f'{target["moves"]} approach moves used')
            return False

        # Approach headings in VICTIM_HEADING_OFFSETS_DEG order; the first one
        # that passes every check is used. The direct bearing (offset 0) is
        # checked exactly as before (same straight-path and keep-out limits as
        # the frontier planner); an offset heading must pass those same checks
        # and, in addition, the live scan, the entrance guard and (before the
        # turn) the existing pre-rotation check. Each offset heading is used
        # at most once per victim. The target stays the confirmed victim: the
        # next decision re-checks it and recomputes the bearing from there.
        bearing = math.atan2(track.y - pose.y, track.x - pose.x)
        points = [
            (r * math.cos(math.radians(k + 0.5)), r * math.sin(math.radians(k + 0.5)))
            for k, r in enumerate(data['analysis']['profile']) if math.isfinite(r)
        ]
        plan_half = self.p.explore_path_half_width_m + self.p.grid_plan_path_margin_m
        candidates = []
        chosen = None
        for offset in VICTIM_HEADING_OFFSETS_DEG:
            cand = self._victim_candidate(pose, rng, bearing, offset, points, plan_half, target)
            candidates.append(cand)
            if cand['result'] == 'reached':
                self._victim_enter_hold(now_sec, pose, f'range {rng:.2f} m')
                return True
            if cand['result'] == 'ok':
                chosen = cand
                break
        self._victim_record({
            't': now_sec, 'event': 'approach_candidates', 'nav': [pose.x, pose.y],
            'track': track.as_dict(), 'range_m': rng, 'bearing_deg': math.degrees(bearing),
            'candidates': candidates})
        if chosen is None:
            self._victim_abandon(
                now_sec, pose, f'no safe approach heading (range {rng:.2f} m; '
                + '; '.join(f'{c["offset_deg"]:+.0f}deg {c["result"]}' for c in candidates)
                + ')')
            return False
        heading = math.radians(chosen['heading_deg'])
        move, free, keep = chosen['move_m'], chosen['free_m'], chosen['keepout_m']
        if chosen['offset_deg'] != 0.0:
            target['offsets_used'].append(chosen['offset_deg'])
            if not self._keepout_before_rotation(now_sec):
                # Position-level refusal (same for every heading): the
                # existing keep-out escape / hold takes over.
                self._victim_log(
                    f'approach heading {chosen["offset_deg"]:+.0f}deg: in-place turn refused '
                    f'here (keep-out escape / hold started)', 'warn')
                return True

        target['moves'] += 1
        self.explore_plan = {
            'sector': 'VICTIM', 'heading': heading, 'target': move,
            'start': pose, 'turn_start': now_sec, 'offset_deg': chosen['offset_deg'],
        }
        via = (f' via approach heading {chosen["offset_deg"]:+.0f}deg (direct blocked)'
               if chosen['offset_deg'] != 0.0 else '')
        self._victim_log(
            f'approach move {target["moves"]}: victim nav ({track.x:.2f},{track.y:.2f}) range '
            f'{rng:.2f} m heading {math.degrees(heading):+.1f}deg -> move {move:.2f} m '
            f'(free {free:.2f} m, keep-out {keep:.2f} m, stop at {stop:.2f} m){via}')
        self._victim_record({
            't': now_sec, 'event': 'approach_move', 'n': target['moves'],
            'nav': [pose.x, pose.y], 'track': track.as_dict(), 'range_m': rng,
            'heading_deg': math.degrees(heading), 'move_m': move, 'free_m': free,
            'keepout_m': keep, 'offset_deg': chosen['offset_deg']})
        self._enter_state(State.EXPLORE_TURN, f'victim_approach_{target["moves"]}')
        return True

    def _victim_candidate(self, pose, rng, bearing, offset_deg, points, plan_half, target,
                          quiet=False):
        """
        Check one approach heading (offset_deg off the victim bearing).

        Returns a record whose 'result' is 'ok' (move_m may be driven),
        'reached' (direct only) or the rejection reason. Nothing moves here;
        quiet=True (prioritizer scoring) only skips the log line.
        """
        heading = normalize_angle(bearing + math.radians(offset_deg))
        stop = self.p.victim_stop_distance_m
        free = self._explore_free_travel(points, heading, plan_half)
        keep, _ = narrow_keepout_travel(
            self.narrow_plan, heading, self.p.explore_step_m, self._keepout_radius(),
            tolerance_m=self.p.narrow_obstacle_tolerance_m)
        cand = {'offset_deg': offset_deg, 'heading_deg': math.degrees(heading),
                'free_m': free, 'keepout_m': keep, 'move_m': 0.0}
        direct = offset_deg == 0.0
        if not direct and offset_deg in target['offsets_used']:
            cand['result'] = 'already_used'
            return self._victim_candidate_log(cand, quiet)
        wanted = victim_offset_wanted(rng, math.radians(offset_deg), stop)
        cand['wanted_m'] = wanted
        move, outcome = victim_approach_move(
            stop + wanted, stop, free, keep, self.p.grid_min_step_m)
        if outcome == 'reached':
            cand['result'] = 'reached' if direct else 'no_progress'
            return self._victim_candidate_log(cand, quiet)
        if outcome == 'blocked':
            cand['result'] = ('corridor_blocked' if free < min(self.p.grid_min_step_m, wanted)
                              else 'narrow_keepout')
            return self._victim_candidate_log(cand, quiet)
        cand['move_m'] = move
        if self._explore_toward_entrance(pose, heading, move + 0.1):
            cand['result'] = 'entrance'
            return self._victim_candidate_log(cand, quiet)
        if not direct:
            live = self._victim_live_check(pose, heading, move)
            cand.update(live)
            if live['live'] != 'ok':
                cand['result'] = live['live']
                return self._victim_candidate_log(cand, quiet)
        cand['result'] = 'ok'
        return self._victim_candidate_log(cand, quiet)

    def _victim_candidate_log(self, cand, quiet=False):
        if quiet:
            return cand
        extra = ''
        if 'live_free_m' in cand:
            extra = (f' live free {cand["live_free_m"]:.2f} m, live keep-out '
                     f'{cand["live_keepout_m"]:.2f} m,')
        self._victim_log(
            f'approach heading {cand["offset_deg"]:+.0f}deg ({cand["heading_deg"]:+.1f}deg): '
            f'free {cand["free_m"]:.2f} m, keep-out {cand["keepout_m"]:.2f} m,{extra} '
            f'move {cand["move_m"]:.2f} m -> {cand["result"]}')
        return cand

    def _victim_live_check(self, pose, heading, move):
        """
        Check an offset approach heading against the latest raw scan.

        The same checks the move itself runs on every new scan
        (handle_explore_move): live narrow keep-out and straight-path
        blocker, so a heading the move would stop on at once is rejected.
        """
        if self.latest_scan is None or not self._is_scan_fresh(
                self.get_clock().now().nanoseconds):
            return {'live': 'stale_scan'}
        rel = normalize_angle(heading - pose.yaw)
        c, s_ = math.cos(rel), math.sin(rel)
        pts = [(x * c + y * s_, -x * s_ + y * c) for x, y in self._scan_points_xy()]
        blocker = straight_path_blocker(
            pts, move, self.p.explore_path_half_width_m, self.p.explore_stop_margin_m)
        live_keep, _ = narrow_keepout_travel(
            self._narrow_from_live_scan(), rel, move, self._keepout_radius(),
            tolerance_m=self.p.narrow_obstacle_tolerance_m)
        ahead = [x for x, y in pts if x > 0.0 and abs(y) < self.p.explore_path_half_width_m]
        live = {'live_free_m': max(0.0, min(ahead, default=9.9) - self.p.explore_stop_margin_m),
                'live_keepout_m': live_keep, 'live': 'ok'}
        if blocker is not None:
            live['live'] = 'live_blocked'
        elif live_keep < move - 1e-6:
            live['live'] = 'live_narrow_keepout'
        return live

    # ---------------------------------------------------------------------
    # Victim prioritizer (victim_prioritization_enabled). Runs only from
    # _victim_after_scan (stationary, after a complete scan, inside): it
    # scores candidates with the existing approach checks (quiet, nothing
    # moves) and returns a track; the approach code above stays unchanged.
    # ---------------------------------------------------------------------

    def _victim_assess(self, pose, data, track):
        """
        Score one confirmed track (approach checks only; nothing moves).

        The live-scan check of an offset heading runs the narrow-obstacle
        detector, which counts structure-filtered detections in
        keepout_stats; those counters are restored afterwards so scoring
        leaves the keep-out diagnostics exactly as the approach alone would.
        """
        saved_stats = dict(self.keepout_stats)
        try:
            return self._victim_assess_checks(pose, data, track)
        finally:
            self.keepout_stats.clear()
            self.keepout_stats.update(saved_stats)

    def _victim_assess_checks(self, pose, data, track):
        stop = self.p.victim_stop_distance_m
        rng = self._victim_range(pose, track)
        dx, dy = track.x - pose.x, track.y - pose.y
        bearing = math.atan2(dy, dx)
        points = [
            (r * math.cos(math.radians(k + 0.5)), r * math.sin(math.radians(k + 0.5)))
            for k, r in enumerate(data['analysis']['profile']) if math.isfinite(r)
        ]
        plan_half = self.p.explore_path_half_width_m + self.p.grid_plan_path_margin_m
        scratch = {'offsets_used': []}
        access = None
        results = []
        for offset in VICTIM_HEADING_OFFSETS_DEG:
            cand = self._victim_candidate(
                pose, rng, bearing, offset, points, plan_half, scratch, quiet=True)
            results.append(f'{offset:+.0f}:{cand["result"]}')
            if cand['result'] in ('ok', 'reached'):
                access = offset
                break
        # Risk is measured on the direct robot -> stop-point line (whatever
        # heading passed above); see victim_prioritizer.risk_score.
        seg = max(0.0, math.hypot(dx, dy) - stop)
        risk = victim_risk(
            [(o['x'], o['y']) for o in self.narrow_plan],
            seg * math.cos(bearing), seg * math.sin(bearing), self._keepout_radius(),
            victim_xy=(dx, dy), person_radius_m=self.p.victim_confirm_radius_m)
        return VictimCandidate(track.id, track.mean_confidence(), rng, access, risk, stop), results

    def _victim_rank_record(self, now_sec, pose, kind, scored, chosen, reason):
        ranked = victim_rank([c for c, _ in scored])
        checks = {c.cid: res for c, res in scored}
        self._victim_record({
            't': now_sec, 'event': kind, 'nav': [pose.x, pose.y],
            'chosen': chosen.cid if chosen else None, 'reason': reason,
            'ranking': [dict(c.as_dict(), checks=checks[c.cid]) for c in ranked]})
        self._victim_log(
            f'{kind}: ' + ', '.join(
                f'#{c.cid} {c.score:.3f}{"" if c.feasible else " (no safe heading)"}'
                for c in ranked)
            + f' -> {"#" + str(chosen.cid) if chosen else "none"} ({reason})')

    def _victim_prioritize(self, now_sec, pose, data):
        """Return the best-ranked confirmed track that has a safe heading, or None."""
        tracks = self.victim_memory.confirmed_all(now_sec, self.victim_abandoned)
        if not tracks:
            return None
        scored = [self._victim_assess(pose, data, tr) for tr in tracks]
        chosen, reason = victim_choose([c for c, _ in scored])
        self._victim_rank_record(now_sec, pose, 'prioritized', scored, chosen, reason)
        if chosen is None:
            return None
        self.victim_priority = {'score': chosen.score, 'considered': len(scored)}
        return next(tr for tr in tracks if tr.id == chosen.cid)

    def _victim_maybe_switch(self, now_sec, pose, data, target):
        """
        Re-rank before the first approach move; returns the (possibly new) target.

        Called only while target['moves'] == 0: the target lock (no switch
        once an approach move has started) is enforced by that caller guard
        in _victim_after_scan, so choose() is never asked with locked=True here.
        """
        current = target['track']
        others = self.victim_memory.confirmed_all(
            now_sec, self.victim_abandoned + [(current.x, current.y)])
        if not others:
            return target
        tracks = [current] + others
        scored = [self._victim_assess(pose, data, tr) for tr in tracks]
        chosen, reason = victim_choose([c for c, _ in scored], current_id=current.id)
        self._victim_rank_record(now_sec, pose, 'reprioritized', scored, chosen, reason)
        if chosen is None or chosen.cid == current.id:
            return target
        new = next(tr for tr in others if tr.id == chosen.cid)
        self.victim_memory.tracks.remove(new)
        self.victim_memory.tracks.append(current)          # back to a candidate, not abandoned
        self.victim_priority = {'score': chosen.score, 'considered': len(scored)}
        self._victim_log(
            f'target switched #{current.id} -> #{new.id} ({reason}) at nav '
            f'({new.x:.2f},{new.y:.2f}) before any approach move')
        self._victim_record({'t': now_sec, 'event': 'switched', 'from': current.id,
                             'to': new.id, 'reason': reason, 'track': new.as_dict()})
        target = self.victim_target = {
            'track': new, 'missed': 0, 'moves': 0, 'checked': now_sec, 'offsets_used': []}
        return target

    def _victim_rank_cues(self, now_sec, pose):
        """Return the best cue group for the one-turn look (confidence and distance only)."""
        groups = self.victim_cues.confirmed_all(
            now_sec, self.victim_abandoned + self.victim_looked)
        if not groups:
            return None
        stop = self.p.victim_stop_distance_m
        scored = [(VictimCandidate(g.id, g.mean_confidence(), self._victim_range(pose, g),
                                   0.0, 0.0, stop), ['cue']) for g in groups]
        chosen, reason = victim_choose([c for c, _ in scored])
        if len(groups) > 1:
            self._victim_rank_record(now_sec, pose, 'cues_prioritized', scored, chosen, reason)
        return next(g for g in groups if g.id == chosen.cid)

    def _victim_publish_found(self, now_sec):
        if self.victim_found_publisher is not None:
            msg = String()
            msg.data = json.dumps(self.victim_report)
            self.victim_found_publisher.publish(msg)
        self.victim_hold['last_publish'] = now_sec

    def _victim_enter_hold(self, now_sec, pose, reason):
        self.publish_velocity(0.0, 0.0)
        if self.victim_report is None:
            track = self.victim_target['track']
            self.victim_report = {
                'victim_x': round(track.x, 3),
                'victim_y': round(track.y, 3),
                'frame': 'nav: odometry distance along the IMU heading, origin = spawn pose '
                         '(dead-reckoned estimate, not ground truth)',
                'distance_m': round(self._victim_range(pose, track), 3),
                'last_measured_distance_m': round(track.distance, 3),
                'confidence': round(track.confidence, 3),
                'observations': track.count,
                'robot_x': round(pose.x, 3),
                'robot_y': round(pose.y, 3),
                'timestamp': round(now_sec, 3),
                'state': 'VICTIM_HOLD',
                'reason': reason,
            }
            if self.victim_prioritizing:
                # Extra keys only; the existing ones are unchanged. Note:
                # 'confidence' above stays the max over readings, while
                # priority_score was computed from the mean confidence.
                prio = self.victim_priority or {}
                self.victim_report['candidate_id'] = track.id
                self.victim_report['priority_score'] = prio.get('score')
                self.victim_report['candidates_considered'] = prio.get('considered', 1)
        vx, vy = self.victim_report['victim_x'], self.victim_report['victim_y']
        # Facing turn: in place, right after a complete 360-degree scan at this
        # same spot (its pre-rotation keep-out check passed).
        self.victim_hold = {'heading': math.atan2(vy - pose.y, vx - pose.x),
                            'start': now_sec, 'facing': True, 'last_publish': now_sec}
        self._victim_publish_found(now_sec)
        self._victim_log(f'REACHED ({reason}): /victim_found {json.dumps(self.victim_report)}')
        self._victim_record({'t': now_sec, 'event': 'victim_hold', 'report': self.victim_report,
                             'stats': self.victim_stats})
        self._enter_state(State.VICTIM_HOLD, reason)

    def handle_victim_hold(self, sectors, now_sec):
        hold = self.victim_hold
        if hold['facing']:
            error = normalize_angle(hold['heading'] - self.imu_yaw)
            if (abs(math.degrees(error)) > self.p.entry_turn_tolerance_deg and
                    now_sec - hold['start'] < VICTIM_FACE_TIMEOUT_S and
                    self._is_imu_fresh(self.get_clock().now().nanoseconds)):
                self._entry_rotate_to(hold['heading'])
                return
            hold['facing'] = False
            self._victim_log(
                f'holding: ready for rescue (heading error to victim '
                f'{math.degrees(error):+.1f}deg)')
        self.publish_velocity(0.0, 0.0)
        if now_sec - hold['last_publish'] >= VICTIM_REPUBLISH_S:
            self._victim_publish_found(now_sec)

    # =====================================================================
    # STATE: FAULT_HOLD
    # =====================================================================

    def _handle_fault_hold(self, scan_ok, odom_ok, now_sec):
        self.publish_velocity(0.0, 0.0)

        if self.fault_terminal:
            self.get_logger().warning(
                'FAULT_HOLD is terminal (recovery exhausted) — restart the '
                'node to resume.',
                throttle_duration_sec=5.0,
            )
            return

        if scan_ok and odom_ok:
            if self.fault_fresh_since is None:
                self.fault_fresh_since = now_sec
            if now_sec - self.fault_fresh_since >= self.p.sensor_fresh_confirm_sec:
                self.fault_fresh_since = None
                if self.mission_phase == 'INSIDE':
                    # Inside the house never fall back to the old reactive
                    # exploration (it can wander back out of the entrance):
                    # re-scan from here, or hold.
                    if self.explore_move is not None and not self.explore_move['reason']:
                        self.explore_move['reason'] = 'interrupted_by_fault'
                    if self.p.explore_enabled and self.nav_pose is not None:
                        self._explore_log(
                            'sensors recovered inside -> fresh STOP_AND_SCAN', 'warn'
                        )
                        self._scan_begin(now_sec)
                    else:
                        self._enter_state(State.INSIDE_HOLD, 'sensors_recovered_inside')
                    return
                self._enter_state(State.EXPLORING, 'sensors_recovered')
        else:
            self.fault_fresh_since = None

    # =====================================================================
    # MAIN CONTROL LOOP
    # =====================================================================

    def control_loop(self):
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9

        if self.latest_pose is not None:
            self._update_path_length()
            self._maybe_record_visited()

        scan_ok = self._is_scan_fresh(now.nanoseconds)
        odom_ok = self._is_odom_fresh(now.nanoseconds)

        if self.state == State.STARTUP:
            self.publish_velocity(0.0, 0.0)
            if scan_ok and odom_ok:
                self._enter_state(State.EXPLORING, 'sensors_ready')
            else:
                self.get_logger().warning(
                    'Waiting for fresh /scan and /odom...',
                    throttle_duration_sec=5.0,
                )
            return

        if self.tilt_latched is not None and not (
                self.state == State.FAULT_HOLD and self.fault_terminal):
            self.publish_velocity(0.0, 0.0)
            self._enter_fault_hold('tilt_guard', terminal=True)
            return

        if self.state == State.FAULT_HOLD:
            self._handle_fault_hold(scan_ok, odom_ok, now_sec)
            return

        if not scan_ok or not odom_ok:
            self._enter_fault_hold('scan_stale' if not scan_ok else 'odom_stale', terminal=False)
            self.publish_velocity(0.0, 0.0)
            return

        sectors = self._compute_sectors()

        # An entry attempt interrupted by a fault resumes from the persisted
        # entrance once the robot is back in the ordinary states.
        if self.mission_phase == 'ENTERING' and self.state not in ENTRY_STATES:
            self._set_mission_phase('OUTSIDE_SEARCH', f'entry_interrupted_in_{self.state.name}')

        imu_ok = self._is_imu_fresh(now.nanoseconds) and self.nav_pose is not None

        if self.state in ENTRY_STATES:
            if not imu_ok:
                self.publish_velocity(0.0, 0.0)
                self.get_logger().warning(
                    'Entry states need fresh /imu heading -- holding.',
                    throttle_duration_sec=2.0,
                )
                return

        elif (self.mission_phase == 'OUTSIDE_SEARCH' and imu_ok and
                self.state != State.CROSSING_OPENING):
            if self._entry_outside_search_tick(now_sec):
                self._log_nav(now_sec)
                return

        self._handlers[self.state](sectors, now_sec)

        self._update_stuck_tracking(now_sec)

        self._log_nav(now_sec)

    # =====================================================================
    # SHUTDOWN
    # =====================================================================

    def shutdown(self):
        try:
            self.publish_velocity(0.0, 0.0)
        except Exception:
            pass


def main(args=None):

    rclpy.init(args=args)

    node = AutonomousSearch()

    try:
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        node.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
