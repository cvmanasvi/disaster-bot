"""Earthquake disaster-site layout: single source of truth (world frame, metres).

Every static obstacle is a box extruded from the floor (yaw-only rotation), so
the LiDAR plane (z = 0.171 m) cuts every obstacle at its full footprint: there
is nothing low enough to hide under the scan, and nothing overhangs the robot.
generate_world.py writes the model SDFs / world from this file and
check_geometry.py measures clearances on the same boxes.

Building: 8.0 x 7.2 m, outer wall centre lines x = 0 / 8, y = 0 / 7.2.
Robot starts OUTSIDE (south yard) like the house tests and has to enter
through the facade doorway.

    y=7.2 +---------------------+-------------------------------+
          |  C  collapsed room  |  D  victim room     [pile] V  |
          |  (dead end)         |                               |
    y=4.8 +--- B->C ---~~~~~~~~-+ alc +------ D door -----------+
          |  B  west room       | cor |  E  east hall           |
          |                     | rid |  ====fallen beam====    |
    y=2.6 +-- open --------- ---+ or  +-- narrow ---- wide -----+
          |  A  lobby                    [island]               |
    y=0   +--------- DOOR ----------- [blocked door] -----------+
         x=0       2.0  2.9          5.6  6.5                  8
"""

import math

T = 0.15            # wall thickness (centred on the wall line)
H = 2.0             # intact wall height
LIDAR_Z = 0.171

SPAWN = (0.0, -1.2, 0.0)          # x, y, yaw (outside, facing +x along the facade)
VICTIM = (7.35, 6.65, 0.0)        # person model origin (static)

# colours (r, g, b)
CONCRETE = (0.58, 0.57, 0.54)
CONCRETE_DMG = (0.40, 0.39, 0.36)
RUBBLE = [(0.50, 0.48, 0.44), (0.36, 0.34, 0.31), (0.62, 0.58, 0.50), (0.28, 0.26, 0.24),
          (0.47, 0.41, 0.33)]
DARK = (0.16, 0.15, 0.14)
WOOD = (0.42, 0.27, 0.12)
FLOOR = (0.40, 0.38, 0.34)


def box(name, cx, cy, sx, sy, h, yaw_deg=0.0, color=CONCRETE):
    return {'name': name, 'cx': cx, 'cy': cy, 'sx': sx, 'sy': sy, 'h': h,
            'yaw': math.radians(yaw_deg), 'color': color}


def hwall(name, y, x0, x1, h=H, color=CONCRETE):
    return box(name, 0.5 * (x0 + x1), y, x1 - x0, T, h, 0.0, color)


def vwall(name, x, y0, y1, h=H, color=CONCRETE):
    return box(name, x, 0.5 * (y0 + y1), T, y1 - y0, h, 0.0, color)


def jagged_h(name, y, x0, x1, heights, color=CONCRETE_DMG):
    """Broken wall span: equal pieces with different heights (all floor-based)."""
    n = len(heights)
    w = (x1 - x0) / n
    return [hwall(f'{name}_{k}', y, x0 + k * w, x0 + (k + 1) * w, h, color)
            for k, h in enumerate(heights)]


O = 0.5 * T  # 0.075: half wall thickness

# --------------------------------------------------------------------------
# damaged_room: building shell + interior partitions (openings = span gaps)
# --------------------------------------------------------------------------
DAMAGED_ROOM = [
    # south facade y=0: doorway x 2.0-2.9 (0.90 clear), blocked doorway x 5.6-6.5
    hwall('facade_w', 0.0, -O, 2.0),
    hwall('facade_mid', 0.0, 2.9, 5.6),
    hwall('facade_e', 0.0, 6.5, 8.0 + O),
    hwall('north', 7.2, -O, 8.0 + O),
    vwall('west', 0.0, O, 7.2 - O),
    vwall('east', 8.0, O, 7.2 - O),
    # lobby north wall y=2.6: B opening x 0.5-1.5, corridor mouth 3.375-4.125,
    # narrow passage x 4.6-5.3 (0.70 clear), wide opening x 6.6-7.6 (1.0 clear)
    *jagged_h('lobby_n_stub', 2.6, O, 0.5, [1.1, 0.7]),
    *jagged_h('lobby_n_b', 2.6, 1.5, 1.8, [0.8, 1.4]),
    hwall('lobby_n_1', 2.6, 1.8, 3.3 + O),
    hwall('lobby_n_2', 2.6, 4.2 - O, 4.6),
    hwall('lobby_n_3', 2.6, 5.3, 6.6),
    hwall('lobby_n_4', 2.6, 7.6, 8.0 - O),
    # corridor walls (B | corridor | E), y 2.675-4.725
    vwall('corr_w', 3.3, 2.6 + O, 4.8 - O),
    vwall('corr_e', 4.2, 2.6 + O, 4.8 - O),
    # y=4.8 wall: B->C door x 1.2-2.1, collapsed section x 2.4-3.375,
    # corridor top 3.375-4.125 (alcove from D), D door x 5.8-6.7 (0.90 clear)
    hwall('mid_n_1', 4.8, O, 1.2),
    hwall('mid_n_2', 4.8, 2.1, 2.4),
    *jagged_h('mid_n_collapsed', 4.8, 2.4, 3.3 + O, [0.9, 0.55, 0.7, 0.5]),
    hwall('mid_n_3', 4.8, 4.2 - O, 5.8),
    hwall('mid_n_4', 4.8, 6.7, 8.0 - O),
    # C | D partition
    vwall('cd', 3.3, 4.8 + O, 7.2 - O),
]

# --------------------------------------------------------------------------
# collapsed_wall: rubble choking the corridor (blocked passage) and the
# blocked facade doorway, plus fallen wall chunks
# --------------------------------------------------------------------------
COLLAPSED_WALL = [
    # corridor choke (corridor clear width 3.375-4.125), y ~3.2-4.0
    box('corr_fill_1', 3.56, 3.42, 0.42, 0.50, 0.85, 14, RUBBLE[1]),
    box('corr_fill_2', 3.95, 3.50, 0.42, 0.62, 1.05, -10, RUBBLE[0]),
    box('corr_fill_3', 3.74, 3.92, 0.62, 0.36, 0.70, 6, RUBBLE[3]),
    box('corr_fill_4', 3.72, 3.20, 0.50, 0.30, 0.45, -20, RUBBLE[2]),
    # blocked facade doorway x 5.6-6.5: fill flush with the wall line + pile behind
    box('door_fill_1', 5.79, 0.00, 0.38, 0.30, 0.95, 8, RUBBLE[0]),
    box('door_fill_2', 6.11, 0.02, 0.36, 0.32, 1.15, -6, RUBBLE[1]),
    box('door_fill_3', 6.38, -0.01, 0.30, 0.30, 0.75, 12, RUBBLE[3]),
    box('door_pile_in', 6.02, 0.45, 0.70, 0.40, 0.60, -10, RUBBLE[4]),
    box('door_pile_out', 6.05, -0.45, 0.60, 0.35, 0.55, 5, RUBBLE[2]),
    # fallen chunks of the collapsed y=4.8 section (B side)
    box('mid_chunk_1', 2.95, 4.45, 0.45, 0.28, 0.35, 20, CONCRETE_DMG),
]

# --------------------------------------------------------------------------
# fallen_beam: large structural beam lying across the east hall, west end
# resting against the corridor wall (no gap), east end 1.06 m from the wall
# --------------------------------------------------------------------------
_B0, _B1 = (4.20, 3.44), (6.85, 3.80)
FALLEN_BEAM = [
    box('beam', 0.5 * (_B0[0] + _B1[0]), 0.5 * (_B0[1] + _B1[1]),
        math.hypot(_B1[0] - _B0[0], _B1[1] - _B0[1]), 0.30, 0.40,
        math.degrees(math.atan2(_B1[1] - _B0[1], _B1[0] - _B0[0])), WOOD),
]

# --------------------------------------------------------------------------
# debris: medium / large pieces (lobby island, victim-cover pile, room debris)
# --------------------------------------------------------------------------
DEBRIS = [
    box('lobby_island', 4.50, 1.25, 1.00, 0.55, 0.45, 20, CONCRETE_DMG),
    box('b_slab', 2.55, 3.55, 0.70, 0.50, 0.60, 35, RUBBLE[1]),
    # victim-cover pile in D (between D door and the NE corner)
    box('pile_1', 6.78, 5.66, 0.55, 0.60, 1.80, 12, CONCRETE_DMG),
    box('pile_2', 7.20, 5.82, 0.50, 0.55, 1.40, -18, RUBBLE[3]),
    # collapsed room C
    box('c_slab_1', 0.60, 6.50, 0.80, 0.60, 0.80, 10, CONCRETE_DMG),
    box('c_slab_2', 2.70, 6.30, 0.90, 0.50, 0.70, -25, RUBBLE[1]),
    box('d_slab', 4.75, 6.55, 0.70, 0.45, 0.65, -15, CONCRETE_DMG),
]

# --------------------------------------------------------------------------
# rubble: small blocks (all >= 0.25 m wide, >= 0.35 m tall)
# --------------------------------------------------------------------------
RUBBLE_BLOCKS = [
    box('r_lobby_sw_1', 0.55, 0.60, 0.50, 0.45, 0.50, 15, RUBBLE[0]),
    box('r_lobby_sw_2', 0.40, 1.15, 0.30, 0.30, 0.35, -20, RUBBLE[2]),
    box('r_lobby_mid', 5.25, 1.62, 0.35, 0.30, 0.35, -25, RUBBLE[3]),
    box('r_lobby_corr', 2.90, 2.20, 0.35, 0.30, 0.40, 20, RUBBLE[4]),
    box('r_lobby_se_1', 7.50, 0.50, 0.60, 0.50, 0.60, 25, RUBBLE[1]),
    box('r_lobby_se_2', 7.65, 1.05, 0.35, 0.35, 0.40, 0, RUBBLE[0]),
    box('r_b_1', 0.45, 4.30, 0.40, 0.35, 0.40, -15, RUBBLE[2]),
    box('r_c_1', 1.70, 6.60, 0.50, 0.50, 0.60, 35, RUBBLE[0]),
    box('r_c_2', 2.75, 5.35, 0.40, 0.40, 0.50, 15, RUBBLE[4]),
    box('r_c_3', 0.45, 5.40, 0.35, 0.50, 0.45, 0, RUBBLE[3]),
    box('r_d_1', 5.55, 6.85, 0.40, 0.35, 0.45, 25, RUBBLE[2]),
    box('r_e_1', 4.75, 4.45, 0.35, 0.30, 0.40, -30, RUBBLE[1]),
    # outdoor rubble (yard), away from the doorway approach
    box('r_yard_1', -1.20, -0.55, 0.60, 0.45, 0.50, 20, RUBBLE[1]),
    box('r_yard_2', 4.20, -0.90, 0.50, 0.40, 0.45, -15, RUBBLE[0]),
    box('r_yard_3', 5.00, -1.70, 0.35, 0.35, 0.35, 0, RUBBLE[3]),
    box('r_yard_4', 7.40, -0.90, 0.45, 0.40, 0.50, 30, RUBBLE[4]),
]

MODELS = {
    'damaged_room': DAMAGED_ROOM,
    'collapsed_wall': COLLAPSED_WALL,
    'fallen_beam': FALLEN_BEAM,
    'debris': DEBRIS,
    'rubble': RUBBLE_BLOCKS,
}


def all_boxes():
    return [b for v in MODELS.values() for b in v]


def corners(b):
    c, s = math.cos(b['yaw']), math.sin(b['yaw'])
    hx, hy = 0.5 * b['sx'], 0.5 * b['sy']
    return [(b['cx'] + c * dx - s * dy, b['cy'] + s * dx + c * dy)
            for dx, dy in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]
