"""
Victim approach in autonomous_search (victim_approach_enabled).

/victim_distance readings -> nav-frame victim memory -> confirmation -> approach with the
ordinary EXPLORE_TURN / EXPLORE_MOVE plan -> VICTIM_HOLD + /victim_found. Deterministic: the
node is driven by direct calls, no simulator.
"""

import json
import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from my_first_robot.autonomous_search import (
    AutonomousSearch, Pose2D, State, victim_approach_move, victim_measurement,
    victim_nav_position, VictimMemory)
import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

LIDAR_X = -0.032


@pytest.fixture
def make_node():
    made = []

    def make(enabled=True):
        flag = 'true' if enabled else 'false'
        rclpy.init(args=['--ros-args', '-p', f'victim_approach_enabled:={flag}'])
        n = AutonomousSearch()
        n.p.scan_record_dir = ''
        made.append(n)
        return n
    yield make
    for n in made:
        n.destroy_node()
    rclpy.shutdown()


def reading(distance, angle_deg=0.0, confidence=0.9, age=0.05):
    m = String()
    m.data = json.dumps({'victim_id': 1, 'class': 'person', 'confidence': confidence,
                         'distance_m': distance, 'camera_x': 160.0, 'angle_deg': angle_deg,
                         'detection_age_s': age})
    return m


def inside(node, x=0.0, y=0.0, yaw=0.0, state=State.EXPLORE_MOVE, rotating=False):
    node.mission_phase = 'INSIDE'
    node.nav_pose = Pose2D(x, y, yaw)
    node.imu_yaw = yaw
    node.imu_history.clear()
    end = yaw + (0.5 if rotating else 0.0)
    for k in range(13):
        node.imu_history.append((0.1 * k, yaw + (end - yaw) * k / 12))
    node.state = state
    node.entry_phase = 'SETTLE' if state == State.STOP_AND_SCAN else None   # stationary phase
    node.narrow_plan = []


def scan_data(r=3.0):
    return {'analysis': {'profile': [r] * 360}, 'complete': True, 'reason': 'complete',
            'result': {}}


def confirm(node, distance, angle_deg=0.0, n=3):
    for _ in range(n):
        node.victim_callback(reading(distance, angle_deg))


# ---------------------------------------------------------------- 1
def test_01_disabled_flag_no_victim_behaviour(make_node):
    node = make_node(enabled=False)
    assert node.p.victim_approach_enabled is False
    assert node.victim_found_publisher is None
    assert '/victim_distance' not in [s.topic_name for s in node.subscriptions]
    assert '/victim_found' not in [p.topic_name for p in node.publishers]
    # Even with a confirmed track in memory the decision point explores as before.
    inside(node, state=State.STOP_AND_SCAN)
    for _ in range(3):
        node.victim_memory.observe(node._now_sec(), 2.0, 0.0, 0.9, 2.0)
    calls = []
    node._grid_update = lambda pose, data: None
    node._grid_stop_reason = lambda pose, last: None
    node._grid_select = lambda now, pose, data: calls.append('frontier')
    node.scan_data = scan_data()
    node.explore_move = None
    node._explore_after_scan(node._now_sec())
    assert calls == ['frontier'] and node.victim_target is None


# ---------------------------------------------------------------- 2
def test_02_reading_converts_to_nav_frame():
    x, y = victim_nav_position(Pose2D(1.0, 2.0, math.pi / 2), 2.0, 0.0, LIDAR_X)
    assert x == pytest.approx(1.0, abs=1e-9) and y == pytest.approx(2.0 - 0.032 + 2.0)
    x, y = victim_nav_position(Pose2D(0.0, 0.0, 0.0), 1.5, math.radians(90.0), LIDAR_X)
    assert x == pytest.approx(-0.032) and y == pytest.approx(1.5)


def test_02_node_uses_current_nav_pose(make_node):
    node = make_node()
    inside(node, x=1.0, y=-1.0, yaw=math.radians(90.0))
    node.victim_callback(reading(2.0, 0.0))
    tr = node.victim_memory.tracks[0]
    assert tr.x == pytest.approx(1.0, abs=1e-6) and tr.y == pytest.approx(-1.0 - 0.032 + 2.0)


# ---------------------------------------------------------------- 3
@pytest.mark.parametrize('payload', [
    {'distance_m': 2.0, 'angle_deg': 0.0},                                   # missing field
    {'distance_m': 'nan', 'angle_deg': 0.0, 'confidence': 0.9},
    {'distance_m': 0.1, 'angle_deg': 0.0, 'confidence': 0.9},                # too close
    {'distance_m': 5.0, 'angle_deg': 0.0, 'confidence': 0.9},                # beyond LiDAR
    {'distance_m': 2.0, 'angle_deg': 150.0, 'confidence': 0.9},              # outside camera
    {'distance_m': 2.0, 'angle_deg': 0.0, 'confidence': 0.4},                # weak
    {'distance_m': 2.0, 'angle_deg': 0.0, 'confidence': 0.9, 'detection_age_s': 2.0},  # stale
])
def test_03_invalid_or_stale_measurement_rejected(payload):
    assert victim_measurement(payload) is None


def test_03_node_rejects_bad_message(make_node):
    node = make_node()
    inside(node)
    node.victim_callback(reading(2.0, age=3.0))
    bad = String()
    bad.data = 'not json'
    node.victim_callback(bad)
    assert node.victim_stats['rejected_invalid'] == 2 and not node.victim_memory.tracks
    assert victim_measurement({'distance_m': 2.0, 'angle_deg': -10.0, 'confidence': 0.9,
                               'detection_age_s': 0.1}) == pytest.approx(
        (2.0, math.radians(-10.0), 0.9))


# ---------------------------------------------------------------- 4
def test_04_rotating_rejects_measurement(make_node):
    node = make_node()
    inside(node, rotating=True)
    node.victim_callback(reading(2.0))
    assert node.victim_stats['rejected_rotating'] == 1 and not node.victim_memory.tracks
    inside(node, state=State.STOP_AND_SCAN)
    node.entry_phase = 'ROTATE'                    # the 360-degree turn itself
    node.victim_callback(reading(2.0))
    assert node.victim_stats['rejected_rotating'] == 2 and not node.victim_memory.tracks


# ---------------------------------------------------------------- 5 / 6
def test_05_three_agreeing_observations_confirm():
    mem = VictimMemory(0.5, 3, 30.0)
    for k, (x, y) in enumerate([(2.0, 0.0), (2.1, 0.05), (1.95, -0.05)]):
        mem.observe(float(k), x, y, 0.8, 2.0)
        assert (mem.confirmed(float(k)) is None) == (k < 2)
    tr = mem.confirmed(3.0)
    assert tr.count == 3 and tr.x == pytest.approx(2.0167, abs=1e-3)


def test_06_inconsistent_observations_do_not_confirm():
    mem = VictimMemory(0.5, 3, 30.0)
    for k, (x, y) in enumerate([(2.0, 0.0), (3.0, 0.0), (2.0, 1.0), (4.0, 1.0)]):
        mem.observe(float(k), x, y, 0.9, 2.0)
    assert mem.confirmed(4.0) is None and len(mem.tracks) == 4
    mem2 = VictimMemory(0.5, 3, 30.0)
    for t in (0.0, 1.0):
        mem2.observe(t, 2.0, 0.0, 0.9, 2.0)
    mem2.observe(100.0, 2.0, 0.0, 0.9, 2.0)        # first two expired (timeout)
    assert mem2.confirmed(100.0) is None


# ---------------------------------------------------------------- 7
def test_07_victim_takes_priority_over_frontier(make_node):
    node = make_node()
    inside(node, state=State.STOP_AND_SCAN)
    node.entry_phase = 'STOPPING'
    calls = []
    node._grid_update = lambda pose, data: None
    node._grid_stop_reason = lambda pose, last: None
    node._grid_select = lambda now, pose, data: calls.append('frontier')
    node.scan_data = scan_data()
    node.explore_move = None
    node._explore_after_scan(node._now_sec())      # nothing confirmed yet
    assert calls == ['frontier']
    inside(node, state=State.STOP_AND_SCAN)
    node.entry_phase = 'STOPPING'
    confirm(node, 2.5)
    step = node.explore_step
    node._explore_after_scan(node._now_sec())
    assert calls == ['frontier']                   # frontier selection skipped
    assert node.state == State.EXPLORE_TURN and node.explore_plan['sector'] == 'VICTIM'
    assert node.explore_step == step               # approach moves are not frontier steps


# ---------------------------------------------------------------- 8
def test_08_move_never_below_stop_distance():
    for rng in [0.5, 0.8, 0.84, 0.86, 0.9, 1.0, 1.3, 2.0, 3.4]:
        for free in [0.0, 0.05, 0.2, 0.5, 1.0]:
            for keep in [0.0, 0.3, 1.0]:
                move, outcome = victim_approach_move(rng, 0.80, free, keep, 0.30)
                if outcome == 'move':
                    assert 0.0 < move <= rng - 0.80 + 1e-9
                    assert move <= free + 1e-9 and move <= keep + 1e-9
                else:
                    assert move == 0.0
                if rng - 0.80 <= 0.05:
                    assert outcome == 'reached'


def test_08_node_plan_stops_at_stop_distance(make_node):
    node = make_node()
    inside(node, state=State.STOP_AND_SCAN)
    confirm(node, 1.4)
    assert node._victim_after_scan(node._now_sec(), node._entry_pose(), scan_data())
    assert node.explore_plan['target'] == pytest.approx(1.4 - 0.80, abs=1e-6)


def test_08_unsafe_corridor_abandons(make_node):
    node = make_node()
    inside(node, state=State.STOP_AND_SCAN)
    confirm(node, 2.5)
    # Something 0.5 m ahead in the path: free travel 0.1 m -> do not force it.
    data = scan_data()
    data['analysis']['profile'][0] = data['analysis']['profile'][359] = 0.5
    assert not node._victim_after_scan(node._now_sec(), node._entry_pose(), data)
    assert node.victim_target is None and node.victim_abandoned


# ---------------------------------------------------------------- 9
def test_09_lost_for_two_scans_abandons(make_node):
    node = make_node()
    inside(node, state=State.STOP_AND_SCAN)
    confirm(node, 3.0)
    pose = node._entry_pose()
    assert node._victim_after_scan(node._now_sec(), pose, scan_data())       # move 1
    assert node._victim_after_scan(node._now_sec() + 1, pose, scan_data())   # miss 1, move 2
    assert node.victim_target['missed'] == 1
    assert not node._victim_after_scan(node._now_sec() + 2, pose, scan_data())  # miss 2
    assert node.victim_target is None and not node.victim_memory.tracks
    assert node.victim_abandoned


def test_09_seen_again_resets_missed(make_node):
    node = make_node()
    inside(node, state=State.STOP_AND_SCAN)
    confirm(node, 3.0)
    pose = node._entry_pose()
    node._victim_after_scan(node._now_sec(), pose, scan_data())
    node._victim_after_scan(node._now_sec(), pose, scan_data())             # miss 1
    assert node.victim_target['missed'] == 1
    inside(node, state=State.EXPLORE_MOVE)
    node.victim_callback(reading(3.0))                                       # seen again
    inside(node, state=State.STOP_AND_SCAN)
    assert node._victim_after_scan(node._now_sec(), pose, scan_data())
    assert node.victim_target['missed'] == 0


# ---------------------------------------------------------------- 10
def test_10_within_stop_distance_enters_victim_hold(make_node):
    node = make_node()
    inside(node, state=State.STOP_AND_SCAN)
    confirm(node, 0.82)
    assert node._victim_after_scan(node._now_sec(), node._entry_pose(), scan_data())
    assert node.state == State.VICTIM_HOLD and node.victim_report is not None


def test_10_close_range_loss_enters_victim_hold(make_node):
    node = make_node()
    inside(node, state=State.STOP_AND_SCAN)
    confirm(node, 1.6)
    pose = node._entry_pose()
    assert node._victim_after_scan(node._now_sec(), pose, scan_data())       # move 0.8 m
    moved = Pose2D(0.7, 0.0, 0.0)                                            # now 0.93 m away
    node.nav_pose = moved
    assert node._victim_after_scan(node._now_sec() + 1, moved, scan_data())  # not seen
    assert node.state == State.VICTIM_HOLD
    assert 'close range' in node.victim_report['reason']


# ---------------------------------------------------------------- 11
def test_11_victim_found_message_fields(make_node):
    node = make_node()
    got = []
    node.create_subscription(String, '/victim_found', got.append,
                             node.victim_found_publisher.qos_profile)
    inside(node, state=State.STOP_AND_SCAN)
    confirm(node, 0.80)
    node._victim_after_scan(node._now_sec(), node._entry_pose(), scan_data())
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
        if got:
            break
    assert got, 'no /victim_found received'
    report = json.loads(got[0].data)
    for key in ('victim_x', 'victim_y', 'distance_m', 'confidence', 'timestamp', 'state',
                'frame'):
        assert key in report
    assert report['state'] == 'VICTIM_HOLD' and 'not ground truth' in report['frame']
    assert report['victim_x'] == pytest.approx(0.80 - 0.032, abs=1e-3)


# ---------------------------------------------------------------- 12
def test_12_victim_hold_publishes_zero_velocity(make_node):
    node = make_node()
    inside(node, state=State.STOP_AND_SCAN)
    confirm(node, 0.80)
    node._victim_after_scan(node._now_sec(), node._entry_pose(), scan_data())
    node.victim_hold['facing'] = False
    sent = []
    node.cmd_publisher.publish = sent.append
    for _ in range(5):
        node.handle_victim_hold(None, node._now_sec())
    assert sent and all(m.twist.linear.x == 0.0 and m.twist.angular.z == 0.0 for m in sent)
    assert node.state == State.VICTIM_HOLD
    # Facing: an in-place turn only (never forward), then zero.
    node.victim_hold.update({'facing': True, 'heading': math.radians(30.0),
                             'start': node._now_sec()})
    node.latest_imu_stamp = node.get_clock().now().to_msg()
    sent.clear()
    node.handle_victim_hold(None, node._now_sec())
    assert sent[-1].twist.linear.x == 0.0 and sent[-1].twist.angular.z > 0.0


# ---------------------------------------------------------------- 13
@pytest.mark.parametrize('state,phase', [
    (State.ENTRANCE_APPROACH, 'ENTERING'), (State.ENTRANCE_ALIGN, 'ENTERING'),
    (State.ENTRANCE_CROSS, 'ENTERING'), (State.VERIFY_INSIDE, 'INSIDE'),
    (State.EXPLORING, 'OUTSIDE_SEARCH'), (State.INSIDE_HOLD, 'INSIDE')])
def test_13_entrance_states_ignore_victims(make_node, state, phase):
    node = make_node()
    inside(node, state=state)
    node.mission_phase = phase
    confirm(node, 2.0, n=5)
    assert node.victim_stats['rejected_state'] == 5 and not node.victim_memory.tracks
    assert node.victim_target is None and node.state == state
    # Rotating during an entrance-protected state: still ignored, no cue either.
    inside(node, state=state, rotating=True)
    node.mission_phase = phase
    confirm(node, 2.0, n=3)
    assert not node.victim_cues.tracks and node.victim_stats['cues'] == 0


# ---------------------------------------------------------------- 14
def laser_with_post(post_x, post_y, half=0.021):
    """Raw scan from the robot at the origin (LiDAR at x=-0.032): 3 m room + one thin post."""
    m = LaserScan()
    m.header.stamp = Time(sec=7, nanosec=0)
    m.angle_min, m.angle_increment = 0.0, 2 * math.pi / 360
    m.angle_max = m.angle_min + 359 * m.angle_increment
    m.range_min, m.range_max = 0.12, 3.5
    ranges = []
    for i in range(360):
        a = i * m.angle_increment
        dx, dy = math.cos(a), math.sin(a)
        t = 3.0
        # ray vs square post
        tmin, tmax = -math.inf, math.inf
        for o, d, lo, hi in ((LIDAR_X, dx, post_x - half, post_x + half),
                             (0.0, dy, post_y - half, post_y + half)):
            if abs(d) < 1e-12:
                if o < lo or o > hi:
                    tmin = math.inf
                continue
            t0, t1 = (lo - o) / d, (hi - o) / d
            tmin, tmax = max(tmin, min(t0, t1)), min(tmax, max(t0, t1))
        if tmax >= tmin > 0:
            t = min(t, tmin)
        ranges.append(t)
    m.ranges = ranges
    return m


def test_14_approach_uses_existing_command_and_safety_path(make_node):
    node = make_node()
    twist_pubs = [p for p in node.publishers if p.msg_type is TwistStamped]
    assert [p.topic_name for p in twist_pubs] == ['/cmd_vel_raw']
    inside(node, state=State.STOP_AND_SCAN)
    confirm(node, 2.5)
    assert node._victim_after_scan(node._now_sec(), node._entry_pose(), scan_data())
    assert node.state == State.EXPLORE_TURN
    plan = node.explore_plan
    assert plan['sector'] == 'VICTIM' and plan['target'] == pytest.approx(1.0)
    # The existing turn handler takes over (already facing -> begins the move) ...
    node.latest_scan = laser_with_post(0.75, 0.25)
    node.latest_scan_stamp = node.latest_scan.header.stamp
    node.handle_explore_turn(None, node._now_sec())
    assert node.state == State.EXPLORE_MOVE
    # ... and the existing live narrow keep-out still cuts the victim move.
    sent = []
    node.cmd_publisher.publish = sent.append
    node.handle_explore_move(None, node._now_sec())
    assert node.explore_move.get('keepout_cut') and plan['target'] < 1.0
    assert all(m.twist.linear.x <= node.p.explore_speed for m in sent)


# ================================================================ look toward cue
def cues(node, distance, angle_deg=0.0, n=3, state=State.STOP_AND_SCAN):
    inside(node, state=state)
    node.entry_phase = 'ROTATE'
    for _ in range(n):
        node.victim_callback(reading(distance, angle_deg))
    inside(node, state=State.STOP_AND_SCAN)
    node.entry_phase = 'STOPPING'


def test_15_rotating_reading_creates_cue_only(make_node):
    node = make_node()
    inside(node, rotating=True)                    # EXPLORE_MOVE but turning
    node.victim_callback(reading(2.0, -30.0))
    assert node.victim_stats['rejected_rotating'] == 1 and node.victim_stats['cues'] == 1
    assert not node.victim_memory.tracks and node.victim_target is None
    cue = node.victim_cues.tracks[0]
    assert math.degrees(math.atan2(cue.y, cue.x + 0.032)) == pytest.approx(-30.0, abs=0.1)


def test_16_conflicting_cues_no_look(make_node):
    node = make_node()
    for ang in (0.0, 60.0, -60.0, 120.0):          # four directions, nothing agrees
        cues(node, 2.0, ang, n=1)
    inside(node, state=State.STOP_AND_SCAN)
    assert not node._victim_after_scan(node._now_sec(), node._entry_pose(), scan_data())
    assert node.victim_stats['looks'] == 0 and not node.victim_cues.tracks


def test_17_agreeing_cues_start_one_turn_only_look(make_node):
    node = make_node()
    cues(node, 2.3, -87.0)
    node.latest_scan = laser_with_post(9.0, 9.0)   # nothing near: rotation allowed
    node.latest_scan_stamp = node.latest_scan.header.stamp
    assert node._victim_after_scan(node._now_sec(), node._entry_pose(), scan_data())
    plan = node.explore_plan
    assert node.state == State.EXPLORE_TURN and plan['sector'] == 'VICTIM_LOOK'
    assert plan['target'] == 0.0 and math.degrees(plan['heading']) == pytest.approx(-87.8, abs=1.0)
    assert node.victim_target is None and not node.victim_memory.tracks   # look != confirm
    assert node.victim_stats['looks'] == 1


def test_18_cue_expiry_no_look(make_node):
    node = make_node()
    now = node._now_sec()
    for _ in range(3):
        node.victim_cues.observe(now - 60.0, 2.0, 0.0, 0.9, 2.0)    # older than 30 s
    inside(node, state=State.STOP_AND_SCAN)
    assert not node._victim_after_scan(now, node._entry_pose(), scan_data())
    assert node.victim_stats['looks'] == 0


def test_19_one_look_per_cue_and_new_evidence(make_node):
    node = make_node()
    node.latest_scan = laser_with_post(9.0, 9.0)
    node.latest_scan_stamp = node.latest_scan.header.stamp
    cues(node, 2.0, 0.0)
    assert node._victim_after_scan(node._now_sec(), node._entry_pose(), scan_data())
    cues(node, 2.0, 0.0)                           # same place again: consumed
    assert not node._victim_after_scan(node._now_sec(), node._entry_pose(), scan_data())
    assert node.victim_stats['looks'] == 1
    cues(node, 2.0, 90.0)                          # genuinely new place (2.8 m away)
    assert node._victim_after_scan(node._now_sec(), node._entry_pose(), scan_data())
    assert node.victim_stats['looks'] == 2


def test_20_look_zero_translation_existing_turn_then_scan(make_node):
    node = make_node()
    node.latest_scan = laser_with_post(9.0, 9.0)
    node.latest_scan_stamp = node.latest_scan.header.stamp
    cues(node, 2.0, 60.0)
    node._victim_after_scan(node._now_sec(), node._entry_pose(), scan_data())
    sent = []
    node.cmd_publisher.publish = sent.append
    node.handle_explore_turn(None, node._now_sec())          # existing turn handler
    assert sent[-1].twist.linear.x == 0.0 and sent[-1].twist.angular.z > 0.0
    node.imu_yaw = node.explore_plan['heading']               # turn reached
    node.nav_pose = Pose2D(0.0, 0.0, node.imu_yaw)
    node.handle_explore_turn(None, node._now_sec())
    assert node.state == State.EXPLORE_MOVE
    node.handle_explore_move(None, node._now_sec())
    assert node.state == State.STOP_AND_SCAN and node.entry_phase == 'SETTLE'
    assert node.explore_move is None               # not a (no-)progress cycle
    assert sent and all(m.twist.linear.x == 0.0 for m in sent)


def test_21_unsafe_turn_refuses_look_via_existing_keepout(make_node):
    node = make_node()
    cues(node, 2.0, 90.0)
    node.latest_scan = laser_with_post(0.40, 0.0)  # thin post inside the keep-out
    node.latest_scan_stamp = node.latest_scan.header.stamp
    node.keepout_escapes = 0
    assert node._victim_after_scan(node._now_sec(), node._entry_pose(), scan_data())
    assert node.explore_plan['sector'] == 'ESCAPE'  # existing escape, not the look
    assert node.victim_stats['looks'] == 1          # cue consumed, no retry loop


def test_22_stationary_readings_after_look_confirm_and_approach(make_node):
    node = make_node()
    node.latest_scan = laser_with_post(9.0, 9.0)
    node.latest_scan_stamp = node.latest_scan.header.stamp
    cues(node, 2.5, 0.0)
    node._victim_after_scan(node._now_sec(), node._entry_pose(), scan_data())   # look
    inside(node, state=State.STOP_AND_SCAN)        # settle after the look: still
    confirm(node, 2.5)
    assert node._victim_after_scan(node._now_sec(), node._entry_pose(), scan_data())
    assert node.victim_target is not None and node.explore_plan['sector'] == 'VICTIM'
    assert node.explore_plan['target'] == pytest.approx(1.0)


def test_23_flag_off_ignores_cues(make_node):
    node = make_node(enabled=False)
    inside(node, state=State.STOP_AND_SCAN)
    for _ in range(3):
        node.victim_cues.observe(node._now_sec(), 2.0, 0.0, 0.9, 2.0)
    calls = []
    node._grid_update = lambda pose, data: None
    node._grid_stop_reason = lambda pose, last: None
    node._grid_select = lambda now, pose, data: calls.append('frontier')
    node.scan_data = scan_data()
    node.explore_move = None
    node._explore_after_scan(node._now_sec())
    assert calls == ['frontier'] and node.victim_stats['looks'] == 0


# ================================================================ approach heading offsets
def profile_with(points, r=3.0):
    """Build a merged scan profile (world-aligned, robot at the origin) with extra points."""
    data = scan_data(r)
    profile = data['analysis']['profile']
    for x, y in points:
        k = int(math.floor(math.degrees(math.atan2(y, x)))) % 360
        profile[k] = min(profile[k], math.hypot(x, y))
    return data


def fresh_live(node, post_x=9.0, post_y=9.0, half=0.021):
    node.latest_scan = laser_with_post(post_x, post_y, half)
    node.latest_scan.header.stamp = node.get_clock().now().to_msg()
    node.latest_scan_stamp = node.latest_scan.header.stamp


def decide(node, data, distance=2.5):
    """Confirm a victim `distance` straight ahead and run one post-scan decision."""
    events = []
    node._victim_record = events.append
    inside(node, state=State.STOP_AND_SCAN)
    confirm(node, distance)
    started = node._victim_after_scan(node._now_sec(), node._entry_pose(), data)
    return started, candidate_results(events)


def candidate_results(events):
    rec = [e for e in events if e['event'] == 'approach_candidates']
    return [(c['offset_deg'], c['result']) for c in rec[-1]['candidates']] if rec else []


V2_CORNER = (0.29, 0.23)          # V2 live blocker: 0.29 m ahead, 0.23 m to the side


def test_24_direct_approach_succeeds_no_offsets(make_node):
    node = make_node()
    fresh_live(node)
    started, results = decide(node, profile_with([]))
    assert started and results == [(0.0, 'ok')]
    plan = node.explore_plan
    assert plan['sector'] == 'VICTIM' and plan['offset_deg'] == 0.0
    assert plan['heading'] == pytest.approx(0.0) and plan['target'] == pytest.approx(1.0)
    assert node.victim_target['offsets_used'] == []


def test_25_direct_blocked_tries_plus_10(make_node):
    node = make_node()
    fresh_live(node)
    started, results = decide(node, profile_with([(0.6, -0.15)]))   # blocker on the right
    assert started and results == [(0.0, 'corridor_blocked'), (10.0, 'ok')]
    assert math.degrees(node.explore_plan['heading']) == pytest.approx(10.0, abs=1e-6)
    assert node.victim_target['offsets_used'] == [10.0]


def test_26_direct_blocked_tries_minus_10(make_node):
    node = make_node()
    fresh_live(node)
    started, results = decide(node, profile_with([V2_CORNER]))       # corner on the left
    assert started and results == [
        (0.0, 'corridor_blocked'), (10.0, 'corridor_blocked'), (-10.0, 'ok')]
    assert math.degrees(node.explore_plan['heading']) == pytest.approx(-10.0, abs=1e-6)


@pytest.mark.parametrize('point,chosen', [((0.10, -0.22), 20.0), ((0.10, 0.22), -20.0)])
def test_27_both_10_blocked_tries_20(make_node, point, chosen):
    node = make_node()
    fresh_live(node)
    started, results = decide(node, profile_with([point]))
    assert started and results[-1] == (chosen, 'ok')
    assert [r for _, r in results[:3]] == ['corridor_blocked'] * 3
    assert math.degrees(node.explore_plan['heading']) == pytest.approx(chosen, abs=1e-6)


def test_28_candidate_order_is_deterministic(make_node):
    from my_first_robot.autonomous_search import VICTIM_HEADING_OFFSETS_DEG
    assert VICTIM_HEADING_OFFSETS_DEG == (0.0, 10.0, -10.0, 20.0, -20.0)
    node = make_node()
    fresh_live(node)
    runs = []
    for _ in range(2):
        node.victim_abandoned = []                   # same situation again from scratch
        runs.append(decide(node, profile_with([(0.10, -0.22), (0.10, 0.22)]))[1])
    assert runs[0] == runs[1]
    assert [o for o, _ in runs[0]] == [0.0, 10.0, -10.0, 20.0, -20.0]


def test_29_unsafe_candidate_rejected_without_translation(make_node):
    node = make_node()
    fresh_live(node)
    sent = []
    node.cmd_publisher.publish = sent.append
    started, results = decide(node, profile_with([(0.10, -0.22), (0.10, 0.22)]))
    assert not started and all(r != 'ok' for _, r in results)
    assert node.state == State.STOP_AND_SCAN and node.explore_plan is None
    assert all(m.twist.linear.x <= 0.0 for m in sent)


def test_30_candidate_inside_024_corridor_rejected(make_node):
    node = make_node()
    assert node.p.explore_path_half_width_m + node.p.grid_plan_path_margin_m == \
        pytest.approx(0.24)
    fresh_live(node)
    # The V2 corner sits 0.23 m beside the direct path: blocked, +10 (toward it) blocked,
    # -10 puts it ~0.28 m to the side (genuinely clear), so -10 is the one used.
    started, results = decide(node, profile_with([V2_CORNER]))
    assert results[0] == (0.0, 'corridor_blocked') and results[1] == (10.0, 'corridor_blocked')
    # A point 0.23 m beside the -10 heading too: -10 is rejected as well.
    node.victim_target, node.victim_abandoned = None, []
    node.state = State.STOP_AND_SCAN
    h = math.radians(-10.0)
    side = (0.40 * math.cos(h) + 0.23 * math.sin(h), 0.40 * math.sin(h) - 0.23 * math.cos(h))
    _, results2 = decide(node, profile_with([V2_CORNER, side]))
    assert (-10.0, 'corridor_blocked') in results2


def test_31_candidate_violating_narrow_keepout_rejected(make_node):
    node = make_node()
    fresh_live(node)
    # Remembered thin post 0.75 m along the -10 heading: inside the keep-out.
    h = math.radians(-10.0)
    inside(node, state=State.STOP_AND_SCAN)
    plan_post = {'x': 0.75 * math.cos(h), 'y': 0.75 * math.sin(h), 'width': 0.04}
    events = []
    node._victim_record = events.append
    confirm(node, 2.5)
    node.narrow_plan = [plan_post]
    node._victim_after_scan(node._now_sec(), node._entry_pose(), profile_with([V2_CORNER]))
    results = candidate_results(events)
    assert (-10.0, 'narrow_keepout') in results
    assert all(o != -10.0 or r != 'ok' for o, r in results)


def test_31_live_narrow_keepout_rejected(make_node):
    node = make_node()
    h = math.radians(-10.0)
    # Thin post (live scan only) 0.35 m beside the -10 path: outside the 0.20 m path
    # blocker, inside the narrow keep-out.
    fresh_live(node, 0.9 * math.cos(h) + 0.35 * math.sin(h),
               0.9 * math.sin(h) - 0.35 * math.cos(h))
    _, results = decide(node, profile_with([V2_CORNER]))
    assert (-10.0, 'live_narrow_keepout') in results


def test_32_candidate_violating_live_clearance_rejected(make_node):
    node = make_node()
    h = math.radians(-10.0)
    fresh_live(node, 0.9 * math.cos(h), 0.9 * math.sin(h), half=0.15)   # wide box, live only
    _, results = decide(node, profile_with([V2_CORNER]))
    assert (-10.0, 'live_blocked') in results


def test_32_stale_live_scan_rejected(make_node):
    node = make_node()
    node.latest_scan = laser_with_post(9.0, 9.0)                   # stamp 7 s: stale
    node.latest_scan_stamp = node.latest_scan.header.stamp
    started, results = decide(node, profile_with([V2_CORNER]))
    assert (-10.0, 'stale_scan') in results and not started


def test_33_passing_candidate_uses_existing_turn_and_move(make_node):
    node = make_node()
    fresh_live(node)
    sent = []
    node.cmd_publisher.publish = sent.append
    decide(node, profile_with([V2_CORNER]))
    assert node.state == State.EXPLORE_TURN and node.explore_plan['sector'] == 'VICTIM'
    node.latest_imu_stamp = node.get_clock().now().to_msg()
    node.handle_explore_turn(None, node._now_sec())                  # existing turn handler
    assert sent[-1].twist.linear.x == 0.0 and sent[-1].twist.angular.z < 0.0
    node.imu_yaw = node.explore_plan['heading']
    node.nav_pose = Pose2D(0.0, 0.0, node.imu_yaw)
    node.handle_explore_turn(None, node._now_sec())
    assert node.state == State.EXPLORE_MOVE
    fresh_live(node)
    node.handle_explore_move(None, node._now_sec())                  # existing move handler
    assert 0.0 < sent[-1].twist.linear.x <= node.p.explore_speed


def test_33_pre_rotation_refusal_hands_over_to_existing_escape(make_node):
    node = make_node()
    fresh_live(node, 0.40, 0.0)                    # thin post inside the keep-out
    node.keepout_escapes = 0
    inside(node, state=State.STOP_AND_SCAN)
    confirm(node, 2.5)
    node.narrow_plan = []
    node._victim_live_check = lambda pose, h, move: {'live': 'ok'}   # isolate the turn check
    assert node._victim_after_scan(node._now_sec(), node._entry_pose(),
                                   profile_with([V2_CORNER]))
    assert node.explore_plan['sector'] == 'ESCAPE'
    assert node.victim_target['moves'] == 0 and node.victim_target['offsets_used'] == [-10.0]


def after_detour(node):
    """Victim 2.5 m ahead, V2 corner -> -10 deg detour planned, then 1.0 m driven."""
    fresh_live(node)
    decide(node, profile_with([V2_CORNER]))
    h = node.explore_plan['heading']
    moved = Pose2D(1.0 * math.cos(h), 1.0 * math.sin(h), h)
    inside(node, x=moved.x, y=moved.y, yaw=h, state=State.STOP_AND_SCAN)
    fresh_live(node)
    return moved


def test_34_target_recomputed_after_candidate_move(make_node):
    node = make_node()
    track = None
    moved = after_detour(node)
    track = node.victim_target['track']
    confirm(node, math.hypot(track.x - moved.x, track.y - moved.y) + 0.032,
            math.degrees(math.atan2(track.y - moved.y, track.x - moved.x) - moved.yaw))
    events = []
    node._victim_record = events.append
    assert node._victim_after_scan(node._now_sec(), moved, profile_with([]))
    bearing = math.atan2(track.y - moved.y, track.x - moved.x)
    assert node.explore_plan['heading'] == pytest.approx(bearing, abs=0.02)   # same victim
    assert node.victim_target['missed'] == 0 and node.victim_target['moves'] == 2


def test_35_victim_loss_after_detour_uses_existing_rule(make_node):
    node = make_node()
    moved = after_detour(node)
    node._victim_after_scan(node._now_sec(), moved, profile_with([]))          # miss 1
    assert node.victim_target['missed'] == 1
    assert not node._victim_after_scan(node._now_sec(), moved, profile_with([]))   # miss 2
    assert node.victim_target is None and node.victim_abandoned


def test_36_same_offset_not_retried(make_node):
    node = make_node()
    fresh_live(node)
    decide(node, profile_with([V2_CORNER]))
    assert node.victim_target['offsets_used'] == [-10.0]
    inside(node, state=State.STOP_AND_SCAN)
    confirm(node, 2.5)                                                   # still seen
    events = []
    node._victim_record = events.append
    node._victim_after_scan(node._now_sec(), node._entry_pose(), profile_with([V2_CORNER]))
    results = candidate_results(events)
    assert (-10.0, 'already_used') in results
    assert node.victim_target is None or node.victim_target['offsets_used'].count(-10.0) == 1


def test_37_at_most_five_headings_and_four_detours_per_victim(make_node):
    node = make_node()
    fresh_live(node)
    layouts = [[(0.6, -0.15)], [V2_CORNER], [(0.10, -0.22)], [(0.10, 0.22)], [(0.6, -0.15)]]
    decisions = []
    for points in layouts:
        inside(node, state=State.STOP_AND_SCAN)
        confirm(node, 2.5)                                   # still seen every decision
        events = []
        node._victim_record = events.append
        node._victim_after_scan(node._now_sec(), node._entry_pose(), profile_with(points))
        decisions.append(candidate_results(events))
    assert all(len(d) <= 5 for d in decisions)
    used = [o for d in decisions for o, r in d if r == 'ok']
    assert used == [10.0, -10.0, 20.0, -20.0]               # each offset once, in order found
    assert (10.0, 'already_used') in decisions[-1]
    assert node.victim_target is None and node.victim_abandoned          # then given up


def test_38_all_candidates_blocked_abandons_and_explores(make_node):
    node = make_node()
    fresh_live(node)
    started, results = decide(node, profile_with([(0.10, -0.22), (0.10, 0.22)]))
    assert not started and len(results) == 5
    assert node.victim_target is None and node.victim_abandoned and not node.victim_memory.tracks
    # The frontier explorer gets the decision exactly as before (map untouched).
    calls = []
    node._grid_update = lambda pose, data: None
    node._grid_stop_reason = lambda pose, last: None
    node._grid_select = lambda now, pose, data: calls.append('frontier')
    inside(node, state=State.STOP_AND_SCAN)
    confirm(node, 2.5)                                    # same place: excluded
    node.scan_data = profile_with([(0.10, -0.22), (0.10, 0.22)])
    node.explore_move = None
    node._explore_after_scan(node._now_sec())
    assert calls == ['frontier']


def test_39_successful_detour_then_normal_direct_approach(make_node):
    node = make_node()
    moved = after_detour(node)
    track = node.victim_target['track']
    confirm(node, math.hypot(track.x - moved.x, track.y - moved.y) + 0.032,
            math.degrees(math.atan2(track.y - moved.y, track.x - moved.x) - moved.yaw))
    events = []
    node._victim_record = events.append
    node._victim_after_scan(node._now_sec(), moved, profile_with([]))
    assert candidate_results(events) == [(0.0, 'ok')]
    assert node.explore_plan['offset_deg'] == 0.0


def test_40_reaching_stop_distance_after_detour_holds(make_node):
    node = make_node()
    after_detour(node)
    track = node.victim_target['track']
    near = Pose2D(track.x - 0.80, track.y, 0.0)
    inside(node, x=near.x, y=near.y, state=State.STOP_AND_SCAN)
    confirm(node, 0.832)
    sent = []
    node.cmd_publisher.publish = sent.append
    assert node._victim_after_scan(node._now_sec(), near, profile_with([]))
    assert node.state == State.VICTIM_HOLD and node.victim_report['state'] == 'VICTIM_HOLD'
    assert sent and sent[-1].twist.linear.x == 0.0


def test_41_victim_found_unchanged_after_detour(make_node):
    node = make_node()
    got = []
    node.create_subscription(String, '/victim_found', got.append,
                             node.victim_found_publisher.qos_profile)
    after_detour(node)
    track = node.victim_target['track']
    near = Pose2D(track.x - 0.80, track.y, 0.0)
    inside(node, x=near.x, y=near.y, state=State.STOP_AND_SCAN)
    confirm(node, 0.832)
    node._victim_after_scan(node._now_sec(), near, profile_with([]))
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
        if got:
            break
    report = json.loads(got[0].data)
    assert set(report) == {'victim_x', 'victim_y', 'frame', 'distance_m',
                           'last_measured_distance_m', 'confidence', 'observations',
                           'robot_x', 'robot_y', 'timestamp', 'state', 'reason'}
    assert report['victim_x'] == pytest.approx(track.x, abs=1e-3)


def test_42_flag_off_never_evaluates_candidates(make_node):
    node = make_node(enabled=False)
    calls = []
    node._victim_candidate = lambda *a: calls.append('candidate')
    node._grid_update = lambda pose, data: None
    node._grid_stop_reason = lambda pose, last: None
    node._grid_select = lambda now, pose, data: calls.append('frontier')
    inside(node, state=State.STOP_AND_SCAN)
    for _ in range(3):
        node.victim_memory.observe(node._now_sec(), 2.0, 0.0, 0.9, 2.0)
    node.scan_data = profile_with([V2_CORNER])
    node.explore_move = None
    node._explore_after_scan(node._now_sec())
    assert calls == ['frontier']


@pytest.mark.parametrize('state', [State.ENTRANCE_APPROACH, State.ENTRANCE_ALIGN,
                                   State.ENTRANCE_CROSS, State.VERIFY_INSIDE])
def test_43_entrance_states_unaffected(make_node, state):
    node = make_node()
    calls = []
    node._victim_candidate = lambda *a: calls.append('candidate')
    inside(node, state=state)
    node.mission_phase = 'ENTERING'
    confirm(node, 2.0, n=5)
    assert not node.victim_memory.tracks and node.victim_target is None and not calls


def test_43_offset_heading_into_entrance_rejected(make_node):
    node = make_node()
    fresh_live(node)
    node._explore_toward_entrance = (
        lambda pose, h, d: abs(math.degrees(h) + 10.0) < 1e-6)      # only -10 ends in the door
    _, results = decide(node, profile_with([V2_CORNER]))
    assert (-10.0, 'entrance') in results and results[-1] != (-10.0, 'ok')
