"""
Read-only mission monitor (disaster_bot_monitor).

Pure model tests on real autonomous_search log lines (master-launch mission, 2026-10-08) plus
node tests: the monitor must create no motion publishers and survive missing / bad data.
"""

import ast
import inspect
import json
import time

from my_first_robot import disaster_bot_monitor as dbm
from my_first_robot.disaster_bot_monitor import DisasterBotMonitor, MonitorModel
import pytest
import rclpy

# Real /rosout message texts (autonomous_search), in mission order.
MISSION = [
    '[NAV] t=6.10 state=EXPLORING phase=OUTSIDE_SEARCH nav=(0.00,0.00) imu_yaw=+0.0 '
    'odom=(0.00,-0.00,-0.0)',
    '[STATE] t=6.04 STARTUP -> EXPLORING (sensors_ready)',
    't=7.90 [MISSION] OUTSIDE_SEARCH -> ENTERING (entrance_confirmed)',
    't=70.50 [MISSION] ENTERING -> INSIDE (verified)',
    't=94.40 [EXPLORE] step 1: turned to +43.1deg in 2.6s -> moving target 1.00m',
    't=105.50 [EXPLORE] step 1 move done: target=1.00m achieved=1.00m reason=target_reached '
    'duration=11.1s min_front_cone=1.01m nav=(3.25,2.50) from_entrance a=+1.38 lat=-0.75',
    't=861.09 [VICTIM] reading 1.59 m -41.9deg conf 0.82 -> nav (7.26,7.69) candidate '
    '(7.26,7.69) n=1 in EXPLORE_MOVE',
    '[STATE] t=864.90 EXPLORE_MOVE -> STOP_AND_SCAN (inside_verified_scan)',
    't=865.92 [VICTIM] cue (turning) 1.37 m bearing +32.4deg conf 0.67 -> group (7.23,7.71) '
    'n=1 in STOP_AND_SCAN',
    't=886.20 [VICTIM] prioritized: #1 0.585 -> #1 (best)',
    't=886.20 [VICTIM] CONFIRMED at nav (7.23,7.71) after 22 readings (spread 0.00 m, conf '
    '0.87) -- takes priority over the next frontier',
    't=886.20 [VICTIM] approach heading +0deg (+31.5deg): free 0.00 m, keep-out 0.78 m, move '
    '0.00 m -> corridor_blocked',
    't=886.20 [VICTIM] approach heading +10deg (+41.5deg): free 0.91 m, keep-out 0.80 m, live '
    'free 0.95 m, live keep-out 0.59 m, move 0.59 m -> ok',
    't=886.20 [VICTIM] approach move 1: victim nav (7.23,7.71) range 1.38 m heading +41.5deg -> '
    'move 0.59 m (free 0.91 m, keep-out 0.80 m, stop at 0.80 m) via approach heading +10deg '
    '(direct blocked)',
    '[STATE] t=886.20 STOP_AND_SCAN -> EXPLORE_TURN (victim_approach_1)',
]
REPORT = {'victim_x': 7.294, 'victim_y': 7.67, 'frame': 'nav', 'distance_m': 0.844,
          'last_measured_distance_m': 1.035, 'confidence': 0.87, 'observations': 30,
          'robot_x': 6.53, 'robot_y': 7.4, 'timestamp': 916.9, 'state': 'VICTIM_HOLD',
          'reason': 'range 0.84 m <= 0.80 m', 'candidate_id': 1, 'priority_score': 0.585448,
          'candidates_considered': 1}
REACHED = ('t=916.90 [VICTIM] REACHED (range 0.84 m <= 0.80 m): /victim_found '
           + json.dumps(REPORT))
HOLD = '[STATE] t=916.90 STOP_AND_SCAN -> VICTIM_HOLD (range 0.84 m <= 0.80 m)'


def feed(model, lines, wall=100.0):
    for text in lines:
        model.on_search_log(text, wall)
    return model


# ---------------------------------------------------------------- read-only guarantees
def test_module_never_publishes():
    src = inspect.getsource(dbm)
    tree = ast.parse(src)
    calls = {n.func.attr for n in ast.walk(tree)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
    assert 'create_publisher' not in calls and 'publish' not in calls
    assert 'create_service' not in calls


@pytest.fixture
def monitor():
    rclpy.init(args=['--ros-args', '-p', 'refresh_sec:=0.5'])
    node = DisasterBotMonitor()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_node_creates_no_motion_or_application_publishers(monitor):
    topics = {p.topic_name for p in monitor.publishers}
    assert '/cmd_vel' not in topics and '/cmd_vel_raw' not in topics
    assert topics <= {'/parameter_events'}           # rclpy-internal only; rosout disabled
    graph = {t for t, _ in monitor.get_publisher_names_and_types_by_node(
        'disaster_bot_monitor', '/')}
    assert '/cmd_vel' not in graph and '/cmd_vel_raw' not in graph and '/rosout' not in graph
    subs = {s.topic_name for s in monitor.subscriptions}
    assert {'/rosout', '/victim_detections', '/victim_distance', '/victim_found',
            '/cmd_vel_raw', '/cmd_vel', '/imu'} <= subs
    assert monitor.refresh_sec == 0.5 and monitor.record_dir == ''


def test_node_tick_without_any_data(monitor, capsys):
    monitor._tick()                                  # nothing published anywhere yet
    out = capsys.readouterr().out
    assert 'WAITING' in out and 'DISASTER BOT MISSION MONITOR' in out


def test_node_refreshes_without_sim_clock(monitor, capsys):
    # use_sim_time is on, but nothing publishes /clock: the wall-clock timer must still fire.
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        rclpy.spin_once(monitor, timeout_sec=0.1)
    assert capsys.readouterr().out.count('DISASTER BOT MISSION MONITOR') >= 3


def test_node_reads_ranking_terms_from_record_dir(tmp_path):
    rclpy.init(args=['--ros-args', '-p', f'record_dir:={tmp_path}'])
    node = DisasterBotMonitor()
    try:
        ev = {'event': 'prioritized', 'ranking': [
            {'id': 1, 'score': 0.585, 'feasible': True, 'confidence': 0.697, 'range_m': 1.38,
             'access_offset_deg': 10.0, 'risk': 0.0,
             'terms': {'confidence': 0.243, 'distance': 0.787, 'access': 0.75}}]}
        with open(tmp_path / 'victim_events.jsonl', 'w') as f:
            f.write(json.dumps({'event': 'reading'}) + '\n' + json.dumps(ev) + '\n{"event": "pa')
        node._poll_record_dir()
        assert node.model.terms[1]['terms']['access'] == 0.75
        pos = node.events_pos
        node._poll_record_dir()                      # partial last line is not consumed
        assert node.events_pos == pos
        feed(node.model, MISSION)
        frame = node.model.render(100.0)
        assert '0.70' in frame and '1.38' in frame and '0.75' in frame
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ---------------------------------------------------------------- mission picture
def test_waiting_render_without_data():
    frame = MonitorModel().render(0.0)
    assert 'WAITING (no autonomous_search log yet)' in frame
    assert 'no confirmed candidate ranked yet' in frame and 'Selected              : none' in frame
    assert 'VICTIM REACHED' not in frame and 'FAULT HOLD' not in frame


def test_mission_progress_from_real_log_lines():
    m = feed(MonitorModel(), MISSION)
    m.max_steps = 30
    assert m.state == 'EXPLORE_TURN' and m.phase == 'INSIDE' and m.step == 1
    assert m.last_move == (1, 1.0, 1.0, 'target_reached')
    frame = m.render(100.5)
    assert 'Exploration steps     : 1 / 30' in frame
    assert '1 candidate ranked at t=886.2 -> #1 (best)' in frame
    assert 'Victim ID             : #1' in frame and 'Priority score        : 0.585' in frame
    assert 'highest feasible priority of 1 candidate ranked' in frame
    assert 'Approach status       : APPROACHING' in frame
    assert '+0 deg corridor_blocked, +10 deg ok' in frame
    assert '+10 deg offset (abs +41.5 deg), move 0.59 m' in frame
    assert 'not published live; pass record_dir' in frame


def test_victim_hold_banner():
    m = feed(MonitorModel(), MISSION + [REACHED, HOLD])
    frame = m.render(100.0)
    assert m.state == 'VICTIM_HOLD' and m.selected['status'] == 'REACHED'
    for text in ('VICTIM REACHED', 'Victim ID             : #1',
                 'Priority score        : 0.585', 'Candidates considered : 1',
                 'Distance (LiDAR est.) : 0.84 m', 'Mission status        : SUCCESS',
                 'Robot is holding position'):
        assert text in frame


def test_victim_found_topic_alone_is_enough():
    m = MonitorModel()
    m.on_victim_found(json.dumps(REPORT))
    assert 'VICTIM REACHED' in m.render(0.0)


def test_fault_hold_banner():
    m = feed(MonitorModel(), MISSION[:5] + [
        't=120.00 [EXPLORE] [TILT_GUARD] roll=+12.0deg pitch=+1.0deg reaches 10.0deg -- '
        'exploration stopped (terminal FAULT_HOLD)',
        '[STATE] t=120.00 EXPLORE_MOVE -> FAULT_HOLD (tilt_guard)'])
    frame = m.render(100.0)
    assert 'MISSION FAULT HOLD' in frame and 'stopped for safety' in frame
    assert 'Reason                : tilt_guard' in frame and 'Fault hold            : YES' in frame
    assert m.tilt_guard == 1 and 'VICTIM REACHED' not in frame


def test_inside_hold_without_victim():
    m = feed(MonitorModel(), MISSION[:6] + [
        't=900.00 [EXPLORE] finished: 30 moves, 31 scans -- holding',
        '[STATE] t=900.00 STOP_AND_SCAN -> INSIDE_HOLD (explore_steps_done)'])
    frame = m.render(100.0)
    assert 'EXPLORATION HOLD - NO VICTIM REACHED' in frame
    assert 'explore_steps_done' in frame and 'step budget used (30 moves)' in frame
    assert 'VICTIM REACHED\n' not in frame.replace('NO VICTIM REACHED', '')


def test_wording_is_robotic_not_medical():
    frame = feed(MonitorModel(), MISSION + [REACHED, HOLD]).render(100.0).lower()
    assert 'not medical triage' in frame
    for bad in ('most injured', 'most critical', 'critical patient', 'injury level',
                'severity:'):
        assert bad not in frame


def test_multi_candidate_ranking_and_infeasible():
    m = feed(MonitorModel(), [
        't=90.60 [VICTIM] prioritized: #2 0.690, #1 0.645, #3 0.148 (no safe heading) -> #2 '
        '(best)'])
    frame = m.render(0.0)
    assert '3 candidates ranked' in frame and m.selected['id'] == 2
    assert m.ranking['items'] == [(2, 0.69, True), (1, 0.645, True), (3, 0.148, False)]
    assert '#3      0.148  NO' in frame
    m2 = feed(MonitorModel(), ['t=90.70 [VICTIM] prioritized: #1 0.148 (no safe heading) -> '
                               'none (none_feasible)'])
    assert m2.selected is None and 'none (none_feasible)' in m2.render(0.0)


def test_cue_ranking_switch_and_abandon():
    m = feed(MonitorModel(), MISSION[:10] + [
        't=90.60 [VICTIM] cues_prioritized: #1 0.784, #2 0.773 -> #1 (best)',
        't=95.00 [VICTIM] target switched #1 -> #2 (margin) at nav (3.0,2.0) before any '
        'approach move',
        't=99.00 [VICTIM] target abandoned: no safe approach heading (range 1.4 m) -- back to '
        'frontier exploration'])
    frame = m.render(0.0)
    assert 'Look-direction ranking: 2 cue groups -> #1' in frame
    assert 'Target switch         : #1 -> #2 (margin)' in frame
    assert 'Abandoned             : t=99.0 no safe approach heading' in frame
    assert m.selected is None and 'Selected              : none' in frame


def test_prioritization_off_confirmation():
    m = feed(MonitorModel(), [MISSION[10]])          # CONFIRMED without a ranking line
    frame = m.render(0.0)
    assert 'n/a (prioritization off)' in frame and 'after 22 readings' in frame


def test_late_start_from_nav_lines_only():
    m = feed(MonitorModel(), ['[NAV] t=500.00 state=EXPLORE_MOVE phase=INSIDE nav=(3.00,4.00) '
                              'imu_yaw=+45.0 odom=(3.0,4.0,+45.0)'])
    assert m.state == 'EXPLORE_MOVE' and m.nav == (3.0, 4.0, 45.0)
    assert 'EXPLORE_MOVE' in m.render(100.0)


def test_unknown_state_text_not_displayed_and_stale_log():
    m = feed(MonitorModel(), ['[STATE] t=1.00 STARTUP -> NOT_A_STATE (x)'], wall=0.0)
    frame = m.render(10.0)
    assert 'NOT_A_STATE' not in frame.split('Last transition')[0]
    m2 = feed(MonitorModel(), ['[STATE] t=1.00 STARTUP -> EXPLORING (sensors_ready)'], wall=0.0)
    assert 'no log for 10 s' in m2.render(10.0)
    assert set(dbm.TERMINAL_STATES) <= set(dbm.STATE_NAMES)


def test_bad_topic_data_is_ignored():
    m = MonitorModel()
    for bad in ('not json', '[]', '{"confidence": "x"}', 'null'):
        m.on_detections(bad, 1.0)
        m.on_victim_distance(bad, 1.0)
        m.on_victim_found(bad)
    assert m.detections is None and m.reading is None and m.victim_found is None
    m.on_detections(json.dumps([{'confidence': 0.6}, {'confidence': 0.9}]), 1.0)
    m.on_victim_distance(json.dumps({'confidence': 0.8, 'distance_m': 2.0, 'angle_deg': -5.0}),
                         1.0)
    frame = m.render(3.0)
    assert '2 person box(es), max conf 0.90' in frame and '2.00 m at -5 deg' in frame


def test_safety_intervention_rule():
    m = MonitorModel()
    m.on_cmd(0.0, 0.0, 0.0)                          # no raw yet: not counted
    m.on_cmd_raw(1.00, 0.20, 0.0)
    m.on_cmd_raw(1.10, 0.20, 0.0)
    m.on_cmd(1.20, 0.20, 0.0)                        # passed through
    m.on_cmd(1.21, 0.0, 0.0)                         # zeroed
    m.on_cmd(1.22, 0.10, 0.0)                        # capped
    m.on_cmd_raw(1.30, 0.0, 0.5)
    m.on_cmd_raw(1.40, 0.0, 0.5)
    m.on_cmd(1.45, 0.0, 0.5)                         # rotation only: never an intervention
    assert (m.zeroed, m.capped) == (1, 1)
    m.on_imu(0.5, -1.0)
    m.on_imu(-0.2, 2.0)
    assert m.roll_pitch_max == (0.5, 2.0)
