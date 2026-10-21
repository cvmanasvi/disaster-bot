"""
Disaster Bot mission monitor: a READ-ONLY terminal dashboard.

    ros2 run my_first_robot disaster_bot_monitor [--ros-args -p record_dir:=<scan_record dir>]

Observer only. It creates no application publishers (rosout logging and parameter services
are disabled for this node) and never commands the robot. Everything shown comes from what the
validated, frozen nodes already publish:

  /rosout              autonomous_search log lines: [STATE] / [NAV] / [MISSION] / [EXPLORE] /
                       [VICTIM] / [KEEPOUT] / [TILT_GUARD] (state, steps, ranking, approach)
  /victim_detections   yolo_detector person boxes (count, confidence)
  /victim_distance     victim_distance_estimator reading (confidence, distance, bearing)
  /victim_found        autonomous_search final report (VICTIM_HOLD)
  /cmd_vel_raw, /cmd_vel   compared to OBSERVE SafetyController interventions (same rule as the
                       mission analysis: raw forward > 0.01 m/s zeroed or capped)
  /imu                 roll / pitch
  graph + autonomous_search parameters (read-only queries): who publishes the velocity topics,
                       explore_max_steps, victim_approach_enabled, victim_prioritization_enabled

Accessibility / risk terms of the ranking are only written to <record_dir>/victim_events.jsonl
(autonomous_search scan_record_dir); they are shown only when record_dir is given. Collisions
are not observable live (no contact topic). The priority score is ROBOTIC prioritization
(confidence, distance, approach accessibility, narrow-obstacle risk), not medical triage.
"""

from collections import deque
import json
import math
import os
import re
import sys
import time

from geometry_msgs.msg import TwistStamped
from my_first_robot.autonomous_search import State
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_rosout_default, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String

SEARCH_NODE = 'autonomous_search'
STATE_NAMES = tuple(s.name for s in State)
TERMINAL_STATES = ('VICTIM_HOLD', 'INSIDE_HOLD', 'FAULT_HOLD')
LOG_STALE_S = 5.0                # no autonomous_search log for this long (wall) -> WAITING
SAFETY_FORWARD_MIN = 0.01        # m/s, same intervention rule as the mission analysis
SAFETY_PAIR_WINDOW_S = 0.06
WIDTH = 66

RE_STATE = re.compile(r'\[STATE\] t=([\d.]+) (\w+) -> (\w+) \(([^)]*)\)')
RE_NAV = re.compile(r'\[NAV\] t=([\d.]+) state=(\w+) phase=(\w+) '
                    r'nav=\(([-\d.]+),([-\d.]+)\) imu_yaw=([-+\d.]+)')
RE_MISSION = re.compile(r't=([\d.]+) \[MISSION\] (\w+) -> (\w+) \(([^)]*)\)')
RE_T = re.compile(r'\bt=([\d.]+)')
RE_STEP = re.compile(r'\[EXPLORE\] step (\d+): ')
RE_STEP_DONE = re.compile(r'\[EXPLORE\] step (\d+) move done: target=([\d.]+)m '
                          r'achieved=([\d.]+)m reason=(\w+)')
RE_FINISHED = re.compile(r'\[EXPLORE\] finished: (\d+) moves')
RE_STOPPED = re.compile(r'\[EXPLORE\] exploration stopped: (.*) -- holding')
RE_RANK = re.compile(r'\[VICTIM\] (prioritized|reprioritized|cues_prioritized): (.*) -> '
                     r'(#\d+|none) \(([^)]*)\)')
RE_RANK_ITEM = re.compile(r'#(\d+) ([-\d.]+)( \(no safe heading\))?')
RE_CONFIRMED = re.compile(r'\[VICTIM\] CONFIRMED at nav \(([-\d.]+),([-\d.]+)\) after (\d+) '
                          r'readings \(spread ([\d.]+) m, conf ([\d.]+)\)')
RE_HEADING = re.compile(r'\[VICTIM\] approach heading ([-+]\d+)deg \(([-+\d.]+)deg\):.* -> (\w+)$')
RE_MOVE = re.compile(r'\[VICTIM\] approach move (\d+): victim nav \(([-\d.]+),([-\d.]+)\) '
                     r'range ([\d.]+) m heading ([-+\d.]+)deg -> move ([\d.]+) m')
RE_VIA = re.compile(r'via approach heading ([-+]\d+)deg')
RE_LOOK = re.compile(r'\[VICTIM\] look (\d+):')
RE_ABANDON = re.compile(r'\[VICTIM\] target abandoned: (.*) -- back to frontier exploration')
RE_SWITCH = re.compile(r'\[VICTIM\] target switched #(\d+) -> #(\d+) \((\w+)\)')
RE_REACHED = re.compile(r'\[VICTIM\] REACHED \(([^)]*)\): /victim_found (\{.*\})')

PRIORITY_BASIS = ('detection confidence, estimated distance,',
                  'approach accessibility, narrow-obstacle risk')
PRIORITY_NOTE = 'Robotic prioritization only - not medical triage; injury severity is unknown.'


def _f(value, fmt, missing='WAITING'):
    return missing if value is None else format(value, fmt)


def _clock(seconds):
    if seconds is None:
        return 'WAITING'
    seconds = int(seconds)
    return f'{seconds // 60:02d}:{seconds % 60:02d}'


def quaternion_roll_pitch(q):
    """Return (roll, pitch) in degrees."""
    roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
    return roll, pitch


class MonitorModel:
    """Mission picture built only from observed messages (no ROS dependency)."""

    def __init__(self):
        self.state = None
        self.state_since = None
        self.phase = None
        self.sim_t = None
        self.nav = None
        self.transitions = deque(maxlen=4)
        self.last_log_wall = None
        self.step = 0
        self.max_steps = None
        self.approach_enabled = None
        self.prioritization_enabled = None
        self.last_move = None
        self.explore_end = None
        self.detections = None          # (count, max confidence, wall time)
        self.reading = None             # (confidence, distance, bearing, wall time)
        self.readings = 0
        self.ranking = None             # dict: kind, items, chosen, reason, t
        self.cue_ranking = None
        self.terms = {}                 # id -> ranking dict from victim_events.jsonl
        self.selected = None            # dict: id, score, reason, confirmed, status
        self.headings = []              # [(offset, absolute heading, result)] latest decision
        self.headings_t = None
        self.approach = None            # dict: n, heading, move, offset, range
        self.looks = 0
        self.abandoned = []
        self.switches = []
        self.victim_found = None
        self.hold_reason = None
        self.fault_reason = None
        self.tilt_guard = 0
        self.keepout_cuts = 0
        self.rotation_refusals = 0
        self.raw = deque(maxlen=400)    # (sim t, linear) of /cmd_vel_raw
        self.raw_last = None
        self.cmd_last = None
        self.zeroed = 0
        self.capped = 0
        self.roll_pitch_max = None
        self.cmd_vel_publishers = None
        self.cmd_vel_raw_publishers = None
        self.nodes = None

    # ------------------------------------------------------------ /rosout
    def on_search_log(self, text, wall):
        self.last_log_wall = wall
        m = RE_T.search(text)
        if m:
            self.sim_t = float(m.group(1))
        m = RE_NAV.search(text)
        if m:
            if self.state != m.group(2):
                self.state, self.state_since = m.group(2), float(m.group(1))
            self.phase = m.group(3)
            self.nav = (float(m.group(4)), float(m.group(5)), float(m.group(6)))
            return
        m = RE_STATE.search(text)
        if m:
            t, old, new, why = float(m.group(1)), m.group(2), m.group(3), m.group(4)
            self.state, self.state_since = new, t
            self.transitions.append((t, old, new, why))
            if new == 'INSIDE_HOLD':
                self.hold_reason = why
            if new == 'FAULT_HOLD':
                self.fault_reason = why
            return
        m = RE_MISSION.search(text)
        if m:
            self.phase = m.group(3)
            return
        self._explore(text)
        self._victim(text)
        if '[TILT_GUARD]' in text:
            self.tilt_guard += 1
        if '[KEEPOUT] live:' in text:
            self.keepout_cuts += 1
        if 'scan rotation refused' in text:
            self.rotation_refusals += 1

    def _explore(self, text):
        m = RE_STEP.search(text)
        if m:
            self.step = max(self.step, int(m.group(1)))
        m = RE_STEP_DONE.search(text)
        if m:
            self.last_move = (int(m.group(1)), float(m.group(2)), float(m.group(3)), m.group(4))
        m = RE_FINISHED.search(text)
        if m:
            self.explore_end = f'step budget used ({m.group(1)} moves)'
        m = RE_STOPPED.search(text)
        if m:
            self.explore_end = m.group(1)

    def _victim(self, text):
        t = self.sim_t
        m = RE_RANK.search(text)
        if m:
            items = [(int(a), float(b), not c) for a, b, c in RE_RANK_ITEM.findall(m.group(2))]
            rank = {'kind': m.group(1), 'items': items, 'chosen': m.group(3),
                    'reason': m.group(4), 't': t}
            if m.group(1) == 'cues_prioritized':
                self.cue_ranking = rank
            else:
                self.ranking = rank
                chosen = int(m.group(3)[1:]) if m.group(3).startswith('#') else None
                if chosen is not None and (self.selected is None
                                           or self.selected['id'] != chosen):
                    score = next((s for i, s, _ in items if i == chosen), None)
                    self.selected = {'id': chosen, 'score': score, 'reason': m.group(4),
                                     'confirmed': None, 'status': 'SELECTED',
                                     'considered': len(items)}
            return
        m = RE_CONFIRMED.search(text)
        if m:
            if self.selected is None:       # prioritization off: confirmed without a ranking
                self.selected = {'id': None, 'score': None, 'reason': 'confirmed',
                                 'confirmed': None, 'status': 'SELECTED', 'considered': None}
            self.selected['confirmed'] = (float(m.group(1)), float(m.group(2)),
                                          int(m.group(3)), float(m.group(5)))
            return
        m = RE_HEADING.search(text)
        if m:
            if self.headings_t != t:
                self.headings, self.headings_t = [], t
            self.headings.append((int(m.group(1)), float(m.group(2)), m.group(3)))
            return
        m = RE_MOVE.search(text)
        if m:
            via = RE_VIA.search(text)
            self.approach = {'n': int(m.group(1)), 'range': float(m.group(4)),
                             'heading': float(m.group(5)), 'move': float(m.group(6)),
                             'offset': int(via.group(1)) if via else 0}
            if self.selected is not None:
                self.selected['status'] = 'APPROACHING'
            return
        m = RE_LOOK.search(text)
        if m:
            self.looks = int(m.group(1))
            return
        m = RE_ABANDON.search(text)
        if m:
            self.abandoned.append((t, m.group(1)))
            if self.selected is not None:
                self.selected['status'] = 'ABANDONED'
            self.selected, self.approach, self.headings = None, None, []
            return
        m = RE_SWITCH.search(text)
        if m:
            self.switches.append((t, int(m.group(1)), int(m.group(2)), m.group(3)))
            return
        m = RE_REACHED.search(text)
        if m:
            self.hold_reason = m.group(1)
            self.on_victim_found(m.group(2))

    # ------------------------------------------------------------ topics
    def on_victim_found(self, data):
        try:
            report = json.loads(data)
        except (TypeError, ValueError):
            return
        if isinstance(report, dict) and 'victim_x' in report and 'state' in report:
            self.victim_found = report
            if self.selected is not None:
                self.selected['status'] = 'REACHED'

    def on_detections(self, data, wall):
        try:
            boxes = json.loads(data)
            conf = max(float(b.get('confidence', 0.0)) for b in boxes)
        except (TypeError, ValueError, AttributeError):
            return
        self.detections = (len(boxes), conf, wall)

    def on_victim_distance(self, data, wall):
        try:
            d = json.loads(data)
            reading = (float(d['confidence']), float(d['distance_m']), float(d['angle_deg']), wall)
        except (TypeError, ValueError, KeyError):
            return
        self.reading = reading
        self.readings += 1

    def on_ranking_event(self, event):
        """Ranking terms from victim_events.jsonl (only when record_dir is given)."""
        if event.get('event') in ('prioritized', 'reprioritized'):
            for r in event.get('ranking', []):
                self.terms[r.get('id')] = r

    def on_cmd_raw(self, t, linear, angular):
        self.raw.append((t, linear))
        self.raw_last = (linear, angular)

    def on_cmd(self, t, linear, angular):
        """Count SafetyController interventions (raw forward zeroed or capped)."""
        self.cmd_last = (linear, angular)
        before = [r for r in self.raw if r[0] <= t - SAFETY_PAIR_WINDOW_S]
        upto = [r for r in self.raw if r[0] <= t]
        if not before or not upto:
            return
        raw = min(before[-1][1], upto[-1][1])
        if raw > SAFETY_FORWARD_MIN and abs(linear) < 1e-4:
            self.zeroed += 1
        elif raw > SAFETY_FORWARD_MIN and linear < raw - 0.005:
            self.capped += 1

    def on_imu(self, roll, pitch):
        r, p = abs(roll), abs(pitch)
        if self.roll_pitch_max is None:
            self.roll_pitch_max = (r, p)
        else:
            self.roll_pitch_max = (max(self.roll_pitch_max[0], r), max(self.roll_pitch_max[1], p))

    # ------------------------------------------------------------ rendering
    def render(self, wall):
        out = []
        bar = '=' * WIDTH
        line = '-' * WIDTH

        def head(title):
            out.extend(['', title, line])

        def row(name, value):
            out.append(f'{name:<22}: {value}')

        out += [bar, 'DISASTER BOT MISSION MONITOR  (read-only observer)'.center(WIDTH), bar]
        stale = self.last_log_wall is None or wall - self.last_log_wall > LOG_STALE_S

        head('MISSION')
        state = self.state if self.state in STATE_NAMES else None
        row('State', 'WAITING (no autonomous_search log yet)' if state is None else
            state + (f'  (since t={self.state_since:.1f} s)' if self.state_since else ''))
        if state is not None and stale:
            row('Search node', f'no log for {wall - self.last_log_wall:.0f} s (stopped?)')
        row('Mission phase', self.phase or 'WAITING')
        row('Mission time (sim)', _clock(self.sim_t))
        steps = f'{self.step} / {self.max_steps}' if self.max_steps else f'{self.step} / WAITING'
        row('Exploration steps', steps)
        if self.last_move:
            n, target, done, why = self.last_move
            row('Last move', f'step {n}: {done:.2f} of {target:.2f} m ({why})')
        if self.explore_end:
            row('Exploration ended', self.explore_end)
        if self.nav:
            row('Nav pose (estimate)', f'({self.nav[0]:.2f}, {self.nav[1]:.2f}) m, '
                f'yaw {self.nav[2]:+.0f} deg')
        row('Victim approach', _onoff(self.approach_enabled))
        row('Prioritization', _onoff(self.prioritization_enabled))
        if self.transitions:
            t, old, new, why = self.transitions[-1]
            row('Last transition', f't={t:.1f} {old} -> {new} ({why})')

        head('VICTIM DETECTION')
        if self.detections:
            n, conf, w = self.detections
            row('YOLO (latest)', f'{n} person box(es), max conf {conf:.2f}, {wall - w:.0f} s ago')
        else:
            row('YOLO (latest)', 'WAITING (no person detected yet)')
        if self.reading:
            conf, dist, ang, w = self.reading
            row('Distance reading', f'{dist:.2f} m at {ang:+.0f} deg, conf {conf:.2f}, '
                f'{wall - w:.0f} s ago ({self.readings} total)')
        else:
            row('Distance reading', 'WAITING')
        row('Look-toward-cue turns', str(self.looks))

        head('VICTIM PRIORITIZATION')
        row('Priority basis', PRIORITY_BASIS[0])
        out.append(' ' * 24 + PRIORITY_BASIS[1])
        out.append(f'  {PRIORITY_NOTE}')
        if self.ranking:
            items = self.ranking['items']
            row('Latest ranking', f'{len(items)} candidate{"s" if len(items) != 1 else ""} '
                f'ranked at t={self.ranking["t"]:.1f} -> {self.ranking["chosen"]} '
                f'({self.ranking["reason"]})')
            out.append(f'  {"ID":<5}{"Score":>8}  {"Feasible":<9}{"Conf":>6}{"Range":>9}'
                       f'{"Access":>8}{"Risk":>6}')
            for cid, score, feasible in items:
                tm = self.terms.get(cid, {})
                conf = _f(tm.get('confidence'), '.2f', '-')
                rng = _f(tm.get('range_m'), '.2f', '-')
                acc = _f(tm.get('terms', {}).get('access'), '.2f', '-')
                risk = _f(tm.get('risk'), '.2f', '-')
                out.append(f'  #{cid:<4}{score:>8.3f}  {"yes" if feasible else "NO":<9}'
                           f'{conf:>6}{rng:>9}{acc:>8}{risk:>6}')
            if not self.terms:
                out.append('  (conf/range/access/risk terms: not published live; '
                           'pass record_dir to show them)')
        else:
            row('Latest ranking', 'no confirmed candidate ranked yet')
        if self.cue_ranking:
            c = self.cue_ranking
            row('Look-direction ranking', f'{len(c["items"])} cue groups -> {c["chosen"]}')

        head('SELECTED VICTIM')
        s = self.selected
        if s is None:
            row('Selected', 'none')
        else:
            row('Victim ID', f'#{s["id"]}' if s['id'] is not None else 'n/a (prioritization off)')
            row('Priority score', _f(s['score'], '.3f', 'n/a'))
            if s['considered'] is not None:
                row('Why selected', f'{s["reason"]}: highest feasible priority of '
                    f'{s["considered"]} candidate{"s" if s["considered"] != 1 else ""} ranked')
            if s['confirmed']:
                x, y, n, conf = s['confirmed']
                row('Confirmed', f'nav ({x:.2f}, {y:.2f}) after {n} readings, max conf {conf:.2f}')
            row('Approach status', s['status'])
        if self.headings:
            row('Heading candidates', ', '.join(f'{o:+d} deg {r}' for o, _, r in self.headings))
        if self.approach:
            a = self.approach
            row('Approach heading', f'{a["offset"]:+d} deg offset (abs {a["heading"]:+.1f} deg), '
                f'move {a["move"]:.2f} m from range {a["range"]:.2f} m (move {a["n"]})')
        for t, why in self.abandoned[-2:]:
            row('Abandoned', f't={t:.1f} {why}' if t is not None else why)
        for t, a, b, why in self.switches[-2:]:
            row('Target switch', f'#{a} -> #{b} ({why})')

        head('SAFETY  (observed; SafetyController untouched)')
        row('/cmd_vel publisher', self.cmd_vel_publishers or 'WAITING')
        row('/cmd_vel_raw publisher', self.cmd_vel_raw_publishers or 'WAITING')
        row('SC interventions', f'{self.zeroed} zeroed / {self.capped} capped '
            '(cmd_vel_raw vs cmd_vel, since monitor start)')
        if self.raw_last and self.cmd_last:
            row('Latest command', f'raw {self.raw_last[0]:+.2f} m/s {self.raw_last[1]:+.2f} rad/s'
                f' -> sent {self.cmd_last[0]:+.2f} m/s {self.cmd_last[1]:+.2f} rad/s')
        rp = self.roll_pitch_max
        row('Tilt', 'WAITING' if rp is None else
            f'max |roll| {rp[0]:.2f} deg, |pitch| {rp[1]:.2f} deg; tilt-guard events '
            f'{self.tilt_guard}')
        row('Keep-out', f'live cuts {self.keepout_cuts}, rotation refusals '
            f'{self.rotation_refusals}')
        row('Fault hold', 'YES' if self.state == 'FAULT_HOLD' else 'no')
        row('Collisions', 'not observable live (no contact topic); see mission report')

        out += self._banner()
        if self.nodes:
            out += ['', 'Nodes: ' + self.nodes]
        out.append(bar)
        return '\n'.join(out)

    def _banner(self):
        bar = '=' * WIDTH
        if self.state == 'VICTIM_HOLD' or (self.victim_found and self.state is None):
            r = self.victim_found or {}
            cid = r.get('candidate_id')
            return ['', bar, 'VICTIM REACHED'.center(WIDTH), bar,
                    f'Victim ID             : {"#" + str(cid) if cid is not None else "n/a"}',
                    f'Priority score        : {_f(r.get("priority_score"), ".3f", "n/a")}',
                    f'Candidates considered : {r.get("candidates_considered", "n/a")}',
                    f'Distance (LiDAR est.) : {_f(r.get("distance_m"), ".2f", "WAITING")} m',
                    f'Max confidence        : {_f(r.get("confidence"), ".2f", "WAITING")}',
                    f'Victim (nav estimate) : ({_f(r.get("victim_x"), ".2f")}, '
                    f'{_f(r.get("victim_y"), ".2f")})',
                    f'Reason                : {self.hold_reason or r.get("reason", "WAITING")}',
                    'Mission status        : SUCCESS',
                    'Robot is holding position (VICTIM_HOLD).']
        if self.state == 'FAULT_HOLD':
            return ['', bar, 'MISSION FAULT HOLD'.center(WIDTH), bar,
                    f'Reason                : {self.fault_reason or "WAITING"}',
                    'The robot has stopped for safety (zero velocity is commanded).']
        if self.state == 'INSIDE_HOLD':
            return ['', bar, 'EXPLORATION HOLD - NO VICTIM REACHED'.center(WIDTH), bar,
                    f'Reason                : {self.hold_reason or "WAITING"}',
                    f'Exploration           : {self.explore_end or "ended"}',
                    'The robot is holding position without a victim.']
        return []


def _onoff(value):
    if value is None:
        return 'WAITING'
    return 'ENABLED' if value else 'disabled'


class DisasterBotMonitor(Node):
    """Subscribes and displays; creates no publishers and sends no commands."""

    def __init__(self):
        super().__init__(
            'disaster_bot_monitor', enable_rosout=False, start_parameter_services=False,
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.record_dir = str(self.declare_parameter('record_dir', '').value)
        self.refresh_sec = max(0.2, float(self.declare_parameter('refresh_sec', 1.0).value))
        self.model = MonitorModel()
        self.tty = sys.stdout.isatty()
        self.events_pos = 0
        self.create_subscription(Log, '/rosout', self._rosout, qos_profile_rosout_default)
        self.create_subscription(String, '/victim_detections', self._detections, 10)
        self.create_subscription(String, '/victim_distance', self._distance, 10)
        self.create_subscription(String, '/victim_found', self._found, 10)
        self.create_subscription(TwistStamped, '/cmd_vel_raw', self._raw, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self._cmd, 10)
        self.create_subscription(Imu, '/imu', self._imu, qos_profile_sensor_data)
        self.param_client = self.create_client(GetParameters, f'/{SEARCH_NODE}/get_parameters')
        self.param_future = None
        # Wall-clock refresh: the node uses sim time (for pairing velocity messages), and a
        # sim-time timer would never fire before Gazebo publishes /clock.
        self.create_timer(self.refresh_sec, self._tick,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _rosout(self, msg):
        if msg.name == SEARCH_NODE:
            self.model.on_search_log(msg.msg, time.monotonic())

    def _detections(self, msg):
        self.model.on_detections(msg.data, time.monotonic())

    def _distance(self, msg):
        self.model.on_victim_distance(msg.data, time.monotonic())

    def _found(self, msg):
        self.model.on_victim_found(msg.data)

    def _raw(self, msg):
        self.model.on_cmd_raw(self._now(), msg.twist.linear.x, msg.twist.angular.z)

    def _cmd(self, msg):
        self.model.on_cmd(self._now(), msg.twist.linear.x, msg.twist.angular.z)

    def _imu(self, msg):
        self.model.on_imu(*quaternion_roll_pitch(msg.orientation))

    def _poll_parameters(self):
        m = self.model
        if m.max_steps is not None:
            return
        if self.param_future is not None and self.param_future.done():
            try:
                values = self.param_future.result().values
                if len(values) == 3:
                    m.max_steps = values[0].integer_value or None
                    m.approach_enabled = values[1].bool_value
                    m.prioritization_enabled = values[2].bool_value
            except Exception:  # noqa: BLE001 - search node gone mid-call: retry later
                pass
            self.param_future = None
        if self.param_future is None and self.param_client.service_is_ready():
            req = GetParameters.Request()
            req.names = ['explore_max_steps', 'victim_approach_enabled',
                         'victim_prioritization_enabled']
            self.param_future = self.param_client.call_async(req)

    def _poll_graph(self):
        def pubs(topic):
            names = [i.node_name for i in self.get_publishers_info_by_topic(topic)]
            return ', '.join(f'{n} ({names.count(n)})' for n in sorted(set(names))) or 'none'
        self.model.cmd_vel_publishers = pubs('/cmd_vel')
        self.model.cmd_vel_raw_publishers = pubs('/cmd_vel_raw')
        names = list(self.get_node_names())
        ours = ('autonomous_search', 'safety_controller', 'yolo_detector',
                'victim_distance_estimator')
        self.model.nodes = ', '.join(f'{n} x{names.count(n)}' for n in ours)

    def _poll_record_dir(self):
        if not self.record_dir:
            return
        path = os.path.join(self.record_dir, 'victim_events.jsonl')
        try:
            with open(path, 'rb') as f:
                f.seek(self.events_pos)
                for line in f:
                    if not line.endswith(b'\n'):
                        break                      # partial line: read it next time
                    self.events_pos += len(line)
                    try:
                        self.model.on_ranking_event(json.loads(line))
                    except ValueError:
                        continue
        except OSError:
            return

    def _tick(self):
        self._poll_parameters()
        self._poll_graph()
        self._poll_record_dir()
        frame = self.model.render(time.monotonic())
        if self.tty:
            sys.stdout.write('\033[H\033[2J' + frame + '\n')
        else:
            sys.stdout.write(frame + '\n\n')
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = DisasterBotMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
