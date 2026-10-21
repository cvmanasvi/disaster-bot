"""
Victim prioritizer for autonomous_search (victim_prioritization_enabled).

Pure ranking of victim candidates: nothing here publishes, subscribes or
commands motion. autonomous_search calls it only at its post-scan decision
point (robot stationary after a complete 360-degree scan) and hands the
chosen candidate to its unchanged confirmation / approach code, so every
approach move still goes through the existing checks, /cmd_vel_raw and the
SafetyController.

Only information the system actually measures is used. There is no severity
term: the YOLO detector reports a person box and a confidence, nothing about
the person's condition.

    confidence     MEAN YOLO confidence of the candidate's agreeing readings
                   (VictimTrack.mean_confidence). /victim_found keeps
                   reporting the MAX (VictimTrack.confidence) as before; the
                   two are deliberately different figures.
    distance       range from the robot to the candidate's nav-frame estimate
    accessibility  first approach heading (0, +-10, +-20 deg off the victim
                   bearing) that passes autonomous_search's own approach
                   checks (corridor, remembered narrow keep-out, entrance
                   guard, live scan); none passes -> not selectable now.
                   The pre-rotation check is NOT part of scoring (it starts an
                   escape as a side effect); the approach still runs it.
                   ACCESS_BY_OFFSET must cover every |offset| in
                   autonomous_search.VICTIM_HEADING_OFFSETS_DEG (tested).
    risk           how close a scanned / remembered narrow obstacle comes to
                   the DIRECT robot -> stop-point line, beyond the keep-out
                   radius (also when the accessible heading is an offset);
                   distance is to the segment, so an obstacle near the
                   robot's own position counts in any direction

    score = 0.35 * confidence + 0.35 * distance + 0.30 * accessibility
            - 0.20 * risk                          (all terms in [0, 1])

Ranking is deterministic: feasible first, then higher score (rounded to
1e-6), shorter range, higher confidence, lower candidate id. A current
target is only replaced by a candidate scoring at least SWITCH_MARGIN more
(no switching on small fluctuations), and never while locked.
"""

import math

MIN_CONFIDENCE = 0.60          # autonomous_search VICTIM_MIN_CONFIDENCE
MAX_RANGE_M = 3.5              # autonomous_search VICTIM_MAX_RANGE_M
WEIGHT_CONFIDENCE = 0.35
WEIGHT_DISTANCE = 0.35
WEIGHT_ACCESS = 0.30
WEIGHT_RISK = 0.20
SWITCH_MARGIN = 0.10           # score advantage needed to replace a target
RISK_BAND_M = 0.30             # risk falls from 1 at the keep-out radius to 0 this far beyond
# Accessibility by |offset| of the first approach heading that passes.
ACCESS_BY_OFFSET = {0.0: 1.0, 10.0: 0.75, 20.0: 0.50}
SCORE_DECIMALS = 6


def clamp01(value):
    return max(0.0, min(1.0, value))


def confidence_score(confidence):
    """Map a confidence in [MIN_CONFIDENCE, 1] to [0, 1]."""
    return clamp01((confidence - MIN_CONFIDENCE) / (1.0 - MIN_CONFIDENCE))


def distance_score(range_m, stop_m):
    """Return 1 at the stop distance, 0 at MAX_RANGE_M (closer = better)."""
    span = MAX_RANGE_M - stop_m
    if span <= 0.0:
        return 1.0
    return clamp01(1.0 - (range_m - stop_m) / span)


def accessibility_score(offset_deg):
    """Return the accessibility of an approach heading offset (None = no safe heading)."""
    if offset_deg is None:
        return 0.0
    return ACCESS_BY_OFFSET.get(abs(float(offset_deg)), 0.0)


def segment_distance(px, py, ex, ey):
    """Return the distance from (px, py) to the segment (0, 0)-(ex, ey)."""
    length_sq = ex * ex + ey * ey
    if length_sq <= 0.0:
        return math.hypot(px, py)
    t = max(0.0, min(1.0, (px * ex + py * ey) / length_sq))
    return math.hypot(px - t * ex, py - t * ey)


def risk_score(obstacles, end_x, end_y, keep_radius_m, victim_xy=None, person_radius_m=0.0):
    """
    Return the narrow-obstacle risk of the straight approach (0 = none, 1 = at keep-out).

    obstacles are (x, y) relative to the robot, the approach runs from the
    robot to (end_x, end_y). Obstacles within person_radius_m of victim_xy
    (same relative frame) are the person's own legs and are ignored.
    """
    risk = 0.0
    for ox, oy in obstacles:
        if victim_xy is not None and math.hypot(
                ox - victim_xy[0], oy - victim_xy[1]) <= person_radius_m:
            continue
        d = segment_distance(ox, oy, end_x, end_y)
        risk = max(risk, clamp01(1.0 - (d - keep_radius_m) / RISK_BAND_M))
    return risk


def priority_score(conf_s, dist_s, access_s, risk_s):
    return (WEIGHT_CONFIDENCE * conf_s + WEIGHT_DISTANCE * dist_s
            + WEIGHT_ACCESS * access_s - WEIGHT_RISK * risk_s)


class VictimCandidate:
    """One scored victim candidate (a confirmed track or a cue group)."""

    def __init__(self, cid, confidence, range_m, access_offset_deg, risk=0.0, stop_m=0.80):
        self.cid = cid
        self.confidence = confidence
        self.range_m = range_m
        self.access_offset_deg = access_offset_deg
        self.risk = risk
        self.feasible = access_offset_deg is not None
        self.conf_score = confidence_score(confidence)
        self.dist_score = distance_score(range_m, stop_m)
        self.access_score = accessibility_score(access_offset_deg)
        self.score = round(priority_score(
            self.conf_score, self.dist_score, self.access_score, risk), SCORE_DECIMALS)

    def as_dict(self):
        return {'id': self.cid, 'score': self.score, 'feasible': self.feasible,
                'confidence': round(self.confidence, 3), 'range_m': round(self.range_m, 3),
                'access_offset_deg': self.access_offset_deg, 'risk': round(self.risk, 3),
                'terms': {'confidence': round(self.conf_score, 3),
                          'distance': round(self.dist_score, 3),
                          'access': round(self.access_score, 3)}}


def rank_key(c):
    return (0 if c.feasible else 1, -c.score, round(c.range_m, SCORE_DECIMALS),
            -round(c.confidence, SCORE_DECIMALS), c.cid)


def rank(candidates):
    """Return the candidates best first (deterministic)."""
    return sorted(candidates, key=rank_key)


def choose(candidates, current_id=None, locked=False, margin=SWITCH_MARGIN):
    """
    Pick the target among candidates.

    Returns (candidate or None, reason). With no current target the best
    feasible candidate wins ('best'; None = 'none_feasible'). With a current
    target (current_id among candidates):
      locked                        -> keep it ('locked'); autonomous_search
                                       enforces its lock by never calling
                                       this after the first approach move
      current not feasible          -> best feasible other ('current_infeasible'),
                                       or keep it when there is none
      best other >= current + margin -> switch ('margin')
      otherwise                     -> keep it ('hysteresis')
    """
    ranked = rank(candidates)
    feasible = [c for c in ranked if c.feasible]
    current = None
    if current_id is not None:
        current = next((c for c in candidates if c.cid == current_id), None)
    if current is None:
        return (feasible[0], 'best') if feasible else (None, 'none_feasible')
    if locked:
        return current, 'locked'
    others = [c for c in feasible if c.cid != current_id]
    if not current.feasible:
        return (others[0], 'current_infeasible') if others else (current, 'no_alternative')
    if others and others[0].score >= current.score + margin - 10.0 ** -SCORE_DECIMALS:
        return others[0], 'margin'
    return current, 'hysteresis'
