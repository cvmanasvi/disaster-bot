"""Synthetic behavioral tests for the CROSSING_OPENING ALIGN early-abandon
mechanism added to autonomous_search.py.

These drive AutonomousSearch.handle_crossing_opening() directly with a fake
pose object and a monotonically advancing fake sim clock, so no Gazebo,
scan, or odom messages are needed. They only touch ALIGN-phase behavior and
do not exercise EXPLORING/TURNING/FOLLOWING_BOUNDARY/RECOVERY/FAULT_HOLD.
"""

import math

import pytest
import rclpy

from my_first_robot.autonomous_search import AutonomousSearch, Pose2D, State


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class FakeClock:
    """Lets a test move the node's sim time forward deterministically."""

    def __init__(self, start_sec=100.0):
        self._sec = start_sec

    def advance(self, dt):
        self._sec += dt

    def now_ns(self):
        return int(self._sec * 1e9)


def make_node(fake_clock):
    node = AutonomousSearch()

    # Route _now_sec() through the fake clock instead of the ROS clock so
    # tests control simulated time precisely and deterministically.
    node._now_sec = lambda: fake_clock.now_ns() / 1e9

    return node


def begin_align(node, fake_clock, target_yaw, start_yaw, fallback=State.FOLLOWING_BOUNDARY):
    """Puts the node directly into CROSSING_OPENING / ALIGN, bypassing
    opening detection (which is not what this fix touches)."""

    node.latest_pose = Pose2D(0.0, 0.0, start_yaw)
    node.crossing_fallback_state = fallback
    node.crossing_phase = 'ALIGN'
    node.crossing_target_yaw = target_yaw
    node.crossing_target_distance = 1.0
    node.crossing_align_start_time = fake_clock.now_ns() / 1e9
    node.crossing_align_best_error_deg = None
    node.crossing_align_best_error_time = 0.0
    node.state = State.CROSSING_OPENING


def run_align_ticks(node, fake_clock, yaw_schedule, dt):
    """Feeds one control tick per (dt, yaw) pair. yaw_schedule is a list of
    yaw values applied *before* each tick (simulating odom updates)."""

    sectors = {'front': 5.0, 'front_left': 5.0, 'front_right': 5.0, 'left': 5.0, 'right': 5.0, 'rear': 5.0}

    for yaw in yaw_schedule:
        fake_clock.advance(dt)
        node.latest_pose = Pose2D(node.latest_pose.x, node.latest_pose.y, yaw)
        node.handle_crossing_opening(sectors, fake_clock.now_ns() / 1e9)
        if node.state != State.CROSSING_OPENING:
            break


def test_alignment_improving_continues_crossing():
    """A: heading error shrinks steadily -> should reach DRIVE, never abandon."""
    fake_clock = FakeClock()
    node = make_node(fake_clock)
    try:
        target_yaw = math.radians(0.0)
        begin_align(node, fake_clock, target_yaw, start_yaw=math.radians(40.0))

        # Steadily closing the gap from 40deg toward 0deg, well within the
        # crossing_align_stall_timeout_sec window at each step.
        yaw_schedule = [math.radians(y) for y in (34, 28, 22, 16, 10, 6)]
        run_align_ticks(node, fake_clock, yaw_schedule, dt=0.5)

        assert node.state == State.CROSSING_OPENING
        assert node.crossing_phase == 'DRIVE'
    finally:
        node.destroy_node()


def test_alignment_not_improving_early_abandons():
    """B: heading error stays flat (stuck oscillating around one value) ->
    should abandon well before the full opening_align_timeout_sec (6.0s)."""
    fake_clock = FakeClock()
    node = make_node(fake_clock)
    try:
        target_yaw = math.radians(0.0)
        begin_align(node, fake_clock, target_yaw, start_yaw=math.radians(40.0))

        # Error hovers around 40deg the whole time -- no real progress.
        yaw_schedule = [math.radians(y) for y in (40, 41, 40, 39, 40, 41, 40, 40, 40, 40, 40, 40)]
        run_align_ticks(node, fake_clock, yaw_schedule, dt=0.5)

        assert node.state == node.crossing_fallback_state
        assert node.state == State.FOLLOWING_BOUNDARY

        elapsed = fake_clock.now_ns() / 1e9 - node.crossing_align_start_time
        # Must abandon well before the old fixed 6.0s timeout.
        assert elapsed < node.p.opening_align_timeout_sec
    finally:
        node.destroy_node()


def test_noisy_but_improving_alignment_does_not_abandon():
    """C: one noisy/worse tick in the middle of an otherwise-improving
    sequence must not trigger early abandonment."""
    fake_clock = FakeClock()
    node = make_node(fake_clock)
    try:
        target_yaw = math.radians(0.0)
        begin_align(node, fake_clock, target_yaw, start_yaw=math.radians(40.0))

        # Overall improving trend with one noisy regression (24 -> 27)
        # inserted *after* a baseline best-error has already been recorded,
        # so it genuinely exercises the "ignore one bad sample" path.
        yaw_schedule = [math.radians(y) for y in (38, 34, 30, 24, 27, 18, 12, 6)]
        run_align_ticks(node, fake_clock, yaw_schedule, dt=0.5)

        assert node.state == State.CROSSING_OPENING
        assert node.crossing_phase == 'DRIVE'
    finally:
        node.destroy_node()


def test_existing_full_timeout_fallback_still_works():
    """D: the original opening_align_timeout_sec backstop must still fire
    on its own. Isolated from the new mechanism by neutralizing
    crossing_align_stall_timeout_sec (set far larger than the timeout), so
    a passing test here specifically exercises the pre-existing timeout
    branch, not the new early-abandon branch."""
    fake_clock = FakeClock()
    node = make_node(fake_clock)
    try:
        node.p.crossing_align_stall_timeout_sec = 100.0

        target_yaw = math.radians(0.0)
        begin_align(node, fake_clock, target_yaw, start_yaw=math.radians(40.0))

        # Never improves, never reaches tolerance -- with early-abandon
        # neutralized, only the 6.0s opening_align_timeout_sec can end this.
        yaw_schedule = [math.radians(40.0)] * 20
        run_align_ticks(node, fake_clock, yaw_schedule, dt=0.5)

        assert node.state == node.crossing_fallback_state
        elapsed = fake_clock.now_ns() / 1e9 - node.crossing_align_start_time
        assert elapsed >= node.p.opening_align_timeout_sec
    finally:
        node.destroy_node()
