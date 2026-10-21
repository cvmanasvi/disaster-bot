"""MVP victim rescue controller.

Sits between autonomous_search and safety_controller:

    autonomous_search --/cmd_vel_search--> victim_rescue --/cmd_vel_raw--> safety_controller

Run autonomous_search with:
    -r /cmd_vel_raw:=/cmd_vel_search -r /odom:=/odom_search

While SEARCHING, search commands and odometry are relayed unchanged.
While APPROACHING / STOPPED, the odometry relay is paused so
autonomous_search sees stale odom and parks itself in its own
(non-terminal) FAULT_HOLD; relaying again lets it recover to EXPLORING.

This node never publishes /cmd_vel directly: safety_controller stays
responsible for collision avoidance.
"""

import json
import math
from collections import deque
from enum import Enum, auto
from types import SimpleNamespace

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class State(Enum):
    SEARCHING = auto()
    CONFIRMING = auto()
    APPROACHING = auto()
    STOPPED = auto()
    RESUMING = auto()


def clamp(value, low, high):
    return max(low, min(high, value))


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class VictimRescue(Node):

    def __init__(self):
        super().__init__('victim_rescue')

        scalar_defaults = {
            'control_period_sec': 0.10,

            # Confirmation.
            'confirm_min_readings': 3,
            'confirm_window_sec': 1.5,
            'min_confidence': 0.60,
            'max_confirm_distance_m': 4.0,
            'known_victim_radius_m': 1.0,

            # Approach.
            'max_linear_speed': 0.10,
            'min_linear_speed': 0.03,
            'linear_gain': 0.20,
            'angular_gain': 0.60,
            'max_angular_speed': 0.25,
            'angle_deadband_deg': 3.0,
            'align_before_drive_deg': 20.0,
            'stop_distance_m': 0.80,
            'approach_timeout_sec': 20.0,

            # Freshness of /victim_distance while approaching.
            'victim_hold_timeout_sec': 0.5,
            'victim_lost_timeout_sec': 1.5,

            # The low fisheye camera loses the person at close range. If the
            # victim is lost when the last distance was already this close,
            # treat it as reached instead of aborting.
            'close_range_fallback_m': 1.2,

            # Stop / resume.
            'stop_hold_sec': 3.0,
            'reacquire_cooldown_sec': 5.0,
            'odom_stale_timeout_sec': 1.0,
        }

        for name, default in scalar_defaults.items():
            self.declare_parameter(name, default)

        self.p = SimpleNamespace(**{
            name: self.get_parameter(name).value for name in scalar_defaults
        })

        # =================================================
        # STATE
        # =================================================

        self.state = State.SEARCHING
        self.state_enter_time = self._now_sec()

        self.pose = None            # (x, y, yaw) in the odom frame
        self.pose_time = None

        # Qualifying readings while confirming: (time, reading).
        self.confirm_readings = deque()

        # Latest reading of the current target: used for steering / distance.
        self.target = None
        self.target_time = None

        # Best (highest-confidence) valid reading since confirmation: used for
        # the /victim_found position and confidence, so a later weak
        # detection cannot overwrite the confirmed victim.
        self.confirmed = None

        self.known_victims = []     # [(x, y), ...]
        self.next_victim_id = 1
        self.cooldown_until = 0.0

        # =================================================
        # SUBSCRIBERS
        # =================================================

        self.create_subscription(
            TwistStamped, '/cmd_vel_search', self.search_cmd_callback, 10
        )
        self.create_subscription(
            String, '/victim_distance', self.victim_callback, 10
        )
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # =================================================
        # PUBLISHERS
        # =================================================

        self.cmd_publisher = self.create_publisher(
            TwistStamped, '/cmd_vel_raw', 10
        )
        self.victim_found_publisher = self.create_publisher(
            String, '/victim_found', 10
        )
        self.odom_publisher = self.create_publisher(
            Odometry, '/odom_search', 10
        )

        self.timer = self.create_timer(
            self.p.control_period_sec, self.control_loop
        )

        self.get_logger().info('🚑 Victim rescue controller started')

        if not self.get_parameter('use_sim_time').value:
            self.get_logger().warning(
                '⚠️ use_sim_time is false. Run with -p use_sim_time:=true '
                'in simulation.'
            )

    # =====================================================================
    # HELPERS
    # =====================================================================

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _search_has_authority(self):
        return self.state in (State.SEARCHING, State.CONFIRMING, State.RESUMING)

    def _enter_state(self, new_state, reason):
        self.get_logger().info(
            f'[RESCUE] {self.state.name} -> {new_state.name} ({reason})'
        )
        self.state = new_state
        self.state_enter_time = self._now_sec()

    def _near_known_victim(self, x, y):
        return any(
            math.hypot(x - vx, y - vy) <= self.p.known_victim_radius_m
            for vx, vy in self.known_victims
        )

    def publish_velocity(self, linear_x, angular_z):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = linear_x
        cmd.twist.angular.z = angular_z
        self.cmd_publisher.publish(cmd)

    # =====================================================================
    # CALLBACKS
    # =====================================================================

    def search_cmd_callback(self, msg):
        # Pass through unchanged while autonomous_search is in charge.
        if self._search_has_authority():
            self.cmd_publisher.publish(msg)

    def odom_callback(self, msg):
        self.pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quaternion_to_yaw(msg.pose.pose.orientation),
        )
        self.pose_time = self._now_sec()

        # Paused while approaching / stopped so autonomous_search holds.
        if self._search_has_authority():
            self.odom_publisher.publish(msg)

    def victim_callback(self, msg):
        try:
            data = json.loads(msg.data)
            distance = float(data['distance_m'])
            angle = math.radians(float(data['angle_deg']))
            confidence = float(data['confidence'])
        except Exception as e:
            self.get_logger().error(f'❌ Bad /victim_distance message: {e}')
            return

        now = self._now_sec()

        if self.pose is None or now - self.pose_time > self.p.odom_stale_timeout_sec:
            return

        # Victim position in the odom frame (LiDAR offset ignored for MVP).
        rx, ry, ryaw = self.pose
        vx = rx + distance * math.cos(ryaw + angle)
        vy = ry + distance * math.sin(ryaw + angle)

        if self._near_known_victim(vx, vy):
            return

        reading = SimpleNamespace(
            distance=distance, angle=angle, confidence=confidence, x=vx, y=vy
        )

        if self.state == State.APPROACHING:
            # Any detection keeps steering fresh; only a strong one may
            # replace the confirmed victim used for the report.
            self.target = reading
            self.target_time = now

            if (
                confidence >= self.p.min_confidence
                and confidence > self.confirmed.confidence
            ):
                self.confirmed = reading
            return

        if self.state not in (State.SEARCHING, State.CONFIRMING):
            return

        if now < self.cooldown_until:
            return

        if confidence < self.p.min_confidence:
            return

        if distance >= self.p.max_confirm_distance_m:
            return

        self.confirm_readings.append((now, reading))
        self.target = reading
        self.target_time = now

        if self.state == State.SEARCHING:
            self._enter_state(
                State.CONFIRMING,
                f'candidate at {distance:.2f} m, {math.degrees(angle):+.1f}°'
            )

    # =====================================================================
    # STATE HANDLERS
    # =====================================================================

    def handle_confirming(self, now):
        while (
            self.confirm_readings
            and now - self.confirm_readings[0][0] > self.p.confirm_window_sec
        ):
            self.confirm_readings.popleft()

        if len(self.confirm_readings) >= self.p.confirm_min_readings:
            self.confirmed = max(
                (reading for _, reading in self.confirm_readings),
                key=lambda r: r.confidence,
            )
            self.confirm_readings.clear()
            self._enter_state(
                State.APPROACHING,
                f'victim confirmed at {self.target.distance:.2f} m, '
                f'{math.degrees(self.target.angle):+.1f}° '
                f'(confidence {self.confirmed.confidence:.2f})'
            )
            return

        if not self.confirm_readings:
            self._enter_state(State.SEARCHING, 'candidate not confirmed')

    def handle_approaching(self, now):
        p = self.p
        target_age = now - self.target_time

        if target_age > p.victim_lost_timeout_sec:
            self.publish_velocity(0.0, 0.0)

            # Close enough already: the camera simply can't see the person
            # at this range any more, so count it as reached.
            if self.target.distance <= p.close_range_fallback_m:
                self._enter_stopped(
                    f'victim lost at {self.target.distance:.2f} m '
                    f'(close-range fallback)'
                )
            else:
                self._abort(
                    f'victim lost at {self.target.distance:.2f} m'
                )
            return

        if now - self.state_enter_time > p.approach_timeout_sec:
            self.publish_velocity(0.0, 0.0)
            self._abort('approach timeout')
            return

        # Victim data briefly missing: hold still, don't drive blind.
        if target_age > p.victim_hold_timeout_sec:
            self.publish_velocity(0.0, 0.0)
            return

        distance = self.target.distance
        angle = self.target.angle

        if distance <= p.stop_distance_m:
            self.publish_velocity(0.0, 0.0)
            self._enter_stopped(f'reached {distance:.2f} m')
            return

        # Steering.
        if abs(math.degrees(angle)) < p.angle_deadband_deg:
            angular = 0.0
        else:
            angular = clamp(
                p.angular_gain * angle, -p.max_angular_speed, p.max_angular_speed
            )

        # Speed: rotate in place first if badly misaligned, then slow down
        # as the victim gets closer.
        if abs(math.degrees(angle)) > p.align_before_drive_deg:
            linear = 0.0
        else:
            linear = clamp(
                p.linear_gain * (distance - p.stop_distance_m),
                p.min_linear_speed,
                p.max_linear_speed,
            )

        self.publish_velocity(linear, angular)

    def _enter_stopped(self, reason):
        # Position / confidence come from the confirmed victim reading;
        # distance is the robot's last measured distance to the victim.
        victim = self.confirmed
        distance = self.target.distance
        victim_id = self.next_victim_id
        self.next_victim_id += 1

        self.known_victims.append((victim.x, victim.y))

        report = {
            'victim_id': victim_id,
            'x': round(victim.x, 3),
            'y': round(victim.y, 3),
            'distance_m': round(distance, 3),
            'confidence': round(victim.confidence, 3),
        }

        msg = String()
        msg.data = json.dumps(report)
        self.victim_found_publisher.publish(msg)

        self.get_logger().info(
            f'🆘 VICTIM {victim_id} FOUND at ({victim.x:.2f}, {victim.y:.2f}) '
            f'| {distance:.2f} m | confidence {victim.confidence:.2f}'
        )

        self._enter_state(State.STOPPED, f'victim {victim_id}: {reason}')

    def handle_stopped(self, now):
        self.publish_velocity(0.0, 0.0)

        if now - self.state_enter_time >= self.p.stop_hold_sec:
            self._enter_state(State.RESUMING, 'hold complete')

    def _abort(self, reason):
        self.cooldown_until = self._now_sec() + self.p.reacquire_cooldown_sec
        self._enter_state(State.RESUMING, reason)

    def handle_resuming(self, now):
        # Command and odom relays are already live again in this state
        # (_search_has_authority); autonomous_search leaves its FAULT_HOLD
        # once it sees fresh /odom_search.
        self.target = None
        self.target_time = None
        self.confirmed = None
        self.confirm_readings.clear()
        self._enter_state(State.SEARCHING, 'search resumed')

    # =====================================================================
    # MAIN CONTROL LOOP
    # =====================================================================

    def control_loop(self):
        now = self._now_sec()

        if self.state == State.CONFIRMING:
            self.handle_confirming(now)
        elif self.state == State.APPROACHING:
            self.handle_approaching(now)
        elif self.state == State.STOPPED:
            self.handle_stopped(now)
        elif self.state == State.RESUMING:
            self.handle_resuming(now)

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

    node = VictimRescue()

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
