import math
from types import SimpleNamespace

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


def angle_in_sector(angle_deg, min_deg, max_deg):
    if min_deg <= max_deg:
        return min_deg <= angle_deg <= max_deg
    return angle_deg >= min_deg or angle_deg <= max_deg


class SafetyController(Node):

    def __init__(self):
        super().__init__('safety_controller')

        scalar_defaults = {
            # Forward safety (unchanged behavior from the original design).
            'safe_distance': 0.55,
            'emergency_distance': 0.30,
            'max_forward_speed': 0.60,
            'slow_speed': 0.15,
            'front_sector_half_width_deg': 30.0,

            # Reverse safety (new).
            'rear_safe_distance': 0.45,
            'rear_emergency_distance': 0.30,
            'max_reverse_speed': 0.15,
            'reverse_slow_speed': 0.08,
            'rear_sector_half_width_deg': 30.0,

            # Command / sensor freshness.
            'cmd_stale_timeout_sec': 0.5,
            'scan_stale_timeout_sec': 1.5,
        }

        for name, default in scalar_defaults.items():
            self.declare_parameter(name, default)

        self.p = SimpleNamespace(**{
            name: self.get_parameter(name).value for name in scalar_defaults
        })

        # =================================================
        # STATE
        # =================================================

        self.latest_cmd = TwistStamped()
        self.latest_cmd_stamp = None
        self.latest_scan = None
        self.latest_scan_stamp = None

        # =================================================
        # SUBSCRIBERS
        # =================================================

        self.cmd_subscriber = self.create_subscription(
            TwistStamped,
            '/cmd_vel_raw',
            self.cmd_callback,
            10
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # =================================================
        # PUBLISHER
        # =================================================

        self.cmd_publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # =================================================
        # CONTROL LOOP
        # =================================================

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            '🛡️ Disaster Bot Safety Controller Started'
        )

        self.get_logger().info(
            f'🛡️ Front safe/emergency: {self.p.safe_distance:.2f} / '
            f'{self.p.emergency_distance:.2f} m'
        )

        self.get_logger().info(
            f'🛡️ Rear safe/emergency: {self.p.rear_safe_distance:.2f} / '
            f'{self.p.rear_emergency_distance:.2f} m'
        )

    # =====================================================
    # COMMAND CALLBACK
    # =====================================================

    def cmd_callback(self, msg):
        self.latest_cmd = msg
        self.latest_cmd_stamp = msg.header.stamp

    # =====================================================
    # LIDAR CALLBACK
    # =====================================================

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.latest_scan_stamp = msg.header.stamp

    # =====================================================
    # FRESHNESS
    # =====================================================

    def _age_seconds(self, stamp, now_ns):
        msg_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        return max(0.0, (now_ns - msg_ns) / 1e9)

    def _is_cmd_fresh(self, now_ns):
        if self.latest_cmd_stamp is None:
            return False
        return self._age_seconds(self.latest_cmd_stamp, now_ns) <= self.p.cmd_stale_timeout_sec

    def _is_scan_fresh(self, now_ns):
        if self.latest_scan is None or self.latest_scan_stamp is None:
            return False
        return self._age_seconds(self.latest_scan_stamp, now_ns) <= self.p.scan_stale_timeout_sec

    # =====================================================
    # SECTOR DISTANCE
    # =====================================================

    def _sector_min_distance(self, scan, half_width_deg, center_deg):

        min_deg = ((center_deg - half_width_deg + 180.0) % 360.0) - 180.0
        max_deg = ((center_deg + half_width_deg + 180.0) % 360.0) - 180.0

        values = []

        for i, distance in enumerate(scan.ranges):

            if not math.isfinite(distance):
                continue

            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            angle_deg = math.degrees(angle)

            if angle_in_sector(angle_deg, min_deg, max_deg):
                values.append(distance)

        if not values:
            return float('inf')

        return min(values)

    # =====================================================
    # CONTROL LOOP
    # =====================================================

    def control_loop(self):

        now_ns = self.get_clock().now().nanoseconds

        # No usable LiDAR: cannot verify safety in any direction.
        if not self._is_scan_fresh(now_ns):
            self.publish_velocity(0.0, 0.0)
            return

        # Controller has stopped talking to us: fail safe.
        if not self._is_cmd_fresh(now_ns):
            self.publish_velocity(0.0, 0.0)
            return

        scan = self.latest_scan

        requested_linear = self.latest_cmd.twist.linear.x
        requested_angular = self.latest_cmd.twist.angular.z

        if requested_linear < 0.0:
            self._control_reverse(scan, requested_linear, requested_angular)
        else:
            self._control_forward(scan, requested_linear, requested_angular)

    # =====================================================
    # FORWARD SAFETY
    # =====================================================

    def _control_forward(self, scan, requested_linear, requested_angular):

        front_distance = self._sector_min_distance(
            scan, self.p.front_sector_half_width_deg, 0.0
        )

        # =================================================
        # EMERGENCY STOP
        # =================================================

        if front_distance <= self.p.emergency_distance:

            self.publish_velocity(
                0.0,
                requested_angular
            )

            return

        # =================================================
        # CLOSE OBSTACLE
        # =================================================

        if front_distance <= self.p.safe_distance:

            if requested_linear > 0.0:

                self.publish_velocity(
                    min(
                        requested_linear,
                        self.p.slow_speed
                    ),
                    requested_angular
                )

            else:

                self.publish_velocity(
                    requested_linear,
                    requested_angular
                )

            return

        # =================================================
        # PATH CLEAR
        # =================================================

        linear = min(requested_linear, self.p.max_forward_speed)

        self.publish_velocity(
            linear,
            requested_angular
        )

    # =====================================================
    # REVERSE SAFETY
    # =====================================================

    def _control_reverse(self, scan, requested_linear, requested_angular):

        rear_distance = self._sector_min_distance(
            scan, self.p.rear_sector_half_width_deg, 180.0
        )

        # =================================================
        # EMERGENCY STOP (reverse)
        # =================================================

        if rear_distance <= self.p.rear_emergency_distance:

            self.publish_velocity(
                0.0,
                requested_angular
            )

            return

        # =================================================
        # CLOSE OBSTACLE (reverse)
        # =================================================

        if rear_distance <= self.p.rear_safe_distance:

            capped = max(requested_linear, -self.p.reverse_slow_speed)

            self.publish_velocity(
                capped,
                requested_angular
            )

            return

        # =================================================
        # PATH CLEAR (reverse)
        # =================================================

        capped = max(requested_linear, -self.p.max_reverse_speed)

        self.publish_velocity(
            capped,
            requested_angular
        )

    # =====================================================
    # PUBLISH
    # =====================================================

    def publish_velocity(
        self,
        linear_x,
        angular_z
    ):

        cmd = TwistStamped()

        cmd.header.stamp = (
            self.get_clock().now().to_msg()
        )

        cmd.twist.linear.x = linear_x
        cmd.twist.angular.z = angular_z

        self.cmd_publisher.publish(cmd)


# =========================================================
# MAIN
# =========================================================

def main(args=None):

    rclpy.init(args=args)

    node = SafetyController()

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
