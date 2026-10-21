import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class MissionControl(Node):

    def __init__(self):
        super().__init__('mission_control')

        # -------------------------------------------------
        # ROS INTERFACES
        # -------------------------------------------------

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.status_publisher = self.create_publisher(
            String,
            '/mission_status',
            10
        )

        # -------------------------------------------------
        # SETTINGS
        # -------------------------------------------------

        self.obstacle_distance = 0.80

        # Current mission state
        self.current_state = 'SEARCHING'

        self.get_logger().info(
            '🚨 Disaster Bot Mission Control Started'
        )

        self.get_logger().info(
            f'Obstacle detection distance: '
            f'{self.obstacle_distance:.2f} m'
        )

        self.publish_state()

    # =====================================================
    # LIDAR CALLBACK
    # =====================================================

    def scan_callback(self, msg):

        front_ranges = []

        total = len(msg.ranges)

        # Approximately ±45 degrees
        angle_count = int(
            math.radians(45.0) / msg.angle_increment
        )

        # Front: 0° → +45°
        for i in range(
            0,
            min(angle_count + 1, total)
        ):

            distance = msg.ranges[i]

            if math.isfinite(distance):

                if (
                    msg.range_min
                    <= distance
                    <= msg.range_max
                ):
                    front_ranges.append(distance)

        # Front: 315° → 360°
        for i in range(
            max(0, total - angle_count),
            total
        ):

            distance = msg.ranges[i]

            if math.isfinite(distance):

                if (
                    msg.range_min
                    <= distance
                    <= msg.range_max
                ):
                    front_ranges.append(distance)

        # -------------------------------------------------
        # No valid front measurements
        # -------------------------------------------------

        if not front_ranges:
            return

        closest_distance = min(front_ranges)

        # -------------------------------------------------
        # DETERMINE STATE
        # -------------------------------------------------

        if closest_distance <= self.obstacle_distance:

            new_state = 'OBSTACLE_DETECTED'

        else:

            new_state = 'SEARCHING'

        # -------------------------------------------------
        # Publish only when state changes
        # -------------------------------------------------

        if new_state != self.current_state:

            self.current_state = new_state

            self.publish_state()

            self.get_logger().info(
                f'Front distance: '
                f'{closest_distance:.2f} m'
            )

    # =====================================================
    # PUBLISH MISSION STATE
    # =====================================================

    def publish_state(self):

        msg = String()

        msg.data = (
            f'DISASTER BOT STATE: '
            f'{self.current_state}'
        )

        self.status_publisher.publish(msg)

        self.get_logger().info(msg.data)


# =========================================================
# MAIN
# =========================================================

def main(args=None):

    rclpy.init(args=args)

    node = MissionControl()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


