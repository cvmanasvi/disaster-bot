import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.threshold = 0.8

        self.get_logger().info('Obstacle Detector Started')

    def scan_callback(self, msg):

        valid_ranges = [
            r for r in msg.ranges
            if r != float('inf')
            and r > msg.range_min
            and r < msg.range_max
        ]

        if not valid_ranges:
            return

        closest_distance = min(valid_ranges)

        if closest_distance < self.threshold:

            stop = Twist()
            stop.linear.x = 0.0
            stop.angular.z = 0.0

            self.publisher.publish(stop)

            self.get_logger().warning(
                f'OBSTACLE! {closest_distance:.2f} m - ROBOT STOPPED'
            )

        else:
            self.get_logger().info(
                f'Path clear: {closest_distance:.2f} m'
            )


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

