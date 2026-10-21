import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionMonitor(Node):

    def __init__(self):
        super().__init__("mission_monitor")

        self.subscription = self.create_subscription(
            String,
            "mission_status",
            self.listener_callback,
            10
        )

        self.get_logger().info("Mission Monitor Started!")

    def listener_callback(self, msg):

        self.get_logger().info(
            f"Received: {msg.data}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = MissionMonitor()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()