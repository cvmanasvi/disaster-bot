import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


class BatteryServer(Node):

    def __init__(self):
        super().__init__("battery_server")

        self.srv = self.create_service(
            AddTwoInts,
            "battery_service",
            self.callback
        )

        self.get_logger().info("Battery Service Started")

    def callback(self, request, response):

        self.get_logger().info(
            f"Received request: {request.a} + {request.b}"
        )

        response.sum = request.a + request.b

        return response


def main(args=None):

    rclpy.init(args=args)

    node = BatteryServer()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
