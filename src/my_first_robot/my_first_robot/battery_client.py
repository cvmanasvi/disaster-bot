import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class BatteryClient(Node):

    def __init__(self):
        super().__init__("battery_client")

        self.client = self.create_client(
            AddTwoInts,
            "battery_service"
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Battery Service...")

        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        self.request.a = a
        self.request.b = b

        future = self.client.call_async(self.request)

        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main(args=None):

    rclpy.init(args=args)

    client = BatteryClient()

    response = client.send_request(20, 22)

    client.get_logger().info(f"Battery Service Response = {response.sum}")

    client.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
