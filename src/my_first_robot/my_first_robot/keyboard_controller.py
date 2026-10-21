import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class KeyboardController(Node):

    def __init__(self):
        super().__init__('keyboard_controller')

        self.publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel_raw',
            10
        )

        self.speed = 0.20
        self.turn_speed = 0.8

        self.get_logger().info(
            '⌨️ Disaster Bot Keyboard Controller Started'
        )

        self.get_logger().info(
            'w = forward | s = backward'
        )
        self.get_logger().info(
            'a = left | d = right | x = stop | q = quit'
        )

    def publish_velocity(self, linear, angular):

        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = linear
        msg.twist.angular.z = angular

        self.publisher.publish(msg)


def get_key():

    settings = termios.tcgetattr(sys.stdin)

    tty.setraw(sys.stdin.fileno())

    key = sys.stdin.read(1)

    termios.tcsetattr(
        sys.stdin,
        termios.TCSADRAIN,
        settings
    )

    return key


def main(args=None):

    rclpy.init(args=args)

    node = KeyboardController()

    try:

        while rclpy.ok():

            key = get_key()

            if key == 'w':

                node.publish_velocity(
                    node.speed,
                    0.0
                )

            elif key == 's':

                node.publish_velocity(
                    -node.speed,
                    0.0
                )

            elif key == 'a':

                node.publish_velocity(
                    0.0,
                    node.turn_speed
                )

            elif key == 'd':

                node.publish_velocity(
                    0.0,
                    -node.turn_speed
                )

            elif key == 'x':

                node.publish_velocity(
                    0.0,
                    0.0
                )

            elif key == 'q':

                node.publish_velocity(
                    0.0,
                    0.0
                )

                break

            rclpy.spin_once(
                node,
                timeout_sec=0.01
            )

    except KeyboardInterrupt:
        pass

    finally:

        node.publish_velocity(
            0.0,
            0.0
        )

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

