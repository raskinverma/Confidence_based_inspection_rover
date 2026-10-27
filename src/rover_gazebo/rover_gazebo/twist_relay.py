#!/usr/bin/env python3
"""
Twist and Odometry Relay Node for Inspection Rover
Ensures compatibility with PLAN.md topic contract:
  - Bridges unstamped /cmd_vel (geometry_msgs/Twist) -> stamped /diff_drive_controller/cmd_vel
  - Bridges /diff_drive_controller/odom -> /odom
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry


class TwistRelay(Node):
    def __init__(self):
        super().__init__('twist_relay')

        # Subscriptions
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.sub_odom = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.pub_stamped_cmd_vel = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )
        self.pub_odom = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.get_logger().info('TwistRelay started: /cmd_vel -> /diff_drive_controller/cmd_vel & /diff_drive_controller/odom -> /odom')

    def cmd_vel_callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_footprint'
        stamped.twist = msg
        self.pub_stamped_cmd_vel.publish(stamped)

    def odom_callback(self, msg: Odometry):
        self.pub_odom.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistRelay()
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
