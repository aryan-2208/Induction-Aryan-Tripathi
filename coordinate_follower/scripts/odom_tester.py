#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomTestNode(Node):
    def __init__(self):
        super().__init__('odom_test_node')
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f'📍 Position → x: {x:.2f}, y: {y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = OdomTestNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
