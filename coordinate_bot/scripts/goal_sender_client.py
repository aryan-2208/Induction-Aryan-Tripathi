#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from coordinate_bot.action import GoToCoordinate
import time

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender_client')
        self._client = ActionClient(self, GoToCoordinate, 'go_to_coordinate')

    def send_goals_from_file(self, file_path):
        self._client.wait_for_server()
        self.get_logger().info(" Action server ready")

        with open(file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    x_str, y_str = line.split()
                    x, y = float(x_str.strip()), float(y_str.strip())
                except ValueError:
                    self.get_logger().error(f" Invalid line format: {line}")
                    continue

                goal_msg = GoToCoordinate.Goal()
                goal_msg.x = x
                goal_msg.y = y
                self.get_logger().info(f" Sending goal: ({x}, {y})")

                future = self._client.send_goal_async(goal_msg)
                rclpy.spin_until_future_complete(self, future)
                goal_handle = future.result()

                if not goal_handle.accepted:
                    self.get_logger().warn(f"Goal ({x}, {y}) was rejected")
                    continue

                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                result = result_future.result().result

                if result.success:
                    self.get_logger().info(f" Goal result: success = True")
                else:
                    self.get_logger().warn(f" Goal result: success = False")

                time.sleep(1)

def main():
    rclpy.init()
    client = GoalSender()
    client.send_goals_from_file('/home/aryan/ros2_ws/src/coordinate_bot/scripts/coordinates.txt')
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
