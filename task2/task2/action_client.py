#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import sys

# Import your custom action interface
from my_custom_interfaces.action import MoveRobot

class ManhattanClient(Node):
    def __init__(self):
        super().__init__('manhattan_client')
        # Create the client. Name MUST match the server: 'move_robot'
        self._client = ActionClient(self, MoveRobot, 'move_robot')

    def send_goal(self, x, y):
        self.get_logger().info(f'Waiting for Action Server...')
        self._client.wait_for_server()

        goal_msg = MoveRobot.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)

        self.get_logger().info(f'Sending goal: ({x}, {y})')

        # Send the goal and register the feedback callback
        self._send_goal_future = self._client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by the server!')
            return

        self.get_logger().info('Goal accepted! Robot is moving...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """This function runs every time the server sends a progress update."""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Current: ({fb.current_x:.2f}, {fb.current_y:.2f}) | '
            f'Remaining: {fb.distance_to_goal:.2f}m',
            throttle_duration_sec=0.5) # Don't flood the terminal

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Mission accomplished! Goal reached.')
        else:
            self.get_logger().error('Mission failed.')
        
        # Shut down the node after we get the result
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    # Check if user gave coordinates in the terminal
    if len(sys.argv) < 3:
        print("Usage: ros2 run task2 action_client <x> <y>")
        return

    client = ManhattanClient()
    client.send_goal(sys.argv[1], sys.argv[2])

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
