#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
import math

class PoseTracker(Node):
    def __init__(self):
        super().__init__('pose_tracker')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0


        self.LINEAR_SCALE  = 0.7203
        self.ANGULAR_SCALE = 0.8

        self.linear_v  = 0.0
        self.angular_v = 0.0
        self.last_time = self.get_clock().now()

        self.cmd_sub  = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_cb, 10)
        self.pose_pub = self.create_publisher(
            Pose2D, '/estimated_pose', 10)

        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info(
            f'Pose Tracker ready — '
            f'LINEAR_SCALE={self.LINEAR_SCALE}  '
            f'ANGULAR_SCALE={self.ANGULAR_SCALE}')

    def cmd_cb(self, msg):
        self.linear_v  = msg.linear.x  * self.LINEAR_SCALE
        self.angular_v = msg.angular.z * self.ANGULAR_SCALE

    def update(self):
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0 or dt > 0.5:
            return

        self.theta += self.angular_v * dt
        self.theta  = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.x += self.linear_v * math.cos(self.theta) * dt
        self.y += self.linear_v * math.sin(self.theta) * dt

        msg = Pose2D()
        msg.x     = self.x
        msg.y     = self.y
        msg.theta = self.theta
        self.pose_pub.publish(msg)

        self.get_logger().info(
            f'x={self.x:.3f}  y={self.y:.3f}  '
            f'θ={math.degrees(self.theta):.1f}°',
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    rclpy.spin(PoseTracker())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
