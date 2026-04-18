import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarMonitor(Node):
    def __init__(self):
        super().__init__('lidar_monitor')
        # We subscribe to the '/scan' topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10) # QoS (Queue size)
        self.get_logger().info('LiDAR Monitor Node has started!')

    def listener_callback(self, msg):
        # 'ranges' is an array of distances. 
        # msg.ranges[0] is usually directly in front of the robot.
        center_index = len(msg.ranges) // 2
        distance = msg.ranges[center_index]

        if distance < 0.5:
            self.get_logger().warn(f'WALL TOO CLOSE! Distance: {distance:.2f}m')
        else:
            self.get_logger().info(f'Path clear. Distance: {distance:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = LidarMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
