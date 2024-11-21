'''Node to implement wall following
    Riley Anderson
    Created: 08/01/2024
    Last Edited: 08/01/2024
'''

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wall_follow_node')
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.get_logger().info('WallFollowNode has been started.')

    def scan_callback(self, msg):
        # Publish a standard velo, steering messages to push car to center of track
        #define distance to left and right of car
        front = min(msg.ranges[520:600])
        left = min(msg.ranges[420:480])
        right = min(msg.ranges[600:660])
        #steer left if within 2 meters (units?) of wall to right, right if within 1 meter of wall to left, center if centered
        if right <= 1:
            self.publish_ackermann_drive(2.0, -15.0)
        if left <= 1:
            self.publish_ackermann_drive(2.0, 15.0)
        else:
            self.publish_ackermann_drive(front, 0.0)
    def odom_callback(self, msg):
        self.get_logger().info('Received Odometry data')
        # Process Odometry data here

    def publish_ackermann_drive(self, speed, steering_angle):
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header = Header()
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.drive.speed = speed
        ackermann_msg.drive.steering_angle = steering_angle

        self.ackermann_publisher.publish(ackermann_msg)
        self.get_logger().info(f'Published AckermannDriveStamped message: speed={speed}, steering_angle={steering_angle}')
def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
