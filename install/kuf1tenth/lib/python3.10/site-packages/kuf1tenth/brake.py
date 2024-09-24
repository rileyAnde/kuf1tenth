'''Node to operate emergency brake
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

class BrakeNode(Node):
    def __init__(self):
        super().__init__('brake_node')
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.get_logger().info('BrakeNode has been started.')

    def scan_callback(self, msg):
        # Publish a velo 0 message if wall is within .5 meters of the front of the car
        lookahead = min(msg.ranges[480:600])
        if lookahead <= 0.75:
            self.publish_ackermann_drive(0.0, 0.0)
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
    node = BrakeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
