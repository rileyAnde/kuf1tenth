'''
RILEY ANDERSON
8/20/2024
'''

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class TranslateNode(Node):
    def __init__(self):
        super().__init__('translate_node')
        self.steering_publisher = self.create_publisher(float, '/autodrive/f1tenth_1/steering_control', 10)
        self.throttle_publisher = self.create_publisher(float, '/autodrive/f1tenth_1/throttle_control', 10)
        self.odom_subscription = self.create_subscription(AckermannDriveStamped, 'drive', self.drive_callback, 10)
        self.get_logger().info('TranslateNode has been started.')
    
    def drive_callback(self, msg):
        speed = msg.drive.speed
        steering_angle = msg.drive.steering_angle
        self.throttle_publisher.publish(speed)
        self.steering_publisher.publish(steering_angle)

def main(args=None):
    rclpy.init(args=args)
    node = TranslateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    

