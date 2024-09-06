'''
RILEY ANDERSON
8/20/2024
'''

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

class TranslateNode(Node):
    def __init__(self):
        super().__init__('translate_node')
        self.steering_publisher = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.throttle_publisher = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        self.odom_subscription = self.create_subscription(AckermannDriveStamped, 'drive', self.drive_callback, 10)
        self.get_logger().info('TranslateNode has been started.')
    
    def drive_callback(self, msg):
        speed = Float32()
        steering_angle = Float32()
        speed.data = float(msg.drive.speed)
        steering_angle.data = msg.drive.steering_angle
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

    

