import rclpy
from math import pi
from statistics import mean
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class FtgNode(Node):
    def __init__(self):
        super().__init__('ftg_node')
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        #self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.get_logger().info('FTGNode has been started.')

    def scan_callback(self, msg):
        '''STEERING CALCULATION'''
        #process where the farthest point is, and then the farthest safe point
        #dead center is 540, each degree has 4 scans -- assume all measurements are in m
        target = self.safe_point(msg)
        unchecked_angle = self.point_to_steering(target)
        if -30 <= unchecked_angle <= 30:
            steering_angle = unchecked_angle
        elif -30 > unchecked_angle:
            steering_angle = -30
        else:
            steering_angle = 30
        
        if min(msg.ranges[120:220]) > 0.5 and min(msg.ranges[760:840]) > 0.5:
            pass
        elif min(msg.ranges[120:220]) <= 0.5:
            steering_angle = 3.0
        elif min(msg.ranges[760:840]) <= 0.5:
            steering_angle = -3.0
        '''SPEED CALCULATION'''
        
        lookahead = msg.ranges[540]
        if lookahead/2 < 8:
            speed = lookahead/2
        else:
            speed = 8

        self.publish_ackermann_drive(speed, self.deg2rad(steering_angle))
    
    def safe_point(self, msg):
        car_width = .296
        tolerance = 1.0
        laser_arr = msg.ranges
        goal_dist = max(msg.ranges)
        goal_point = laser_arr.index(goal_dist)
        safe_steps_away = int((car_width+tolerance) // self.dbs(goal_dist) + 1)
        self.get_logger().info(f'num scans: {safe_steps_away}')
        try:
            if laser_arr[goal_point-safe_steps_away] > laser_arr[goal_point+safe_steps_away]:
                return goal_point - safe_steps_away
            else:
                return goal_point + safe_steps_away
        except:
            return 540
        
    
    def deg2rad(self, deg):
        return (deg*pi)/180    
    
    def point_to_steering(self, target):
        #self.get_logger().info(f'Target: {target}')
        if target == 540:
            return 0
        else:
            return (target-540)/4


    def dbs(self, dist):
        #calc distance between scans
        return (pi*dist)/540


    def publish_ackermann_drive(self, speed, steering_angle):
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header = Header()
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.drive.speed = float(speed)
        ackermann_msg.drive.steering_angle = float(steering_angle)

        self.ackermann_publisher.publish(ackermann_msg)
        self.get_logger().info(f'Published AckermannDriveStamped message: speed={speed}, steering_angle={steering_angle}')

def main(args=None):
    rclpy.init(args=args)
    node = FtgNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()