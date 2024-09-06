import rclpy
from math import pi
from rclpy.node import Node
from statistics import mean
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class FtgNode(Node):
    def __init__(self):
        super().__init__('ftg_node')
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/autodrive/f1tenth_1/lidar', self.scan_callback, 10)
        #self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.get_logger().info('FTGNode has been started.')

    def scan_callback(self, msg):
        car_width = .296
        tolerance = 0.5
        '''STEERING CALCULATION'''
        #process where the farthest point is, and then the farthest safe point
        #dead center is 540, each degree has 4 scans -- assume all measurements are in m
        target = self.safe_point(msg)
        unchecked_angle = self.point_to_steering(target)

        #check the angle to make sure the car is physically able to turn that far
        if -30 <= unchecked_angle <= 30:
            steering_angle = unchecked_angle
        elif -30 > unchecked_angle:
            steering_angle = -30
        else:
            steering_angle = 30
        
        # #safety check! if car is within .5 meters of the wall, steer away slightly to provent side collision
        if min(msg.ranges[120:220]) > car_width+tolerance and min(msg.ranges[760:840]) > car_width+tolerance:
            pass
        elif min(msg.ranges[120:220]) <= car_width+tolerance and steering_angle < 0:
            steering_angle = 0.75
        elif min(msg.ranges[760:840]) <= car_width+tolerance and steering_angle > 0:
            steering_angle = -0.75
        '''SPEED CALCULATION'''

        #really need to improve this part. Hopefully a system that integrates turning angle + distance to lookahead
        # if steering_angle!=0 and mean(msg.ranges[520:560])*(0.9-steering_angle/100) < 8:
        #     speed = mean(msg.ranges[520:560])*(0.9-steering_angle/100)
        # else:
        #     speed = 8
        if msg.ranges[540]/2 >= 8:
            speed = 8
        else:
            speed = msg.ranges[540]/2
        self.get_logger().info(f'steering angle:{speed}')
        self.publish_ackermann_drive(speed, self.deg2rad(steering_angle))
    
    def safe_point(self, msg):
        car_width = .296
        tolerance = 0.5
        laser_arr = msg.ranges
        goal_dist = max(msg.ranges)
        goal_point = laser_arr.index(goal_dist)
        Lsafe_point = self.safe_point_to_left(laser_arr, goal_point, car_width/2 + tolerance)
        Rsafe_point = self.safe_point_to_right(laser_arr, goal_point, car_width/2 + tolerance)
        try:
            if (Lsafe_point != 0) and (Rsafe_point != 0) and laser_arr[goal_point + Lsafe_point] >= laser_arr[goal_point+Rsafe_point]:
                target = Lsafe_point
                self.get_logger().info(f'num scans: {target}')
                return goal_point+Lsafe_point
            elif Rsafe_point != 0:
                target = Rsafe_point
                self.get_logger().info(f'num scans: {target}')
                return goal_point+Rsafe_point
            else:
                return 540
        except:
            return 540
        
    def safe_point_to_right(self, ranges, index_of_furthest, width):
        act_distance = 0
        steps_away = 1
        try:
            while act_distance < width:
                act_distance += self.dbs(ranges[index_of_furthest+steps_away])
                steps_away += 1
        except:
            return 0
        return steps_away
    
    def safe_point_to_left(self, ranges, index_of_furthest, width):
        act_distance = 0
        steps_away = -1
        try:
            while act_distance < width:
                act_distance += self.dbs(ranges[index_of_furthest+steps_away])
                steps_away -= 1
        except:
            return 0
        return steps_away
        
    
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
