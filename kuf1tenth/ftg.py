import rclpy
from math import pi
from rclpy.node import Node
from statistics import mean
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from collections import deque

class FtgNode(Node):
    def __init__(self):
        super().__init__('ftg_node')
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.get_logger().info('FTGNode has been started.')
        self.scan = []
        self.window_size = 10
        # Initialize a deque with a maximum length of 20 to store speed values
        self.speed_queue = deque(maxlen=10)
        
        # Lap counter variables
        self.lap_count = 0
        self.start_finish_line_x = 0.0  # Define the x-coordinate of the start/finish line
        self.previous_x_position = None  # To track the previous position of the car
        self.crossed_line = False  # To track whether the car has crossed the line

        self.position = None
        self.orientation = None

    def scan_callback(self, msg):
        car_width = .296
        tolerance = 0.5
        '''STEERING CALCULATION'''
        self.scan = msg.ranges

        target = self.safe_point(msg)
        unchecked_angle = self.point_to_steering(target)

        # Check the angle to make sure the car is physically able to turn that far
        if -30 <= unchecked_angle <= 30:
            steering_angle = unchecked_angle
        elif unchecked_angle < -30:
            steering_angle = -30
        else:
            steering_angle = 30
        
        # Safety check for walls
        if min(msg.ranges[120:220]) > car_width + tolerance and min(msg.ranges[760:840]) > car_width + tolerance:
            pass
        elif min(msg.ranges[120:220]) <= car_width + tolerance and steering_angle < 0:
            steering_angle = 0.75
        elif min(msg.ranges[760:840]) <= car_width + tolerance and steering_angle > 0:
            steering_angle = -0.75

        # Speed calculation
        if msg.ranges[540] / 2 >= 8:
            speed = 8
        else:
            speed = msg.ranges[540] / 2
        
        self.speed_queue.append(speed)

        if self.detect_crash():
            self.get_logger().info("Crash")
            self.destroy_node()
            rclpy.shutdown()
        
        self.publish_ackermann_drive(speed, self.deg2rad(steering_angle))

    def odom_callback(self, msg):
        # Extract position and orientation from the Odometry message
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation

        # Check if the car has crossed the start/finish line
        self.check_lap(self.position)

        #self.get_logger().info(f'Odometry received: position={self.position}, orientation={self.orientation}')

    def check_lap(self, current_position):
        # Check if we have a previous position to compare
        if self.previous_x_position is None:
            self.previous_x_position = current_position.x
            return
        # Check if the car crossed the start/finish line going forward
        if self.previous_x_position < self.start_finish_line_x <= current_position.x:
            if not self.crossed_line:
                self.lap_count += 1
                self.crossed_line = True
        
        self.get_logger().info(f'Lap {self.lap_count} completed!')
        
        # Reset the line crossing flag if the car moves away from the line
        if current_position.x < self.start_finish_line_x:
            self.crossed_line = False

        # Update previous x position for the next callback
        self.previous_x_position = current_position.x

    def safe_point(self, msg):
        car_width = .296
        tolerance = 0.5
        laser_arr = msg.ranges
        goal_dist = max(msg.ranges)
        goal_point = laser_arr.index(goal_dist)
        Lsafe_point = self.safe_point_to_left(laser_arr, goal_point, car_width / 2 + tolerance)
        Rsafe_point = self.safe_point_to_right(laser_arr, goal_point, car_width / 2 + tolerance)

        try:
            if Lsafe_point != 0 and Rsafe_point != 0 and laser_arr[goal_point + Lsafe_point] >= laser_arr[goal_point + Rsafe_point]:
                return goal_point + Lsafe_point
            elif Rsafe_point != 0:
                return goal_point + Rsafe_point
            else:
                return 540
        except:
            return 540

    def safe_point_to_right(self, ranges, index_of_furthest, width):
        act_distance = 0
        steps_away = 1
        try:
            while act_distance < width:
                act_distance += self.dbs(ranges[index_of_furthest + steps_away])
                steps_away += 1
        except:
            return 0
        return steps_away

    def safe_point_to_left(self, ranges, index_of_furthest, width):
        act_distance = 0
        steps_away = -1
        try:
            while act_distance < width:
                act_distance += self.dbs(ranges[index_of_furthest + steps_away])
                steps_away -= 1
        except:
            return 0
        return steps_away

    def deg2rad(self, deg):
        return (deg * pi) / 180

    def point_to_steering(self, target):
        if target == 540:
            return 0
        else:
            return (target - 540) / 4

    def dbs(self, dist):
        return (pi * dist) / 540

    def publish_ackermann_drive(self, speed, steering_angle):
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header = Header()
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.drive.speed = float(speed)
        ackermann_msg.drive.steering_angle = float(steering_angle)

        self.ackermann_publisher.publish(ackermann_msg)
        #self.get_logger().info(f'Published AckermannDriveStamped message: speed={speed}, steering_angle={steering_angle}')

    def detect_crash(self):
        for i in range(len(self.scan) - self.window_size + 1):
            window = self.scan[i:i + self.window_size]
            if all(val <= 0.2 for val in window):
                return True

        if len(self.speed_queue) == self.speed_queue.maxlen and all(speed < 0.05 for speed in self.speed_queue):
            return True
        return False

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
