import rclpy
from math import pi
from rclpy.node import Node
from statistics import mean
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np

class FtgNode(Node):
    def __init__(self):
        super().__init__('ftg_node')
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/autodrive/f1tenth_1/lidar', self.scan_callback, 10)
        #self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.get_logger().info('Jackson FTGNode has been started.')

        # from RileyFTG
        self.car_width = .296
        self.tolerance = 0.5

        #adapted parameters from BDEvan5 f1tenth benchmarks FTG implementation
        self.bubble_radius: 160
        self.preprocess_conv_size: 3
        self.best_point_conv_size: 80
        self.max_lidar_dist: 10.0
        self.fast_speed: 5
        self.straights_speed: 5.0
        self.corners_speed: 3.0
        self.straights_steering_angle: 0.174
        self.fast_steering_angle: 0.0785
        self.safe_threshold: 5
        self.max_steer: 0.4


    def scan_callback(self, msg):
        proc_ranges = self.preprocess_lidar(msg.ranges)
        
        closest = np.argmin(proc_ranges)

        #eliminate all points inside of bubble
        min_index = closest - self.bubble_radius
        max_index = closest + self.bubble_radius
        if min_index < 0: min_index = 0
        if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        # Find max length gap
        gap_start, gap_end = self.find_max_gap(proc_ranges)

        # Find the best point in the gap
        best = self.find_best_point(gap_start, gap_end, proc_ranges)

        # Publish Drive message
        steering_angle = self.get_angle(best, len(proc_ranges))
        if abs(steering_angle) > self.straights_steering_angle:
            speed = self.corners_speed
        elif abs(steering_angle) > self.fast_steering_angle:
            speed = self.straights_speed
        else:
            speed = self.fast_speed


        self.publish_ackermann_drive(speed, steering_angle)



        # '''STEERING CALCULATION'''
        # #process where the farthest point is, and then the farthest safe point
        # #dead center is 540, each degree has 4 scans -- assume all measurements are in m
        # target = self.safe_point(msg)
        # unchecked_angle = self.point_to_steering(target)

        # #check the angle to make sure the car is physically able to turn that far
        # if -30 <= unchecked_angle <= 30:
        #     steering_angle = unchecked_angle
        # elif -30 > unchecked_angle:
        #     steering_angle = -30
        # else:
        #     steering_angle = 30
        
        # # #safety check! if car is within .5 meters of the wall, steer away slightly to provent side collision
        # if min(msg.ranges[120:220]) > car_width+tolerance and min(msg.ranges[760:840]) > car_width+tolerance:
        #     pass
        # elif min(msg.ranges[120:220]) <= car_width+tolerance and steering_angle < 0:
        #     steering_angle = 0.75
        # elif min(msg.ranges[760:840]) <= car_width+tolerance and steering_angle > 0:
        #     steering_angle = -0.75
        # '''SPEED CALCULATION'''

        # #really need to improve this part. Hopefully a system that integrates turning angle + distance to lookahead
        # # if steering_angle!=0 and mean(msg.ranges[520:560])*(0.9-steering_angle/100) < 8:
        # #     speed = mean(msg.ranges[520:560])*(0.9-steering_angle/100)
        # # else:
        # #     speed = 8
        # if msg.ranges[540]/2 >= 8:
        #     speed = 8
        # else:
        #     speed = msg.ranges[540]/2
        # self.get_logger().info(f'steering angle:{speed}')
        # self.publish_ackermann_drive(speed, self.deg2rad(steering_angle))
    
    def preprocess_lidar(self, ranges):
            """ Preprocess the LiDAR scan array. Expert implementation includes:
                1.Setting each value to the mean over some window
                2.Rejecting high values (eg. > 3m)
            """
            self.radians_per_elem = (2 * np.pi) / len(ranges)
            # we won't use the LiDAR data from directly behind us
            proc_ranges = np.array(ranges[135:-135])
            # sets each value to the mean over a given window
            proc_ranges = np.convolve(proc_ranges, np.ones(self.preprocess_conv_size), 'same') / self.preprocess_conv_size
            proc_ranges = np.clip(proc_ranges, 0, self.max_lidar_dist)
            return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
            free_space_ranges: list of LiDAR data which contains a 'bubble' of zeros
        """
        # mask the bubble
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        # get a slice for each contigous sequence of non-bubble data
        slices = np.ma.notmasked_contiguous(masked)
        # print(slices)
        # max_len = slices[-1].stop - slices[-1].start
        # chosen_slice = slices[-1]
        # I think we will only ever have a maximum of 2 slices but will handle an
        # indefinitely sized list for portablility
        for sl in slices[::-1]:
            # print(sl)
            sl_len = sl.stop - sl.start
            if sl_len > self.planner_params.safe_threshold:
                chosen_slice = sl
                # print("Slice choosen")
                return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        # do a sliding window average over the data in the max gap, this will
        # help the car to avoid hitting corners
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.planner_params.best_point_conv_size),
                                       'same') / self.planner_params.best_point_conv_size
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        """ Get the angle of a particular element in the lidar data and transform it into an appropriate steering angle
        """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2
        steering_angle = np.clip(steering_angle, -self.planner_params.max_steer, self.planner_params.max_steer)
        return steering_angle
    
        
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
