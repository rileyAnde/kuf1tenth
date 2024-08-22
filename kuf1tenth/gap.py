import rclpy
from math import pi
from statistics import mean
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class GapNode(Node):
    def __init__(self):
        super().__init__('gap_node')
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        #self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.get_logger().info('GapNode has been started.')

    def scan_callback(self, msg):
        '''STEERING CALCULATION'''
        #process where are the gaps, and then determine which is the "best" gap
        #dead center is 540, each degree has 4 scans -- assume all measurements are in m
        car_width = .296
        tolerance = .08
        #at any given distance, there is a varying distance between degrees, pi(d)/540
        #assume turning is [-30:30] (120 scans either way)
        gaps = self.reachable_gaps(msg, car_width, tolerance)
        farthest_gap = [0,0,0]
        for i in gaps:
            if i[2] > farthest_gap[2]:
                farthest_gap = i
        steering_angle = self.gap_to_steering(farthest_gap)
        '''SPEED CALCULATION'''
        # TODO aim for .5 sec ittc at all times, if impossible just set to max (For testing just set speed to 3)
        self.publish_ackermann_drive(3, steering_angle)
        
    
    def reachable_gaps(self, msg, car_width, tolerance):
        working_range = msg[420:680]
        gap_size = car_width + (tolerance*2)
        gaps = []
        for i in range(len(working_range)-1):
            if i > min(working_range)+.03:
                dbs = self.dbs(working_range[i])
                scans_for_gap = (gap_size // dbs) + 1
                #determine if there exists a gap 
                if min(working_range[i:i+scans_for_gap]) >= working_range[i]-.03:
                    x=0
                    if working_range[i+scans_for_gap+x] >= working_range[i]-.03:
                        x+=1
                    #gap format is [start indice, end indice, avg dist]
                    gap = [420+i, 420+i+scans_for_gap+x, mean(working_range[i:i+scans_for_gap+x])]
                    gaps.append[gap]
        return gaps
    
    def gap_to_steering(self, gap):
        min_indice = gap[0]
        max_indice = gap[1]
        target = (min_indice + max_indice) // 2
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
        ackermann_msg.drive.speed = speed
        ackermann_msg.drive.steering_angle = steering_angle

        self.ackermann_publisher.publish(ackermann_msg)
        self.get_logger().info(f'Published AckermannDriveStamped message: speed={speed}, steering_angle={steering_angle}')

def main(args=None):
    rclpy.init(args=args)
    node = GapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()