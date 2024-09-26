import rclpy
import numpy as np
import time
import tensorflow as tf
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from collections import deque
from nav_msgs.msg import Odometry
import time


class TLNNode(Node):
    def __init__(self):
        super().__init__('tln_node')
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.get_logger().info('TLNNode has been started.')

        self.model_path = "/home/r478a194/Downloads/f1_tenth_model_small_noquantized.tflite"
        self.interpreter = tf.lite.Interpreter(model_path=self.model_path)
        self.interpreter.allocate_tensors()
        self.input_index = self.interpreter.get_input_details()[0]["index"]
        self.output_index = self.interpreter.get_output_details()[0]["index"]
        self.scan_buffer = np.zeros((2, 20))

        #Evaluation additions
        self.scan = []
        self.window_size = 10
        # Initialize a deque with a maximum length of 20 to store speed values
        self.speed_queue = deque(maxlen=10)
        
        # Lap counter variables
        self.lap_count = 0
        self.lap_time = time.time()

        self.start_finish_line_x = 0.0  # Define the x-coordinate of the start/finish line
        self.previous_x_position = None  # To track the previous position of the car
        self.crossed_line = False  # To track whether the car has crossed the line

        self.position = None
        self.orientation = None

    def linear_map(self, x, x_min, x_max, y_min, y_max):
        return (x - x_min) / (x_max - x_min) * (y_max - y_min) + y_min    

    def scan_callback(self, msg):
        scans = np.array(msg.ranges)
        scans = np.append(scans, [20])
        self.get_logger().info(f'num scans:{len(scans)}')
        noise = np.random.normal(0, 0.5, scans.shape)
        scans = scans + noise
        scans[scans > 10] = 10
        scans = scans[::2]  # Use every other value
        scans = np.expand_dims(scans, axis=-1).astype(np.float32)
        scans = np.expand_dims(scans, axis=0)

        self.interpreter.set_tensor(self.input_index, scans)
        start_time = time.time()
        self.interpreter.invoke()
        inf_time = (time.time() - start_time) * 1000  # in milliseconds
        self.get_logger().info(f'Inference time: {inf_time:.2f} ms')

        output = self.interpreter.get_tensor(self.output_index)
        steer = output[0, 0]
        speed = output[0, 1]

        min_speed = 0
        max_speed = 8
        speed = self.linear_map(speed, 0, 1, min_speed, max_speed)
        
        self.speed_queue.append(speed)

        if self.detect_crash():
            self.get_logger().info("Crash")
            self.get_logger().info(f"Lap time: {time.time() - self.lap_time}")
            self.destroy_node()
            rclpy.shutdown()
            quit()
        self.publish_ackermann_drive(speed, steer)

    def publish_ackermann_drive(self, speed, steering_angle):
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header = Header()
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.drive.speed = float(speed)
        ackermann_msg.drive.steering_angle = float(steering_angle)

        self.ackermann_publisher.publish(ackermann_msg)
        self.get_logger().info(f'Published AckermannDriveStamped message: speed={speed}, steering_angle={steering_angle}')
    
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
    
        if self.lap_count == 1:
            self.get_logger().info(f"Lap time: {time.time() - self.lap_time}")
            self.destroy_node()
            quit()
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
    node = TLNNode()
    lap_time = time.time()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')

        node.get_logger().info(f"Lap time: {time.time() - lap_time}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
