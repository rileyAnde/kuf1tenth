import rclpy
import numpy as np
import time
import tensorflow as tf
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import sys
sys.path.append("/home/r478a194/f1tenth_omniverse/ros2_ws/src/f1_omni/f1_omni")
from birdseye import Birdseye


class TLNNode(Node):
    def __init__(self):
        super().__init__('tln_node')
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info('TLNNode has been started.')

        self.model_path = "/home/r478a194/f1tenth_omniverse/ros2_ws/models/birdseye_noquantized.tflite"
        self.interpreter = tf.lite.Interpreter(model_path=self.model_path)
        self.interpreter.allocate_tensors()
        self.input_index = self.interpreter.get_input_details()[0]["index"]
        self.output_index = self.interpreter.get_output_details()[0]["index"]
        self.img_proc = Birdseye(100)

    def linear_map(self, x, x_min, x_max, y_min, y_max):
        return (x - x_min) / (x_max - x_min) * (y_max - y_min) + y_min    

    def scan_callback(self, msg):
        scans = np.array(msg.ranges)
        scans[scans > 10] = 10
        #scans = scans[::2]

        # Create grayscale Birdseye image
        img = self.img_proc.make(scans)               
        img = img.astype(np.float32)        
        img = np.expand_dims(img, axis=0)

        # Run inference
        self.interpreter.set_tensor(self.input_index, img)
        start_time = time.time()
        self.interpreter.invoke()
        inf_time = (time.time() - start_time) * 1000  # ms
        self.get_logger().info(f'Inference time: {inf_time:.2f} ms')

        output = self.interpreter.get_tensor(self.output_index)
        steer = output[0, 0]
        speed = output[0, 1]

        # Denormalize speed
        min_speed = 1
        max_speed = 8
        speed = self.linear_map(speed, 0, 1, min_speed, max_speed)

        self.publish_ackermann_drive(speed, steer)

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
    node = TLNNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
