import gym
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
import csv
import random
from collections import deque
import torch
import torch.nn as nn
import torch.optim as optim
from math import sin, cos

# DQN model definition
class DQN(nn.Module):
    def __init__(self, state_size, action_size):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(state_size, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc3 = nn.Linear(128, action_size)
    
    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)

# DQN Agent
class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=2000)
        self.gamma = 0.95  # discount rate
        self.epsilon = 1.0  # exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.001
        self.model = DQN(state_size, action_size)
        self.optimizer = optim.Adam(self.model.parameters(), lr=self.learning_rate)
        self.criterion = nn.MSELoss()

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        state = torch.FloatTensor(state)
        act_values = self.model(state)
        return torch.argmax(act_values[0]).item()

    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        for state, action, reward, next_state, done in minibatch:
            state = torch.FloatTensor(state)
            next_state = torch.FloatTensor(next_state)
            target = reward
            if not done:
                target = reward + self.gamma * torch.max(self.model(next_state)[0])
            target_f = self.model(state)
            target_f[0][action] = target
            self.optimizer.zero_grad()
            loss = self.criterion(target_f, self.model(state))
            loss.backward()
            self.optimizer.step()
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def load(self, name):
        self.model.load_state_dict(torch.load(name))

    def save(self, name):
        torch.save(self.model.state_dict(), name)

# Reward function based on distance along centerline
def calculate_reward(pose, centerline):
    car_x, car_y = pose.position.x, pose.position.y
    closest_distance = min(np.linalg.norm(np.array([car_x, car_y]) - np.array([cx, cy])) for cx, cy in centerline)
    return 1.0 / (closest_distance + 1e-5)  # Avoid division by zero

# ROS2 node for controlling the car using rclpy
class F1TenthEnv(Node):
    def __init__(self):
        super().__init__('f1tenth_dqn_node')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/initialpose', self.pose_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        self.state = None
        self.done = False
        self.reward = 0
        self.centerline = self.load_centerline()
    
    def scan_callback(self, data):
        self.state = np.array(data.ranges)
        self.get_logger().info("Received scan data, updated state.")
        self.reward = calculate_reward(data.pose, self.centerline)
        if self.check_collision(data.ranges):
            self.done = True
            self.get_logger().warn("Collision detected, resetting...")


    def check_collision(self, ranges):
        '''IMPLEMENT COLLISION LOGIC'''
        degrees = ranges[0::4]
        for i in degrees:
            point = [sin(degrees.index(i)-45)*i, cos(degrees.index(i)-45)*i]
            if abs(point[0])<= 0.1516 and abs(point[1]) <= 0.0760:
                return True
        return False

    def reset(self):
        start_pose = random.choice(self.centerline)
        self.publish_pose(start_pose)
        self.done = False
        self.get_logger().info(f"Reset car to start position: {start_pose}")
        return self.state

    def publish_pose(self, pose):
        reset_pose = PoseStamped()
        reset_pose.pose.position.x, reset_pose.pose.position.y = pose
        self.drive_pub.publish(reset_pose)

    def step(self, action):
        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.speed = action[0]
        drive_cmd.drive.steering_angle = action[1]
        self.drive_pub.publish(drive_cmd)
        self.get_logger().info(f"Applied action - Speed: {action[0]}, Steering: {action[1]}")
        return self.state, self.reward, self.done, {}

    def load_centerline(self):
        try:
            with open('jayhawk_raceway_s.csv', newline='') as f:
                reader = csv.reader(f)
                centerline = [tuple(map(float, row)) for row in reader]
            self.get_logger().info("Centerline loaded successfully.")
            return centerline
        except Exception as e:
            self.get_logger().error(f"Error loading centerline: {e}")
            return []

# Main function to run the training loop
def main():
    rclpy.init()

    env = F1TenthEnv()

    env.get_logger().info("Waiting for sensor data...")
    while env.state is None and rclpy.ok():
        rclpy.spin_once(env, timeout_sec=0.1)

    state_size = len(env.state)
    action_size = 2  # Speed and steering angle
    agent = DQNAgent(state_size, action_size)
    
    batch_size = 32
    episode_count = 0
    
    env.get_logger().info("Starting Deep Q-Learning Training...")
    
    while rclpy.ok():
        state = env.reset()
        done = False
        episode_reward = 0
        
        while not done and rclpy.ok():
            action = agent.act(state)
            next_state, reward, done, _ = env.step(action)
            episode_reward += reward
            agent.remember(state, action, reward, next_state, done)
            state = next_state
            
            if len(agent.memory) > batch_size:
                agent.replay(batch_size)
        
        episode_count += 1
        env.get_logger().info(f"Episode: {episode_count}, Reward: {episode_reward}")
    
    agent.save("f1tenth_dqn_model.pth")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
