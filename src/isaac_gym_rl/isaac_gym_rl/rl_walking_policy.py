#!/usr/bin/env python3

"""
Reinforcement Learning Walking Policy for Humanoid Robots

This module implements a pre-trained RL policy for humanoid walking
that can be deployed on real robots after training in Isaac Gym.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import torch
import torch.nn as nn
from collections import deque


class ActorCritic(nn.Module):
    """
    Simple Actor-Critic network for humanoid walking control
    """
    def __init__(self, state_dim, action_dim, hidden_dim=512):
        super(ActorCritic, self).__init__()

        # Actor network (policy)
        self.actor = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Tanh()
        )

        # Critic network (value function)
        self.critic = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )

    def forward(self, state):
        action = self.actor(state)
        value = self.critic(state)
        return action, value


class RLWalkingPolicy(Node):
    """
    A node that implements a reinforcement learning walking policy
    trained in Isaac Gym and deployed on a humanoid robot.
    """

    def __init__(self):
        super().__init__('rl_walking_policy')

        # Initialize neural network
        self.state_dim = 48  # Example: joint positions, velocities, IMU data, command
        self.action_dim = 12  # Example: 12 joint position commands for legs
        self.hidden_dim = 512

        self.policy_network = ActorCritic(self.state_dim, self.action_dim, self.hidden_dim)

        # Load pre-trained model weights
        self.load_model()

        # Robot state variables
        self.joint_positions = np.zeros(24)  # Example: 24 total joints
        self.joint_velocities = np.zeros(24)
        self.imu_data = np.zeros(6)  # orientation + angular velocity
        self.target_velocity = np.array([0.0, 0.0, 0.0])  # linear x, y, angular z
        self.prev_actions = deque(maxlen=10)  # Store previous actions for stability

        # Publishers and subscribers
        qos_profile = QoSProfile(depth=10)

        self.joint_cmd_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            qos_profile
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            qos_profile
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info('RL Walking Policy initialized')

    def load_model(self):
        """
        Load pre-trained model weights
        In a real implementation, this would load from a file
        """
        # For demonstration, we'll initialize with random weights
        # In practice, this would load a model trained in Isaac Gym
        self.get_logger().info('Initializing policy network with random weights')
        # In a real implementation, you would use:
        # self.policy_network.load_state_dict(torch.load('path/to/pretrained/model.pth'))

    def joint_state_callback(self, msg):
        """
        Callback for joint state messages
        """
        if len(msg.position) == len(self.joint_positions):
            self.joint_positions = np.array(msg.position)
        if len(msg.velocity) == len(self.joint_velocities):
            self.joint_velocities = np.array(msg.velocity)

    def imu_callback(self, msg):
        """
        Callback for IMU data
        """
        # Extract orientation (convert quaternion to euler angles)
        w, x, y, z = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arcsin(2 * (w * y - z * x))

        # Extract angular velocity
        ang_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        self.imu_data = np.array([roll, pitch, 0.0, ang_vel[0], ang_vel[1], ang_vel[2]])

    def cmd_vel_callback(self, msg):
        """
        Callback for velocity commands
        """
        self.target_velocity = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.angular.z
        ])

    def build_state_vector(self):
        """
        Build state vector for the RL policy
        Combines joint states, IMU data, and target velocity
        """
        # Normalize joint positions (example ranges)
        norm_joint_pos = np.clip(self.joint_positions / np.pi, -1.0, 1.0)

        # Normalize joint velocities (example ranges)
        norm_joint_vel = np.clip(self.joint_velocities / 10.0, -1.0, 1.0)

        # Combine all state components
        state = np.concatenate([
            norm_joint_pos,      # Joint positions
            norm_joint_vel,      # Joint velocities
            self.imu_data,       # IMU data (orientation, angular velocity)
            self.target_velocity # Target velocity command
        ])

        return torch.FloatTensor(state).unsqueeze(0)

    def control_loop(self):
        """
        Main control loop that runs the RL policy
        """
        # Build state vector
        state = self.build_state_vector()

        # Get action from policy (inference mode)
        with torch.no_grad():
            action, _ = self.policy_network(state)

        # Convert action to joint commands
        # This is a simplified example - real implementation would depend on robot structure
        joint_commands = action.squeeze().numpy()

        # Apply action scaling if needed
        joint_commands = np.clip(joint_commands, -1.0, 1.0)  # Limit to reasonable range

        # Create and publish joint command message
        joint_cmd_msg = JointState()
        joint_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        joint_cmd_msg.header.frame_id = 'base_link'

        # For this example, we'll command only the leg joints
        # In practice, you'd map the RL output to specific joints
        joint_cmd_msg.name = [
            'left_hip_roll', 'left_hip_yaw', 'left_hip_pitch',
            'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
            'right_hip_roll', 'right_hip_yaw', 'right_hip_pitch',
            'right_knee', 'right_ankle_pitch', 'right_ankle_roll'
        ]

        joint_cmd_msg.position = joint_commands.tolist()

        self.joint_cmd_pub.publish(joint_cmd_msg)

        # Store action for stability
        self.prev_actions.append(joint_commands)

        self.get_logger().debug(f'Published joint commands: {joint_commands[:3]}...')


def main(args=None):
    rclpy.init(args=args)

    rl_walking_policy = RLWalkingPolicy()

    try:
        rclpy.spin(rl_walking_policy)
    except KeyboardInterrupt:
        pass
    finally:
        rl_walking_policy.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()