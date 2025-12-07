#!/usr/bin/env python3

"""
Training Script for RL Walking Policy

This script demonstrates how the RL walking policy would be trained
in Isaac Gym (simulated environment) before deployment to real robots.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.distributions import Normal
from collections import deque
import random
import os


class ActorCritic(nn.Module):
    """
    Actor-Critic network for humanoid walking control
    """
    def __init__(self, state_dim, action_dim, hidden_dim=512):
        super(ActorCritic, self).__init__()

        # Actor network (policy)
        self.actor_mean = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Tanh()
        )

        self.actor_std = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Softplus()  # Ensure std is positive
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
        action_mean = self.actor_mean(state)
        action_std = self.actor_std(state)
        value = self.critic(state)
        return action_mean, action_std, value

    def get_action(self, state):
        action_mean, action_std, value = self.forward(state)
        dist = Normal(action_mean, action_std)
        action = dist.sample()
        log_prob = dist.log_prob(action)
        return action, log_prob, value


class PPOAgent:
    """
    Proximal Policy Optimization (PPO) agent for training
    """
    def __init__(self, state_dim, action_dim, lr=3e-4, gamma=0.99, eps_clip=0.2, k_epochs=4):
        self.lr = lr
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.k_epochs = k_epochs

        self.policy = ActorCritic(state_dim, action_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=lr)
        self.policy_old = ActorCritic(state_dim, action_dim)
        self.policy_old.load_state_dict(self.policy.state_dict())

        self.MseLoss = nn.MSELoss()

    def update(self, memory):
        # Monte Carlo estimate of state rewards
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(memory.rewards), reversed(memory.is_terminals)):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)

        # Normalizing the rewards
        rewards = torch.tensor(rewards, dtype=torch.float32)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-5)

        # Convert list to tensor
        old_states = torch.stack(memory.states).detach()
        old_actions = torch.stack(memory.actions).detach()
        old_logprobs = torch.stack(memory.logprobs).detach()

        # Optimize policy for K epochs
        for _ in range(self.k_epochs):
            # Evaluating old actions and values
            logprobs, state_values, dist_entropy = self.evaluate(old_states, old_actions)

            # Finding the ratio (pi_theta / pi_theta__old)
            ratios = torch.exp(logprobs - old_logprobs.detach())

            # Finding Surrogate Loss
            advantages = rewards - state_values.detach()
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1 - self.eps_clip, 1 + self.eps_clip) * advantages
            loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values, rewards) - 0.01 * dist_entropy

            # Take gradient step
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()

        # Copy new weights into old policy
        self.policy_old.load_state_dict(self.policy.state_dict())

    def evaluate(self, state, action):
        action_mean, action_std, state_value = self.policy_old(state)

        dist = Normal(action_mean, action_std)

        action_logprobs = dist.log_prob(action)
        dist_entropy = dist.entropy()

        return action_logprobs, state_value, dist_entropy


class Memory:
    """
    Memory buffer for storing training data
    """
    def __init__(self):
        self.actions = []
        self.states = []
        self.logprobs = []
        self.rewards = []
        self.is_terminals = []

    def clear_memory(self):
        del self.actions[:]
        del self.states[:]
        del self.logprobs[:]
        del self.rewards[:]
        del self.is_terminals[:]


class RLWalkingTrainer(Node):
    """
    A node that simulates training of RL walking policies in Isaac Gym
    """

    def __init__(self):
        super().__init__('rl_walking_trainer')

        # Training parameters
        self.state_dim = 48  # Example: joint positions, velocities, IMU data, command
        self.action_dim = 12  # Example: 12 joint position commands for legs
        self.hidden_dim = 512
        self.max_episodes = 1000
        self.max_timesteps = 1000
        self.update_timestep = 2000  # Update policy every n timesteps
        self.lr = 0.0003
        self.gamma = 0.99
        self.eps_clip = 0.2
        self.k_epochs = 4

        # Initialize PPO agent
        self.agent = PPOAgent(self.state_dim, self.action_dim, self.lr, self.gamma, self.eps_clip, self.k_epochs)
        self.memory = Memory()

        # Training metrics
        self.running_reward = 0
        self.avg_length = 0
        self.log_interval = 20  # Print avg reward in the interval
        self.reeward_threshold = 500  # Threshold to stop training

        # Robot state simulation
        self.current_episode = 0
        self.timestep = 0
        self.episode_reward = 0

        # Timer for training loop
        self.train_timer = self.create_timer(0.001, self.train_step)  # Fast timer for simulation

        self.get_logger().info('RL Walking Policy Trainer initialized')

    def reset_environment(self):
        """
        Reset the simulated environment
        """
        # In a real Isaac Gym simulation, this would reset the robot to a stable position
        # For this example, we'll just return a random state
        return torch.FloatTensor(np.random.randn(self.state_dim)).unsqueeze(0)

    def simulate_environment(self, action):
        """
        Simulate the environment response to an action
        """
        # In a real Isaac Gym simulation, this would simulate physics and return next state, reward, done
        # For this example, we'll return a random next state and a simple reward function

        # Simple reward: encourage forward movement while maintaining balance
        reward = 0.0
        if action[0] > 0:  # If moving forward
            reward += 1.0  # Reward forward movement
        else:
            reward -= 0.5  # Penalize backward movement

        # Penalize large deviations from upright position (simplified)
        reward -= abs(action[1]) * 0.1  # Penalize excessive lateral movement

        # Random next state (in real implementation, this would come from Isaac Gym)
        next_state = torch.FloatTensor(np.random.randn(self.state_dim)).unsqueeze(0)

        # Random done condition (in real implementation, this would be based on robot falling)
        done = random.random() < 0.01  # 1% chance of episode ending

        return next_state, reward, done

    def train_step(self):
        """
        Single training step
        """
        if self.timestep == 0:
            # Reset environment at the start of each episode
            self.state = self.reset_environment()
            self.episode_reward = 0

        # Select action with the policy
        with torch.no_grad():
            action, logprob, _ = self.agent.policy_old.get_action(self.state)

        # Store state, action, logprob in memory
        self.memory.states.append(self.state)
        self.memory.actions.append(action)
        self.memory.logprobs.append(logprob)

        # Simulate environment step
        next_state, reward, done = self.simulate_environment(action.numpy())

        # Store reward and terminal state
        self.memory.rewards.append(reward)
        self.memory.is_terminals.append(done)

        # Update state and metrics
        self.state = next_state
        self.episode_reward += reward
        self.timestep += 1

        # If max timesteps reached or episode ended, update policy
        if self.timestep % self.update_timestep == 0:
            self.agent.update(self.memory)
            self.memory.clear_memory()
            self.timestep = 0

        if done:
            self.running_reward += self.episode_reward
            self.current_episode += 1

            # Print average reward every log interval
            if self.current_episode % self.log_interval == 0:
                avg_reward = self.running_reward / self.log_interval
                self.get_logger().info(f'Episode {self.current_episode}, Average Reward: {avg_reward:.2f}')
                self.running_reward = 0

            # Check if training is complete
            if self.current_episode >= self.max_episodes:
                self.save_model()
                self.get_logger().info('Training completed!')
                self.train_timer.cancel()

    def save_model(self):
        """
        Save the trained model
        """
        model_path = 'isaac_gym_rl/models/trained_walking_policy.pth'
        os.makedirs(os.path.dirname(model_path), exist_ok=True)
        torch.save(self.agent.policy.state_dict(), model_path)
        self.get_logger().info(f'Model saved to {model_path}')


def main(args=None):
    rclpy.init(args=args)

    rl_walking_trainer = RLWalkingTrainer()

    try:
        rclpy.spin(rl_walking_trainer)
    except KeyboardInterrupt:
        pass
    finally:
        rl_walking_trainer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()