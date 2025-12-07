#!/usr/bin/env python3

"""
RL Walking Policy for Humanoid Robot using Isaac Gym

This script implements a reinforcement learning approach for humanoid walking
using NVIDIA Isaac Gym. It defines the environment, policy network, and
training loop for learning stable walking gaits.
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.distributions import Normal
import pickle

# Check if isaacgym is available
try:
    import isaacgym
    from isaacgym import gymapi, gymtorch
    from isaacgym.torch_utils import *
    ISAAC_GYM_AVAILABLE = True
except ImportError:
    print("Isaac Gym not found. This is expected in non-Isaac Gym environments.")
    ISAAC_GYM_AVAILABLE = False


class HumanoidEnv:
    """
    Humanoid environment for Isaac Gym
    """
    def __init__(self, cfg):
        if not ISAAC_GYM_AVAILABLE:
            print("Isaac Gym not available. Using mock environment for demonstration.")
            self.mock_env = True
            self.cfg = cfg
            return

        self.mock_env = False
        self.gym = gymapi.acquire_gym()
        self.sim = None
        self.envs = []
        self.cfg = cfg

        # Initialize simulation
        self._create_sim()
        self._create_envs()

    def _create_sim(self):
        """Create the simulation"""
        # Set up sim params
        sim_params = gymapi.SimParams()
        sim_params.dt = self.cfg['dt']
        sim_params.up_axis = gymapi.UP_AXIS_Z
        sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)

        # Set PhysX params
        sim_params.physx.solver_type = 1
        sim_params.physx.num_position_iterations = 8
        sim_params.physx.num_velocity_iterations = 1
        sim_params.physx.max_gpu_contact_pairs = 2**23
        sim_params.physx.max_gpu_found_lost_pairs = 2**24
        sim_params.physx.use_gpu = True

        # Set cache sizes
        sim_params.physx.max_gpu_dense_articulations = 2**10
        sim_params.physx.max_gpu_rigid_body_pairs = 2**12
        sim_params.physx.max_gpu_contacts = 2**12

        # Create sim
        self.sim = self.gym.create_sim(self.cfg['compute_device_id'],
                                       self.cfg['graphics_device_id'],
                                       self.cfg['physics_engine'],
                                       sim_params)

    def _create_envs(self):
        """Create environments"""
        # Create asset root
        asset_root = self.cfg['asset_root']
        asset_file = self.cfg['asset_file']

        # Load humanoid asset
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = False
        asset_options.default_dof_drive_mode = gymapi.DOF_MODE_EFFORT
        asset_options.height_field_resolution = 4
        asset_options.armature = 0.01
        asset_options.thickness = 0.01
        asset_options.angular_damping = 0.01
        asset_options.linear_damping = 0.01

        asset = self.gym.load_asset(self.sim, asset_root, asset_file, asset_options)

        # Get default pose
        default_pose = gymapi.Transform()
        default_pose.p = gymapi.Vec3(0.0, 0.0, 1.0)
        default_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)

        # Create environments
        spacing = self.cfg['env_spacing']
        env_lower = gymapi.Vec3(-spacing, -spacing, 0.0)
        env_upper = gymapi.Vec3(spacing, spacing, spacing)

        for i in range(self.cfg['num_envs']):
            # Create env
            env = self.gym.create_env(self.sim, env_lower, env_upper, 1)

            # Create actor
            actor_handle = self.gym.create_actor(env, asset, default_pose, "humanoid", i, 0, 0)

            # Configure DOFs
            props = self.gym.get_actor_dof_properties(env, actor_handle)
            props["driveMode"].fill(gymapi.DOF_MODE_EFFORT)
            props["stiffness"].fill(500.0)
            props["damping"].fill(50.0)
            self.gym.set_actor_dof_properties(env, actor_handle, props)

            # Store environment
            self.envs.append(env)

    def reset(self):
        """Reset the environment"""
        if self.mock_env:
            # Mock state for demonstration
            state_dim = self.cfg.get('state_dim', 48)
            return torch.randn(1, state_dim, dtype=torch.float32)

        # Reset simulation
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)

        # Get state
        state = self._get_state()
        return state

    def step(self, actions):
        """Step the environment"""
        if self.mock_env:
            # Mock step for demonstration
            state = torch.randn(1, self.cfg.get('state_dim', 48), dtype=torch.float32)
            reward = torch.tensor([np.random.random()], dtype=torch.float32)
            done = torch.tensor([np.random.random() < 0.1], dtype=torch.bool)
            return state, reward, done, {}

        # Apply actions
        self._apply_actions(actions)

        # Step simulation
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)

        # Get next state
        next_state = self._get_state()
        reward = self._compute_reward()
        done = self._check_termination()

        return next_state, reward, done, {}

    def _get_state(self):
        """Get the current state of the environment"""
        # This would involve getting joint positions, velocities, etc.
        # from the Isaac Gym simulation
        if self.mock_env:
            return torch.randn(1, self.cfg.get('state_dim', 48), dtype=torch.float32)

        # In real Isaac Gym environment, this would get actual state tensors
        # from the simulation
        # For now, returning a placeholder - in real implementation this would
        # extract state information from the Isaac Gym simulation
        state_dim = self.cfg.get('state_dim', 48)
        return torch.randn(1, state_dim, dtype=torch.float32)

    def _apply_actions(self, actions):
        """Apply actions to the environment"""
        # This would involve setting joint torques or positions
        # based on the policy output
        if self.mock_env:
            # In mock environment, just return
            return

        # In real Isaac Gym environment, this would apply actions
        # to the simulation - setting joint torques or positions
        # For now, this is a placeholder
        pass

    def _compute_reward(self):
        """Compute reward based on current state"""
        # This would involve calculating a reward function based on
        # forward progress, balance, energy efficiency, etc.
        if self.mock_env:
            # Simple mock reward - in real implementation this would be more sophisticated
            return torch.tensor([np.random.random()], dtype=torch.float32)

        # In real Isaac Gym environment, this would compute
        # actual reward from simulation state
        # For now, returning a placeholder - in real implementation this would
        # calculate reward based on humanoid's performance (forward progress, balance, etc.)
        return torch.tensor([0.0], dtype=torch.float32)

    def _check_termination(self):
        """Check if the episode should terminate"""
        # This would involve checking if the robot has fallen
        # or reached the goal
        if self.mock_env:
            # Simple mock termination - in real implementation this would be more sophisticated
            return torch.tensor([np.random.random() < 0.1], dtype=torch.bool)

        # In real Isaac Gym environment, this would check
        # termination conditions from simulation
        # For now, returning a placeholder - in real implementation this would
        # check if the humanoid has fallen, reached a goal, or exceeded time limit
        return torch.tensor([False], dtype=torch.bool)


class ActorCritic(nn.Module):
    """
    Actor-Critic network for humanoid walking policy
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
            nn.Linear(hidden_dim, action_dim)
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

        # Log std for action distribution
        self.log_std = nn.Parameter(torch.zeros(action_dim))

    def forward(self, state):
        """Forward pass for both actor and critic"""
        value = self.critic(state)
        mean = self.actor(state)
        std = torch.exp(self.log_std)

        return mean, std, value

    def get_action(self, state):
        """Sample action from the policy"""
        mean, std, value = self.forward(state)
        dist = Normal(mean, std)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(dim=-1)

        return action, log_prob, value

    def evaluate(self, state, action):
        """Evaluate the action for training"""
        mean, std, value = self.forward(state)
        dist = Normal(mean, std)
        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)

        return log_prob, entropy, value


class PPOAgent:
    """
    PPO Agent for training the humanoid walking policy
    """
    def __init__(self, state_dim, action_dim, lr=3e-4, gamma=0.99,
                 eps_clip=0.2, k_epochs=4, hidden_dim=512):
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.k_epochs = k_epochs

        self.policy = ActorCritic(state_dim, action_dim, hidden_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=lr)
        self.policy_old = ActorCritic(state_dim, action_dim, hidden_dim)
        self.policy_old.load_state_dict(self.policy.state_dict())

        self.MseLoss = nn.MSELoss()

    def update(self, state_batch, action_batch, log_prob_batch,
               reward_batch, done_batch):
        """Update the policy using PPO"""
        # Convert to tensors
        state_batch = torch.stack(state_batch).detach()
        action_batch = torch.stack(action_batch).detach()
        old_log_prob_batch = torch.stack(log_prob_batch).detach()
        reward_batch = torch.stack(reward_batch).detach()
        done_batch = torch.stack(done_batch).detach()

        # Monte Carlo estimate of rewards
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(reward_batch), reversed(done_batch)):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)

        # Normalizing the rewards
        rewards = torch.tensor(rewards, dtype=torch.float32)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-7)

        # Optimize policy for K epochs
        for _ in range(self.k_epochs):
            # Evaluate old actions and values
            log_prob, entropy, state_values = self.policy.evaluate(state_batch, action_batch)

            # Finding the ratio (pi_theta / pi_theta__old)
            ratios = torch.exp(log_prob - old_log_prob_batch.detach())

            # Finding Surrogate Loss
            advantages = rewards - state_values.detach()
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1 - self.eps_clip, 1 + self.eps_clip) * advantages
            actor_loss = -torch.min(surr1, surr2).mean()
            critic_loss = self.MseLoss(state_values, rewards.unsqueeze(1))

            # Total loss
            loss = actor_loss + 0.5 * critic_loss - 0.01 * entropy.mean()

            # Take gradient step
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

        # Copy new weights into old policy
        self.policy_old.load_state_dict(self.policy.state_dict())


def train_humanoid_walking(cfg):
    """
    Train the humanoid walking policy using PPO
    """
    print("Starting humanoid walking training...")

    # Create environment
    env = HumanoidEnv(cfg)

    # Get state and action dimensions
    if ISAAC_GYM_AVAILABLE:
        state_dim = 48  # Example state dimension
        action_dim = 12  # Example action dimension (12 joints for simplified humanoid)
    else:
        state_dim = cfg.get('state_dim', 48)
        action_dim = cfg.get('action_dim', 12)

    # Create agent
    agent = PPOAgent(state_dim, action_dim,
                     lr=cfg.get('learning_rate', 3e-4),
                     gamma=cfg.get('gamma', 0.99),
                     eps_clip=cfg.get('eps_clip', 0.2),
                     k_epochs=cfg.get('k_epochs', 4))

    # Training variables
    max_episodes = cfg.get('max_episodes', 1000)
    max_timesteps = cfg.get('max_timesteps', 1000)
    update_timestep = cfg.get('update_timestep', 2000)

    # Storage for training data
    state_batch = []
    action_batch = []
    log_prob_batch = []
    reward_batch = []
    done_batch = []

    timestep = 0

    # Training loop
    for episode in range(max_episodes):
        state = env.reset()
        episode_reward = 0

        for t in range(max_timesteps):
            timestep += 1

            # Select action
            with torch.no_grad():
                action, log_prob, _ = agent.policy_old.get_action(state)

            # Store old state, action, log_prob
            state_batch.append(state)
            action_batch.append(action)
            log_prob_batch.append(log_prob)

            # Take action
            state, reward, done, _ = env.step(action)

            # Store results
            reward_batch.append(reward)
            done_batch.append(done)
            episode_reward += reward.item()

            # Update if it's time
            if timestep % update_timestep == 0:
                agent.update(state_batch, action_batch, log_prob_batch,
                             reward_batch, done_batch)

                # Clear batch buffers
                state_batch.clear()
                action_batch.clear()
                log_prob_batch.clear()
                reward_batch.clear()
                done_batch.clear()

            if done:
                break

        # Print episode results
        print(f'Episode {episode}: Reward = {episode_reward:.2f}')

        # Save model periodically
        if episode % cfg.get('save_interval', 100) == 0:
            save_model(agent, cfg.get('model_save_path', 'models/'), episode)

    # Save final model
    save_model(agent, cfg.get('model_save_path', 'models/'), 'final')
    print("Training completed!")


def save_model(agent, model_path, episode):
    """Save the trained model"""
    import os
    os.makedirs(model_path, exist_ok=True)

    model_dict = {
        'policy_state_dict': agent.policy.state_dict(),
        'optimizer_state_dict': agent.optimizer.state_dict(),
        'episode': episode
    }

    torch.save(model_dict, f'{model_path}/humanoid_walking_model_{episode}.pth')
    print(f"Model saved at {model_path}/humanoid_walking_model_{episode}.pth")


def load_model(model_path, state_dim=48, action_dim=12):
    """Load a trained model"""
    model_dict = torch.load(model_path)

    # Recreate the agent with the same dimensions
    agent = PPOAgent(state_dim, action_dim)

    # Load the saved state dictionaries
    agent.policy.load_state_dict(model_dict['policy_state_dict'])
    agent.optimizer.load_state_dict(model_dict['optimizer_state_dict'])

    # Copy policy to old policy for inference
    agent.policy_old.load_state_dict(agent.policy.state_dict())

    print(f"Model loaded from {model_path}")
    print(f"Episode: {model_dict['episode']}")

    return agent


def main():
    """
    Main function to configure and start training
    """
    # Configuration
    cfg = {
        'dt': 1.0/60.0,  # Simulation timestep
        'num_envs': 4096,  # Number of parallel environments
        'env_spacing': 5.0,  # Spacing between environments
        'asset_root': './assets',  # Root path to assets
        'asset_file': 'humanoid.urdf',  # Humanoid asset file
        'compute_device_id': 0,  # Compute device (GPU)
        'graphics_device_id': 0,  # Graphics device (GPU)
        'physics_engine': gymapi.SIM_PHYSX,  # Physics engine

        # Training parameters
        'max_episodes': 10000,
        'max_timesteps': 1000,
        'update_timestep': 2000,
        'learning_rate': 3e-4,
        'gamma': 0.99,
        'eps_clip': 0.2,
        'k_epochs': 4,
        'save_interval': 500,
        'model_save_path': 'models/',

        # State and action dimensions (for mock environment)
        'state_dim': 48,
        'action_dim': 12,
    }

    # Start training
    train_humanoid_walking(cfg)


if __name__ == '__main__':
    main()