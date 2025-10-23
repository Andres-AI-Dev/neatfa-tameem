#!/usr/bin/env python3
"""
PPO Training with CPFA Foraging Environment
Single agent training for food foraging task
"""

import os
import sys
import time
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
from torch.distributions import Categorical
import logging
from datetime import datetime
from collections import deque
import json

# Setup paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
ARGOS_DIR = os.path.join(ROOT_DIR, "argos")
XML = os.path.join(ARGOS_DIR, "experiments", "cpfa_ppo_training.argos")

# Change to argos directory so it can find libraries
os.chdir(ARGOS_DIR)

# Import ARGoS environment
sys.path.append(os.path.join(ARGOS_DIR, "build"))
try:
    import iant_rl
except ImportError:
    print("Error: Could not import iant_rl. Make sure ARGoS is built.")
    sys.exit(1)

# Environment and Model Configuration
OBS_DIM = 15  # Food (1), proximity (8), light (6) sensors
NUM_ACTIONS = 15
ACTIONS = [
    # Full speed movements
    (16.0, 16.0, False),   # 0: Full speed forward
    (16.0, -16.0, False),  # 1: Full speed turn right
    (-16.0, 16.0, False),  # 2: Full speed turn left

    # Medium speed movements
    (8.0, 8.0, False),     # 3: Medium speed forward
    (8.0, -8.0, False),    # 4: Medium speed turn right
    (-8.0, 8.0, False),    # 5: Medium speed turn left

    # Slow speed movements (for precise food pickup)
    (4.0, 4.0, False),     # 6: Slow forward
    (4.0, -4.0, False),    # 7: Slow turn right
    (-4.0, 4.0, False),    # 8: Slow turn left

    # Stop and pheromone actions
    (0.0, 0.0, False),     # 9: Stop
    (0.0, 0.0, True),      # 10: Stop + lay pheromone

    # Pheromone-enhanced movements
    (16.0, 16.0, True),    # 11: Forward + pheromone
    (8.0, 8.0, True),      # 12: Medium forward + pheromone
    (12.0, 8.0, True),     # 13: Forward-right + pheromone
    (8.0, 12.0, True),     # 14: Forward-left + pheromone
]

# PPO Hyperparameters
LEARNING_RATE = 3e-4
GAMMA = 0.99
GAE_LAMBDA = 0.95
PPO_EPSILON = 0.2
PPO_EPOCHS = 4
BATCH_SIZE = 64
VALUE_COEF = 0.5
ENTROPY_COEF = 0.01
MAX_GRAD_NORM = 0.5

# Training Configuration
NUM_EPISODES = 1000  # Test run first (change to 200000+ for real training)
UPDATE_EVERY = 2048  # Number of steps before PPO update
LOG_INTERVAL = 10
SAVE_INTERVAL = 50

# Device configuration
DEVICE = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(f"Using device: {DEVICE}")
if torch.cuda.is_available():
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"GPU Memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.2f} GB")

class ActorCritic(nn.Module):
    """Actor-Critic network for PPO"""
    def __init__(self, obs_dim=OBS_DIM, num_actions=NUM_ACTIONS):
        super(ActorCritic, self).__init__()

        # Shared feature extraction layers
        self.feature_net = nn.Sequential(
            nn.Linear(obs_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
        )

        # Actor (policy) head
        self.actor = nn.Sequential(
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, num_actions)
        )

        # Critic (value) head
        self.critic = nn.Sequential(
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )

    def forward(self, obs):
        features = self.feature_net(obs)
        action_probs = F.softmax(self.actor(features), dim=-1)
        value = self.critic(features)
        return action_probs, value

    def get_action(self, obs, action=None):
        action_probs, value = self.forward(obs)
        dist = Categorical(action_probs)

        if action is None:
            action = dist.sample()

        return action, dist.log_prob(action), dist.entropy(), value


class PPOMemory:
    """Memory buffer for PPO"""
    def __init__(self):
        self.states = []
        self.actions = []
        self.log_probs = []
        self.values = []
        self.rewards = []
        self.dones = []

    def add(self, state, action, log_prob, value, reward, done):
        self.states.append(state)
        self.actions.append(action)
        self.log_probs.append(log_prob)
        self.values.append(value)
        self.rewards.append(reward)
        self.dones.append(done)

    def clear(self):
        self.states.clear()
        self.actions.clear()
        self.log_probs.clear()
        self.values.clear()
        self.rewards.clear()
        self.dones.clear()

    def get_batch(self):
        return (
            torch.stack(self.states).to(DEVICE),
            torch.stack(self.actions).to(DEVICE),
            torch.stack(self.log_probs).to(DEVICE),
            torch.stack(self.values).squeeze(-1).to(DEVICE),
            torch.tensor(self.rewards).to(DEVICE),
            torch.tensor(self.dones).to(DEVICE)
        )


def compute_gae(rewards, values, dones, next_value, gamma=GAMMA, lam=GAE_LAMBDA):
    """Compute Generalized Advantage Estimation"""
    advantages = []
    gae = 0

    for t in reversed(range(len(rewards))):
        if t == len(rewards) - 1:
            next_val = next_value
        else:
            next_val = values[t + 1]

        delta = rewards[t] + gamma * next_val * (1 - dones[t]) - values[t]
        gae = delta + gamma * lam * (1 - dones[t]) * gae
        advantages.insert(0, gae)

    return torch.tensor(advantages).to(DEVICE)


def update_ppo(model, optimizer, memory, epochs=PPO_EPOCHS):
    """Update model using PPO algorithm"""
    states, actions, old_log_probs, old_values, rewards, dones = memory.get_batch()

    # Compute returns and advantages
    with torch.no_grad():
        _, next_value = model(states[-1].unsqueeze(0))
        advantages = compute_gae(rewards, old_values, dones, next_value.item())
        returns = advantages + old_values

    # Normalize advantages
    advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

    # PPO update
    for _ in range(epochs):
        # Get current predictions
        _, new_log_probs, entropy, values = model.get_action(states, actions)
        values = values.squeeze(-1)

        # Compute ratios
        ratios = torch.exp(new_log_probs - old_log_probs)

        # Compute surrogate losses
        surr1 = ratios * advantages
        surr2 = torch.clamp(ratios, 1 - PPO_EPSILON, 1 + PPO_EPSILON) * advantages

        # Compute losses
        actor_loss = -torch.min(surr1, surr2).mean()
        critic_loss = F.mse_loss(values, returns)
        entropy_loss = -entropy.mean()

        # Total loss
        loss = actor_loss + VALUE_COEF * critic_loss + ENTROPY_COEF * entropy_loss

        # Update
        optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), MAX_GRAD_NORM)
        optimizer.step()

    return actor_loss.item(), critic_loss.item(), entropy_loss.item()


def compute_reward(obs, action_idx, food_collected, at_nest, carrying_food):
    """
    Compute reward for foraging task

    Rewards:
    - Finding food: +10
    - Returning food to nest: +20
    - Moving towards food/nest when appropriate: +1
    - Using pheromones effectively: +0.5
    - Collision/stuck: -1
    """
    reward = 0.0

    # Major rewards
    if food_collected:
        reward += 10.0
    if at_nest and carrying_food:
        reward += 20.0

    # Check sensor readings
    has_food = obs[0] > 0.5  # Robot is carrying food
    proximity = obs[1:9]  # Proximity sensors
    light_sensors = obs[9:15]  # Light sensors

    # Penalty for collisions
    if max(proximity) > 0.9:
        reward -= 1.0

    # Reward for using pheromones when carrying food
    if has_food and ACTIONS[action_idx][2]:  # Laying pheromone
        reward += 0.5

    # Small reward for movement towards light sources
    if max(light_sensors) > 0.2:
        forward_actions = [0, 3, 6, 11, 12]
        if action_idx in forward_actions:
            reward += 0.2

    # Small step penalty to encourage efficiency
    reward -= 0.01

    return reward


def train():
    """Main training loop"""
    # Setup logging
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = os.path.join(ARGOS_DIR, "..", "logs", f"ppo_cpfa_{timestamp}")
    os.makedirs(log_dir, exist_ok=True)

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(os.path.join(log_dir, "training.log")),
            logging.StreamHandler()
        ]
    )

    # Initialize model and optimizer
    model = ActorCritic().to(DEVICE)
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)

    print(f"\nModel initialized on {DEVICE}")
    print(f"Model parameters: {sum(p.numel() for p in model.parameters()):,}")

    # Training metrics
    episode_rewards = deque(maxlen=100)
    episode_lengths = deque(maxlen=100)
    total_steps = 0

    # Initialize environment once
    env = iant_rl.IAntRLEnv(XML)

    # Training loop
    for episode in range(NUM_EPISODES):

        # Episode variables
        memory = PPOMemory()
        episode_reward = 0
        episode_length = 0
        done = False

        # Get initial observation
        obs = env.reset()
        obs_tensor = torch.FloatTensor(obs).to(DEVICE)

        # Episode loop
        while not done:
            # Get action from model
            with torch.no_grad():
                action_idx, log_prob, _, value = model.get_action(obs_tensor.unsqueeze(0))
                action_idx = action_idx.item()

            # Execute action
            left_speed, right_speed, lay_pheromone = ACTIONS[action_idx]
            next_obs, reward, terminated, truncated, info = env.step(
                float(left_speed), float(right_speed), bool(lay_pheromone)
            )
            done = terminated or truncated

            # Use reward from environment (CPFA already computes proper foraging rewards)

            # Store transition
            memory.add(
                obs_tensor,
                torch.tensor(action_idx),
                log_prob.squeeze(),
                value.squeeze(),
                reward,
                float(done)
            )

            # Update counters
            episode_reward += reward
            episode_length += 1
            total_steps += 1

            # Update observation
            obs = next_obs
            obs_tensor = torch.FloatTensor(obs).to(DEVICE)

            # PPO update
            if total_steps % UPDATE_EVERY == 0:
                actor_loss, critic_loss, entropy_loss = update_ppo(model, optimizer, memory)
                memory.clear()

                logging.info(f"Step {total_steps} - Actor Loss: {actor_loss:.4f}, "
                           f"Critic Loss: {critic_loss:.4f}, Entropy: {entropy_loss:.4f}")

        # Episode complete
        episode_rewards.append(episode_reward)
        episode_lengths.append(episode_length)

        # Logging
        if episode % LOG_INTERVAL == 0:
            avg_reward = np.mean(episode_rewards) if episode_rewards else 0
            avg_length = np.mean(episode_lengths) if episode_lengths else 0

            logging.info(f"Episode {episode}/{NUM_EPISODES} - "
                        f"Avg Reward: {avg_reward:.2f}, Avg Length: {avg_length:.1f}, "
                        f"Last Reward: {episode_reward:.2f}")

        # Save checkpoint
        if episode % SAVE_INTERVAL == 0:
            checkpoint_path = os.path.join(log_dir, f"ppo_cpfa_episode_{episode}.pt")
            torch.save({
                'episode': episode,
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'episode_rewards': list(episode_rewards),
                'episode_lengths': list(episode_lengths),
            }, checkpoint_path)
            logging.info(f"Checkpoint saved: {checkpoint_path}")

    # Save final model
    final_model_path = os.path.join(log_dir, "ppo_cpfa_final.pt")
    torch.save({
        'episode': NUM_EPISODES,
        'model_state_dict': model.state_dict(),
        'optimizer_state_dict': optimizer.state_dict(),
        'episode_rewards': list(episode_rewards),
        'episode_lengths': list(episode_lengths),
        'obs_dim': OBS_DIM,
        'num_actions': NUM_ACTIONS,
        'actions': ACTIONS,
    }, final_model_path)

    logging.info(f"Training complete! Final model saved: {final_model_path}")

    # Save training summary
    summary = {
        'total_episodes': NUM_EPISODES,
        'final_avg_reward': float(np.mean(episode_rewards)),
        'final_avg_length': float(np.mean(episode_lengths)),
        'hyperparameters': {
            'learning_rate': LEARNING_RATE,
            'gamma': GAMMA,
            'gae_lambda': GAE_LAMBDA,
            'ppo_epsilon': PPO_EPSILON,
            'batch_size': BATCH_SIZE,
            'value_coef': VALUE_COEF,
            'entropy_coef': ENTROPY_COEF,
        }
    }

    with open(os.path.join(log_dir, "training_summary.json"), 'w') as f:
        json.dump(summary, f, indent=2)

    return model, log_dir


if __name__ == "__main__":
    print("Starting PPO training with CPFA foraging environment...")
    print(f"Configuration file: {XML}")
    print(f"Episodes: {NUM_EPISODES}")
    print(f"Actions: {NUM_ACTIONS}")

    model, log_dir = train()
    print(f"\nTraining complete! Logs and model saved in: {log_dir}")