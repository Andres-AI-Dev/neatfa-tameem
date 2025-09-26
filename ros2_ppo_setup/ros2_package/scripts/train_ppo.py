#!/usr/bin/env python3
"""
PPO Training Script for ARGoS Swarm Foraging
Uses stable-baselines3 with ROS2-ARGoS environment
"""

import os
import argparse
import time
from datetime import datetime
import numpy as np
import torch

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor
from stable_baselines3.common.callbacks import (
    BaseCallback, 
    CheckpointCallback, 
    EvalCallback
)
from stable_baselines3.common.logger import configure
from stable_baselines3.common.utils import set_random_seed

import sys
sys.path.append('../src')
from argos_ros2_env import ArgosROS2Env


class TensorboardCallback(BaseCallback):
    """
    Custom callback for additional tensorboard logging.
    """
    
    def __init__(self, verbose=0):
        super(TensorboardCallback, self).__init__(verbose)
        self.episode_rewards = []
        self.episode_lengths = []
        
    def _on_step(self) -> bool:
        # Log custom metrics
        if 'food_collected' in self.locals.get('infos', [{}])[0]:
            food_collected = self.locals['infos'][0]['food_collected']
            self.logger.record('argos/food_collected', food_collected)
        
        return True
    
    def _on_rollout_end(self) -> None:
        # Log episode statistics
        if len(self.episode_rewards) > 0:
            self.logger.record('rollout/ep_rew_mean', np.mean(self.episode_rewards))
            self.logger.record('rollout/ep_len_mean', np.mean(self.episode_lengths))
            
            # Reset for next rollout
            self.episode_rewards = []
            self.episode_lengths = []


def make_env(env_id: int, num_robots: int = 6):
    """
    Factory function for creating environments.
    """
    def _init():
        env = ArgosROS2Env(env_id, num_robots)
        return env
    set_random_seed(env_id)
    return _init


def train_ppo(args):
    """
    Main training function for PPO on ARGoS swarm foraging.
    """
    
    # Create timestamp for this run
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_name = f"ppo_argos_{timestamp}"
    
    # Set up directories
    log_dir = f"./logs/{run_name}"
    model_dir = f"./models/{run_name}"
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(model_dir, exist_ok=True)
    
    # Create vectorized environment
    print(f"Creating {args.n_envs} parallel environments...")
    env = SubprocVecEnv([make_env(i, args.n_robots) for i in range(args.n_envs)])
    env = VecMonitor(env, log_dir)
    
    # Create evaluation environment
    eval_env = SubprocVecEnv([make_env(100, args.n_robots)])
    
    # Configure logging
    logger = configure(log_dir, ["stdout", "tensorboard"])
    
    # PPO hyperparameters optimized for swarm robotics
    ppo_params = {
        "policy": "MlpPolicy",
        "env": env,
        "learning_rate": 3e-4,
        "n_steps": 2048,  # Steps per environment per update
        "batch_size": 64,
        "n_epochs": 10,
        "gamma": 0.99,  # Discount factor
        "gae_lambda": 0.95,  # GAE lambda
        "clip_range": 0.2,
        "ent_coef": 0.01,  # Entropy coefficient for exploration
        "vf_coef": 0.5,  # Value function coefficient
        "max_grad_norm": 0.5,
        "tensorboard_log": log_dir,
        "policy_kwargs": dict(
            net_arch=[256, 256, 128],  # Neural network architecture
            activation_fn=torch.nn.ReLU
        ),
        "verbose": 1
    }
    
    print("Initializing PPO model...")
    model = PPO(**ppo_params)
    model.set_logger(logger)
    
    # Set up callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=10000,
        save_path=model_dir,
        name_prefix="ppo_checkpoint"
    )
    
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=model_dir,
        log_path=log_dir,
        eval_freq=5000,
        deterministic=True,
        render=False
    )
    
    tensorboard_callback = TensorboardCallback()
    
    callbacks = [checkpoint_callback, eval_callback, tensorboard_callback]
    
    # Training loop
    print(f"Starting training for {args.total_timesteps} timesteps...")
    print(f"This will take approximately {args.total_timesteps / (args.n_envs * 1000)} minutes")
    
    try:
        model.learn(
            total_timesteps=args.total_timesteps,
            callback=callbacks,
            progress_bar=True
        )
        
        print(f"Training completed! Saving final model...")
        model.save(f"{model_dir}/ppo_final")
        
        print(f"Model saved to {model_dir}/ppo_final")
        print(f"Tensorboard logs saved to {log_dir}")
        print(f"To view training progress, run: tensorboard --logdir {log_dir}")
        
    except KeyboardInterrupt:
        print("\nTraining interrupted by user. Saving current model...")
        model.save(f"{model_dir}/ppo_interrupted")
        print(f"Model saved to {model_dir}/ppo_interrupted")
    
    finally:
        env.close()
        eval_env.close()


def evaluate_model(model_path: str, n_episodes: int = 10):
    """
    Evaluate a trained PPO model.
    """
    print(f"Loading model from {model_path}...")
    model = PPO.load(model_path)
    
    # Create environment for evaluation
    env = ArgosROS2Env(env_id=0, num_robots=6)
    
    episode_rewards = []
    episode_lengths = []
    food_collected_list = []
    
    for episode in range(n_episodes):
        obs, info = env.reset()
        episode_reward = 0
        episode_length = 0
        
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            
            episode_reward += reward
            episode_length += 1
        
        episode_rewards.append(episode_reward)
        episode_lengths.append(episode_length)
        food_collected_list.append(info.get('food_collected', 0))
        
        print(f"Episode {episode + 1}: Reward = {episode_reward:.2f}, "
              f"Length = {episode_length}, Food = {info.get('food_collected', 0)}")
    
    print(f"\nEvaluation Results over {n_episodes} episodes:")
    print(f"Mean Reward: {np.mean(episode_rewards):.2f} ± {np.std(episode_rewards):.2f}")
    print(f"Mean Length: {np.mean(episode_lengths):.2f} ± {np.std(episode_lengths):.2f}")
    print(f"Mean Food Collected: {np.mean(food_collected_list):.2f} ± {np.std(food_collected_list):.2f}")
    
    env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PPO Training for ARGoS Swarm Foraging")
    
    parser.add_argument("--n_envs", type=int, default=4,
                       help="Number of parallel environments")
    parser.add_argument("--n_robots", type=int, default=6,
                       help="Number of robots per environment")
    parser.add_argument("--total_timesteps", type=int, default=1_000_000,
                       help="Total training timesteps")
    parser.add_argument("--evaluate", type=str, default=None,
                       help="Path to model for evaluation only")
    parser.add_argument("--n_eval_episodes", type=int, default=10,
                       help="Number of evaluation episodes")
    
    args = parser.parse_args()
    
    if args.evaluate:
        evaluate_model(args.evaluate, args.n_eval_episodes)
    else:
        train_ppo(args)