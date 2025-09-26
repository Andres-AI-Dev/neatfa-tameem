#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from torch.distributions import Categorical
import os
import sys

# PPO Model Architecture (must match training script)
class ActorCritic(nn.Module):
    def __init__(self, obs_dim=15, num_actions=15):
        super(ActorCritic, self).__init__()

        # Shared feature layers
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

    def get_action(self, obs, deterministic=False):
        action_probs, value = self.forward(obs)

        if deterministic:
            action = torch.argmax(action_probs, dim=-1)
        else:
            dist = Categorical(action_probs)
            action = dist.sample()

        return action.item() if action.dim() == 0 else action.numpy()


class PPORos2Controller(Node):
    def __init__(self, model_path, robot_id="fb_0", deterministic=True):
        super().__init__(f'ppo_controller_{robot_id}')

        self.robot_id = robot_id
        self.deterministic = deterministic

        # Load PPO model
        self.get_logger().info(f'Loading PPO model from: {model_path}')
        self.model = ActorCritic()

        if os.path.exists(model_path):
            checkpoint = torch.load(model_path, map_location='cpu')
            self.model.load_state_dict(checkpoint['model_state_dict'])
            self.model.eval()
            self.get_logger().info(f'Model loaded successfully from episode {checkpoint["episode"]}')
        else:
            self.get_logger().error(f'Model file not found: {model_path}')
            sys.exit(1)

        # Create subscriber for observations
        self.obs_sub = self.create_subscription(
            Float32MultiArray,
            f'/iant/{robot_id}/observation',
            self.observation_callback,
            10)

        # Create publisher for actions
        self.action_pub = self.create_publisher(
            Int32,
            f'/iant/{robot_id}/action',
            10)

        self.get_logger().info(f'PPO ROS2 Controller initialized for robot: {robot_id}')
        self.get_logger().info(f'Subscribed to: /iant/{robot_id}/observation')
        self.get_logger().info(f'Publishing to: /iant/{robot_id}/action')

    def observation_callback(self, msg):
        # Convert observation to tensor
        obs = np.array(msg.data, dtype=np.float32)

        if len(obs) != 15:
            self.get_logger().warning(f'Invalid observation size: {len(obs)} (expected 15)')
            return

        obs_tensor = torch.FloatTensor(obs).unsqueeze(0)

        # Get action from model
        with torch.no_grad():
            action = self.model.get_action(obs_tensor, deterministic=self.deterministic)

        # Publish action
        action_msg = Int32()
        action_msg.data = int(action)
        self.action_pub.publish(action_msg)

        # Log for debugging
        self.get_logger().debug(f'Obs: {obs[:6]}... -> Action: {action}')


def main(args=None):
    rclpy.init(args=args)

    # Model path - update this to your actual model location
    model_path = '/root/ppo_argos/ppo_final.pt'

    # Check if model exists in container
    if not os.path.exists(model_path):
        # Try alternative path
        model_path = '/root/ppo_argos/model/ppo_final.pt'
        if not os.path.exists(model_path):
            print(f"ERROR: Model not found at {model_path}")
            print("Please mount your model file into the container")
            sys.exit(1)

    # Create controller for primary robot
    controller = PPORos2Controller(
        model_path=model_path,
        robot_id="fb_0",  # First footbot
        deterministic=True  # Use deterministic actions for testing
    )

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()