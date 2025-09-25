#!/usr/bin/env python3
"""
ARGoS-ROS2 Gym Environment for PPO Training
Interfaces between stable-baselines3 and ARGoS simulator via ROS2
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import threading
import time
from typing import Dict, Tuple, Optional

# Import custom messages (we'll define these later)
# from argos3_ros2_bridge.msg import RobotObservation, RobotAction


class ArgosROS2Env(gym.Env):
    """
    Gym environment that communicates with ARGoS via ROS2.
    Supports iAnt swarm foraging task with 6 robots.
    """
    
    def __init__(self, env_id: int = 0, num_robots: int = 6):
        super().__init__()
        
        self.env_id = env_id
        self.num_robots = num_robots
        
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.init()
        
        self.node = rclpy.create_node(f'argos_env_{env_id}')
        
        # QoS profile for real-time communication
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscribers for robot observations
        self.observations = {}
        self.obs_subscribers = []
        for i in range(num_robots):
            topic = f'/env_{env_id}/robot_{i}/observation'
            # For now, using generic Float32MultiArray
            from std_msgs.msg import Float32MultiArray
            sub = self.node.create_subscription(
                Float32MultiArray,
                topic,
                lambda msg, robot_id=i: self._obs_callback(msg, robot_id),
                qos
            )
            self.obs_subscribers.append(sub)
        
        # Create publishers for robot actions
        self.action_publishers = []
        for i in range(num_robots):
            topic = f'/env_{env_id}/robot_{i}/action'
            pub = self.node.create_publisher(Float32MultiArray, topic, qos)
            self.action_publishers.append(pub)
        
        # Create service client for environment control
        from std_srvs.srv import Empty
        self.reset_client = self.node.create_client(
            Empty, f'/env_{env_id}/reset'
        )
        
        # Define action and observation spaces
        # Actions: [left_wheel, right_wheel, lay_pheromone] per robot
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, 
            shape=(num_robots * 3,), 
            dtype=np.float32
        )
        
        # Observations: sensor readings per robot
        # [proximity_sensors(8), light_sensors(8), ground_sensors(4), 
        #  has_food(1), nest_direction(2), food_direction(2)] = 25 per robot
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf,
            shape=(num_robots * 25,),
            dtype=np.float32
        )
        
        # Start ROS2 spinner in separate thread
        self.spinner_thread = threading.Thread(target=self._spin_node)
        self.spinner_thread.daemon = True
        self.spinner_thread.start()
        
        # Episode tracking
        self.current_step = 0
        self.max_steps = 2000  # 2000 steps per episode
        self.food_collected = 0
        self.total_food = 64
        
    def _spin_node(self):
        """Spin ROS2 node in background thread."""
        rclpy.spin(self.node)
    
    def _obs_callback(self, msg, robot_id: int):
        """Store incoming observations from robots."""
        self.observations[robot_id] = np.array(msg.data)
    
    def _get_combined_observation(self) -> np.ndarray:
        """Combine observations from all robots into single vector."""
        combined = []
        for i in range(self.num_robots):
            if i in self.observations:
                combined.extend(self.observations[i])
            else:
                # Return zeros if observation not yet received
                combined.extend(np.zeros(25))
        return np.array(combined, dtype=np.float32)
    
    def _publish_actions(self, actions: np.ndarray):
        """Publish actions to all robots."""
        from std_msgs.msg import Float32MultiArray
        
        # Reshape actions: [robot_0_left, robot_0_right, robot_0_pheromone, ...]
        actions_reshaped = actions.reshape(self.num_robots, 3)
        
        for i in range(self.num_robots):
            msg = Float32MultiArray()
            msg.data = actions_reshaped[i].tolist()
            self.action_publishers[i].publish(msg)
    
    def _wait_for_observations(self, timeout: float = 1.0) -> bool:
        """Wait for observations from all robots."""
        start_time = time.time()
        while len(self.observations) < self.num_robots:
            if time.time() - start_time > timeout:
                return False
            time.sleep(0.01)
        return True
    
    def reset(self, seed: Optional[int] = None) -> Tuple[np.ndarray, Dict]:
        """Reset the environment."""
        super().reset(seed=seed)
        
        # Clear observations
        self.observations.clear()
        self.current_step = 0
        self.food_collected = 0
        
        # Call reset service
        if self.reset_client.wait_for_service(timeout_sec=1.0):
            from std_srvs.srv import Empty
            request = Empty.Request()
            future = self.reset_client.call_async(request)
            
            # Wait for reset to complete
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        # Wait for initial observations
        self._wait_for_observations()
        
        obs = self._get_combined_observation()
        info = {"food_collected": 0}
        
        return obs, info
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Execute one step in the environment."""
        
        # Publish actions
        self._publish_actions(action)
        
        # Wait for next observations
        self.observations.clear()
        time.sleep(0.05)  # Allow time for simulation step
        self._wait_for_observations()
        
        # Get new observation
        obs = self._get_combined_observation()
        
        # Calculate reward (simplified for now)
        # In real implementation, this would come from ARGoS loop functions
        reward = self._calculate_reward(obs)
        
        # Check termination
        self.current_step += 1
        terminated = self.food_collected >= self.total_food
        truncated = self.current_step >= self.max_steps
        
        info = {
            "food_collected": self.food_collected,
            "steps": self.current_step
        }
        
        return obs, reward, terminated, truncated, info
    
    def _calculate_reward(self, obs: np.ndarray) -> float:
        """
        Calculate reward based on observations.
        This is a simplified version - real implementation would
        get reward from ARGoS loop functions.
        """
        # Base reward for surviving
        reward = 0.01
        
        # Reward for food collection (would come from ARGoS)
        # This is placeholder logic
        for i in range(self.num_robots):
            robot_obs = obs[i*25:(i+1)*25]
            has_food = robot_obs[20]  # Assuming index 20 is has_food flag
            
            if has_food > 0.5:
                reward += 1.0
        
        return reward
    
    def close(self):
        """Clean up ROS2 resources."""
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class VectorizedArgosEnv:
    """
    Vectorized environment for parallel training.
    Creates multiple ARGoS instances for faster learning.
    """
    
    def __init__(self, num_envs: int = 4):
        from stable_baselines3.common.vec_env import SubprocVecEnv
        
        def make_env(env_id):
            def _init():
                return ArgosROS2Env(env_id)
            return _init
        
        self.envs = SubprocVecEnv([make_env(i) for i in range(num_envs)])
    
    def get_envs(self):
        return self.envs