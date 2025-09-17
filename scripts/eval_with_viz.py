#!/usr/bin/env python3
"""
Simple evaluation script with visualization enabled.
Runs the PPO model with ARGoS visualization window.
"""
import os
import sys
import time
import torch
import numpy as np

# Paths
ARGOS_DIR = os.path.join(os.path.dirname(__file__), "..", "argos")
ARGOS_DIR = os.path.abspath(ARGOS_DIR)
XML = os.path.join(ARGOS_DIR, "experiments", "iAnt_rl.xml")

# Import env
sys.path.append(os.path.join(ARGOS_DIR, "build"))
import iant_rl

# Import model architecture from training script
from train_ppo import ActorCritic, NUM_ACTIONS, OBS_DIM, ACTIONS

def load_model(ckpt_path, device):
    model = ActorCritic(OBS_DIM, NUM_ACTIONS).to(device)
    state = torch.load(ckpt_path, map_location=device)
    model.load_state_dict(state["model_state_dict"])
    model.eval()
    return model

def select_action(model, obs, device):
    with torch.no_grad():
        obs_t = torch.from_numpy(obs).float().to(device).unsqueeze(0)
        action_probs, _ = model(obs_t)
        action = int(torch.argmax(action_probs, dim=1).item())
        return action

def main():
    device = torch.device("cpu")  # Use CPU for simplicity
    print("=" * 60)
    print("PPO Model Evaluation with ARGoS Visualization")
    print("=" * 60)
    
    # Find a PPO model to use
    runs_dir = os.path.join(os.path.dirname(__file__), "runs")
    
    # Try to use ppo_final.pt if it exists (best trained model)
    ckpt = os.path.join(runs_dir, "ppo_final.pt")
    if not os.path.exists(ckpt):
        # Otherwise try ppo_episode_100.pt
        ckpt = os.path.join(runs_dir, "ppo_episode_100.pt")
        if not os.path.exists(ckpt):
            # Otherwise find any PPO model
            print("Looking for PPO models...")
            for f in sorted(os.listdir(runs_dir)):
                if f.startswith('ppo_') and f.endswith('.pt'):
                    ckpt = os.path.join(runs_dir, f)
                    break
    
    print(f"\n✓ Using model: {os.path.basename(ckpt)}")
    model = load_model(ckpt, device)
    
    # Create environment WITH VISUALIZATION
    print("\n✓ Creating environment with visualization ENABLED")
    print("  A window should open showing the ARGoS simulator...")
    env = iant_rl.IAntRLEnv(XML, force_no_viz=False)  # False = show visualization!
    time.sleep(2)  # Give the window time to initialize
    
    # Change to ARGoS directory (required for resources)
    original_dir = os.getcwd()
    os.chdir(ARGOS_DIR)
    
    try:
        print("\n✓ Starting simulation...")
        print("  Watch the robots forage for food!")
        print("  Starting in 3 seconds...")
        time.sleep(3)
        
        # Run one episode
        obs = env.reset()
        total_reward = 0
        step_count = 0
        
        print("Episode running... (press Ctrl+C to stop)")
        while True:
            # Select action using trained model
            action_idx = select_action(model, np.array(obs), device)
            left, right, pheromone = ACTIONS[action_idx]
            
            # Step environment
            obs, reward, terminated, truncated, info = env.step(left, right, pheromone)
            total_reward += reward
            step_count += 1
            
            # Print progress every 100 steps
            if step_count % 100 == 0:
                print(f"  Step {step_count}: Food left = {info['food_left']}, "
                      f"Fitness = {info['fitness']:.2f}")
            
            if terminated or truncated:
                break
        
        print("\n" + "=" * 60)
        print(f"Episode Complete!")
        print(f"  Total steps: {step_count}")
        print(f"  Total reward: {total_reward:.2f}")
        print(f"  Final fitness: {info['fitness']:.2f}")
        print(f"  Food collected: {64 - info['food_left']}/64")
        print("=" * 60)
        
    finally:
        env.close()
        os.chdir(original_dir)
        print("\nVisualization closed.")

if __name__ == "__main__":
    main()