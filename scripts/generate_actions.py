#!/usr/bin/env python3
"""
Generates actions from the trained PPO model and saves them to a file
for the C++ controller to read.
"""
import os
import sys
import json
import torch
import numpy as np
import time
from pathlib import Path

# Paths
ARGOS_DIR = os.path.join(os.path.dirname(__file__), "..", "argos")
ARGOS_DIR = os.path.abspath(ARGOS_DIR)
XML = os.path.join(ARGOS_DIR, "experiments", "iAnt_rl.xml")
ACTION_FILE = os.path.join(ARGOS_DIR, "robot_actions.json")
ACTION_FILE_TMP = ACTION_FILE + ".tmp"

# Import env
sys.path.append(os.path.join(ARGOS_DIR, "build"))
import iant_rl

# Import model architecture
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

def write_action_file(actions):
    """Write actions atomically to avoid partial reads."""
    with open(ACTION_FILE_TMP, 'w') as f:
        json.dump(actions, f)
    os.rename(ACTION_FILE_TMP, ACTION_FILE)

def main():
    device = torch.device("cpu")
    print("=" * 60)
    print("Generating Actions from PPO Model")
    print("=" * 60)
    
    # Find model
    runs_dir = os.path.join(os.path.dirname(__file__), "runs")
    ckpt = os.path.join(runs_dir, "ppo_final.pt")
    if not os.path.exists(ckpt):
        ckpt = os.path.join(runs_dir, "ppo_episode_100.pt")
        if not os.path.exists(ckpt):
            for f in sorted(os.listdir(runs_dir)):
                if f.startswith('ppo_') and f.endswith('.pt'):
                    ckpt = os.path.join(runs_dir, f)
                    break
    
    print(f"\n✓ Loading model: {os.path.basename(ckpt)}")
    model = load_model(ckpt, device)
    
    # Create headless environment
    print("\n✓ Creating headless environment")
    env = iant_rl.IAntRLEnv(XML, force_no_viz=True)
    
    # Change to ARGoS directory
    original_dir = os.getcwd()
    os.chdir(ARGOS_DIR)
    
    # Generate all actions for the episode
    all_actions = []
    
    try:
        print("\n✓ Generating actions for full episode...")
        obs = env.reset()
        step_count = 0
        
        while True:
            # Get action from model
            action_idx = select_action(model, np.array(obs), device)
            left, right, pheromone = ACTIONS[action_idx]
            
            # Store action for all 6 robots
            # First robot uses RL model, others can follow or use default
            robot_actions = []
            for i in range(6):
                if i == 0:  # First robot uses RL
                    robot_actions.append({
                        "left": float(left),
                        "right": float(right),
                        "pheromone": bool(pheromone)
                    })
                else:  # Other robots use same action or default
                    robot_actions.append({
                        "left": float(left),
                        "right": float(right),
                        "pheromone": False
                    })
            
            all_actions.append({
                "step": step_count,
                "robots": robot_actions
            })
            
            # Step environment
            obs, reward, terminated, truncated, info = env.step(left, right, pheromone)
            step_count += 1
            
            if step_count % 100 == 0:
                print(f"  Generated {step_count} steps: Food = {info['food_left']}, Fitness = {info['fitness']:.2f}")
            
            if terminated or truncated or step_count >= 2000:
                break
        
        # Write all actions to file
        print(f"\n✓ Writing {len(all_actions)} action steps to {ACTION_FILE}")
        write_action_file(all_actions)
        
        print(f"\n✓ Final statistics:")
        print(f"  Total steps: {step_count}")
        print(f"  Final fitness: {info['fitness']:.2f}")
        print(f"  Food collected: {64 - info['food_left']}/64")
        
        print("\n" + "=" * 60)
        print("Actions saved! Now run ARGoS with the RL controller:")
        print("  cd argos && argos3 -c experiments/iAnt_rl_replay.xml")
        print("=" * 60)
        
    finally:
        env.close()
        os.chdir(original_dir)

if __name__ == "__main__":
    main()