#!/usr/bin/env python3
"""
Runs ARGoS visualization in a subprocess while controlling it with the PPO model.
This solves the Qt/pybind11 incompatibility by separating the processes.
"""
import os
import sys
import subprocess
import time
import json
import torch
import numpy as np
import threading
import queue
from pathlib import Path

# Paths
ARGOS_DIR = os.path.join(os.path.dirname(__file__), "..", "argos")
ARGOS_DIR = os.path.abspath(ARGOS_DIR)
XML = os.path.join(ARGOS_DIR, "experiments", "iAnt_rl.xml")

# Import env for headless evaluation
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

def run_model_controller(model, device, action_queue, stop_event):
    """Run the RL model in headless mode and generate actions."""
    # Create headless environment
    env = iant_rl.IAntRLEnv(XML, force_no_viz=True)
    
    # Change to ARGoS directory
    original_dir = os.getcwd()
    os.chdir(ARGOS_DIR)
    
    try:
        obs = env.reset()
        step_count = 0
        
        while not stop_event.is_set():
            # Get action from model
            action_idx = select_action(model, np.array(obs), device)
            left, right, pheromone = ACTIONS[action_idx]
            
            # Send action to visualization process
            action_data = {
                "step": step_count,
                "left": float(left),
                "right": float(right),
                "pheromone": bool(pheromone)
            }
            action_queue.put(action_data)
            
            # TODO(human): Write action to file for C++ controller to read
            
            # Step environment
            obs, reward, terminated, truncated, info = env.step(left, right, pheromone)
            step_count += 1
            
            if step_count % 100 == 0:
                print(f"  Model step {step_count}: Food = {info['food_left']}, Fitness = {info['fitness']:.2f}")
            
            if terminated or truncated:
                print(f"\nModel evaluation complete: Fitness = {info['fitness']:.2f}")
                break
            
            # Small delay to sync with visualization
            time.sleep(0.01)
            
    finally:
        env.close()
        os.chdir(original_dir)

def main():
    device = torch.device("cpu")
    print("=" * 60)
    print("ARGoS Visualization with PPO Model Control")
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
    
    # Create communication queue
    action_queue = queue.Queue()
    stop_event = threading.Event()
    
    # Start model controller in thread
    print("\n✓ Starting model controller thread")
    model_thread = threading.Thread(
        target=run_model_controller,
        args=(model, device, action_queue, stop_event)
    )
    model_thread.start()
    
    # Start ARGoS visualization in subprocess
    print("\n✓ Starting ARGoS visualization subprocess")
    print("  The ARGoS window should open now...")
    print("  Note: The robots are controlled by the trained PPO model")
    print("  Press Ctrl+C to stop\n")
    
    # Run ARGoS with visualization
    argos_process = subprocess.Popen(
        ["argos3", "-c", "experiments/iAnt_rl.xml"],
        cwd=ARGOS_DIR,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    
    try:
        # Wait for process to complete or user interrupt
        argos_process.wait()
    except KeyboardInterrupt:
        print("\n\nStopping visualization...")
    finally:
        stop_event.set()
        argos_process.terminate()
        model_thread.join(timeout=5)
        print("\n✓ Visualization stopped")
    
    print("=" * 60)

if __name__ == "__main__":
    main()