# ROS2-ARGoS-PPO Visualization Instructions

## Quick Start (3 Terminal Setup)

### Terminal 1: Start ARGoS Simulation with Visualization
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/andres2020/ros2_ws/install/ppo_argos_ros2/lib
argos3 -c test_simple.argos
```

**What you should see:**
- A Qt window opens showing a 3D arena
- One robot (footbot) in the center
- Walls around the arena
- The robot will initially be stationary (waiting for PPO controller)

### Terminal 2: Run PPO Controller
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 ppo_ros2_node.py
```

**What you should see:**
```
[INFO] Loading PPO model from: /home/andres2020/ros2_ws/ppo_final.pt
[INFO] Model loaded successfully from episode 200
[INFO] PPO ROS2 Controller initialized for robot: fb_0
[INFO] Subscribed to: /iant/fb_0/observation
[INFO] Publishing to: /iant/fb_0/action
```

**After both are running:** The robot in the ARGoS window should start moving!

### Terminal 3 (Optional): Monitor ROS2 Communication
```bash
source /opt/ros/humble/setup.bash

# List all topics
ros2 topic list

# Watch actions being sent to robot
ros2 topic echo /iant/fb_0/action

# Check publishing frequency
ros2 topic hz /iant/fb_0/observation
```

## If Things Don't Work

### 1. If you get "cannot open shared object file"
The library path might not be set correctly. Run:
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/andres2020/ros2_ws/install/ppo_argos_ros2/lib
```

### 2. If you get "Model not found"
Make sure the PPO model file exists:
```bash
ls -la ~/ros2_ws/ppo_final.pt
```
If not, copy it from the original location:
```bash
cp ~/Dev/FL-RL-Research/neatfa-tameem/ros2_argos/ppo_implementation/ppo_final.pt ~/ros2_ws/
```

### 3. If ROS2 topics don't appear
Make sure you source ROS2 in each terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 4. If you need to rebuild (after code changes)
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ppo_argos_ros2
source install/setup.bash
```

### 5. If X11 display issues
```bash
echo $DISPLAY  # Should show :1 or similar
xhost +local:  # Allow local connections
```

## Understanding What You're Seeing

1. **Robot Movement**: The robot makes decisions every 0.1 seconds based on the PPO model
2. **Sensor Rays**: Blue lines show proximity sensors (if enabled in config)
3. **Action Bias**: The trained model (200 episodes) has a bias toward action 11 (forward-right)
4. **Arena**: 5m x 5m space with walls to contain the robot

## Key Files Location

- **ARGoS Config**: `~/ros2_ws/test_simple.argos`
- **PPO Model**: `~/ros2_ws/ppo_final.pt` (trained for 200 episodes)
- **PPO Controller**: `~/ros2_ws/ppo_ros2_node.py`
- **C++ ROS2 Controller**: `~/ros2_ws/src/argos_ros2_controller/src/ros2_iant_controller.cpp`

## Architecture Overview

```
ARGoS Simulation (C++/Qt)
    ↓ (publishes sensor data)
/iant/fb_0/observation [ROS2 Topic]
    ↓
PPO Controller (Python)
    ↓ (runs neural network inference)
/iant/fb_0/action [ROS2 Topic]
    ↓
Robot executes action in ARGoS
```

## To Stop Everything

1. Press `Ctrl+C` in Terminal 2 (PPO Controller)
2. Press `Ctrl+C` in Terminal 1 (ARGoS) or close the Qt window
3. Optional cleanup: `killall argos3`

## Success Indicators

✅ ARGoS Qt window opens and shows robot
✅ PPO controller reports "Model loaded successfully"
✅ Robot starts moving in the visualization
✅ `ros2 topic list` shows both `/iant/fb_0/observation` and `/iant/fb_0/action`

---

Last tested: September 25, 2025
Repository: https://github.com/DoggyDog2020/neatfa-tameem.git (branch: RL-FL-ROS-Andres)