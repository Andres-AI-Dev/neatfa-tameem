# ROS2-ARGoS-PPO Pipeline for Swarm Robotics (Linux Workstation Version)

## Overview
This implementation solves the Qt/Python incompatibility issue in ARGoS by using ROS2 as an inter-process communication bridge. This allows ARGoS (C++ with Qt visualization) to run separately from Python-based PPO reinforcement learning controllers.

**IMPORTANT**: This must be run on Linux workstations in the MARS lab. It will NOT work on macOS due to OpenGL limitations in XQuartz.

## System Requirements
- Ubuntu 22.04 LTS (tested on MARS lab workstations)
- X11 display (for Qt visualization)
- At least 8GB RAM
- GPU not required (CPU-only PyTorch is used)

## Quick Start

### Fastest Method - One Command
```bash
cd ~/Dev/FL-RL-Research/neatfa-tameem/ros2_argos
./QUICK_START.sh
```
This opens all necessary terminals and starts the visualization automatically.

### Manual Method - Full Control
See `RUN_VISUALIZATION.md` for detailed step-by-step instructions.

## Installation for New Users

### 1. Clone the Repository
```bash
git clone https://github.com/DoggyDog2020/neatfa-tameem.git
cd neatfa-tameem
git checkout RL-FL-ROS-Andres
cd ros2_argos
```

### 2. Install System Dependencies
```bash
# Install ROS2 and dependencies
sudo bash install_dependencies.sh

# Install remaining Qt/OpenGL dependencies
sudo bash install_remaining_deps.sh

# Install Python packages (as regular user, not sudo)
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
pip3 install setuptools==58.2.0

# Setup ROS2 in bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc

# Update rosdep
rosdep update
```

### 3. Setup ARGoS pkg-config
```bash
sudo bash create_argos_pkgconfig.sh
```

### 4. Build ROS2 Workspace
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cp -r ppo_implementation ~/ros2_ws/src/argos_ros2_controller

# Copy required files
cp ppo_implementation/test_simple.argos ~/ros2_ws/
cp ppo_implementation/floor.png ~/ros2_ws/
cp ppo_implementation/ppo_final.pt ~/ros2_ws/
cp ppo_implementation/ppo_ros2_node.py ~/ros2_ws/

# Update paths in config files
sed -i 's|/root/|/home/'$USER'/|g' ~/ros2_ws/test_simple.argos
sed -i 's|/root/|/home/'$USER'/|g' ~/ros2_ws/ppo_ros2_node.py

# Build
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ppo_argos_ros2
source install/setup.bash
```

## Running the Visualization

### Terminal 1 - ARGoS Simulation
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ros2_ws/install/ppo_argos_ros2/lib
argos3 -c test_simple.argos
```

### Terminal 2 - PPO Controller
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 ppo_ros2_node.py
```

### Terminal 3 - Monitor (Optional)
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /iant/fb_0/action
```

## What You Should See
1. ARGoS Qt window opens showing a 5x5m arena with walls
2. One footbot robot (fb_0) in the center
3. Robot starts moving based on PPO decisions (trained for 200 episodes)
4. Console shows ROS2 communication between ARGoS and Python

## Architecture

```
ARGoS Simulation (C++/Qt)
    ↓ publishes sensor data
/iant/fb_0/observation [ROS2 Topic]
    ↓
PPO Controller (Python)
    ↓ runs neural network inference
/iant/fb_0/action [ROS2 Topic]
    ↓
Robot executes action in ARGoS
```

## Key Files

| File | Description |
|------|-------------|
| `ppo_implementation/ros2_iant_controller.cpp` | C++ ROS2 controller for ARGoS |
| `ppo_implementation/ppo_ros2_node.py` | Python PPO controller node |
| `ppo_implementation/ppo_final.pt` | Trained PPO model (200 episodes, 36k params) |
| `ppo_implementation/test_simple.argos` | ARGoS configuration |
| `QUICK_START.sh` | One-click launcher script |
| `RUN_VISUALIZATION.md` | Detailed running instructions |

## Troubleshooting

### X11 Display Issues
```bash
echo $DISPLAY  # Should show :1 or similar
xhost +local:  # Allow local connections
```

### Library Not Found
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ros2_ws/install/ppo_argos_ros2/lib
```

### Model File Not Found
```bash
cp ~/Dev/FL-RL-Research/neatfa-tameem/ros2_argos/ppo_implementation/ppo_final.pt ~/ros2_ws/
```

### Rebuild After Code Changes
```bash
cd ~/ros2_ws
colcon build --packages-select ppo_argos_ros2
source install/setup.bash
```

## For Team Members

To run this on your MARS lab workstation:
1. Fork this repository
2. Follow the installation steps above
3. Use `./QUICK_START.sh` for easy visualization

## Technical Details

### Why ROS2?
- Solves Qt/pybind11 incompatibility by process isolation
- Industry standard for robotics
- Enables multi-robot scaling
- Real-time communication at 10Hz

### PPO Model Details
- Architecture: 3-layer neural network (128-128-64 units)
- Parameters: 36,000
- Training: 200 episodes
- Action space: 15 discrete actions (movement + pheromone)
- Observation space: 15D (food, proximity, light sensors)

## Citation
If you use this work, please cite:
```
ROS2-ARGoS-PPO Pipeline
MARS Lab, University
September 2025
```

## Support
For issues specific to MARS lab workstations, contact the team via the repository issues page.

---
Last tested: September 25, 2025
Platform: Ubuntu 22.04 LTS (MARS Lab Workstations)