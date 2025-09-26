# ROS2-ARGoS-PPO Setup (Simplified)

## Quick Start (If Already Installed)
```bash
cd ~/ros2_ws
./QUICK_START.sh
```
This opens 3 terminals:
1. ARGoS with Qt/OpenGL visualization
2. PPO controller (trained model)
3. ROS2 monitor

## Architecture
**ARGoS (C++/Qt) ↔️ ROS2 ↔️ Python PPO Controller**

- Solves Qt/Python incompatibility via ROS2 bridge
- Real-time control at ~10Hz
- Full 3D visualization

## Installation (If Starting Fresh)

### 1. Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Humble already installed at `/opt/ros/humble`

### 2. Install Dependencies
```bash
cd ros2_ppo_setup/installation_scripts
sudo bash install_dependencies.sh        # ROS2 packages
sudo bash install_remaining_deps.sh      # Qt5/OpenGL
bash create_argos_pkgconfig.sh          # ARGoS pkg-config
```

### 3. Install Python Packages
```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
pip3 install setuptools==58.2.0
```

### 4. Build ARGoS Components
```bash
cd argos
mkdir -p build && cd build
export PKG_CONFIG_PATH=/home/andres2020/Dev/RL-FL-Research/neatfa-tameem:$PKG_CONFIG_PATH
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

### 5. Setup ROS2 Workspace
```bash
# Copy package to workspace
mkdir -p ~/ros2_ws/src
cp -r ros2_ppo_setup/ros2_package ~/ros2_ws/src/argos_ros2_controller

# Copy required files
cd ~/ros2_ws
cp src/argos_ros2_controller/test_simple.argos .
cp src/argos_ros2_controller/floor.png .
cp src/argos_ros2_controller/ppo_final.pt .
cp src/argos_ros2_controller/ppo_ros2_node.py .

# Build ROS2 package
source /opt/ros/humble/setup.bash
export PKG_CONFIG_PATH=/home/andres2020/Dev/RL-FL-Research/neatfa-tameem:$PKG_CONFIG_PATH
colcon build --packages-select ppo_argos_ros2
source install/setup.bash
```

## File Structure
```
~/ros2_ws/                      # Working directory
├── QUICK_START.sh              # One-click launcher
├── ppo_final.pt                # Trained PPO model (36k params)
├── test_simple.argos           # Simulation config
├── floor.png                   # Arena texture
└── src/argos_ros2_controller/  # ROS2 package

neatfa-tameem/
├── argos/                      # ARGoS RL environment
│   └── build/                  # Built C++ components
├── ros2_ppo_setup/             # Clean organized files
│   ├── installation_scripts/   # Setup scripts
│   └── ros2_package/           # ROS2 source code
└── scripts/                    # Training scripts
```

## Manual Run (3 Terminals)

**Terminal 1 - ARGoS:**
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ros2_ws/install/ppo_argos_ros2/lib
argos3 -c test_simple.argos
```

**Terminal 2 - PPO Controller:**
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 ppo_ros2_node.py
```

**Terminal 3 - Monitor:**
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /iant/fb_0/action
```

## Multi-Robot Support
Ready for federated learning with topics:
- `/iant/fb_0/` - Robot 0
- `/iant/fb_1/` - Robot 1
- `/iant/fb_2/` - Robot 2
- `/iant/fb_3/` - Robot 3

## Troubleshooting

### Model not found error:
Model should be at: `/home/andres2020/ros2_ws/ppo_final.pt`

### Rebuild after changes:
```bash
cd ~/ros2_ws
colcon build --packages-select ppo_argos_ros2
source install/setup.bash
```

## Current Status
✅ Visualization working with Qt/OpenGL
✅ PPO control via ROS2
✅ 200-episode trained model
✅ Single robot working

## Next Steps
- Train better model (current is only 200 episodes)
- Add multiple robots for swarm behavior
- Implement federated learning across robots