#!/bin/bash

# Quick start script to run the ROS2-ARGoS-PPO visualization
# This script opens the necessary terminals with the right commands

echo "=========================================="
echo "ROS2-ARGoS-PPO Visualization Launcher"
echo "=========================================="
echo ""
echo "This will open 3 terminal windows:"
echo "1. ARGoS Simulation (with visualization)"
echo "2. PPO Controller (Python)"
echo "3. ROS2 Monitor (optional)"
echo ""
read -p "Press Enter to continue..."

# Terminal 1: ARGoS Simulation
gnome-terminal --title="ARGoS Simulation" -- bash -c "
echo 'Starting ARGoS Simulation...';
cd ~/ros2_ws;
source /opt/ros/humble/setup.bash;
source install/setup.bash;
export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/home/andres2020/ros2_ws/install/ppo_argos_ros2/lib;
echo 'Press Enter to start ARGoS...';
read;
argos3 -c test_simple.argos;
exec bash"

# Wait for ARGoS to start
sleep 3

# Terminal 2: PPO Controller
gnome-terminal --title="PPO Controller" -- bash -c "
echo 'Starting PPO Controller...';
cd ~/ros2_ws;
source /opt/ros/humble/setup.bash;
source install/setup.bash;
echo 'Waiting for ARGoS to initialize...';
sleep 2;
echo 'Starting PPO controller...';
python3 ppo_ros2_node.py;
exec bash"

# Wait a moment
sleep 2

# Terminal 3: ROS2 Monitor (optional)
gnome-terminal --title="ROS2 Monitor" -- bash -c "
echo 'ROS2 Topic Monitor';
source /opt/ros/humble/setup.bash;
echo '';
echo 'Available commands:';
echo '  ros2 topic list                    - List all topics';
echo '  ros2 topic echo /iant/fb_0/action  - Watch robot actions';
echo '  ros2 topic hz /iant/fb_0/observation - Check observation rate';
echo '';
ros2 topic list;
exec bash"

echo ""
echo "=========================================="
echo "All terminals launched!"
echo "=========================================="
echo ""
echo "You should see:"
echo "1. ARGoS Qt window with a robot"
echo "2. PPO controller connecting to ROS2"
echo "3. Robot starting to move based on PPO decisions"
echo ""
echo "To stop: Press Ctrl+C in each terminal or close windows"