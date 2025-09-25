#!/bin/bash

# Script to install remaining dependencies for ROS2-ARGoS-PPO pipeline
# Run with: sudo bash install_remaining_deps.sh

set -e  # Exit on error

echo "==============================================="
echo "Installing remaining dependencies"
echo "==============================================="

# Install Qt5 and OpenGL dependencies (without qt5-default which is deprecated in 22.04)
echo "Installing Qt5 and OpenGL dependencies..."
apt install -y \
    qtbase5-dev \
    libqt5opengl5-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    libxi-dev \
    libxmu-dev

# Install Python dependencies
echo "Installing Python pip..."
apt install -y python3-pip python3-setuptools

echo "==============================================="
echo "System packages installation complete!"
echo "==============================================="
echo ""
echo "Now please run AS REGULAR USER (not sudo):"
echo ""
echo "# 1. Install Python packages:"
echo "pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu"
echo "pip3 install setuptools==58.2.0"
echo ""
echo "# 2. Add ROS2 to bashrc:"
echo "echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc"
echo "source ~/.bashrc"
echo ""
echo "# 3. Update rosdep:"
echo "rosdep update"
echo "==============================================="