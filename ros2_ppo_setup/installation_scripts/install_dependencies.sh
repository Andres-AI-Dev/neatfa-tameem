#!/bin/bash

# Script to install ROS2 Humble and dependencies for the ROS2-ARGoS-PPO pipeline
# Run with: sudo bash install_dependencies.sh

set -e  # Exit on error

echo "==============================================="
echo "Installing ROS2 Humble and dependencies"
echo "==============================================="

# Update system
echo "Updating system packages..."
apt update
apt install -y software-properties-common
add-apt-repository universe -y

# Add ROS2 repository
echo "Adding ROS2 repository..."
apt update && apt install -y curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
echo "Installing ROS2 Humble..."
apt update
apt install -y ros-humble-desktop python3-colcon-common-extensions

# Install additional ROS2 packages we need
echo "Installing additional ROS2 packages..."
apt install -y ros-humble-ros-base ros-humble-ament-cmake python3-rosdep

# Initialize rosdep
echo "Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi

# Install build tools
echo "Installing build tools..."
apt install -y build-essential cmake git pkg-config

# Install Qt5 and OpenGL dependencies for ARGoS visualization
echo "Installing Qt5 and OpenGL dependencies..."
apt install -y \
    qt5-default \
    libqt5opengl5-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    libxi-dev \
    libxmu-dev

# Install Python dependencies
echo "Installing Python dependencies..."
apt install -y python3-pip python3-setuptools

# Install PyTorch and other Python packages as regular user
echo "==============================================="
echo "Installing Python packages (as user)..."
echo "Please run these commands after the script:"
echo "pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu"
echo "pip3 install setuptools==58.2.0"
echo "==============================================="

# Create ROS2 workspace directory
echo "Creating ROS2 workspace directory..."
mkdir -p ~/ros2_ws/src

echo "==============================================="
echo "Installation complete!"
echo "==============================================="
echo ""
echo "Next steps:"
echo "1. Run as regular user (not sudo):"
echo "   pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu"
echo "   pip3 install setuptools==58.2.0"
echo ""
echo "2. Add to ~/.bashrc:"
echo "   echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc"
echo "   source ~/.bashrc"
echo ""
echo "3. Update rosdep as regular user:"
echo "   rosdep update"
echo "==============================================="