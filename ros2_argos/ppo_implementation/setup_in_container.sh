#!/bin/bash

echo "=== Setting up PPO-ROS2-ARGoS Package ==="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Source ROS2
echo -e "${YELLOW}Sourcing ROS2...${NC}"
source /opt/ros/humble/setup.bash

# Create package structure
echo -e "${YELLOW}Creating ROS2 package structure...${NC}"
cd /root/ros2_ws/src

# Remove old package if exists
rm -rf ppo_argos_ros2

# Create new package
ros2 pkg create --build-type ament_cmake ppo_argos_ros2

# Create directories
mkdir -p ppo_argos_ros2/src
mkdir -p ppo_argos_ros2/include/ppo_argos_ros2
mkdir -p ppo_argos_ros2/scripts

# Copy files
echo -e "${YELLOW}Copying source files...${NC}"
cp /root/ppo_argos/ros2_iant_controller.cpp ppo_argos_ros2/src/ 2>/dev/null || echo "Controller cpp not found"
cp /root/ppo_argos/ros2_iant_controller.h ppo_argos_ros2/include/ppo_argos_ros2/ 2>/dev/null || echo "Controller header not found"
cp /root/ppo_argos/ppo_ros2_node.py ppo_argos_ros2/scripts/ 2>/dev/null || echo "Python node not found"
cp /root/ppo_argos/CMakeLists.txt ppo_argos_ros2/
cp /root/ppo_argos/package.xml ppo_argos_ros2/

# Make Python script executable
chmod +x ppo_argos_ros2/scripts/ppo_ros2_node.py 2>/dev/null || true

# Update the controller include path
echo -e "${YELLOW}Fixing include paths...${NC}"
sed -i 's|#include "ros2_iant_controller.h"|#include "ppo_argos_ros2/ros2_iant_controller.h"|g' ppo_argos_ros2/src/ros2_iant_controller.cpp 2>/dev/null || true

# Build
echo -e "${YELLOW}Building package...${NC}"
cd /root/ros2_ws
colcon build --packages-select ppo_argos_ros2

if [ $? -eq 0 ]; then
    echo -e "${GREEN}Build successful!${NC}"
    echo ""
    echo -e "${GREEN}Package built successfully! Now you can:${NC}"
    echo "1. Source the workspace: source /root/ros2_ws/install/setup.bash"
    echo "2. Run ARGoS (in terminal 1): argos3 -c /root/ppo_argos/iant_ros2_test.argos"
    echo "3. Run PPO node (in terminal 2): python3 /root/ppo_argos/ppo_ros2_node.py"
else
    echo -e "${RED}Build failed! Check the error messages above.${NC}"
    echo ""
    echo "Common issues:"
    echo "- Missing ARGoS headers: Make sure ARGoS is installed"
    echo "- Missing ROS2 packages: Make sure ROS2 is sourced"
fi