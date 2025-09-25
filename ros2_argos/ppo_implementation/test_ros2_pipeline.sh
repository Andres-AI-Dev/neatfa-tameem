#!/bin/bash

echo "=== Testing ROS2-ARGoS-PPO Pipeline ==="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Step 1: Source ROS2 environment
echo -e "${YELLOW}Step 1: Setting up ROS2 environment${NC}"
source /opt/ros/humble/setup.bash

# Step 2: Build the PPO ROS2 package
echo -e "${YELLOW}Step 2: Building PPO ROS2 package${NC}"
cd /root/ros2_ws
cp -r /root/ppo_argos/* /root/ros2_ws/src/ppo_argos_ros2/
colcon build --packages-select ppo_argos_ros2
if [ $? -ne 0 ]; then
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi
echo -e "${GREEN}Build successful!${NC}"

# Step 3: Source the workspace
source /root/ros2_ws/install/setup.bash

# Step 4: Check if model exists
echo -e "${YELLOW}Step 3: Checking for PPO model${NC}"
if [ ! -f "/root/ppo_argos/ppo_final.pt" ]; then
    echo -e "${RED}Model file not found at /root/ppo_argos/ppo_final.pt${NC}"
    echo "Please mount your model file into the container"
    exit 1
fi
echo -e "${GREEN}Model found!${NC}"

# Step 5: Start ARGoS in background
echo -e "${YELLOW}Step 4: Starting ARGoS simulator${NC}"
argos3 -c /root/ppo_argos/iant_ros2_test.argos &
ARGOS_PID=$!
sleep 3

# Step 6: Start PPO ROS2 node
echo -e "${YELLOW}Step 5: Starting PPO ROS2 controller${NC}"
python3 /root/ros2_ws/install/ppo_argos_ros2/lib/ppo_argos_ros2/ppo_ros2_node.py &
PPO_PID=$!

# Step 7: Monitor for a bit
echo -e "${YELLOW}Step 6: Running test for 30 seconds...${NC}"
echo "You should see the robot moving based on PPO decisions"
sleep 30

# Step 8: Cleanup
echo -e "${YELLOW}Cleaning up...${NC}"
kill $PPO_PID 2>/dev/null
kill $ARGOS_PID 2>/dev/null

echo -e "${GREEN}Test complete!${NC}"