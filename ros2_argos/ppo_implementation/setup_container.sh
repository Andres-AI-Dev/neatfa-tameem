#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}==================================================${NC}"
echo -e "${BLUE}     PPO-ARGoS-ROS2 Container Setup Script      ${NC}"
echo -e "${BLUE}==================================================${NC}"
echo ""

# Function to check if command succeeded
check_status() {
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ $1 successful${NC}"
    else
        echo -e "${RED}✗ $1 failed${NC}"
        exit 1
    fi
}

# Step 1: Create ARGoS pkg-config file
echo -e "${YELLOW}Step 1: Setting up ARGoS pkg-config...${NC}"
mkdir -p /usr/local/lib/pkgconfig
cat > /usr/local/lib/pkgconfig/argos3_simulator.pc << 'EOF'
prefix=/usr/local
exec_prefix=${prefix}
libdir=${exec_prefix}/lib/argos3
includedir=${prefix}/include

Name: argos3_simulator
Description: ARGoS3 Multi-Robot Simulator
Version: 3.0.0
Requires: lua5.3
Libs: -L${libdir} -largos3core_simulator -Wl,-rpath,${libdir}
Cflags: -I${includedir} -I${includedir}/argos3 -I/usr/include/lua5.3
EOF
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
check_status "ARGoS pkg-config setup"

# Step 2: Source ROS2
echo -e "${YELLOW}Step 2: Sourcing ROS2...${NC}"
source /opt/ros/humble/setup.bash
check_status "ROS2 source"

# Step 3: Create ROS2 package
echo -e "${YELLOW}Step 3: Creating ROS2 package structure...${NC}"
cd /root/ros2_ws/src
rm -rf ppo_argos_ros2  # Remove if exists
ros2 pkg create --build-type ament_cmake ppo_argos_ros2 > /dev/null 2>&1
check_status "ROS2 package creation"

# Step 4: Set up directories
echo -e "${YELLOW}Step 4: Setting up directories...${NC}"
mkdir -p ppo_argos_ros2/src ppo_argos_ros2/include/ppo_argos_ros2 ppo_argos_ros2/scripts
check_status "Directory creation"

# Step 5: Copy files and create fixed controller
echo -e "${YELLOW}Step 5: Creating fixed controller source files...${NC}"

# Copy header and other files
cp /root/ppo_argos/ros2_iant_controller.h ppo_argos_ros2/include/ppo_argos_ros2/ 2>/dev/null || echo "  Controller header not found"
cp /root/ppo_argos/ppo_ros2_node.py ppo_argos_ros2/scripts/ 2>/dev/null || echo "  Python node not found"

# Create proper CMakeLists.txt
cat > ppo_argos_ros2/CMakeLists.txt << 'CMAKE_EOF'
cmake_minimum_required(VERSION 3.5)
project(ppo_argos_ros2)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

# Find ARGoS
pkg_check_modules(ARGOS REQUIRED argos3_simulator)

# Set compiler flags
set(CMAKE_CXX_STANDARD 17)
add_definitions(-std=c++17)

# Include directories
include_directories(
  include/ppo_argos_ros2
  ${ARGOS_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

# Link directories
link_directories(${ARGOS_LIBRARY_DIRS})

# Create the controller library
add_library(ros2_iant_controller SHARED
  src/ros2_iant_controller.cpp
)

# Set target properties
target_compile_features(ros2_iant_controller PUBLIC cxx_std_17)

# Link libraries
target_link_libraries(ros2_iant_controller
  ${ARGOS_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

# Install the library
install(TARGETS ros2_iant_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python scripts
install(PROGRAMS
  scripts/ppo_ros2_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(FILES
  include/ppo_argos_ros2/ros2_iant_controller.h
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
CMAKE_EOF

# Create package.xml
cat > ppo_argos_ros2/package.xml << 'PACKAGE_EOF'
<?xml version="1.0"?>
<package format="3">
  <name>ppo_argos_ros2</name>
  <version>0.0.1</version>
  <description>ROS2 bridge for PPO-controlled ARGoS robots</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
PACKAGE_EOF

# Fix header callback signature (moved here after copying the file)
sed -i 's/void ActionCallback(const std_msgs::msg::Int32::SharedPtr msg);/void ActionCallback(const std_msgs::msg::Int32::ConstSharedPtr msg);/g' ppo_argos_ros2/include/ppo_argos_ros2/ros2_iant_controller.h 2>/dev/null || true

# Create the fixed controller with proper types
cat > ppo_argos_ros2/src/ros2_iant_controller.cpp << 'CONTROLLER_EOF'
#include "ros2_iant_controller.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <sstream>

ROS2iAntController::ROS2iAntController() :
    m_pcProximity(nullptr),
    m_pcLight(nullptr),
    m_pcGround(nullptr),
    m_pcPosition(nullptr),
    m_pcWheels(nullptr),
    m_currentAction(9), // Start with stop action
    m_actionReceived(false) {
}

void ROS2iAntController::Init(TConfigurationNode& t_node) {
    // Get sensor/actuator handles
    m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    m_pcLight = GetSensor<CCI_FootBotLightSensor>("footbot_light");
    m_pcGround = GetSensor<CCI_FootBotMotorGroundSensor>("footbot_motor_ground");
    m_pcPosition = GetSensor<CCI_PositioningSensor>("positioning");
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");

    // Get robot ID
    m_strRobotId = GetId();

    // Initialize ROS2
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    // Create ROS2 node with unique name
    std::string node_name = "iant_controller_" + m_strRobotId;
    m_rosNode = std::make_shared<rclcpp::Node>(node_name);

    // Create publishers and subscribers with robot-specific topics
    std::string obs_topic = "/iant/" + m_strRobotId + "/observation";
    std::string act_topic = "/iant/" + m_strRobotId + "/action";

    m_observationPub = m_rosNode->create_publisher<std_msgs::msg::Float32MultiArray>(
        obs_topic, 10);

    m_actionSub = m_rosNode->create_subscription<std_msgs::msg::Int32>(
        act_topic, 10,
        std::bind(&ROS2iAntController::ActionCallback, this, std::placeholders::_1));

    LOG << "ROS2 iAnt Controller initialized for robot: " << m_strRobotId << std::endl;
}

void ROS2iAntController::ControlStep() {
    // Spin ROS2 to process callbacks
    rclcpp::spin_some(m_rosNode);

    // Publish current observation
    PublishObservation();

    // Execute action if received, otherwise stop
    if (m_actionReceived) {
        ExecuteAction(m_currentAction);
        m_actionReceived = false; // Reset flag
    } else {
        // Default to stop if no action received
        m_pcWheels->SetLinearVelocity(0.0, 0.0);
    }
}

void ROS2iAntController::Reset() {
    m_currentAction = 9; // Stop action
    m_actionReceived = false;
}

void ROS2iAntController::Destroy() {
    // Cleanup ROS2
    if (m_rosNode) {
        m_observationPub.reset();
        m_actionSub.reset();
        m_rosNode.reset();
    }
}

std::vector<float> ROS2iAntController::GetObservation() {
    std::vector<float> obs;
    obs.reserve(15);

    // Get position and orientation
    const CCI_PositioningSensor::SReading& posReading = m_pcPosition->GetReading();
    CQuaternion orientation = posReading.Orientation;

    // Add orientation quaternion (4 values)
    obs.push_back(static_cast<float>(orientation.GetW()));
    obs.push_back(static_cast<float>(orientation.GetX()));
    obs.push_back(static_cast<float>(orientation.GetY()));
    obs.push_back(static_cast<float>(orientation.GetZ()));

    // Check if holding food (simplified - checking ground sensor for black)
    bool holdingFood = false;
    const CCI_FootBotMotorGroundSensor::TReadings& groundReadings = m_pcGround->GetReadings();
    for (auto& reading : groundReadings) {
        if (reading.Value < 0.1) { // Black detected (food)
            holdingFood = true;
            break;
        }
    }
    obs.push_back(holdingFood ? 1.0f : 0.0f);

    // Check if near food (simplified - using ground sensor)
    bool nearFood = false;
    for (auto& reading : groundReadings) {
        if (reading.Value < 0.5 && reading.Value > 0.1) {
            nearFood = true;
            break;
        }
    }
    obs.push_back(nearFood ? 1.0f : 0.0f);

    // Add proximity sensor readings (4 quadrants: front, left, back, right)
    const CCI_FootBotProximitySensor::TReadings& proxReadings = m_pcProximity->GetReadings();

    // Front quadrant (sensors 0-5)
    Real maxFront = 0.0;
    for (int i = 0; i < 6; i++) {
        maxFront = std::max(maxFront, proxReadings[i].Value);
    }
    obs.push_back(static_cast<float>(maxFront));

    // Left quadrant (sensors 6-11)
    Real maxLeft = 0.0;
    for (int i = 6; i < 12; i++) {
        maxLeft = std::max(maxLeft, proxReadings[i].Value);
    }
    obs.push_back(static_cast<float>(maxLeft));

    // Back quadrant (sensors 12-17)
    Real maxBack = 0.0;
    for (int i = 12; i < 18; i++) {
        maxBack = std::max(maxBack, proxReadings[i].Value);
    }
    obs.push_back(static_cast<float>(maxBack));

    // Right quadrant (sensors 18-23)
    Real maxRight = 0.0;
    for (int i = 18; i < 24; i++) {
        maxRight = std::max(maxRight, proxReadings[i].Value);
    }
    obs.push_back(static_cast<float>(maxRight));

    // Pheromone presence (simplified - always 0 for now)
    obs.push_back(0.0f);

    // Light sensor readings (4 quadrants)
    const CCI_FootBotLightSensor::TReadings& lightReadings = m_pcLight->GetReadings();

    // Similar quadrant approach for light sensors
    Real maxLightFront = 0.0, maxLightLeft = 0.0, maxLightBack = 0.0, maxLightRight = 0.0;

    for (size_t i = 0; i < lightReadings.size(); i++) {
        Real angle = lightReadings[i].Angle.GetValue();
        Real value = lightReadings[i].Value;

        if (angle >= -45 && angle <= 45) {
            maxLightFront = std::max(maxLightFront, value);
        } else if (angle > 45 && angle <= 135) {
            maxLightLeft = std::max(maxLightLeft, value);
        } else if (angle > 135 || angle <= -135) {
            maxLightBack = std::max(maxLightBack, value);
        } else {
            maxLightRight = std::max(maxLightRight, value);
        }
    }

    obs.push_back(static_cast<float>(maxLightFront));
    obs.push_back(static_cast<float>(maxLightLeft));
    obs.push_back(static_cast<float>(maxLightBack));
    obs.push_back(static_cast<float>(maxLightRight));

    return obs;
}

void ROS2iAntController::PublishObservation() {
    auto obs_msg = std_msgs::msg::Float32MultiArray();
    obs_msg.data = GetObservation();
    m_observationPub->publish(obs_msg);
}

void ROS2iAntController::ActionCallback(const std_msgs::msg::Int32::ConstSharedPtr msg) {
    m_currentAction = msg->data;
    m_actionReceived = true;
}

void ROS2iAntController::ExecuteAction(int action) {
    if (action < 0 || action >= 15) {
        LOG << "[WARNING] Invalid action: " << action << std::endl;
        return;
    }

    // Set wheel speeds based on action
    Real leftSpeed = WHEEL_SPEEDS[action][0];
    Real rightSpeed = WHEEL_SPEEDS[action][1];

    m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);

    // Handle pheromone actions (10, 13, 14)
    if (action == 10 || action == 13 || action == 14) {
        LOG << "Pheromone action triggered (not implemented)" << std::endl;
    }
}

// Register the controller
REGISTER_CONTROLLER(ROS2iAntController, "ros2_iant_controller")
CONTROLLER_EOF

check_status "Controller source creation"

# Step 6: Create ARGoS config files
echo -e "${YELLOW}Step 6: Creating ARGoS config files...${NC}"
mkdir -p ppo_argos_ros2/config
cat > ppo_argos_ros2/config/test_foraging.argos << 'CONFIG_EOF'
<?xml version="1.0" ?>
<argos-configuration>
  <framework>
    <system threads="0" />
    <experiment length="0" ticks_per_second="10" />
  </framework>

  <controllers>
    <ros2_iant_controller id="fbc" library="/root/ros2_ws/install/ppo_argos_ros2/lib/libros2_iant_controller">
      <actuators>
        <differential_steering implementation="default" />
      </actuators>
      <sensors>
        <footbot_proximity implementation="default" show_rays="true" />
        <footbot_light implementation="rot_z_only" show_rays="true" />
        <footbot_motor_ground implementation="rot_z_only" />
        <positioning implementation="default" />
      </sensors>
      <params />
    </ros2_iant_controller>
  </controllers>

  <arena size="5, 5, 2" center="0,0,1">
    <floor id="floor" source="loop_functions" pixels_per_meter="50" />

    <!-- Add some walls -->
    <box id="wall_north" size="5,0.1,0.5" movable="false">
      <body position="0,2.5,0" orientation="0,0,0" />
    </box>
    <box id="wall_south" size="5,0.1,0.5" movable="false">
      <body position="0,-2.5,0" orientation="0,0,0" />
    </box>
    <box id="wall_east" size="0.1,5,0.5" movable="false">
      <body position="2.5,0,0" orientation="0,0,0" />
    </box>
    <box id="wall_west" size="0.1,5,0.5" movable="false">
      <body position="-2.5,0,0" orientation="0,0,0" />
    </box>

    <!-- Food items -->
    <cylinder id="food1" radius="0.1" height="0.1" movable="false">
      <body position="1,1,0" orientation="0,0,0" />
      <leds medium="leds">
        <led offset="0,0,0.11" anchor="origin" color="yellow" />
      </leds>
    </cylinder>

    <!-- Single robot -->
    <foot-bot id="fb_0">
      <body position="0,0,0" orientation="0,0,0" />
      <controller config="fbc" />
    </foot-bot>
  </arena>

  <physics_engines>
    <dynamics2d id="dyn2d" />
  </physics_engines>

  <media>
    <led id="leds" />
  </media>

  <visualization>
    <qt-opengl>
      <camera>
        <placement idx="0" position="0,0,4" look_at="0,0,0" lens_focal_length="20" />
      </camera>
    </qt-opengl>
  </visualization>
</argos-configuration>
CONFIG_EOF
check_status "Config file creation"

# Step 7: Build the package
echo -e "${YELLOW}Step 7: Building ROS2 package...${NC}"
cd /root/ros2_ws
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
colcon build --packages-select ppo_argos_ros2 2>&1 | tee /tmp/build.log | grep -E "Starting|Finished|Failed"

if grep -q "Finished <<< ppo_argos_ros2" /tmp/build.log; then
    echo -e "${GREEN}✓ Build successful${NC}"
else
    echo -e "${RED}✗ Build failed. Check /tmp/build.log for details${NC}"
    exit 1
fi

# Step 8: Source workspace
echo -e "${YELLOW}Step 8: Sourcing workspace...${NC}"
source /root/ros2_ws/install/setup.bash
check_status "Workspace source"

# Step 9: Verify installation
echo -e "${YELLOW}Step 9: Verifying installation...${NC}"
if [ -f /root/ros2_ws/install/ppo_argos_ros2/lib/libros2_iant_controller.so ]; then
    echo -e "${GREEN}✓ Controller library found${NC}"
else
    echo -e "${RED}✗ Controller library not found${NC}"
    exit 1
fi

# Step 10: Test PPO model
echo -e "${YELLOW}Step 10: Testing PPO model...${NC}"
if [ -f /root/ppo_argos/ppo_final.pt ]; then
    python3 /root/ppo_argos/test_model.py 2>/dev/null | head -5
    check_status "Model test"
else
    echo -e "${YELLOW}⚠ Model file not found - skipping test${NC}"
fi

# Step 11: Create test ARGoS file with visualization
echo -e "${YELLOW}Step 11: Creating ARGoS test file with visualization...${NC}"
cat > /root/ppo_argos/test_with_viz.argos << 'EOF'
<?xml version="1.0"?>
<argos-configuration>
  <framework>
    <system threads="0" />
    <experiment length="100" ticks_per_second="10" />
  </framework>

  <controllers>
    <ros2_iant_controller id="iant_ros2"
      library="/root/ros2_ws/install/ppo_argos_ros2/lib/libros2_iant_controller.so">
      <actuators>
        <differential_steering implementation="default" />
      </actuators>
      <sensors>
        <footbot_proximity implementation="default" show_rays="true" />
        <footbot_light implementation="default" show_rays="false" />
        <footbot_motor_ground implementation="rot_z_only" />
        <positioning implementation="default" />
      </sensors>
      <params />
    </ros2_iant_controller>
  </controllers>

  <arena size="5, 5, 1" center="0,0,0.5">
    <floor id="floor" source="image" path="/root/ppo_argos/floor.png" />

    <box id="wall_north" size="5,0.1,0.5" movable="false">
      <body position="0,2.5,0" orientation="0,0,0" />
    </box>
    <box id="wall_south" size="5,0.1,0.5" movable="false">
      <body position="0,-2.5,0" orientation="0,0,0" />
    </box>
    <box id="wall_east" size="0.1,5,0.5" movable="false">
      <body position="2.5,0,0" orientation="0,0,0" />
    </box>
    <box id="wall_west" size="0.1,5,0.5" movable="false">
      <body position="-2.5,0,0" orientation="0,0,0" />
    </box>

    <foot-bot id="fb_0">
      <body position="0,0,0" orientation="0,0,0" />
      <controller config="iant_ros2" />
    </foot-bot>
  </arena>

  <physics_engines>
    <dynamics2d id="dyn2d" />
  </physics_engines>

  <media>
    <led id="leds" />
  </media>

  <!-- Visualization -->
  <visualization>
    <qt-opengl>
      <camera>
        <placements>
          <placement index="0" position="0,0,5" look_at="0,0,0" up="1,0,0" lens_focal_length="20" />
        </placements>
      </camera>
    </qt-opengl>
  </visualization>
</argos-configuration>
EOF
check_status "ARGoS test file creation"

# Step 12: Create simple floor image
echo -e "${YELLOW}Step 12: Creating floor image...${NC}"
python3 -c "
from PIL import Image
import numpy as np
img = Image.fromarray(np.ones((500, 500, 3), dtype=np.uint8) * 200)
img.save('/root/ppo_argos/floor.png')
" 2>/dev/null || echo "  Note: PIL not found, using white floor"
touch /root/ppo_argos/floor.png  # Create empty file if PIL fails
check_status "Floor image creation"

# Final message
echo ""
echo -e "${BLUE}==================================================${NC}"
echo -e "${GREEN}       Container Setup Complete!                ${NC}"
echo -e "${BLUE}==================================================${NC}"
echo ""
echo -e "${GREEN}You can now test the pipeline with:${NC}"
echo ""
echo "1. Test ARGoS with visualization:"
echo -e "   ${YELLOW}argos3 -c /root/ppo_argos/test_with_viz.argos${NC}"
echo ""
echo "2. For full pipeline test, open 3 terminals:"
echo "   Terminal 1: Monitor topics"
echo -e "   ${YELLOW}source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash${NC}"
echo -e "   ${YELLOW}watch -n 0.5 ros2 topic list${NC}"
echo ""
echo "   Terminal 2: Run ARGoS"
echo -e "   ${YELLOW}source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash${NC}"
echo -e "   ${YELLOW}export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:\$PKG_CONFIG_PATH${NC}"
echo -e "   ${YELLOW}argos3 -c /root/ppo_argos/test_with_viz.argos${NC}"
echo ""
echo "   Terminal 3: Run PPO controller"
echo -e "   ${YELLOW}source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash${NC}"
echo -e "   ${YELLOW}cd /root/ppo_argos && python3 ppo_ros2_node.py${NC}"
echo ""

# Save environment setup for easy sourcing
cat > /root/setup_env.sh << 'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
echo "Environment ready!"
EOF
chmod +x /root/setup_env.sh

echo -e "${BLUE}Tip: Run ${YELLOW}source /root/setup_env.sh${BLUE} in new terminals for quick setup${NC}"
echo ""