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
    obs.push_back(orientation.GetW());
    obs.push_back(orientation.GetX());
    obs.push_back(orientation.GetY());
    obs.push_back(orientation.GetZ());

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
    float maxFront = 0.0f;
    for (int i = 0; i < 6; i++) {
        maxFront = std::max(maxFront, proxReadings[i].Value);
    }
    obs.push_back(maxFront);

    // Left quadrant (sensors 6-11)
    float maxLeft = 0.0f;
    for (int i = 6; i < 12; i++) {
        maxLeft = std::max(maxLeft, proxReadings[i].Value);
    }
    obs.push_back(maxLeft);

    // Back quadrant (sensors 12-17)
    float maxBack = 0.0f;
    for (int i = 12; i < 18; i++) {
        maxBack = std::max(maxBack, proxReadings[i].Value);
    }
    obs.push_back(maxBack);

    // Right quadrant (sensors 18-23)
    float maxRight = 0.0f;
    for (int i = 18; i < 24; i++) {
        maxRight = std::max(maxRight, proxReadings[i].Value);
    }
    obs.push_back(maxRight);

    // Pheromone presence (simplified - always 0 for now)
    obs.push_back(0.0f);

    // Light sensor readings (4 quadrants)
    const CCI_FootBotLightSensor::TReadings& lightReadings = m_pcLight->GetReadings();

    // Similar quadrant approach for light sensors
    float maxLightFront = 0.0f, maxLightLeft = 0.0f, maxLightBack = 0.0f, maxLightRight = 0.0f;

    for (size_t i = 0; i < lightReadings.size(); i++) {
        float angle = lightReadings[i].Angle.GetValue();
        float value = lightReadings[i].Value;

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

    obs.push_back(maxLightFront);
    obs.push_back(maxLightLeft);
    obs.push_back(maxLightBack);
    obs.push_back(maxLightRight);

    return obs;
}

void ROS2iAntController::PublishObservation() {
    auto obs_msg = std_msgs::msg::Float32MultiArray();
    obs_msg.data = GetObservation();
    m_observationPub->publish(obs_msg);
}

void ROS2iAntController::ActionCallback(const std_msgs::msg::Int32::SharedPtr msg) {
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
    // Note: ARGoS doesn't have direct pheromone laying for footbot
    // This would need to be implemented through loop functions
    if (action == 10 || action == 13 || action == 14) {
        // Would trigger pheromone laying through loop functions
        LOG << "Pheromone action triggered (not implemented)" << std::endl;
    }
}

// Register the controller
REGISTER_CONTROLLER(ROS2iAntController, "ros2_iant_controller")