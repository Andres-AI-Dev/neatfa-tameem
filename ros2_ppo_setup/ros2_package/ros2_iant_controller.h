#ifndef ROS2_IANT_CONTROLLER_H
#define ROS2_IANT_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/utility/math/rng.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <memory>
#include <vector>

using namespace argos;

class ROS2iAntController : public CCI_Controller {
public:
    ROS2iAntController();
    virtual ~ROS2iAntController() = default;

    virtual void Init(TConfigurationNode& t_node) override;
    virtual void ControlStep() override;
    virtual void Reset() override;
    virtual void Destroy() override;

private:
    // Sensors
    CCI_FootBotProximitySensor* m_pcProximity;
    CCI_FootBotLightSensor* m_pcLight;
    CCI_FootBotMotorGroundSensor* m_pcGround;
    CCI_PositioningSensor* m_pcPosition;

    // Actuators
    CCI_DifferentialSteeringActuator* m_pcWheels;

    // ROS2
    std::shared_ptr<rclcpp::Node> m_rosNode;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_observationPub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_actionSub;

    // State
    int m_currentAction;
    bool m_actionReceived;
    std::string m_strRobotId;

    // Helper methods
    std::vector<float> GetObservation();
    void PublishObservation();
    void ActionCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void ExecuteAction(int action);

    // Constants for actions
    static constexpr Real WHEEL_SPEEDS[15][2] = {
        {16.0, 16.0},   // 0: Full speed forward
        {16.0, -16.0},  // 1: Full speed turn right
        {-16.0, 16.0},  // 2: Full speed turn left
        {8.0, 8.0},     // 3: Medium speed forward
        {8.0, -8.0},    // 4: Medium speed turn right
        {-8.0, 8.0},    // 5: Medium speed turn left
        {4.0, 4.0},     // 6: Slow forward
        {4.0, -4.0},    // 7: Slow turn right
        {-4.0, 4.0},    // 8: Slow turn left
        {0.0, 0.0},     // 9: Stop
        {0.0, 0.0},     // 10: Stop + pheromone (handled separately)
        {12.0, 8.0},    // 11: Forward with slight right bias
        {8.0, 12.0},    // 12: Forward with slight left bias
        {12.0, 8.0},    // 13: Forward right + pheromone
        {8.0, 12.0},    // 14: Forward left + pheromone
    };
};

#endif