#include "iAnt_rl_controller.h"
#include <iostream>
#include <sstream>

iAnt_rl_controller::iAnt_rl_controller() :
    iAnt_controller(),
    m_current_step(0),
    m_actions_loaded(false),
    m_action_file_path("robot_actions.json")
{
}

void iAnt_rl_controller::Init(TConfigurationNode& node) {
    // Initialize base controller
    iAnt_controller::Init(node);
    
    // Get wheels actuator
    m_wheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    
    // Try to get action file path from XML if specified
    GetNodeAttributeOrDefault(node, "action_file", m_action_file_path, m_action_file_path);
    
    // Load actions from file
    m_actions_loaded = LoadActionsFromFile();
    if (m_actions_loaded) {
        LOG << "[RL Controller] Successfully loaded actions from " << m_action_file_path << std::endl;
    } else {
        LOG << "[RL Controller] Could not load actions from " << m_action_file_path 
            << ", using default behavior" << std::endl;
    }
    
    m_current_step = 0;
}

bool iAnt_rl_controller::LoadActionsFromFile() {
    std::ifstream file(m_action_file_path);
    if (!file.is_open()) {
        return false;
    }
    
    Json::CharReaderBuilder builder;
    Json::String errs;
    
    if (!Json::parseFromStream(builder, file, &m_actions, &errs)) {
        LOGERR << "Failed to parse JSON: " << errs << std::endl;
        return false;
    }
    
    file.close();
    return true;
}

void iAnt_rl_controller::GetRobotAction(size_t robot_id, size_t step,
                                        Real& left_speed, Real& right_speed, 
                                        bool& lay_pheromone) {
    // Default values
    left_speed = 0.0;
    right_speed = 0.0;
    lay_pheromone = false;
    
    if (!m_actions_loaded || !m_actions.isArray()) {
        return;
    }
    
    // Find the action for this step
    for (const auto& step_data : m_actions) {
        if (step_data["step"].asUInt() == step) {
            const auto& robots = step_data["robots"];
            if (robots.isArray() && robot_id < robots.size()) {
                const auto& robot_action = robots[static_cast<int>(robot_id)];
                left_speed = robot_action["left"].asDouble();
                right_speed = robot_action["right"].asDouble();
                lay_pheromone = robot_action["pheromone"].asBool();
                return;
            }
        }
    }
}

void iAnt_rl_controller::ControlStep() {
    if (m_actions_loaded) {
        // Get robot ID from the entity ID (e.g., "fb_0" -> 0)
        std::string id = GetId();
        size_t robot_id = 0;
        
        // Extract number from ID
        size_t pos = id.find_last_of("_");
        if (pos != std::string::npos) {
            robot_id = std::stoi(id.substr(pos + 1));
        }
        
        // Get action for this robot and step
        Real left_speed, right_speed;
        bool lay_pheromone;
        GetRobotAction(robot_id, m_current_step, left_speed, right_speed, lay_pheromone);
        
        // Apply wheel speeds directly
        if (m_wheels != nullptr) {
            m_wheels->SetLinearVelocity(left_speed, right_speed);
        }
        
        // Note: We can't handle pheromone laying from here due to private members
        // Would need to modify base class or use loop functions
        
        // Increment step globally (shared among all robots)
        m_current_step++;
    } else {
        // Fall back to default behavior
        iAnt_controller::ControlStep();
    }
}

void iAnt_rl_controller::Reset() {
    iAnt_controller::Reset();
    m_current_step = 0;
    
    // Reload actions in case they've been updated
    m_actions_loaded = LoadActionsFromFile();
}

REGISTER_CONTROLLER(iAnt_rl_controller, "iAnt_rl_controller")