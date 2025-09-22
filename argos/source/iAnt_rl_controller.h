#ifndef IANT_RL_CONTROLLER_H
#define IANT_RL_CONTROLLER_H

#include "iAnt_controller.h"
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <fstream>
#include <json/json.h>
#include <vector>

class iAnt_rl_controller : public iAnt_controller {

public:
    iAnt_rl_controller();
    virtual ~iAnt_rl_controller() {}
    
    virtual void Init(TConfigurationNode& node);
    virtual void ControlStep();
    virtual void Reset();

protected:
    // Need access to actuator
    CCI_DifferentialSteeringActuator* m_wheels;
    
private:
    bool LoadActionsFromFile();
    void GetRobotAction(size_t robot_id, size_t step, 
                       Real& left_speed, Real& right_speed, bool& lay_pheromone);
    
    Json::Value m_actions;
    size_t m_current_step;
    bool m_actions_loaded;
    std::string m_action_file_path;
};

#endif