#ifndef IANT_RL_CONTROLLER_LIVE_H
#define IANT_RL_CONTROLLER_LIVE_H

#include "iAnt_controller.h"
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <atomic>

// Shared memory structure for real-time communication
struct RLSharedMemory {
    std::atomic<bool> ready;          // Python has written new action
    std::atomic<bool> obs_ready;      // C++ has written new observation
    float left_speed;                  // Action: left wheel speed
    float right_speed;                 // Action: right wheel speed
    bool lay_pheromone;               // Action: lay pheromone
    float observation[15];            // Current observation
    float reward;                     // Current reward
    bool terminated;                  // Episode done
    bool truncated;                   // Time limit
    int episode;                      // Current episode
    int step;                         // Current step
    int food_collected;               // Food collected this episode
};

class iAnt_rl_controller_live : public iAnt_controller {

public:
    iAnt_rl_controller_live();
    virtual ~iAnt_rl_controller_live();

    virtual void Init(TConfigurationNode& node);
    virtual void ControlStep();
    virtual void Reset();

protected:
    CCI_DifferentialSteeringActuator* m_wheels;

private:
    RLSharedMemory* shared_mem;
    int shm_fd;
    std::string shm_name;

    void InitSharedMemory();
    void CleanupSharedMemory();
    void WriteObservation();
    void WaitForAction();
};

#endif