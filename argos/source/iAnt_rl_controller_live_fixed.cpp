#include "iAnt_rl_controller_live.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <chrono>
#include <thread>

iAnt_rl_controller_live::iAnt_rl_controller_live() :
    iAnt_controller(),
    m_wheels(nullptr),
    shared_mem(nullptr),
    shm_fd(-1),
    shm_name("/iant_rl_live") {}

iAnt_rl_controller_live::~iAnt_rl_controller_live() {
    CleanupSharedMemory();
}

void iAnt_rl_controller_live::Init(TConfigurationNode& node) {
    // Initialize base controller
    iAnt_controller::Init(node);

    // Get wheel actuator
    m_wheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");

    // Initialize shared memory
    InitSharedMemory();

    LOG << "[LIVE RL] Controller initialized with shared memory: " << shm_name << std::endl;
}

void iAnt_rl_controller_live::InitSharedMemory() {
    // Try to open existing shared memory first (created by Python)
    shm_fd = shm_open(shm_name.c_str(), O_RDWR, 0666);

    if (shm_fd == -1) {
        // If doesn't exist, create it
        LOG << "[LIVE RL] Creating new shared memory" << std::endl;
        shm_fd = shm_open(shm_name.c_str(), O_CREAT | O_RDWR, 0666);
        if (shm_fd == -1) {
            LOGERR << "[LIVE RL] Failed to create shared memory" << std::endl;
            return;
        }

        // Set size
        if (ftruncate(shm_fd, sizeof(RLSharedMemory)) == -1) {
            LOGERR << "[LIVE RL] Failed to set shared memory size" << std::endl;
            return;
        }
    } else {
        LOG << "[LIVE RL] Connected to existing shared memory" << std::endl;
    }

    // Map memory
    shared_mem = (RLSharedMemory*)mmap(0, sizeof(RLSharedMemory),
                                       PROT_READ | PROT_WRITE,
                                       MAP_SHARED, shm_fd, 0);
    if (shared_mem == MAP_FAILED) {
        LOGERR << "[LIVE RL] Failed to map shared memory" << std::endl;
        shared_mem = nullptr;
        return;
    }

    LOG << "[LIVE RL] Shared memory mapped successfully" << std::endl;

    // Signal that C++ is ready
    shared_mem->obs_ready = true;
}

void iAnt_rl_controller_live::CleanupSharedMemory() {
    if (shared_mem != nullptr && shared_mem != MAP_FAILED) {
        munmap(shared_mem, sizeof(RLSharedMemory));
    }
    if (shm_fd != -1) {
        close(shm_fd);
    }
}

void iAnt_rl_controller_live::ControlStep() {
    if (!shared_mem) {
        // No shared memory - stop the robot
        m_wheels->SetLinearVelocity(0.0, 0.0);
        return;
    }

    // Always write current observation
    WriteObservation();

    // Signal observation is ready
    shared_mem->obs_ready.store(true);

    // Check if Python has sent an action
    if (shared_mem->ready.load()) {
        // Apply the action from Python
        Real left_speed = shared_mem->left_speed;
        Real right_speed = shared_mem->right_speed;

        // Debug output every 10 steps
        if (shared_mem->step % 10 == 0) {
            LOG << "[LIVE RL] Step " << shared_mem->step
                << " Action: L=" << left_speed << " R=" << right_speed
                << " Food=" << shared_mem->food_collected << std::endl;
        }

        // Apply wheel speeds directly - no modification
        m_wheels->SetLinearVelocity(left_speed, right_speed);

        // Handle pheromone if requested
        if (shared_mem->lay_pheromone) {
            LayPheromone(GetPosition());
        }

        // Clear ready flag to signal we've consumed the action
        shared_mem->ready.store(false);

        // Increment step
        shared_mem->step++;
    } else {
        // No action from Python yet - keep waiting, don't move
        static int wait_count = 0;
        wait_count++;
        if (wait_count % 100 == 0) {
            LOG << "[LIVE RL] Waiting for Python action... (count=" << wait_count << ")" << std::endl;
        }

        // Stop the robot while waiting
        m_wheels->SetLinearVelocity(0.0, 0.0);
    }
}

void iAnt_rl_controller_live::WriteObservation() {
    // Get current observation (same as Python binding)
    std::vector<Real> obs = GetObservation();

    // Copy to shared memory
    for (size_t i = 0; i < obs.size() && i < 15; i++) {
        shared_mem->observation[i] = obs[i];
    }

    // Calculate reward
    shared_mem->reward = 0.0;

    // Reward for holding food
    if (IsHoldingFood()) {
        shared_mem->reward = 1.0;
    }

    // Big reward for returning food to nest
    if (IsHoldingFood() && IsNearNest()) {
        shared_mem->reward = 10.0;
    }

    // Small penalty for each step (encourage efficiency)
    shared_mem->reward -= 0.01;

    // Update termination flags
    shared_mem->terminated = false;
    shared_mem->truncated = false;

    // Track food collection
    static bool prev_holding_food = false;
    bool curr_holding_food = IsHoldingFood();

    if (curr_holding_food && !prev_holding_food) {
        shared_mem->food_collected++;
        LOG << "[LIVE RL] FOOD COLLECTED! Total: " << shared_mem->food_collected << std::endl;
    }

    // Track food delivery
    static bool prev_near_nest = false;
    bool curr_near_nest = IsNearNest();

    if (prev_holding_food && !curr_holding_food && curr_near_nest) {
        LOG << "[LIVE RL] FOOD DELIVERED TO NEST!" << std::endl;
    }

    prev_holding_food = curr_holding_food;
    prev_near_nest = curr_near_nest;
}

void iAnt_rl_controller_live::WaitForAction() {
    // This function is no longer used - we handle waiting in ControlStep
}

void iAnt_rl_controller_live::Reset() {
    iAnt_controller::Reset();

    if (shared_mem) {
        shared_mem->step = 0;
        shared_mem->food_collected = 0;
        shared_mem->episode++;
        LOG << "[LIVE RL] Reset for episode " << shared_mem->episode << std::endl;
    }
}

// Register controller
REGISTER_CONTROLLER(iAnt_rl_controller_live, "iAnt_rl_controller_live")