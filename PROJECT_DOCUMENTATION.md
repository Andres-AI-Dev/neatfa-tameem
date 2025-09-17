# neatfa-tameem RL Framework Documentation

## Project Overview
This is Tameem's Reinforcement Learning framework for the iAnt robots in ARGoS simulator. It removes NEAT/GA dependencies and exposes a Python API through pybind11 for training multi-agent swarm robotics using DQN and PPO algorithms.

## File Structure and Detailed Explanations

### C++ Source Files (`argos/source/`)

#### 1. **py_iant_rl.cpp** (77 lines) - THE CORE FILE
- **Purpose**: Pybind11 wrapper that bridges C++ ARGoS with Python
- **Key Class**: `IAntRLEnv` - Implements the RL environment interface
- **Methods**:
  - `reset()`: Initializes/resets ARGoS simulation, returns initial observation
  - `step(left_speed, right_speed, lay_pheromone)`: Advances simulation by one tick
  - `close()`: Cleanly destroys the simulation
- **Important**: Has `force_no_viz=true` hardcoded (line 11) which disables visualization
- **Returns**: Standard RL tuple (observation, reward, terminated, truncated, info)

#### 2. **iAnt_controller.cpp/h** 
- **Purpose**: Robot controller that executes RL actions
- **Key Changes from NEAT version**:
  - Removed all neural network logic
  - Added `SetAction()`: Caches action from Python
  - Added `GetObservation()`: Returns 15-dim observation vector
  - `ControlStep()`: Applies cached action to robot's wheels
- **Observation Space** (15 dimensions):
  - [0-3]: Orientation quaternion (w,x,y,z)
  - [4]: isHoldingFood (0 or 1)
  - [5]: isNearFood (0 or 1)
  - [6-9]: Proximity sensor readings (front, left, back, right)
  - [10]: isNearPheromone (0 or 1)
  - [11-14]: Light sensor values (front, left, back, right)

#### 3. **iAnt_loop_functions.cpp/h**
- **Purpose**: Manages the overall simulation environment
- **Key RL Methods**:
  - `RLSetAction()`: Forwards action to robot controller
  - `RLGetObservation()`: Gets observation from robot
  - `RLTerminated()`: Returns true when all food collected
  - `RLTruncated()`: Returns true when time limit reached
  - `getFitness()`: Calculates fitness (food collected + 2*food returned)
- **Manages**: Food distribution, pheromone trails, nest location

#### 4. **iAnt_pheromone.cpp/h**
- **Purpose**: Implements pheromone trail mechanics
- **Features**:
  - Pheromone laying by robots
  - Pheromone decay over time
  - Pheromone detection by sensors

#### 5. **iAnt_qt_user_functions.cpp/h**
- **Purpose**: Qt GUI visualization functions
- **Note**: Currently disabled due to `force_no_viz=true`
- **Would show**: Robot positions, food, pheromone trails, nest

#### 6. **main.cpp**
- **Purpose**: Standalone executable for running ARGoS
- **Note**: Not used in RL training (Python controls instead)

### Python Training Scripts (`scripts/`)

#### 1. **train_dqn.py** (191 lines) - WORKING
- **Purpose**: Deep Q-Network training implementation
- **Key Components**:
  - `ArgosEnvWrapper`: Handles ARGoS directory changes
  - `QNetwork`: Neural network for Q-values (128-128-6 architecture)
  - **Action Space**: 6 discrete actions
    - Forward, Backward, Turn Right, Turn Left, Stop, Forward+Pheromone
  - **Training Parameters**:
    - 200 episodes
    - Learning rate: 1e-3
    - Epsilon decay: 1.0 → 0.1 over 20,000 steps
    - Replay buffer: 100,000 transitions
    - Target network update: Every 1,000 steps

#### 2. **train_ppo.py** (447 lines) - NOT WORKING
- **Purpose**: Proximal Policy Optimization implementation
- **Architecture**: Actor-Critic with shared features
- **Issue**: Tameem mentioned it doesn't work (reason unknown)
- **Uses**: Same 6 discrete actions as DQN

#### 3. **test.py** (22 lines)
- **Purpose**: Basic test to verify environment works
- **Usage**: Quick sanity check that module loads and steps

#### 4. **eval_greedy.py** & **eval_ppo.py**
- **Purpose**: Evaluate trained models
- **Note**: Would enable visualization if working

### Configuration Files (`argos/experiments/`)

#### 1. **iAnt.xml** (Original)
- **Purpose**: NEAT/GA configuration with chromosome data
- **Not for RL**: Contains evolution-specific parameters

#### 2. **iAnt_rl.xml** (Created by us, needs Tameem's version)
- **Purpose**: RL-specific environment configuration
- **Key Parameters**:
  ```xml
  MaxSimTime = "125"  <!-- For quick tests (2000 steps) -->
  MaxSimTime = "1800" <!-- For full training (28,800 steps) -->
  FoodItemCount = "64"
  entity quantity = "6" <!-- 6 robots for swarm -->
  ```

### Build Files

#### 1. **argos/CMakeLists.txt**
- **Purpose**: Main build configuration
- **Key sections**:
  - Finds ARGoS, GSL, Lua, Qt dependencies
  - Builds pybind11 module if found
  - Links all libraries

#### 2. **argos/source/CMakeLists.txt**
- **Purpose**: Builds individual components
- **Creates**: 
  - `libiAnt_controller.dylib`
  - `libiAnt_loop_functions.dylib`
  - `iant_main` executable
  
## Complete Setup Guide for macOS (M-series)

### Prerequisites
- macOS with Apple Silicon (M1/M2/M3/M4)
- Homebrew installed
- Python 3.11+ 
- Git

### Step-by-Step Installation

#### 1. Install Dependencies
```bash
# Install ARGoS and build tools
brew tap ilpincy/argos3
brew install argos3 cmake qt gsl lua python@3.13 pybind11

# Install Python packages
pip3 install numpy torch pybind11
```

#### 2. Clone Repository
```bash
git clone https://github.com/MARSLab-UTRGV/neatfa-tameem.git
cd neatfa-tameem
git checkout RL-Training-Tameem
```

#### 3. Fix Build Configuration for macOS
Edit `argos/CMakeLists.txt`:
```cmake
# Change line 1:
cmake_minimum_required(VERSION 3.5)  # Was 2.8.12

# Add after line 10:
set(ARGOS_CMAKE_DIR "/opt/homebrew/share/argos3/cmake")

# Change line 28:
link_directories(/opt/homebrew/lib/argos3)  # Was /usr/lib/argos3
```

#### 4. Build the Project
```bash
cd argos
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
```

You should see:
- Building libraries: `libiAnt_controller.dylib`, `libiAnt_loop_functions.dylib`
- Building Python module: `iant_rl.cpython-313-darwin.so`

#### 5. Create Missing Configuration
```bash
cd ../experiments
cp iAnt.xml iAnt_rl.xml
```

Edit `iAnt_rl.xml`:
- Remove the entire `Chromosome = "..."` line
- Add `<footbot_light implementation = "rot_z_only" show_rays = "false"/>` to sensors
- Verify: `entity quantity="6"` for multi-agent
- Verify: `FoodItemCount = "64"`
- Set: `MaxSimTime = "125"` for testing or `"1800"` for full training

#### 6. Fix Python Script Paths
Edit `scripts/test.py` and `scripts/train_dqn.py`:
```python
# Replace hardcoded path:
ARGOS_DIR = os.path.join(os.path.dirname(__file__), "..", "argos")
ARGOS_DIR = os.path.abspath(ARGOS_DIR)
```

#### 7. Test the Installation
```bash
cd ../../scripts
python3 test.py
```

Expected output: 15-dimensional observation vector

#### 8. Run DQN Training
```bash
python3 train_dqn.py
```

Monitor progress:
```bash
tail -f log_*.txt
```

## Training Parameters Explained

### Quick Test Mode
- MaxSimTime: 125 seconds
- Steps per episode: 2000
- Real time: ~2 seconds per episode
- Use for: Debugging, quick iterations

### Full Training Mode
- MaxSimTime: 1800 seconds
- Steps per episode: 28,800
- Real time: ~30 seconds per episode
- Use for: Actual learning, paper results

## Known Issues

1. **PPO Not Working**: Algorithm fails to learn (reason unknown)
2. **Visualization Disabled**: `force_no_viz=true` hardcoded in py_iant_rl.cpp
3. **Missing iAnt_rl.xml**: Need Tameem's actual configuration file

## Reward Structure

- **Picking up food**: +1 reward
- **Returning food to nest**: +2 reward
- **Dense reward**: Calculated as delta fitness between steps

## Multi-Agent Details

- **6 robots** in the arena
- Only first robot (`fb_0`) is controlled by RL
- All robots contribute to:
  - Food collection
  - Pheromone trails
  - Environment dynamics
- This is true swarm robotics - collective behavior emerges

## Next Steps for Federated Learning

1. Get Tameem's actual `iAnt_rl.xml` configuration
2. Fix visualization for debugging
3. Debug PPO implementation
4. Add federated learning layer on top:
   - Multiple environments
   - Parameter aggregation
   - Communication protocols
   - Privacy preservation

## Contact
- Original Author: Tameem (Discord: @Tameem.uz.zaman)
- Current Maintainer: Andres G.
- Lab: MARS Lab (Multi-Autonomous Robotic Swarms), UTRGV