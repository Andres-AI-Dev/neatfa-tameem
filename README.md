## RL Environment (Pybind11)

This repository now includes a reinforcement learning (RL) interface to ARGoS that removes the NEAT/GA dependency in the simulation loop and exposes a Python API through pybind11. The RL environment treats ARGoS as a step-able simulator that returns observations, rewards, and termination/truncation signals.

### What changed and wh

- `argos/source/iAnt_controller.cpp` and `argos/source/iAnt_controller.h`
  - Removed all NEAT/GA neural-network logic and dependencies.
  - Added RL action interface: `SetAction(left_speed, right_speed, lay_pheromone)` caches an action each tick.
  - Controller `ControlStep()` now applies the cached action to the robot’s differential drive and optionally lays pheromone.
  - Added `GetObservation()` to collect the agent’s observation vector directly from sensors and local state.
  - Why: The controller becomes a pure executor of external actions and a producer of observations (agent policy lives in Python).

- `argos/source/iAnt_loop_functions.cpp` and `argos/source/iAnt_loop_functions.h`
  - Removed chromosome parsing/output and any NEAT-specific utilities.
  - Added RL helper methods:
    - `RLSetAction(left,right,lay)` forwards the action to the primary robot controller.
    - `RLGetObservation()` returns the primary robot’s observation vector.
    - `RLTerminated()` signals task completion (all food collected).
    - `RLTruncated()` signals time limit reached.
  - Maintains global simulation state (food, pheromones, ranges) and computes fitness.
  - Why: The loop functions act as the RL environment boundary for Python bindings and manage episode lifecycle.

- `argos/source/main.cpp`
  - Removed GA/NEAT includes; kept a minimal CLI runner that can still load and execute an ARGoS experiment if desired.
  - Why: Keeps native executable working without NEAT dependencies.

- `argos/source/py_iant_rl.cpp`
  - New pybind11 module exposing a Python class `IAntRLEnv` with `reset()`, `step(left,right,lay)`, and `close()`.
  - `reset()` loads the XML, initializes ARGoS without visualization, and returns the initial observation.
  - `step()` applies an action via `RLSetAction`, advances simulation by one tick, and returns `(obs, reward, terminated, truncated, info)`.
  - Why: Allows training/evaluation with standard Python RL libraries while keeping physics and environment in C++.

- CMake updates
  - Removed all `nn/` and `ga/` sources from `argos/source/CMakeLists.txt`.
  - Added pybind11 module build in `argos/CMakeLists.txt` to produce `iant_rl` Python extension.

### Observation, action, reward, and termination

- Observation vector (length 15) from `iAnt_controller::GetObservation()`:
  - Indices 0–3: Orientation quaternion `(w, x, y, z)` from the positioning sensor.
  - Index 4: `isHoldingFood` (1 if carrying food, else 0).
  - Index 5: `isNearFood` (1 if within food pickup radius, else 0).
  - Indices 6–9: Maximum proximity sensor readings in the four sectors (front, left, back, right).
  - Index 10: `isNearPheromone` (1 if near an active pheromone, else 0).
  - Indices 11–14: Maximum light sensor values in the four sectors (front, left, back, right).

- Action space (applied each tick via `iAnt_controller::ControlStep`) and ranges:
  - `left_speed`: linear velocity for left wheel, truncated to `[-16, 16]`.
  - `right_speed`: linear velocity for right wheel, truncated to `[-16, 16]`.
  - `lay_pheromone`: boolean flag; if true, lay or refresh pheromone at the robot’s current location.

- Reward function (dense), computed in Python as delta fitness per step:
  - `fitness = (FoodItemCount - FoodList.size()) + 2 * foodReturned` (see `iAnt_loop_functions::getFitness()`).
  - Reward at time t is `fitness_t - fitness_{t-1}`; picking food and returning to nest increase reward (nest return is weighted higher).

- Termination and truncation conditions:
  - `terminated` is true when all food is collected (`FoodList.size() == 0`).
  - `truncated` is true when the time limit is reached (`SimTime >= MaxSimTime`, note `MaxSimTime` is scaled by ticks per second in initialization).

### Step semantics

Each `step()`:
- Calls `RLSetAction()` to cache the action on the primary controller (first `foot-bot`).
- Calls `UpdateSpace()` to advance ARGoS by one tick, which triggers `PreStep()`, per-robot `ControlStep()`, and `PostStep()`.
- Reads the next observation, computes reward as delta fitness, and returns termination flags and info.

### XML for RL

- `argos/experiments/iAnt_rl.xml` is an RL-friendly configuration derived from `iAnt.xml`:
  - Removes the `Chromosome` attribute entirely.
  - Loads libraries `build/source/libiAnt_controller` and `build/source/libiAnt_loop_functions`.
  - Spawns a single `foot-bot` by default (the RL API currently controls the first robot).
  - Keeps CPFA/environment parameters (nest, food radius, distribution) used by the environment.
  - Adds required sensors (`footbot_proximity`, `positioning`, `footbot_motor_ground`, `footbot_light`) and differential steering actuator.

### Build

```bash
cd argos
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

If `pybind11` is found, this produces a Python extension `iant_rl` in `argos/build` and shared libs in `argos/build/source`.

### Python usage

Minimal script (`scripts/test.py`):
```python
import os, sys

ARGOS_DIR = "/home/tameem/Tameem-Work/neatfa-tameem/argos"  # adjust to your path
sys.path.append(os.path.join(ARGOS_DIR, "build"))
import iant_rl

xml = os.path.join(ARGOS_DIR, "experiments", "iAnt_rl.xml")

# chdir so XML can resolve relative library paths like build/source/lib...
```

### Training Algorithms

#### DQN (Deep Q-Network)
- **File**: `scripts/train_dqn.py`
- **Architecture**: Simple MLP with Q-value outputs
- **Action Selection**: Epsilon-greedy exploration
- **Experience Replay**: Uses replay buffer for stable training
- **Usage**: `python scripts/train_dqn.py`

#### PPO (Proximal Policy Optimization)
- **File**: `scripts/train_ppo.py`
- **Architecture**: Actor-Critic with shared feature layers
- **Action Selection**: Stochastic policy with entropy bonus
- **Advantages**: GAE (Generalized Advantage Estimation)
- **Stability**: Clipped objective prevents large policy updates
- **Usage**: `python scripts/train_ppo.py`

### Evaluation

#### DQN Evaluation
- **File**: `scripts/eval_greedy.py`
- **Purpose**: Load trained DQN checkpoint and run with visualization
- **Usage**: `python scripts/eval_greedy.py`

#### PPO Evaluation
- **File**: `scripts/eval_ppo.py`
- **Purpose**: Load trained PPO checkpoint and run with visualization
- **Usage**: `python scripts/eval_ppo.py`

### Key Differences: DQN vs PPO

| Aspect | DQN | PPO |
|--------|-----|-----|
| **Algorithm Type** | Value-based | Policy-based |
| **Action Space** | Discrete (15 actions) | Discrete (15 actions) |
| **Wheel Speeds** | Full range [-16, 16] for exploration | Full range [-16, 16] for exploration |
| **Exploration** | Epsilon-greedy | Stochastic policy |
| **Training Stability** | Moderate (experience replay) | High (clipped updates) |
| **Sample Efficiency** | Lower | Higher |
| **Hyperparameter Sensitivity** | High | Low |
| **Best For** | Simple discrete control | Complex discrete control |
cwd = os.getcwd()
os.chdir(ARGOS_DIR)
try:
    env = iant_rl.IAntRLEnv(xml)
    obs = env.reset()
    obs, reward, done, trunc, info = env.step(0.0, 0.0, False)
    print(obs)
finally:
    env.close()
    os.chdir(cwd)
```

To run without changing directories, make the `library` attributes in the XML absolute paths to the built shared libraries.

### Multi-robot and visualization

- Multi-robot: increase `entity quantity` in the XML. The RL API currently addresses the first `foot-bot` as the primary controller; extending to multi-agent is straightforward by exposing per-robot actions/observations.
- Visualization: The Python wrapper loads ARGoS with `b_force_no_viz = true` for faster training. This can be made optional if you want to render episodes.
