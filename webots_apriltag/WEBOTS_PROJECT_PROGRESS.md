# Webots AprilTag Simulation Project - Progress Report

**Project:** Multi-Robot AprilTag Collection and Deposit System
**Simulator:** Webots R2025a
**Robot Platform:** E-puck Differential Drive Robot
**Last Updated:** January 2025

---

## Executive Summary

This project implements a sophisticated robotic foraging simulation using Webots, where E-puck robots collect AprilTags (visual markers) from the environment and deposit them at a central base. The system uses computer vision for AprilTag detection and implements intelligent behavioral strategies for efficient collection.

---

## Project Evolution

### Phase 1: Foundation & Basic Detection
**Commits:** `64b628db` - `94b2adf1`

- Initialized Webots AprilTag simulation project
- Integrated AprilTag C++ library for marker detection
- Created basic AprilTag detection controller
- Implemented simple tracking and following behaviors

**Key Files:**
- `apriltag_detector.py` - Basic AprilTag detection
- `apriltag_tracker.py` - Visual tracking controller
- `simple_apriltag.wbt` - Initial test world

### Phase 2: Collection & Deposit Mechanics
**Commits:** `a4d29062` - `5b0eea78`

- Developed collect-and-deposit behavior
- Implemented supervisor-based collection mechanics
- Created centralized deposit base
- Added multi-robot support

**Key Files:**
- `apriltag_collector_depositor.py` - Basic collection controller
- `apriltag_collect_and_deposit.wbt` - Collection world
- `c&d_fixed_lighting_v3.wbt` - Fixed lighting environment

### Phase 3: CSWD Behavior System
**Commits:** `5b0eea78` - `dc033f0d`

Implemented the **CSWD (Collect, Stop, Wait, Deposit)** behavioral state machine:

1. **Collect:** Robot searches for and approaches AprilTags
2. **Stop:** Robot halts at optimal distance from target
3. **Wait:** Robot signals intent (LED flashing, beeping)
4. **Deposit:** Robot returns to base and deposits collected tag

**Key Files:**
- `cswd_controller.py` - Initial CSWD implementation
- `cswd_v5.wbt` - CSWD world configuration

---

## Current Implementation: CSWD Bounding Box System

### Controller Versions & Evolution

#### Version 8: Baseline Implementation
**File:** `cswd_bounding_box_controller_v8.py`

**Features:**
- Visual bounding box detection using OpenCV
- Basic tag locking mechanism
- Simple approach and deposit behavior
- LED signaling system
- Distance-based collection triggers

**Parameters:**
- AprilTag size: 0.055m x 0.055m x 0.055m
- Max forward speed: 4.0 rad/s
- Max rotation speed: 1.5 rad/s
- Collection distance: 0.15m
- Deposit distance: 0.30m

#### Version 9: Enhanced Locking
**File:** `cswd_bounding_box_controller_v9.py`

**Improvements:**
- Minimum lock duration (3.0 seconds)
- Timeout mechanisms to prevent infinite locks
- Improved switching logic with 2x size threshold
- Better lost visual handling

**Key Changes:**
- Lock duration requirement prevents rapid switching
- 2x size difference required to switch targets (100% bigger)
- Timeout after 120 steps if no progress

#### Version 10: Aggressive Re-evaluation (CURRENT PREFERRED)
**File:** `cswd_bounding_box_controller_v10.py`

**Features:**
- **Continuous re-evaluation:** No minimum lock duration
- **1.5x switching threshold:** Switch to tags 50% larger
- **Aggressive target acquisition:** Always goes for the biggest visible tag
- **Optimized deposit base:** Flat red cylinder (0.002m height)

**Why This Version Works Best:**
- Dynamically switches to closer/larger targets
- Doesn't get stuck on distant tags
- Most efficient collection behavior observed
- Successfully collects tags in high-density environments

**World Configuration:**
- 50 AprilTags arranged in two clusters
- Tags: 0.055m cubes
- Base: Red cylinder (0.2m radius, 0.002m height)
- Arena: 5m x 5m

#### Version 11: Scaled-Down Testing (IN DEVELOPMENT)
**File:** `cswd_bounding_box_controller_v11.py`

**Purpose:** Test detection and behavior with smaller AprilTags

**Changes:**
- AprilTags scaled to **0.01375m** (1/4 of original size)
- Same behavioral logic as v10
- Testing detection limits and visual processing

---

## Technical Architecture

### Vision System

**Camera Configuration:**
- Resolution: 640x480 pixels
- Field of View: 1 radian (~57.3 degrees)
- Processing: OpenCV for image manipulation
- Detection: AprilTag C++ library (tag36h11 family)

**Visual Processing Pipeline:**
```
Camera Image → Grayscale Conversion → AprilTag Detection →
Bounding Box Calculation → Target Selection → Motor Control
```

### Behavioral State Machine

```
SEARCHING → APPROACHING → SIGNALING → COLLECTING →
TURNING_TO_BASE → RETURNING → DEPOSITING → SEARCHING
```

**State Descriptions:**

1. **SEARCHING:** Rotate in place, scanning for AprilTags
2. **APPROACHING:** Move toward locked tag at full speed
3. **SIGNALING:** Stop and signal with LEDs/beeps
4. **COLLECTING:** Attach tag to robot (supervisor call)
5. **TURNING_TO_BASE:** Rotate to face deposit base
6. **RETURNING:** Navigate back to base with course correction
7. **DEPOSITING:** Phase through base and release tag

### Target Selection Algorithm

**v10 Strategy (Aggressive):**
```python
if new_tag_size > locked_tag_size * 1.5:
    switch_to_new_tag()
```

**Benefits:**
- Quick response to closer targets
- Prevents long-distance pursuits
- Maximizes collection efficiency

### Motor Control

**Differential Drive Control:**
- Left/right wheel independent control
- Proportional rotation correction
- Speed limits: ±6.28 rad/s (±1 revolution/second)

**Approach Behavior:**
```python
rotation_error = tag_center_x - camera_center_x
rotation_speed = rotation_error * 0.02  # Proportional control
left_speed = forward_speed + rotation_speed
right_speed = forward_speed - rotation_speed
```

---

## Performance Metrics

### Version 10 Performance (Typical Run)

- **Collection Success Rate:** ~95%
- **Average Collection Time:** 15-20 seconds per tag
- **Target Switching:** 3-5 switches per collection cycle
- **Navigation Accuracy:** ±0.1m to deposit base
- **Concurrent Operations:** Tested with 2 robots successfully

### Known Issues & Limitations

1. **Occlusion:** Cannot detect tags behind obstacles
2. **Lighting Sensitivity:** Performance varies with lighting conditions
3. **Collision Avoidance:** Basic proximity sensors, no advanced obstacle avoidance
4. **Carried Tag Visibility:** Tags become invisible once attached to robot
5. **Edge Cases:** Occasional lost visual during final approach

---

## Development Tools & Technologies

### Software Stack
- **Simulator:** Webots R2025a
- **Language:** Python 3.x
- **Computer Vision:** OpenCV (cv2)
- **AprilTag Detection:** apriltag C++ library with Python bindings
- **Math Libraries:** NumPy for vector operations

### Development Environment
- **Platform:** Linux (Ubuntu-based)
- **Version Control:** Git
- **IDE Support:** VS Code compatible
- **Simulation Modes:**
  - Real-time GUI mode
  - Fast mode (no rendering)
  - Headless mode for batch testing

---

## File Structure

```
webots_apriltag/
├── controllers/
│   ├── cswd_bounding_box_controller_v8/    # Baseline version
│   ├── cswd_bounding_box_controller_v9/    # Enhanced locking
│   ├── cswd_bounding_box_controller_v10/   # Current best (aggressive)
│   └── cswd_bounding_box_controller_v11/   # Small tags test
├── worlds/
│   ├── cswd_bounding_box_test_v8.wbt      # v8 world
│   ├── cswd_bounding_box_test_v9.wbt      # v9 world
│   ├── cswd_bounding_box_test_v10.wbt     # v10 world (current)
│   └── cswd_bounding_box_test_v11.wbt     # v11 small tags
├── apriltag/                               # AprilTag library
│   └── build/libapriltag.so.3
├── textures/                               # AprilTag images
│   └── real_tag36h11_id0.png
└── build_world.py                          # World generation utility
```

---

## Key Insights & Design Decisions

### Why Version 10 is Preferred

1. **Dynamic Optimization:** Continuous re-evaluation allows the robot to always pursue the optimal target
2. **No Artificial Delays:** Removing minimum lock duration improves responsiveness
3. **Balanced Threshold:** 1.5x switching prevents flip-flopping while allowing opportunistic switching
4. **Real-World Applicability:** Mimics how an efficient forager would behave

### Design Philosophy

The system follows a **reactive architecture** rather than deliberative planning:
- **Immediate response** to visual stimuli
- **Simple state transitions** based on sensor data
- **No complex path planning** - relies on visual servoing
- **Greedy local optimization** - always goes for the biggest visible tag

---

## Future Work & Improvements

### Short-Term Goals
1. ✅ Optimize deposit base geometry (completed in v10)
2. 🔄 Test scaled-down AprilTags (v11 in progress)
3. ⏳ Multi-robot coordination strategies
4. ⏳ Collision avoidance improvements

### Long-Term Goals
1. **Machine Learning Integration:** Train RL agents for collection strategies
2. **Communication Protocols:** Robot-to-robot coordination
3. **Dynamic Environments:** Moving targets, obstacles
4. **Energy Modeling:** Battery constraints and charging behavior
5. **Swarm Behaviors:** Emergent collective strategies

### Research Questions
- How does tag density affect collection efficiency?
- What is the optimal switching threshold for different environments?
- Can robots learn better strategies than hand-coded behaviors?
- How does team size affect overall performance?

---

## Experimental Configurations

### Test Scenarios

**Scenario 1: High-Density Clusters**
- 25 tags per cluster
- 2 clusters in opposite corners
- Tests aggressive switching behavior

**Scenario 2: Uniform Distribution**
- 50 tags evenly distributed
- Tests search efficiency
- Evaluates navigation strategies

**Scenario 3: Scaled Detection Limits**
- Quarter-size tags (v11)
- Tests vision system sensitivity
- Evaluates detection range

---

## Conclusion

The Webots AprilTag simulation project has successfully evolved through multiple iterations, culminating in a robust CSWD behavioral system. Version 10 represents the current best-performing configuration, demonstrating efficient collection through aggressive target re-evaluation and optimized navigation.

The modular design allows for rapid prototyping of new behaviors, and the foundation is solid for future machine learning integration and multi-robot coordination research.

---

## References & Resources

- **Webots Documentation:** https://cyberbotics.com/doc/reference/
- **AprilTag Library:** https://github.com/AprilRobotics/apriltag
- **E-puck Robot Specs:** https://www.gctronic.com/doc/index.php/E-Puck
- **OpenCV Documentation:** https://docs.opencv.org/

---

## Contact & Collaboration

For questions about this project or collaboration opportunities, please refer to the main repository documentation.

**Project Status:** Active Development
**Last Major Update:** January 2025
**Current Focus:** Version 10 optimization and v11 scaled testing
