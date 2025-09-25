# Testing the PPO-ARGoS-ROS2 Pipeline

## Prerequisites
- Docker container is running (`root@f6cb392634c5` or similar)
- You're in the Docker terminal
- The ROS2 package has been built successfully

## Commands to test with a fresh container:

1. Start a fresh container (in your host terminal):
  
  cd /Users/andres/Dev/fl-research/RL-FL-Andres/ros2_argos
  ./run_interactive.sh

2. Run the setup script (in the new container):
  
  /root/ppo_argos/setup_container.sh

## How to create Docker Terminal:
docker exec -it ros2_argos_interactive /bin/bash

## Step-by-Step Testing Guide

### 1. Initial Setup (In Docker Terminal)

```bash
# Source ROS2 and your workspace
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Set PKG_CONFIG_PATH for ARGoS
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH

# Verify the controller library exists
ls -la /root/ros2_ws/install/ppo_argos_ros2/lib/libros2_iant_controller.so
```

### 2. Test Basic ROS2 Communication

```bash
# Check ROS2 is working
ros2 topic list
# Should show: /parameter_events, /rosout
```

### 3. Test PPO Model Loading (Simple Test)

First, let's verify the PPO model can be loaded:

```bash
# Test if model loads correctly
cd /root/ppo_argos
python3 -c "
import torch
import torch.nn as nn
import torch.nn.functional as F

class ActorCritic(nn.Module):
    def __init__(self, obs_dim=15, num_actions=15):
        super().__init__()
        self.feature_net = nn.Sequential(
            nn.Linear(obs_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
        )
        self.actor = nn.Sequential(
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, num_actions)
        )
        self.critic = nn.Sequential(
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )
    def forward(self, obs):
        features = self.feature_net(obs)
        action_probs = F.softmax(self.actor(features), dim=-1)
        value = self.critic(features)
        return action_probs, value

model = ActorCritic()
checkpoint = torch.load('/root/ppo_argos/ppo_final.pt', map_location='cpu')
model.load_state_dict(checkpoint['model_state_dict'])
print('✓ Model loaded successfully!')
print(f'  Episode: {checkpoint[\"episode\"]}')
print(f'  Model has {sum(p.numel() for p in model.parameters())} parameters')
"
```

### 4. Test ARGoS with ROS2 Controller (Without Visualization)

Since XQuartz isn't installed, we'll run ARGoS without GUI:

```bash
# Create a simple test experiment file
cat > /root/ppo_argos/test_headless.argos << 'EOF'
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
        <footbot_proximity implementation="default" show_rays="false" />
        <footbot_light implementation="default" show_rays="false" />
        <footbot_motor_ground implementation="rot_z_only" />
        <positioning implementation="default" />
      </sensors>
      <params />
    </ros2_iant_controller>
  </controllers>

  <arena size="10, 10, 1" center="0,0,0.5">
    <floor id="floor" source="image" path="/root/ppo_argos/floor.png" />

    <box id="wall_north" size="10,0.1,0.5" movable="false">
      <body position="0,5,0" orientation="0,0,0" />
    </box>
    <box id="wall_south" size="10,0.1,0.5" movable="false">
      <body position="0,-5,0" orientation="0,0,0" />
    </box>
    <box id="wall_east" size="0.1,10,0.5" movable="false">
      <body position="5,0,0" orientation="0,0,0" />
    </box>
    <box id="wall_west" size="0.1,10,0.5" movable="false">
      <body position="-5,0,0" orientation="0,0,0" />
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

  <!-- No visualization -->
</argos-configuration>
EOF

# Create a simple floor image
python3 -c "
from PIL import Image
import numpy as np
img = Image.fromarray(np.ones((100, 100, 3), dtype=np.uint8) * 255)
img.save('/root/ppo_argos/floor.png')
print('Created floor.png')
"

# Run ARGoS in headless mode (will likely fail without proper loop functions)
# This is just to test if the controller loads
timeout 5 argos3 -c /root/ppo_argos/test_headless.argos 2>&1 | grep -E "ROS2|controller|ERROR|WARNING" || true
```

### 5. Full Integration Test (Requires 3 Terminals)

You'll need three terminal windows, all connected to the Docker container.

#### Terminal 1: Run ARGoS
```bash
# In first Docker terminal
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH

# Run ARGoS (will wait for ROS2 commands)
argos3 -c /root/ppo_argos/test_headless.argos
```

#### Terminal 2: Run PPO ROS2 Node
Open a new terminal on your Mac and connect to the container:
```bash
# On Mac terminal
docker exec -it ros2_argos_interactive /bin/bash
```

Then in the new Docker terminal:
```bash
# In second Docker terminal
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Run the PPO controller
cd /root/ppo_argos
python3 ppo_ros2_node.py
```

#### Terminal 3: Monitor ROS2 Topics
Open another Mac terminal and connect:
```bash
# On Mac terminal
docker exec -it ros2_argos_interactive /bin/bash
```

Then monitor:
```bash
# In third Docker terminal
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# List topics
ros2 topic list

# You should see:
# /iant/fb_0/observation
# /iant/fb_0/action
# /parameter_events
# /rosout

# Monitor observations being published
ros2 topic echo /iant/fb_0/observation

# Monitor actions being sent
ros2 topic echo /iant/fb_0/action

# Check message rates
ros2 topic hz /iant/fb_0/observation
ros2 topic hz /iant/fb_0/action
```

### 6. Verify Communication Flow

If everything is working:
1. ARGoS publishes observations to `/iant/fb_0/observation`
2. PPO node receives observations and publishes actions to `/iant/fb_0/action`
3. ARGoS receives actions and moves the robot
4. You'll see messages flowing in Terminal 3

### Troubleshooting

If ARGoS crashes immediately:
```bash
# It might need loop functions. Try with a simpler controller test:
argos3 -c /root/ppo_argos/test_headless.argos 2>&1 | head -50
```

If no topics appear:
```bash
# Check if controller library is found
ldd /root/ros2_ws/install/ppo_argos_ros2/lib/libros2_iant_controller.so | grep "not found"
```

If PPO node fails:
```bash
# Check model path
ls -la /root/ppo_argos/ppo_final.pt

# Test model loading separately
python3 -c "import torch; print(torch.load('/root/ppo_argos/ppo_final.pt', map_location='cpu').keys())"
```

### Success Criteria

✅ ROS2 topics `/iant/fb_0/observation` and `/iant/fb_0/action` appear
✅ Observations are published at ~10Hz (ARGoS tick rate)
✅ PPO node loads model without errors
✅ Actions are being sent in response to observations
✅ No crashes or error messages

## Next Steps

Once basic communication works:
1. Create proper ARGoS experiment with food items
2. Add loop functions for foraging task
3. Test with visualization (requires XQuartz installation)
4. Measure performance metrics
5. Retrain model with better reward shaping if needed