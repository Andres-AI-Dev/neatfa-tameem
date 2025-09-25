#!/bin/bash

echo "Starting ROS2-ARGoS-PPO test in Docker container..."

# Make sure XQuartz is running (for macOS)
if [[ "$OSTYPE" == "darwin"* ]]; then
    # Check if XQuartz is installed
    if [ -d "/Applications/Utilities/XQuartz.app" ]; then
        # Allow connections from localhost
        xhost +localhost 2>/dev/null || true
    else
        echo "WARNING: XQuartz not found. ARGoS visualization will not work."
        echo "Install XQuartz from https://www.xquartz.org"
    fi
fi

# Run the Docker container with proper mounting and display settings
docker run -it --rm \
    --name ros2_argos_test \
    -e DISPLAY=host.docker.internal:0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$(pwd)/ppo_implementation:/root/ppo_argos" \
    -v "$(pwd)/../argos/experiments:/root/argos_experiments" \
    ros2-argos-ppo:latest \
    /bin/bash -c "chmod +x /root/ppo_argos/test_ros2_pipeline.sh && /root/ppo_argos/test_ros2_pipeline.sh"