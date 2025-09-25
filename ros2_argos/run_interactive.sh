#!/bin/bash

echo "Starting interactive ROS2-ARGoS-PPO Docker session..."
echo ""
echo "This will open a shell in the container where you can test the pipeline manually."
echo ""

# Make sure XQuartz is running (for macOS)
if [[ "$OSTYPE" == "darwin"* ]]; then
    if [ -d "/Applications/Utilities/XQuartz.app" ]; then
        echo "Allowing X11 connections from localhost..."
        xhost +localhost 2>/dev/null || true
    else
        echo "WARNING: XQuartz not found. ARGoS visualization will not work."
        echo "Install XQuartz from https://www.xquartz.org"
    fi
fi

echo ""
echo "Starting Docker container..."
echo ""

docker run -it --rm \
    --name ros2_argos_interactive \
    -e DISPLAY=host.docker.internal:0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$(pwd)/ppo_implementation:/root/ppo_argos" \
    -v "$(pwd)/../argos/experiments:/root/argos_experiments" \
    ros2-argos-ppo:latest \
    /bin/bash

echo "Container exited."