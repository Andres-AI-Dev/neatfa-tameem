#!/bin/bash

# Build and run script for ROS2-ARGoS PPO training

set -e  # Exit on error

echo "======================================"
echo "ROS2-ARGoS PPO Environment Setup"
echo "======================================"

# Check for Docker
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed. Please install Docker Desktop for macOS."
    echo "Visit: https://www.docker.com/products/docker-desktop/"
    exit 1
fi

# Check for XQuartz (needed for GUI on macOS)
if [[ "$OSTYPE" == "darwin"* ]]; then
    if ! command -v xquartz &> /dev/null && ! [ -d "/Applications/Utilities/XQuartz.app" ]; then
        echo "Warning: XQuartz not found. GUI visualization may not work."
        echo "Install XQuartz for visualization: brew install --cask xquartz"
    else
        echo "✓ XQuartz detected for GUI support"
        # Allow connections from localhost
        xhost +localhost 2>/dev/null || true
    fi
fi

# Build or rebuild Docker image
echo ""
echo "Building Docker image..."
echo "This may take 10-15 minutes on first build..."

docker-compose build

echo "✓ Docker image built successfully"

# Create necessary directories
mkdir -p ppo_implementation/{logs,models}

echo ""
echo "======================================"
echo "Setup Complete!"
echo "======================================"
echo ""
echo "To start training, run one of these commands:"
echo ""
echo "1. Interactive shell (for development):"
echo "   docker-compose run --rm ros2-argos-ppo"
echo ""
echo "2. Direct training (headless):"
echo "   docker-compose run --rm ros2-argos-ppo python3 /root/ppo_argos/scripts/train_ppo.py"
echo ""
echo "3. With visualization (requires XQuartz):"
echo "   docker-compose run --rm -e DISPLAY=host.docker.internal:0 ros2-argos-ppo"
echo ""
echo "======================================"