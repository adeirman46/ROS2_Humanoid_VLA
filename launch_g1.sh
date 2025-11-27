#!/bin/bash

# Unitree G1 Quick Start Script
# This script activates the ROS2 environment and launches the robot

set -e  # Exit on error

echo "========================================="
echo "  Unitree G1 Robot - Quick Launch"
echo "========================================="
echo ""

# Check if micromamba is available
if ! command -v micromamba &> /dev/null; then
    echo "❌ Error: micromamba not found"
    echo "Please install micromamba first"
    exit 1
fi

echo "✓ Found micromamba"

# Activate ROS2 environment
echo "🔄 Activating ros2_env environment..."
eval "$(micromamba shell hook --shell bash)"
micromamba activate ros2_env

# Source the workspace
echo "🔄 Sourcing workspace..."
cd "$(dirname "$0")"
source install/setup.bash

echo ""
echo "========================================="
echo "  Launching Unitree G1 Robot..."
echo "========================================="
echo ""
echo "Expected windows:"
echo "  1. RViz - 3D visualization"
echo "  2. Joint State Publisher GUI - Control panel"
echo ""
echo "Use CTRL+C to stop all nodes"
echo ""

# Launch the robot
ros2 launch g1_package g1_robot.launch.xml
