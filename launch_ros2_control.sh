#!/bin/bash

# Unitree G1 with ROS2 Control - Launch Script

set -e  

echo "========================================="
echo "  Unitree G1 - ROS2 Control System"
echo "========================================="
echo ""

# Check if micromamba is available
if ! command -v micromamba &> /dev/null; then
    echo "❌ Error: micromamba not found"
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
echo "  Starting System Components"
echo "========================================="
echo ""
echo "Timeline:"
echo "  0s  - Gazebo starts with robot"
echo "  5s  - Robot spawns"
echo "  8s  - Joint state broadcaster loads"
echo "  10s - Position controller loads"
echo "  14s - GUI opens"
echo ""
echo "This uses proper ROS2 Control with:"
echo "  ✓ gazebo_ros2_control plugin"
echo "  ✓ JointTrajectory action interface"
echo "  ✓ Position control"
echo ""
echo "Use CTRL+C to stop"
echo ""

# Launch
ros2 launch g1_controller robot_ros2_control_gui.launch.py
