#!/bin/bash

# Unitree G1 with GUI Controller - Launch Script

set -e  # Exit on error

echo "========================================="
echo "  Unitree G1 - GUI Controller"
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
echo "  Launching G1 with GUI Controller"
echo "========================================="
echo ""
echo "This will:"
echo "  1. Start Gazebo with the G1 robot"
echo "  2. After 7 seconds, open GUI controller window"
echo ""
echo "GUI Controls:"
echo "  🧍 Stand          - Standing pose"
echo "  🚶 Walk Forward   - Walking motion"
echo "  ↻ Turn CW        - Turn clockwise"
echo "  ↺ Turn CCW       - Turn counter-clockwise"
echo ""
echo "Use CTRL+C to stop (or click Quit in GUI)"
echo ""

# Launch the robot with GUI controller
ros2 launch g1_controller robot_with_gui.launch.py
