#!/bin/bash

# Unitree G1 Gazebo House Simulation Launch Script
# This script activates the ROS2 environment and launches the robot in a house environment

set -e  # Exit on error

echo "========================================="
echo "  Unitree G1 - Gazebo House Simulation"
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
echo "  Launching G1 Robot in House World..."
echo "========================================="
echo ""
echo "Expected windows:"
echo "  1. Gazebo - 3D simulation with house environment"
echo "  2. Robot will spawn inside the house"
echo ""
echo "Robot spawn position:"
echo "  - X: 0.0, Y: 0.0, Z: 0.5 (center of house, 0.5m above ground)"
echo ""
echo "Available launch arguments:"
echo "  spawn_x:=<x>     - X position (default: 0.0)"
echo "  spawn_y:=<y>     - Y position (default: 0.0)"
echo " spawn_z:=<z>     - Z position (default: 0.5)"
echo "  spawn_yaw:=<rad> - Yaw orientation (default: 0.0)"
echo "  rviz:=true       - Also launch RViz (default: false)"
echo ""
echo "Use CTRL+C to stop the simulation"
echo ""

# Launch the gazebo house simulation
ros2 launch g1_gazebo_sim gazebo_house.launch.py
