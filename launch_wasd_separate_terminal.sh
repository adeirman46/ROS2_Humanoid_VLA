#!/bin/bash

# Unitree G1 with WASD Keyboard Control - Separate Terminal Launch Script
# This launches Gazebo in one terminal and WASD controller in a new terminal

set -e  

echo "========================================="
echo "  Unitree G1 - WASD Control (Split Terminals)"
echo "========================================="
echo ""

# Check if micromamba is available
if ! command -v micromamba &> /dev/null; then
    echo "❌ Error: micromamba not found"
    exit 1
fi

echo "✓ Found micromamba"

# Get the workspace directory
WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "🔄 Activating ros2_env environment..."
eval "$(micromamba shell hook --shell bash)"
micromamba activate ros2_env

echo "🔄 Sourcing workspace..."
source "$WORKSPACE_DIR/install/setup.bash"

echo ""
echo "========================================="
echo "  Launching System in 2 Terminals"
echo "========================================="
echo ""
echo "Terminal 1 (this one): Gazebo + Controllers"
echo "Terminal 2 (new): WASD Keyboard Control"
echo ""
echo "Timeline:"
echo "  0s  - Gazebo starts with robot"
echo "  5s  - Robot spawns"
echo "  8s  - Joint state broadcaster loads"
echo "  10s - Position controller loads"
echo "  14s - WASD Controller opens in new terminal"
echo ""
echo "Keyboard Controls (in new terminal):"
echo "  W - Walk Forward"
echo "  S - Walk Backward (Reverse)"
echo "  A - Turn Left (Counterclockwise)"
echo "  D - Turn Right (Clockwise)"
echo "  SPACE - Stand Pose"
echo "  ESC - Quit"
echo ""
echo "Use CTRL+C in THIS terminal to stop everything"
echo ""

# Launch Gazebo and controllers (this terminal)
ros2 launch g1_gazebo_sim gazebo_robot_ros2_control.launch.py &
GAZEBO_PID=$!

# Wait a bit for Gazebo to start
sleep 3

# Spawn controllers
sleep 5
echo "🔧 Spawning joint state broadcaster..."
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager &
BROADCASTER_PID=$!

sleep 2
echo "🔧 Spawning position trajectory controller..."
ros2 run controller_manager spawner position_trajectory_controller --controller-manager /controller_manager &
CONTROLLER_PID=$!

# Wait for controllers to initialize
sleep 4

# Launch WASD controller in a new terminal
echo "🎮 Opening WASD controller in new terminal..."
if command -v gnome-terminal &> /dev/null; then
    gnome-terminal -- bash -c "cd '$WORKSPACE_DIR' && eval \"\$(micromamba shell hook --shell bash)\" && micromamba activate ros2_env && source install/setup.bash && ros2 run g1_controller wasd_controller.py; exec bash"
elif command -v xterm &> /dev/null; then
    xterm -e "cd '$WORKSPACE_DIR' && eval \"\$(micromamba shell hook --shell bash)\" && micromamba activate ros2_env && source install/setup.bash && ros2 run g1_controller wasd_controller.py; exec bash" &
elif command -v konsole &> /dev/null; then
    konsole -e bash -c "cd '$WORKSPACE_DIR' && eval \"\$(micromamba shell hook --shell bash)\" && micromamba activate ros2_env && source install/setup.bash && ros2 run g1_controller wasd_controller.py; exec bash" &
else
    echo "❌ Error: No compatible terminal emulator found (tried gnome-terminal, xterm, konsole)"
    echo "Please install one of these terminal emulators"
    kill $GAZEBO_PID $BROADCASTER_PID $CONTROLLER_PID 2>/dev/null
    exit 1
fi

echo ""
echo "========================================="
echo "  System Running"
echo "========================================="
echo "Monitor this terminal for system messages"
echo "Use the new terminal for WASD keyboard control"
echo "Press CTRL+C here to stop everything"
echo ""

# Wait for gazebo process
wait $GAZEBO_PID
