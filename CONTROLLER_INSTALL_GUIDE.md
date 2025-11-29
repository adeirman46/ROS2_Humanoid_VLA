# ROS2 Controllers Installation Guide

## Problem

The ROS2 control plugins (`joint_state_broadcaster` and `joint_trajectory_controller`) are not available in the `ros2_env` micromamba environment, causing this error:

```
[ERROR] [controller_manager]: Loader for controller 'joint_state_broadcaster' not found.
[ERROR] [controller_manager]: Loader for controller 'position_trajectory_controller' not found.
```

## Solution Options

### Option 1: Build from Source (Recommended for your setup)

Clone and build the ros2_controllers package in your workspace:

```bash
cd /home/irman/ROS2_Humanoid_VLA/src

# Clone ros2_controllers repository
git clone https://github.com/ros-controls/ros2_controllers.git -b humble

# Build the package
cd /home/irman/ROS2_Humanoid_VLA
eval "$(micromamba shell hook --shell bash)"
micromamba activate ros2_env
colcon build --packages-select joint_state_broadcaster joint_trajectory_controller --symlink-install

# Source the workspace
source install/setup.bash
```

### Option 2: Install from ROS apt repository (if available)

If you have ROS2 Humble installed system-wide via apt:

```bash
sudo apt update
sudo apt install ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller
```

Then source the system ROS2 installation in your launch scripts:

```bash
source /opt/ros/humble/setup.bash
```

### Option 3: Use System ROS2 (Quick workaround)

If ROS2 Humble is installed system-wide, modify the launch scripts to source it:

Edit `launch_wasd_control.sh` and add before the workspace sourcing:

```bash
# Source system ROS2 (if available)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
```

## Verification

After installing the controllers, verify they are available:

```bash
ros2 pkg list | grep -E "(joint_state_broadcaster|joint_trajectory_controller)"
```

You should see:
```
joint_state_broadcaster
joint_trajectory_controller
```

Check available controller types:

```bash
ros2 control list_controller_types
```

You should see (among others):
```
joint_state_broadcaster/JointStateBroadcaster
joint_trajectory_controller/JointTrajectoryController
```

## Next Steps

Once the controllers are installed:

1. Try the new separate terminal launch script:
   ```bash
   ./launch_wasd_separate_terminal.sh
   ```

2. Or use the original single-terminal version:
   ```bash
   ./launch_wasd_control.sh
   ```

The separate terminal script opens the WASD keyboard control in a new window, making it easier to see keyboard feedback.
