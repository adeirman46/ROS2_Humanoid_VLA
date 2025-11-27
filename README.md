# Unitree G1 Humanoid Robot - ROS2

Complete URDF model of the Unitree G1 humanoid robot (29 DOF) for ROS2 Humble visualization and control.

## Overview

This package provides a complete ROS2 setup for visualizing and controlling the **Unitree G1** humanoid robot in RViz with interactive joint control.

**Features:**
- 29 degrees of freedom (arms, legs, articulated hands)
- High-quality 3D meshes (60+ STL files)
- Interactive GUI with sliders for all joints
- Ready-to-use launch files
- RViz pre-configured visualization

---

## Prerequisites

- **Linux** (Ubuntu 24.04+)
- **Micromamba** or Conda
- **ROS2 Humble** (via RoboStack)

---

## Installation

### 1. Activate ROS2 Environment

```bash
micromamba activate ros2_env
```

### 2. Install Dependencies

```bash
micromamba install -y \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  ros-humble-xacro \
  -c conda-forge -c robostack-staging
```

**Required packages:**
- `robot-state-publisher` - Publishes robot state to TF tree
- `joint-state-publisher` - Publishes joint states
- `joint-state-publisher-gui` - Interactive GUI with sliders
- `rviz2` - 3D visualization
- `xacro` - URDF processing

### 3. Clone Repository

```bash
cd ~/Documents/HUMA
git https://github.com/adeirman46/ROS2_Humanoid_VLA
```

### 4. Build Workspace

```bash
colcon build --symlink-install
```

Expected output: Build completes in ~1-2 seconds (CMake deprecation warning is harmless)

### 5. Source Workspace

```bash
source install/setup.bash
```

---

## Running the Robot

### Quick Launch (Use the Script)

```bash
./launch_g1.sh
```

### Manual Launch

```bash
# Make sure you're in the workspace and environment is activated
micromamba activate ros2_env
cd ~/Documents/HUMA/ws_g1
source install/setup.bash

# Launch the robot
ros2 launch g1_package g1_robot.launch.xml
```

### What You'll See

Two windows will open:

1. **RViz** - 3D visualization showing the Unitree G1 robot with all body parts
2. **Joint State Publisher GUI** - Control panel with 29 sliders for each joint

**Usage:**
- Move sliders in the GUI
- Watch the robot update in real-time in RViz
- Experiment with different poses

---

## Robot Details

### Technical Specifications

| Specification | Details |
|---------------|---------|
| **Total DOF** | 29 degrees of freedom |
| **Arms** | 7 DOF each (shoulder: pitch/roll/yaw, elbow, wrist: yaw/roll/pitch) |
| **Legs** | 6 DOF each (hip: yaw/roll/pitch, knee, ankle: pitch/roll) |
| **Hands** | Articulated fingers with individual thumb, index, and middle finger control |
| **Meshes** | 60+ high-quality STL files |
| **Format** | Standard ROS2 URDF |

### Joint Groups

**Left & Right Arms (7 DOF each):**
- shoulder_pitch, shoulder_roll, shoulder_yaw
- elbow
- wrist_yaw, wrist_roll, wrist_pitch

**Left & Right Legs (6 DOF each):**
- hip_yaw, hip_roll, hip_pitch
- knee
- ankle_pitch, ankle_roll

**Left & Right Hands:**
- hand_thumb_0, hand_thumb_1, hand_thumb_2
- hand_index_0, hand_index_1
- hand_middle_0, hand_middle_1

### Package Structure

```
ws_g1/
├── README.md                    # This file
├── launch_g1.sh                # Quick launch script
├── g1_package/                 # ROS2 package
│   ├── launch/
│   │   └── g1_robot.launch.xml # Main launch file
│   ├── urdf/
│   │   ├── g1_29dof.urdf      # Robot description
│   │   └── g1.rviz            # RViz configuration
│   └── meshes/                # 3D mesh files (STL)
├── build/                      # Build artifacts
├── install/                    # Installation files
└── log/                        # Build logs
```

---

## ROS2 Topics

When running, the following topics are active:

```bash
# View all topics
ros2 topic list

# Key topics:
/robot_description    # URDF model
/joint_states        # Current joint positions
/tf                  # Transform tree
/tf_static           # Static transforms
```

**Inspect joint states:**
```bash
ros2 topic echo /joint_states
```

---

## Troubleshooting

### "Package 'g1_package' not found"
```bash
source install/setup.bash
```

### Missing Dependencies
```bash
micromamba activate ros2_env
micromamba install -y ros-humble-joint-state-publisher-gui -c robostack-staging
```

### Rebuild Workspace
```bash
cd ~/Documents/HUMA/ws_g1
colcon build --symlink-install
source install/setup.bash
```

### Display Issues (SSH/Remote)
```bash
# Enable X11 forwarding
ssh -X user@host

# Or set display
export DISPLAY=:0
```

---

## Next Steps

**Motion Planning:**
- Integrate with MoveIt2 for advanced trajectory planning

**Physics Simulation:**
- Add collision/inertial properties to URDF
- Set up Gazebo simulation

**Custom Controllers:**
- Write programmatic joint control
- Publish to `/joint_states` topic

**Real Robot:**
- Use URDF as reference model with actual hardware
- Implement robot drivers

---

**Happy Robotics! 🤖**
