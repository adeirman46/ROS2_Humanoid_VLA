# G1 Gazebo Simulation Package

This package provides Gazebo simulation capabilities for the Unitree G1 humanoid robot in the AWS RoboMaker Small House World environment.

## Features

- **Realistic House Environment**: AWS RoboMaker Small House World with 68 furniture models including sofas, tables, beds, kitchen appliances, and more
- **Multiple Rooms**: Living room, bedroom, kitchen, and hallway
- **Configurable Spawn Location**: Spawn the robot at different positions within the house
- **RViz Integration**: Optional RViz visualization alongside Gazebo

## Quick Start

### Using the Launch Script

The easiest way to launch the simulation:

```bash
./launch_gazebo_house.sh
```

### Manuel Launch

If you prefer to launch manually:

```bash
# Activate environment
eval "$(micromamba shell hook --shell bash)"
micromamba activate ros2_env

# Source workspace
source install/setup.bash

# Launch simulation
ros2 launch g1_gazebo_sim gazebo_house.launch.py
```

## Launch Arguments

You can customize the robot's spawn position and enable RViz:

```bash
ros2 launch g1_gazebo_sim gazebo_house.launch.py spawn_x:=2.0 spawn_y:=1.0 spawn_z:=0.5 rviz:=true
```

Available arguments:
- `spawn_x` (default: 0.0) - X position in meters
- `spawn_y` (default: 0.0) - Y position in meters
- `spawn_z` (default: 0.5) - Z position (height) in meters
- `spawn_yaw` (default: 0.0) - Yaw orientation in radians
- `gui` (default: true) - Start Gazebo GUI
- `rviz` (default: false) - Start RViz
- `use_sim_time` (default: true) - Use simulation time

## Recommended Spawn Locations

To avoid furniture collisions, use these tested spawn positions:

### Living Room (Center)
```bash
spawn_x:=0.0 spawn_y:=0.0 spawn_z:=0.5
```

### Kitchen Area
```bash
spawn_x:=7.0 spawn_y:=-3.0 spawn_z:=0.5 spawn_yaw:=1.57
```

### Bedroom
```bash
spawn_x:=-6.0 spawn_y:=0.0 spawn_z:=0.5
```

### Hallway
```bash
spawn_x:=1.0 spawn_y:=-5.0 spawn_z:=0.5
```

## Package Structure

```
g1_gazebo_sim/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── gazebo_house.launch.py    # Main launch file
├── worlds/
│   └── small_house.world          # Gazebo world file
├── models/                         # 68 furniture models
│   ├── aws_robomaker_residential_Bed_01/
│   ├── aws_robomaker_residential_Sofa_01/
│   ├── ...
├── config/
│   └── gazebo.rviz                # RViz configuration
└── README.md
```

## Troubleshooting

### Models Not Loading

If furniture models don't appear in Gazebo:
1. Check that `GAZEBO_MODEL_PATH` includes the models directory
2. Verify all model directories are present in `g1_gazebo_sim/models/`
3. Try launching with verbose output: `gazebo --verbose`

### Robot Spawns Inside Furniture

The default spawn position (0, 0, 0.5) should be clear, but if you experience collisions:
- Try different spawn positions from the list above
- Increase `spawn_z` to spawn the robot higher
- Use the hallway position which has more open space

### Simulation Runs Slowly

- Reduce physics update rate in the world file
- Disable shadows (`<shadows>false</shadows>` is already set)
- Close unnecessary models or simplify the world

## AWS RoboMaker Small House World

This package uses the [AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world), which is designed for indoor robotics simulation and testing.

### House Layout

The house includes:
- **Living Room**: Sofas, coffee table, TV, carpet
- **Kitchen**: Refrigerator, cooking bench, cabinets, dining table with chairs
- **Bedroom**: Bed, nightstands, wardrobe
- **Gym Area**: Fitness equipment, dumbbells
- **Bathroom**: (furniture included)
- **Multiple Windows and Doors**

## License

This package: Apache-2.0
AWS RoboMaker Small House World: Apache-2.0
