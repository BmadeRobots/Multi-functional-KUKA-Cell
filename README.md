# KUKA Dual-Arm Robot Cell - ROS2 Integration

This package enables ROS2 Humble control of a dual KUKA robot cell using RobotDK simulation and MoveIt2 motion planning.

## Prerequisites

Install the following software on Windows:

1. **RobotDK**
2. **ROS2 Humble** (Windows installation)
3. **MoveIt2** package
4. **KukaVarProxy** server

## Setup Instructions

### 1. RobotDK Connection Setup

1. Open the file `Multi-axis Assembly Robot` in RobotDK
2. Establish the connection between master and slave robots
3. On the teach pendant, start the **KukaVarProxy** server
4. Leave the script named `RoboDKSync35` running on **both controllers**
5. In RobotDK, activate the connection
6. Connection is now established

### 2. Trajectory Planning (Predefined Motion)

Start the trajectory planning node:
```bash
ros2 run kuka_robodk_ros2 trajectory_planner.py
```

This node will start planning trajectories for predefined motions.

### 3. MoveIt2 Setup

#### Initial Configuration (One-time setup)

1. Open MoveIt Setup Assistant:
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

2. Import the URDF file from `kuka_robot_descriptions`
3. Complete the MoveIt configuration package
4. Output the package as `kuka_moveit2`

#### Running MoveIt2 Motion Planning

1. Launch the MoveIt demo node:
```bash
ros2 launch kuka_moveit2 demo.launch.py
```

2. This will open RViz with the MoveIt interface

3. Select your desired planner from available options:
   - OMPL planners
   - RRT
   - RRT-Connect
   - Other tested planners

4. Plan and execute:
   - Set target pose in RViz
   - Click "Plan"
   - Click "Execute"
   - Motion will actuate on **both RobotDK simulation and real robots**

## Package Structure
```
kuka_robodk_ros2/
├── trajectory_planner.py    # Predefined trajectory execution
└── ...

kuka_robot_descriptions/
├── urdf/                     # Robot URDF files
└── ...

kuka_moveit2/                 # MoveIt2 configuration package
├── launch/
│   └── demo.launch.py
└── config/
```

## Notes

- Ensure both robot controllers have `RoboDKSync35` running before attempting motion
- The KukaVarProxy server must be active on the teach pendant
- Real robot motion is synchronized with RobotDK simulation
- Always verify motion in simulation before executing on real hardware

## Troubleshooting

- If connection fails, verify KukaVarProxy is running on both controllers
- Ensure RobotDK connection is activated after starting the sync scripts
- Check ROS2 topic communication: `ros2 topic list`
