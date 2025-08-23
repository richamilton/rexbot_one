# rexbot_one
## Background
### Robot State Publisher
![Robot State Publisher](docs/images/robot_state_publisher.png)

# Dependencies
## Controls
1. Install:
- ros-humble-ros2-control
- ros-humble-ros2-controllers
- ros-humble-gazebo-ros2-control

```
sudo apt install <packages>
```

## Getting Started

### 1. Build the Package
```bash
colcon build --packages-select rexbot_one
source install/setup.bash
```

### 2. Launch with Simulation
```bash
ros2 launch rexbot_one launch_sim.launch.py
```

### 3. Visualize the Robot
```bash
rviz2
```

> **Note:** To visualize movable joints, publish their state:
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

# References
- Design robot