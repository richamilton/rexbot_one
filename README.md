# rexbot_one

## Table of Contents
- [Background](#background)
- [Dependencies](#dependencies)
- [Getting Started](#getting-started)
- [References](#references)

## Background

### Robot State Publisher
![Robot State Publisher](docs/images/robot_state_publisher.png)

### Teleops
![Teleops](docs/images/teleops.png)

### SLAM

Simultaneous Localization and Mapping: Builds a map while estimating the robot’s position within that map.

#### Grid SLAM vs Feature SLAM
- `Feature SLAM`: Uses identifiable landmarks (eg. red roof, fence).
- `Grid SLAM`: Divides the world into cells, cells can be occupied , unocupied or unknown (what we use)

#### Online vs Offline
- `Online`: Working on a live data stream
- `Offline`: Working on recorded logs

#### Asynchronous vs Synchronous
- `Asynchronous`: Always process the most recent scan to avoid lagging even if that means skipping scans.
- `Synchronous`: Always process scan in order without skipping, can introduce lagging.

#### Coordinate Frames
- `base_link`: Frame attached to the robot.
- `odom`: Represents the world origin (based on wheel odometry) -- can have drift.
- `map`: Frame tied to an accurate global map -- corrects odometry drift -- but jumpy.
- `base_footprint`: 2D shadow of the robot's position on the ground, used for 2D SLAM

## Dependencies

### Controls
Install the following packages:
- `ros-humble-ros2-control`
- `ros-humble-ros2-controllers`
- `ros-humble-gazebo-ros2-control`

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control
```

### Teleops

Install the following packages:
- `joystick`
- `jstest-gtk`
- `evtest`
- `teleop_twist_joy`

```bash
sudo apt install joystick jstest-gtk evtest teleop_twist_joy
```

Check that joystick works with linux (Option)
- Select a event number and any button presses should cause some new text to come through on the screen.
```bash
evtest
```

### SLAM
Install the following packages:
- `slam_toolbox`

### Navigation
Install the following packages:
- `ros-humble-navigation2`
- `ros-humble-nav2-bringup`
- `ros-humble-twist-mux`

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

### 3. SLAM
1. Make service call to bring all elevators to floor being mapped:
```bash
ros2 service call /bring_all_to_floor example_interfaces/srv/AddTwoInts "{a: <floor>, b: 0}"
```

2. Run online-async slam:
```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/rexbot_one/config/mapper_params_online_async.yaml use_sim_time:=true
```

3. Save and Serialize Map
- `Save Map`: Save in old format for external service that uses map.
- `Serialize Map`: Save in new format, used with slam_toolbox for localization etc.
    a. In Rviz add slam_toolbox plugin
    b. Save and serialize map


> **Note:** Default params file can be copied and modified:
> ```bash
> cp /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml src/rexbot_one/config/
> ```

> **Note:** Stop Slam and rerun (step 2) to generate a new a map -- for example mapping another floor.

### 4a. Localization w/ slam_toolbox
- TBD

### 4b. Adaptive Monte Carlo Localisation (AMCL)
- TBD

## References

- Design robot