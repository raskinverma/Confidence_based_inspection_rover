# Confidence-Based Active Inspection Rover (ASCEND-GROUND)

> Robotics and Intelligent Autonomous Systems — Course Project
> **Institute:** IIIT Dharwad | **Faculty:** Dr. Sunil C K

A **simulation-only** autonomous ground rover (ROS 2 Jazzy + Gazebo Harmonic) that builds a 2D occupancy map via LiDAR SLAM, navigates to inspection waypoints with Nav2, detects visual defects (HSV colour blobs and ArUco markers) with an onboard camera, and logs each detection with global map coordinates.

## Tech Stack

| Layer | Choice |
|---|---|
| OS | Ubuntu 24.04 LTS |
| ROS | ROS 2 Jazzy Jalisco |
| Simulator | Gazebo Harmonic (gz-sim 8) |
| SLAM | slam_toolbox (async) |
| Navigation | Nav2 (NavFn/A* + DWB) |
| Perception | OpenCV 4 + ArUco |

## Install Dependencies

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-rviz2 \
  ros-jazzy-tf2-tools ros-jazzy-tf-transformations \
  ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-vision-msgs \
  ros-jazzy-robot-localization \
  python3-colcon-common-extensions python3-rosdep python3-opencv
```

## Build

```bash
cd ~/College/robotics_main_proj/ros2_ws
colcon build --symlink-install
source install/setup.zsh
```

## Run

```bash
# Full autonomous simulation (Gazebo + Nav2 + perception)
ros2 launch rover_bringup full_sim.launch.py mode:=nav run_perception:=true

# SLAM mapping mode (teleoperate to build map)
ros2 launch rover_bringup full_sim.launch.py mode:=slam
ros2 run teleop_twist_keyboard teleop_twist_keyboard  # separate terminal

# View robot description only
ros2 launch rover_description rsp.launch.py
```

## Package Structure

| Package | Purpose |
|---|---|
| `rover_description` | URDF/Xacro model, inertial macros, RViz config |
| `rover_gazebo` | Gazebo Harmonic worlds, ros2_control config, gz_bridge, twist relay |
| `rover_navigation` | slam_toolbox, AMCL, Nav2 params, saved map, launch files |
| `rover_perception` | Defect detector (HSV+ArUco), TF2 pose projector, defect logger |
| `rover_mission` | Autonomous mission FSM, inspection waypoints |
| `rover_interfaces` | Custom `Defect.msg` |
| `rover_bringup` | Master launch file orchestrating the full simulation stack |

## Output

Detected defects are written to `results/inspection_defects.csv` (created at runtime, not tracked by git) and visualised as persistent `MarkerArray` spheres in RViz on `/defects`.

## License

MIT

## Author

- **Raskin Verma** — Team Lead / Foundation Owner
