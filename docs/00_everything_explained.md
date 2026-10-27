# Inspection Rover — Everything Explained

> A plain-English deep dive into every file, concept, and design decision made so far.
> Read this alongside the actual code files. Updated after every phase.

---

## Table of Contents
1. [Big Picture — What Are We Building?](#1-big-picture)
2. [Workspace Layout — 7 Packages](#2-workspace-layout)
3. [The Robot Model (URDF / Xacro)](#3-the-robot-model)
4. [Sensors](#4-sensors)
5. [Gazebo Simulation](#5-gazebo-simulation)
6. [Control System — How the Robot Moves](#6-control-system)
7. [The Twist Relay](#7-the-twist-relay)
8. [Perception Stack — Seeing Defects](#8-perception-stack)
9. [Mission FSM — Autonomous Behaviour](#9-mission-fsm)
10. [What's Running Right Now](#10-whats-running-right-now)
11. [What's Next](#11-whats-next)
12. [Phase 2 — SLAM Mapping](#phase-2--slam-mapping-added)
13. [Phase 3 — Autonomous Navigation (Nav2 & Localization)](#phase-3--autonomous-navigation-nav2--localization)

---

## 1. Big Picture

We are building an **autonomous inspection rover** in simulation.

The rover:
- Drives around a facility (factory / pipeline / structure)
- Uses a camera to spot **defects** (rust, leaks, ArUco asset markers)
- Uses a **LiDAR** to map the environment and navigate safely
- Has a **brain (Mission Manager)** that gives it waypoints to visit in sequence
- Logs every defect it finds with its real-world map position

Think of it like a Roomba, but instead of vacuuming it patrols a facility and reports damage.

```
              +---------------------------+
              |     Mission Manager       |  <- "The Brain"
              |  (FSM: IDLE->NAV->SCAN)   |
              +------------+--------------+
                           | sends goals
                           v
              +---------------------------+
              |          Nav2             |  <- "The Driver"
              |  (path plan, avoid obs)   |
              +------------+--------------+
                           | /cmd_vel
                           v
              +---------------------------+
              |    4-Wheel Rover          |  <- "The Body"
              |  (Gazebo simulation)      |
              +----+---------------+------+
             /scan |               | /camera/image_raw
                   v               v
          +----------+     +----------------+
          | SLAM/Nav2 |     | Defect Detector|  <- "The Eyes"
          +----------+     +----------------+
```

---

## 2. Workspace Layout

```
ros2_ws/
+-- src/
|   +-- rover_description/   <- Robot model (shape, joints, sensors)
|   +-- rover_gazebo/        <- Simulation (Gazebo world, physics, launching)
|   +-- rover_interfaces/    <- Custom ROS 2 messages (Defect.msg etc.)
|   +-- rover_navigation/    <- SLAM + Nav2 configuration
|   +-- rover_perception/    <- Camera-based defect detection
|   +-- rover_mission/       <- Autonomous mission logic (FSM)
|   +-- rover_bringup/       <- One-click launch of the full system
+-- docs/
    +-- 00_everything_explained.md   <- THIS FILE
    +-- 01_architecture.md
    +-- 02_kinematics_derivation.md
```

### Why 7 separate packages?

ROS 2 best practice is **separation of concerns** — each package has one job.
If you want to swap the camera algorithm, you only touch `rover_perception`.
If you want a different Nav2 planner, you only touch `rover_navigation`.
They communicate via **topics, services, and actions** (ROS 2's communication layer).

---

## 3. The Robot Model

**Files:** `rover_description/urdf/`

### What is URDF / Xacro?

- **URDF** (Unified Robot Description Format) = XML file describing every physical part (link) and connection (joint) of a robot.
- **Xacro** = a macro language on top of URDF. Lets you use variables, math expressions, loops, and reusable macros. Without it you'd copy-paste wheel code 4 times.

### base.xacro — The Chassis

The link/joint tree:
```
base_footprint  (ground plane, z=0)
      |  fixed joint (raised by wheel_radius = 4.25 cm)
      v
  base_link  (the body box: 35x25x8 cm, 2.5 kg)
      +-- front_left_wheel_joint   (type=continuous, can spin forever)
      +-- rear_left_wheel_joint
      +-- front_right_wheel_joint
      +-- rear_right_wheel_joint
```

**Why `base_footprint`?**
It is a ROS convention. Nav2 and many tools expect a frame sitting exactly on the ground (z=0). `base_link` is raised to wheel-axle height so it is not underground. The fixed joint between them lifts `base_link` by exactly one wheel radius.

**Why 4 wheels (Skid-Steer)?**
- More stable than 2-wheel differential drive (does not tip)
- Looks more like a real inspection rover (Clearpath Jackal style)
- Turns by spinning left side faster/slower than right side — like a tank
- No steering joints needed — simpler model

**The Wheel Macro:**
```xml
<xacro:macro name="wheel" params="prefix side_sign x_pos rpy_rot axis_z">
```
This is called 4 times with different parameters:
- `side_sign` = +1 for left wheels, -1 for right wheels (controls Y offset)
- `rpy_rot` rotates the cylinder so it lies flat (cylinders in URDF are vertical by default; we rotate -90 or +90 degrees)
- `axis_z` = 1 or -1 so that both sides spin in the correct real-world direction

**Key dimensions:**

| Property     | Value    | Reason                               |
|---|---|---|
| Chassis      | 35x25x8 cm | Compact but stable                |
| Wheel radius | 4.25 cm  | ~85 mm diameter — small rover        |
| Track width  | 27 cm    | Wide stance = stable                 |
| Wheelbase    | 20 cm    | Front-to-rear axle gap               |
| Wheel mass   | 200 g    | Realistic rubber wheel               |
| Chassis mass | 2.5 kg   | Battery + compute + body             |

**Inertia tensors:**
Each link has an `<inertia>` tag. This tells the physics engine how mass is distributed so the robot does not behave like a frictionless brick. We compute them analytically:
- Box: `Ixx = m*(y^2 + z^2)/12`, etc.
- Cylinder: `Ixx = m*(3r^2 + h^2)/12`, etc.

---

## 4. Sensors

**File:** `rover_description/urdf/sensors.xacro`

### Why is sensor definition split into two files?

`sensors.xacro` defines the **physical shape and position** (links and joints).
`gazebo_plugins.xacro` defines the **simulation behaviour** (what Gazebo actually does — fire rays, capture images, etc.).

This separation means the physical model is hardware-agnostic. On a real robot you would just replace the Gazebo plugin block.

### LiDAR

```
Mounted:     5 cm forward, 14 cm above base_link
Shape:       Small cylinder (35 mm radius, 4 cm tall) — like a real spinning LiDAR
Gazebo type: gpu_lidar (GPU raytracing = much faster than CPU)
Sweep:       360 degrees, 360 rays (1 ray per degree), 10 Hz
Range:       0.12 m to 12.0 m
Noise:       Gaussian stddev = 1 cm (realistic sensor noise)
Topic:       /scan  (sensor_msgs/LaserScan)
```

**What does /scan contain?**
An array of 360 distance measurements, one per degree around the rover, taken 10 times per second. Think of it as a radar ping — the rover "sees" how far away walls and objects are in every direction. SLAM uses this data to build a map.

### Camera

```
Mounted:     17 cm forward, 18 cm above base_link, tilted DOWN 10 degrees
Why down?    To see the ground and walls ahead (inspection use case)
Shape:       Small flat box
Resolution:  640 x 480 pixels @ 30 Hz
HFOV:        60 degrees (1.047 radians)
Noise:       Gaussian stddev = 0.007 (mild image noise)
Topics:      /camera/image_raw   /camera/camera_info
```

**What is camera_optical_frame?**
ROS has a convention (REP-103) for camera coordinate systems:
- Camera link uses robot convention: X=forward, Y=left, Z=up
- Camera *optical* frame uses image/OpenCV convention: Z=forward, X=right, Y=down

We add a `camera_optical_frame` child link rotated by (-90, 0, -90) degrees to satisfy this. The defect detector needs this for 3D projections to be correct.

### IMU

```
Mounted:     Directly on base_link, 8 cm above
Update rate: 100 Hz
Provides:    Linear acceleration (x,y,z) + angular velocity (roll/pitch/yaw rate)
Topic:       /imu  (sensor_msgs/Imu)
```

**What is the IMU used for?**
It helps Nav2 understand how the robot is rotating and accelerating. This is especially important for skid-steer robots, which slip sideways when turning. Wheel odometry alone gives poor orientation estimates during turns. IMU fusion fixes this.

---

## 5. Gazebo Simulation

**Package:** `rover_gazebo/`

### gazebo_plugins.xacro — Making the Physics Work

**Wheel friction (mu1, mu2):**
```xml
<mu1>0.8</mu1>   <!-- Longitudinal friction (forward/backward grip) -->
<mu2>0.1</mu2>   <!-- Lateral friction (side slip — LOW intentionally) -->
```

Skid-steer works by *intentionally sliding* the inner wheels sideways during a turn. If mu2 were high, the rover would fight itself while turning and either not turn or strain its joints. Low mu2 = realistic skid-steer behaviour like a tank.

**ros2_control Hardware Interface:**
```xml
<ros2_control name="GazeboSimSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>
  <joint name="front_left_wheel_joint">
    <command_interface name="velocity"/>  <!-- we send: how fast to spin (rad/s) -->
    <state_interface name="position"/>   <!-- we read: how far it has rotated -->
    <state_interface name="velocity"/>   <!-- we read: current spin speed -->
  </joint>
  ... (x4 for all wheels)
```

This is the glue between ROS 2 and Gazebo physics:
- We declare 4 joints, each accepting velocity commands in rad/s
- Gazebo physics reads those commands and applies motor torques
- Position and velocity state is read back and published as `/joint_states`
- The robot_state_publisher uses `/joint_states` + URDF to compute all TF transforms

### sim.launch.py — What Runs When You Launch

When you run `ros2 launch rover_gazebo sim.launch.py`:

| Process                | What it does |
|---|---|
| `gazebo`               | Opens Gazebo Harmonic with our world SDF file |
| `robot_state_publisher`| Reads URDF, broadcasts all TF transforms (every link's 3D position) |
| `ros_gz_sim create`    | Spawns the rover model into the running Gazebo world |
| `parameter_bridge`     | Translates Gazebo topics -> ROS 2 topics (/scan, /camera/*, /imu, /clock) |
| `twist_relay.py`       | Our adapter node (explained next) |
| `rviz2`                | 3D visualisation — robot model, LiDAR points, camera feed |

**Why `parameter_bridge`?**
Gazebo Harmonic uses its own internal messaging (gz-transport). ROS 2 uses DDS. They cannot talk directly. The bridge translates between the two. Without it, `/scan` would not exist as a ROS 2 topic.

**The /clock bridge:**
Gazebo runs its own simulation clock. All ROS 2 nodes use `use_sim_time: true` so they read from this `/clock` topic instead of real wall-clock time. This lets you pause or speed up simulation without breaking timing-sensitive code.

---

## 6. Control System — How the Robot Moves

**Config:** `rover_gazebo/config/diff_drive_controller.yaml`

### The diff_drive_controller

ROS 2 ships a standard `diff_drive_controller` implementing differential-drive math. We use it for skid-steer too — at the controller level, both are identical (just set left vs right wheel speeds differently).

The controller:
1. Subscribes to `/diff_drive_controller/cmd_vel` (TwistStamped message)
2. Computes left and right wheel velocities from the desired linear + angular motion
3. Sends velocity commands to the 4 wheel joints via ros2_control
4. Reads joint states back and publishes `/diff_drive_controller/odom`

**Skid-steer kinematic equations:**
```
v_left  = v - (omega * L/2)
v_right = v + (omega * L/2)

where:
  v     = desired linear velocity (m/s) from cmd_vel.linear.x
  omega = desired angular velocity (rad/s) from cmd_vel.angular.z
  L     = track width = wheel_separation = 0.27 m
```

To turn RIGHT: set positive angular.z -> right wheels spin faster, left slower (or even backwards). The rover skids left wheels sideways — exactly like a tank turn.

**Converting m/s to rad/s for wheels:**
```
omega_wheel = v_side / wheel_radius    (0.0425 m)
```

---

## 7. The Twist Relay

**File:** `rover_gazebo/rover_gazebo/twist_relay.py`

### The Problem It Solves

`diff_drive_controller` requires a **TwistStamped** message (Twist + header with timestamp + frame_id).
Everything else in ROS 2 ecosystem (teleop keyboard, Nav2, our mission manager) publishes plain **Twist** (no header).

Without this adapter, you could not drive the robot with the keyboard or Nav2 at all.

### What It Does

```
/cmd_vel (geometry_msgs/Twist)
    |
    v  twist_relay.py adds header:
       stamp = current time
       frame_id = 'base_footprint'
    |
    v
/diff_drive_controller/cmd_vel (geometry_msgs/TwistStamped)

/diff_drive_controller/odom (nav_msgs/Odometry)
    |
    v  twist_relay.py re-publishes as-is
    |
    v
/odom (nav_msgs/Odometry)    <- Nav2 reads from /odom
```

It is only 72 lines of code but without it nothing moves. It is a permanent part of the system, always running when the simulation is up.

---

## 8. Perception Stack — Seeing Defects

**Package:** `rover_perception/`

Three nodes form a pipeline:

```
/camera/image_raw
        |
        v
 defect_detector.py -----> /camera/detections_debug (annotated image for RViz)
        |
        | publishes /defect_detections
        | (Defect msg: type, confidence, pixel_u, pixel_v)
        v
  pose_projector.py  (also reads /camera/camera_info, /scan, and TF tree)
        |
        | publishes /defect_events
        | (same Defect msg + map_position filled in)
        v
   defect_logger.py -----> defect_log.csv file on disk
```

### defect_detector.py — What It Detects and How

Runs at camera framerate (30 Hz). Processes every frame.

**1. Red (Rust) Detection — HSV Colour Thresholding**

We convert the image from BGR to HSV (Hue-Saturation-Value) colour space.
HSV is better for colour detection because lighting changes affect Value (brightness) more than Hue. The object stays "red" in HSV even under different lighting conditions.

Red hue wraps around in HSV (appears near both 0 degrees and 180 degrees), so we check two ranges and OR them together:
```python
lower_red1 = [0,   100, 70]   upper_red1 = [10,  255, 255]  # hue 0-10 deg
lower_red2 = [170, 100, 70]   upper_red2 = [180, 255, 255]  # hue 170-180 deg
mask_red = cv2.bitwise_or(mask_red1, mask_red2)
```

Then:
- Morphological opening (erosion then dilation) — removes tiny speckle noise
- Morphological closing (dilation then erosion) — fills small gaps in detected region
- Find contours — get the outlines of detected blobs
- Filter by area (min 500 px^2) — ignore tiny noise detections

**2. Yellow (Leak) Detection — Same approach, different hue**
```python
lower_yellow = [20, 100, 100]  upper_yellow = [35, 255, 255]
```

**3. ArUco Markers — Asset/Equipment Tags**
ArUco markers are fiducial markers (like QR codes but simpler and faster to detect). Used to mark specific equipment to inspect.
OpenCV's `cv2.aruco.ArucoDetector` with `DICT_4X4_50` dictionary detects them.
When detected, publishes `defect_type = "aruco_5"` (with the marker ID embedded in the string).

**Confidence score formula:**
```python
confidence = min(1.0, area / 20000.0 + 0.5)
```
Larger detected region -> higher confidence. Floor at 0.5 (something was detected), cap at 1.0.

**Output — the Defect message (`rover_interfaces/msg/Defect`):**
```
header        <- ROS timestamp + frame_id
defect_type   <- "rust", "leak", "aruco_5", etc.
confidence    <- 0.5 to 1.0
pixel_u       <- column in image (x coordinate)
pixel_v       <- row in image (y coordinate)
map_position  <- Point (empty here, filled in by pose_projector)
```

### pose_projector.py — From Pixel to World Position

A pixel coordinate (u, v) just tells you *where in the 640x480 image* a defect is. To log the defect's actual real-world location, we need to un-project it into 3D space.

**Step 1: Estimate depth using LiDAR**

We do not have a depth camera. Instead, we correlate the pixel's horizontal angle with the nearest LiDAR ray:
```
bearing = atan2(u - cx, fx)     <- horizontal angle in camera frame
angle_lidar = -bearing           <- flip sign for coordinate frame difference
```
Look up the LiDAR scan range at that angle and take the median of 3 nearby rays to get a stable distance Z.

**Step 2: Pinhole camera back-projection**

The camera intrinsic matrix K (provided in /camera/camera_info) contains:
- fx, fy = focal lengths in pixels
- cx, cy = principal point (image centre, usually 320x240)

Back-projection:
```
X = (u - cx) * Z / fx     <- left/right offset in metres
Y = (v - cy) * Z / fy     <- up/down offset in metres
Z = depth from LiDAR step
```
This gives a 3D point in `camera_optical_frame`.

**Step 3: Transform to map frame via TF2**

TF2 is ROS 2's transform tree system. It continuously tracks where every link is relative to every other link at every moment in time.

We ask: "transform this 3D point from camera_optical_frame to map".
TF2 traces the chain:
```
camera_optical_frame -> camera_link -> base_link -> odom -> map
```
The result is the defect's XY position on the map — the same coordinate system that Nav2 uses for navigation.

### defect_logger.py

Subscribes to `/defect_events` and appends each detection to a CSV file:
```
timestamp, defect_type, confidence, pixel_u, pixel_v, map_x, map_y, map_z
```
Simple but permanent record of everything the rover found.

---

## 9. Mission FSM — Autonomous Behaviour

**File:** `rover_mission/rover_mission/mission_manager.py`

### What is a Finite State Machine (FSM)?

The rover can only be in **one state at a time**. The FSM defines:
- What the rover does in each state
- What event triggers a transition to a different state

```
           +----------------------------------------------------------+
           |                                                          |
    +------+------+  Nav2 ready   +------------+  Arrived  +--------+|
    |    IDLE     | ------------> | NAVIGATING | --------> |SCANNING||
    +-------------+               +------+-----+           +----+---+|
                                         ^                      |     |
                                  +------+--------+  Scan done  |     |
                                  |    NEXT_WP    | <-----------+     |
                                  +---+------+----+                   |
                                      |      |                        |
                          more WPs    |      | no more WPs           |
                                      |      v                        |
                                      |   +-----------+              |
                                      |   | COMPLETED |              |
                                      v   +-----------+              |
                               back to NAVIGATING                    |
           +----------------------------------------------------------+
```

### State-by-State Breakdown

**IDLE**
The FSM timer fires at 1 Hz. In IDLE, it waits up to 2 seconds for Nav2's `navigate_to_pose` action server to become available. Once Nav2 is ready, transition to NAVIGATING.

**NAVIGATING**
Sends the current waypoint as a NavigateToPose action goal to Nav2. Nav2 plans a path (avoiding obstacles on the costmap) and drives the rover there. We wait asynchronously via callbacks — we do NOT block.

Nav2 callback chain:
```python
send_goal_async() -> goal_response_callback()    # Was goal accepted?
                  -> get_result_async()          # Did we arrive?
                  -> goal_result_callback()      # Arrived = go to SCANNING
                                                 # Failed  = retry or skip
```

**SCANNING**
The rover has arrived at the waypoint. It rotates gently in place:
```python
cmd.angular.z = 0.4   # rad/s, slow rotation
```
This sweeps the camera across the inspection area. The defect_detector is always running, so it detects anything visible during the rotation. Scan duration is configurable per waypoint (default 3 seconds).

**NEXT_WP**
Increments waypoint index. If more exist: back to NAVIGATING. If all visited: COMPLETED.

**Failure handling:** If Nav2 fails to reach a waypoint, retry once. If retry also fails, skip to next waypoint. The mission never aborts completely.

### Waypoints Format

Loaded from `rover_mission/config/inspection_waypoints.yaml`:
```yaml
waypoints:
  - id: 1
    name: "Panel A"
    x: 2.0           # metres in map frame
    y: 1.5
    yaw: 0.0          # heading at arrival (radians)
    scan_duration: 5.0  # how long to scan at this waypoint
```
Yaw is converted to a quaternion (q.z = sin(yaw/2), q.w = cos(yaw/2)) for the Nav2 goal pose.

---

## 10. What's Running Right Now

The **simulation is live** and healthy. Complete data flow:

```
Gazebo Harmonic (physics at 1000 Hz)
   |
   +-- LiDAR: 360 rays at 10 Hz
   |     --> gz_ros_bridge --> /scan (sensor_msgs/LaserScan)
   |
   +-- Camera: 640x480 frame at 30 Hz
   |     --> gz_ros_bridge --> /camera/image_raw (sensor_msgs/Image)
   |                       --> /camera/camera_info (sensor_msgs/CameraInfo)
   |
   +-- IMU: accel + gyro at 100 Hz
   |     --> gz_ros_bridge --> /imu (sensor_msgs/Imu)
   |
   +-- 4 wheel joints
         <-- gz_ros2_control reads /diff_drive_controller/cmd_vel
         --> /joint_states (with all 4 wheel positions/velocities)
         --> /diff_drive_controller/odom (wheel odometry)

robot_state_publisher:
   Reads URDF + /joint_states
   --> broadcasts /tf (dynamic transforms, updates per joint_states)
   --> broadcasts /tf_static (fixed transforms, published once)

twist_relay.py:
   /cmd_vel (Twist) --> /diff_drive_controller/cmd_vel (TwistStamped)
   /diff_drive_controller/odom --> /odom

rviz2:
   Displays robot 3D model, LiDAR scan points, camera image, TF frames
```

To drive it right now:
```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# i=forward  ,=backward  j=turn left  l=turn right  k=stop
```

---

## 11. What's Next

### Phase 2 — SLAM Mapping (Days 4-5)
**Goal:** Drive the rover and build a 2D map of the environment in real time.

Tool: `slam_toolbox` in asynchronous mode.

How it works:
1. Subscribes to `/scan` and `/tf`
2. As rover moves, detects matching features in overlapping scans (loop closure)
3. Builds a 2D occupancy grid: free=white, obstacle=black, unknown=grey
4. Map grows as new areas are explored
5. Save final map to .pgm + .yaml files (used by Nav2 later)

### Phase 3 — Nav2 Navigation (Days 6-7)
**Goal:** Send the rover to a goal pose and have it autonomously plan + drive there.

Components:
- Global costmap — inflated obstacle zones on the saved map
- Local costmap — real-time obstacle avoidance from live LiDAR
- NavFn planner — global A* path from current pose to goal
- DWB controller — local trajectory that follows the path while avoiding obstacles

### Phase 4 — Live Perception (Days 8-9)
**Goal:** Run defect_detector + pose_projector live in the simulation.

We will place coloured boxes (red/yellow) in the Gazebo world SDF and verify the rover detects them and logs them with correct map positions.

### Phase 5 — Full Mission Autonomy (Days 10-11)
**Goal:** Complete end-to-end autonomous run.
Mission Manager drives to all waypoints, scans each one, defect pipeline logs all findings.

### Phase 6 — Integration and Demo (Days 12-14)
**Goal:** One-command launch of the entire system, robustness testing, final documentation.

---

*Last updated: Phase 1 complete — robot model, simulation, control stack, perception nodes, and mission FSM are all written and building clean.*

---

# PHASE 2 — SLAM MAPPING (Added)

## What is SLAM?

**SLAM = Simultaneous Localisation And Mapping.**

The rover has to solve two problems at the same time:
1. **"Where am I?"** — Localisation
2. **"What does the world look like?"** — Mapping

These are circular: to build a map you need to know where you are, and to know where you are you need a map. SLAM solves both together.

**Our tool: `slam_toolbox` (async mode)**
- One of the best open-source SLAM implementations for ROS 2
- "Async" means it processes scans as fast as they arrive without blocking the main loop
- Uses the **Ceres Solver** (Google's open-source nonlinear least-squares optimizer) to refine the pose graph

---

## How SLAM_Toolbox Works — Step by Step

### Input it needs:
- `/scan` — LiDAR scan (360 rays at 10 Hz)
- `/tf` — Transform tree so it knows how the robot moved between scans
- `/odom` — Wheel odometry (initial guess for robot motion)

### What it does:

**1. Scan Matching (ICP-like)**
Each new LiDAR scan is compared to the previous scan. Features (corners, walls, edges) that appear in both scans are matched. From the shift in matching features, it estimates how much the robot moved and rotated. This is more accurate than wheel odometry alone.

**2. Pose Graph**
Every robot pose (position + orientation) is a **node** in a graph. The constraints between poses (from scan matching) are **edges**. The Ceres solver continuously optimizes this graph to minimize error.

**3. Loop Closure**
When the rover revisits an area it mapped before, SLAM detects the match and adds a constraint between the current pose node and the old pose node. This corrects any accumulated drift in the map — the map "snaps" into consistency.

**4. Occupancy Grid**
The final output: a 2D grid where each cell is:
- **0 (white)** = free space (LiDAR ray passed through)
- **100 (black)** = obstacle (LiDAR ray ended here)
- **-1 (grey)** = unknown (no data yet)

Resolution: 5 cm per cell (configured in slam_toolbox_online_async.yaml).

---

## Key Config Parameters Explained

File: `rover_navigation/config/slam_toolbox_online_async.yaml`

```yaml
base_frame: base_footprint   # Where is the robot's origin?
scan_topic: /scan            # Which LiDAR topic to use?
mode: mapping                # We are building a new map (not localising in existing)
resolution: 0.05             # 5 cm per grid cell
max_laser_range: 12.0        # Ignore LiDAR returns beyond 12 m (matches sensor config)
minimum_travel_distance: 0.15  # Only process new scan after moving 15 cm (avoids redundant processing)
minimum_travel_heading: 0.15   # Or after rotating 0.15 rad (~8.5 degrees)
do_loop_closing: true        # Fix accumulated drift when revisiting areas
transform_publish_period: 0.02  # Publish map->odom transform at 50 Hz (smooth)
map_update_interval: 2.0     # Redraw the occupancy grid every 2 seconds
```

**The transform SLAM publishes:**
SLAM publishes `map -> odom` transform. Combined with the existing `odom -> base_footprint` transform (from wheel odometry), the full chain becomes:
```
map -> odom -> base_footprint -> base_link -> [all sensors]
```
This is how every sensor reading gets a position in the world map.

---

## How to Run SLAM

### Terminal 1 — Start the simulation (if not already running):
```bash
cd ~/College/robotics_main_proj/ros2_ws
source install/setup.bash
ros2 launch rover_gazebo sim.launch.py
```

### Terminal 2 — Start SLAM:
```bash
source install/setup.bash
ros2 launch rover_navigation slam.launch.py
```
You will see: `slam_toolbox` starts and begins publishing `/map` topic.

### Terminal 3 — Drive the rover to build the map:
```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Drive around the entire facility. The map grows in RViz as you explore.

### Save the map when done:
```bash
source install/setup.bash
bash src/rover_navigation/launch/save_map.sh inspection_map
```
Creates `inspection_map.pgm` (image of the map) and `inspection_map.yaml` (metadata).

---

## What You See in RViz During SLAM

- **Grey area** = unexplored (unknown)
- **White area** = free space (rover can drive here)
- **Black lines/walls** = obstacles detected by LiDAR
- **Robot model** = current rover position in the map
- **Green scan points** = live LiDAR scan overlay
- **Small red dots** = pose graph nodes (each scan position recorded)

As you drive, the map expands and white/black areas replace grey.
When loop closure fires, you may see the map slightly shift/correct itself — that is SLAM fixing accumulated error.

*Last updated: Phase 2 SLAM implementation complete.*

---

# PHASE 3 — AUTONOMOUS NAVIGATION (NAV2 & LOCALIZATION)

## What is Nav2?

**Nav2 (Navigation 2)** is the industrial-standard autonomous navigation framework for ROS 2.

If SLAM was about creating the map ("cartography"), Nav2 is about **driving autonomously across that map** from point A to point B without crashing into anything, even when unexpected obstacles appear.

```
       [User Goal / Mission Manager] (e.g. "Go to Room B pipe leak")
                     |
                     v
             +---------------+
             | BT Navigator  |  <- Orchestrates high-level decisions
             +-------+-------+
                     |
         +-----------+-----------+
         |                       |
         v                       v
+-----------------+     +-----------------+
| Planner Server  |     | Controller      |
| (Navfn / A*)    |     | Server (DWB)    |
| Global Path     |     | Local Velocity  |
+--------+--------+     +--------+--------+
         |                       |
         v                       v
 [Global Costmap]         [Local Costmap]
 (Whole facility map)     (3x3m local window)
                                 |
                                 v
                     +-----------------------+
                     |   Velocity Smoother   |
                     |  & Collision Monitor  |
                     +-----------+-----------+
                                 | /cmd_vel
                                 v
                     [Twist Relay -> Rover]
```

---

## The Core Components of Nav2 Explained

### 1. Map Server (`nav2_map_server`)
- Loads our pre-saved facility map (`facility.yaml` and `facility.pgm`).
- Publishes the map onto `/map` as an `OccupancyGrid` message.
- Uses lifecycle management (Unconfigured -> Inactive -> Active) so it only consumes resources when requested.

### 2. AMCL — Adaptive Monte Carlo Localization (`nav2_amcl`)
Before the robot can navigate, it must know **where it is on the map**. This is called **localization**.
- **How it works (The Particle Filter):**
  1. AMCL generates hundreds of "particles" (guesses) scattered around the initial pose guess `(x, y, theta)`.
  2. Each particle represents a hypothetical robot position and orientation.
  3. When the rover moves, wheel odometry updates every particle's position.
  4. When a LiDAR scan arrives, AMCL simulates what the LiDAR *should* see for every particle against the known map.
  5. Particles whose simulated scans match the real LiDAR scan are given high weights; particles that don't match are penalized.
  6. **Resampling:** High-weight particles multiply; low-weight particles vanish.
  7. Within 1-2 seconds of driving, the cloud of particles collapses tightly around the rover's true location.
- **The transform AMCL publishes:**
  AMCL continuously calculates and broadcasts the **`map -> odom`** coordinate frame transform, correcting odometry drift in real time.

### 3. Costmaps (`nav2_costmap_2d`)
A costmap transforms a binary obstacle map into an **energy/cost field** where higher values represent greater danger:
- **Lethal Cost (254, black):** Physical obstacles (walls, pipes, shelf). Driving here causes a crash.
- **Inscribed Cost (253):** Within the bounding box / footprint radius of the rover. If the robot's center is here, its body touches the wall.
- **Inflation Gradient (252 down to 1):** A smooth safety buffer surrounding every obstacle. Paths that stay further away from walls are cheaper and preferred.
- **Free Space (0, transparent/light):** Completely open floor. Safe to drive at full speed.

Nav2 maintains **two separate costmaps**:
1. **Global Costmap:**
   - Covers the entire 12m x 10m facility.
   - Combines the static wall map (`facility.pgm`) + real-time laser scans.
   - Used by the global planner to find the overall route.
2. **Local Costmap:**
   - A 3m x 3m moving window centered on the robot.
   - Updates rapidly (5 Hz) using live LiDAR scans.
   - Detects dynamic or moved objects (e.g., dynamic boxes, people, obstacles not on the original map) to allow real-time avoidance.

### 4. Global Planner (`planner_server` using Navfn)
- Receives the start pose and goal pose.
- Uses **Navfn** (Dijkstra / A* algorithm over the global costmap).
- Computes the shortest, collision-free geometric line of waypoints through the facility.
- Knows about door openings, corridors, and rooms.

### 5. Local Controller (`controller_server` using DWB Local Planner)
- While the Global Planner gives the big-picture path, the Local Controller actually **drives the robot**.
- **DWB = Dynamic Window Approach (expanded):**
  1. Samples hundreds of possible velocity commands `(linear_v, angular_w)` the rover can physically execute in the next 1.5 seconds.
  2. Projects circular trajectory arcs for each sample.
  3. Evaluates ("critics") each arc:
     - `BaseObstacle`: Rejects arcs that collide with any costmap obstacle.
     - `PathAlign` & `PathDist`: Rewards arcs that stay close to the global plan.
     - `GoalAlign` & `GoalDist`: Rewards arcs moving toward the waypoint.
     - `RotateToGoal`: Smoothly aligns heading when arriving at the destination.
  4. Selects the winning trajectory and outputs velocity commands at 20 Hz.

### 6. Velocity Smoother & Collision Monitor
- **Velocity Smoother (`nav2_velocity_smoother`):**
  Applies acceleration limits (0.8 m/s² linear, 2.0 rad/s² angular) to smooth out step changes in commanded velocity, preventing wheel slip and robot jolting.
- **Collision Monitor (`nav2_collision_monitor`):**
  An independent safety watchdog running right before motor output. If an obstacle suddenly appears within the robot's footprint, it instantly zeros the command to prevent an impact.

### 7. Recovery Behaviors (`behavior_server`)
If the rover gets blocked or stuck in an awkward angle:
- `Spin`: Rotates in place to re-clear LiDAR blind spots and scan surroundings.
- `BackUp`: Reverses 15-20 cm away from an obstacle.
- `Wait`: Pauses for dynamic obstacles (e.g. moving objects) to clear the path.

### 8. BT Navigator (`bt_navigator`)
- Coordinates the entire workflow using **Behavior Trees**.
- Instead of brittle hard-coded if/else statements, the behavior tree handles goal planning, retry loops, error handling, and recovery triggers hierarchically.

---

## Tuning for Our Inspection Rover

File: `rover_navigation/config/nav2_params.yaml`

| Parameter | Value | Why this value? |
|---|---|---|
| `footprint` | `[[-0.175, -0.125], [-0.175, 0.125], [0.175, 0.125], [0.175, -0.125]]` | Exact bounding box of our chassis (35cm length × 25cm width). Allows tighter navigation than a conservative bounding circle. |
| `min_vel_x` | `0.0 m/s` | **Forward-only driving:** Prevents the rover from reversing backwards towards goals behind it. Forces rover to rotate in place and drive forward facing the goal. |
| `max_vel_x` | `0.45 m/s` | Safe cruising speed inside industrial facility. |
| `max_vel_theta` | `1.2 rad/s` | ~68 deg/sec for smooth turning without wheel skidding. |
| `xy_goal_tolerance` | `0.20 m` | Rover stops within 20 cm of destination. |
| `yaw_goal_tolerance` | `0.25 rad` (~14 deg) | Sufficient orientation alignment for camera inspection. |
| `inflation_radius` (local) | `0.26 m` | **Critical for doorways:** The doorway is 0.80 m wide. Inflation of 0.26 m on both doorposts leaves `0.80 - (0.26 × 2) = 0.28 m` of zero-cost clear space in the center. |
| `inflation_radius` (global) | `0.28 m` | Guarantees global planner (Navfn) finds valid low-cost paths through 0.8 m doorways. |
| `cost_scaling_factor` | `5.0` | Steeper exponential cost drop-off near walls so clearance channels remain low-cost. |
| `time_before_collision` | `0.5 s` | Keeps collision monitor safety projection compact enough to pass through narrow doorways without false emergency stops. |
| `controller_frequency` | `20.0 Hz` | Fast control loop for responsive obstacle avoidance. |

---

## Troubleshooting & Key Navigation Insights

### 1. Why Did the Rover Move Backwards Towards the Door?
**The Symptom:** When commanded to a goal located behind it (e.g., exiting Room B back out to the corridor), the rover drove in reverse towards the doorway instead of turning around.

**The Root Cause:**
In `DWBLocalPlanner` (the controller that computes instantaneous wheel velocities), `min_vel_x` was previously set to `-0.15 m/s`. When evaluating candidate trajectories:
- Driving forward required first rotating ~180° in place (high heading penalty during the turn).
- Reversing had a shorter Euclidean distance without an immediate 180° rotation.
- DWB's trajectory scoring functions calculated that reversing had lower total cost, so it selected negative linear velocity.

**Why Reversing is Bad for an Inspection Rover:**
1. **Blind Spots:** The camera and main LiDAR are mounted at the front; reversing into a narrow door runs the risk of colliding with unseen obstacles.
2. **Inspection Misses:** The inspection camera cannot inspect targets behind the rover.

**The Fix:**
Set `min_vel_x: 0.0`. This restricts the allowed linear velocity space to $[0, 0.45]\text{ m/s}$. When a goal is behind the rover, DWB can only choose between pure rotation ($v_x = 0, \omega = \pm 1.2$) or forward motion, forcing the rover to pivot on the spot and drive forward through the door.

---

### 2. Why Did the Rover Get Stuck at the Door?
**The Symptom:** The rover approached the 0.8 m doorway between Room A and Room B, stopped, hesitated, and aborted or oscillatingly failed to pass through.

**The Root Causes (Dual Issue):**
1. **Costmap Inflation Radius Overlap:**
   - The doorway opening is $0.80\text{ m}$ wide ($Y \in [-0.4, 0.4]$).
   - The costmap inflation radius was previously $0.35\text{ m}$ (local) and $0.40\text{ m}$ (global).
   - Both doorposts inflated outward into the passage: $0.35\text{ m} + 0.35\text{ m} = 0.70\text{ m}$ of the $0.80\text{ m}$ opening was covered by high-cost inflation zones!
   - This left only $0.10\text{ m}$ of low cost in the center, which is narrower than the $0.25\text{ m}$ wide chassis.
   - Furthermore, `cost_scaling_factor` was $3.0$, which produces a gradual cost decay, making the entire doorway appear hazardous.
2. **Collision Monitor Lookahead Polygon:**
   - `collision_monitor` used `time_before_collision: 1.2 s`.
   - At $0.3\text{ m/s}$, the projected safety footprint extended $0.36\text{ m}$ ahead. At even slight approach angles, this polygon clipped the doorposts, causing the collision monitor to step in and clamp `cmd_vel` to zero.

**The Fix:**
- Set `inflation_radius: 0.26` (local) and `0.28` (global).
- Set `cost_scaling_factor: 5.0` so costs drop off sharply within a few centimeters of walls.
- Set `collision_monitor.time_before_collision: 0.5 s` so the safety lookahead is tight enough to clear 0.8 m doorways while still arresting true imminent collisions.

---

### 3. Self-Adaptability: What Happens When an Obstacle Blocks the Way?
**The Question:** If an unexpected obstacle (like a cardboard box or a fallen pipe) sits in the corridor, does the rover have self-adaptability to avoid it, replan, look around, or find another way?

**Yes! In Nav2, self-adaptability operates on three distinct hierarchical levels:**

```
[Level 1: Local Avoidance]   ->  DWB curves the wheels around the obstacle if space exists.
             | (if corridor is completely blocked / too narrow)
             v
[Level 2: Recovery Behaviors] ->  Spin (look around 360°), back up, and clear stale sensor noise.
             |
             v
[Level 3: Global Replanning]  ->  Navfn / A* computes a complete detour around the building.
```

#### Why Did the Robot Pause at the Box in the Corridor?
1. **The Corridor Geometry:**
   - The corridor is $1.1\text{ m}$ wide.
   - The cardboard box is $0.5\text{ m}$ wide.
   - That leaves only $\approx 0.3\text{ m}$ on each side. With our rover ($0.25\text{ m}$ wide) and safety inflation, the remaining space is physically too narrow to squeeze past.
2. **The 10-Second Wait Window:**
   - In `controller_server`, `progress_checker.movement_time_allowance` was previously set to `10.0 s`.
   - The robot sat stopped in front of the box for up to 10 seconds before officially declaring itself "stuck" and executing a recovery action. To a human spectator, 10 seconds looks like a freeze.
3. **Obstacle Weighting:**
   - `BaseObstacle.scale` was previously `0.02` while `PathDist.scale` was `32.0`. The local controller clung stubbornly to the original line on the floor instead of veering away earlier.

#### The Adaptability Enhancements Applied:
- **Fast Stuck Detection (`movement_time_allowance: 4.0 s`):** If the rover cannot move forward for 4 seconds, it immediately flags a stall and initiates recovery instead of waiting 10 seconds.
- **Proactive Swerving (`BaseObstacle.scale: 0.08`, `PathDist.scale: 24.0`):** Gives higher priority to curving away from detected objects before getting trapped against them.
- **Heuristic Global Detour (`use_astar: true`):** A* rapidly recomputes a path around the obstacle (e.g. through another room or doorway) once the corridor is marked as blocked by live LiDAR.
- **Recovery Behaviors:** The `behavior_server` executes `backup` (controlled reverse), `spin` (rotate to inspect new paths), and `clear_costmap` to adapt autonomously.

---

### 4. Why Did the Rover Abort with 15 Recoveries at the Wall Corner?
**The Symptom:** In the corridor, the rover cut close to the 90° inner wall corner, stopped, attempted 15 recoveries over 74 seconds, failed each one, and finally aborted (`Feedback: aborted`, `[bt_navigator]: Goal failed`).

**The Multi-Stage Failure Chain:**
1. **Corner-Cutting Global Path:**
   - With `global_costmap` inflation set to $0.28\text{ m}$ and `cost_scaling_factor: 5.0`, the cost dropped off too sharply outside the wall.
   - The global planner (Navfn) chose the mathematically shortest path, which shaved the apex of the $90^\circ$ corner.
   - Following this line, the rover cut the corner too tight and pinned its front bumper within $5\text{ cm}$ of the wall.
2. **Zero Reverse Tolerance (`min_vel_x: 0.0`):**
   - Because linear velocity was bounded at $[0.0, 0.45]$, the local planner could not reverse even 1 centimeter to disengage its front bumper.
   - Pure rotation ($v_x = 0, \omega \ne 0$) swung the outer rectangular chassis corners into the wall, causing every forward/turning trajectory to be rejected as lethal collision.
3. **Collision Monitor Choking the Recovery Behaviors:**
   - When the robot stalled, Nav2's Behavior Tree repeatedly ordered recovery actions: `spin` and `backup` (15 times!).
   - BUT the emergency safety node `collision_monitor` sat between Nav2 and the motors. Seeing the wall inside its safety polygon, `collision_monitor` clamped all output velocities to **zero**!
   - Nav2 thought the hardware was executing recoveries, while the collision monitor was silently muting the motors. After 15 attempts, the Behavior Tree aborted.

**The Fix:**
- **Wide Global Cushion (`inflation_radius: 0.38 m`, `cost_scaling_factor: 2.8`):** Pushes global paths into the center of corridors and forces wide, sweeping turns around wall corners.
- **Disengage Reverse Allowance (`min_vel_x: -0.05 m/s`):** Allows a tiny $5\text{ cm/s}$ reverse crawl (forward speed is $45\text{ cm/s}$, 9× faster, so it will never cruise backwards across rooms, but can effortlessly untangle itself from a corner).
- **Disable `collision_monitor` Intervention (`FootprintApproach.enabled: false`):** Restores full motor authority to Nav2's native costmap checking so `backup` and `spin` recoveries can actually execute without being choked.
- **Increased Obstacle Buffer in DWB (`BaseObstacle.scale: 0.12`):** Actively steers the rover away from walls before getting pinned.

---

## How to Run Autonomous Navigation

### Option 1 — Complete Simulation Bringup (Recommended)
Launches Gazebo world, robot, ROS-Gazebo bridge, relay, Nav2 stack, RViz, and perception in a single command:

```bash
cd ~/College/robotics_main_proj/ros2_ws
source install/setup.bash
ros2 launch rover_bringup full_sim.launch.py mode:=nav
```

### Option 2 — Modular Launch (Two Terminals)

**Terminal 1 — Start the Simulation:**
```bash
cd ~/College/robotics_main_proj/ros2_ws
source install/setup.bash
ros2 launch rover_gazebo sim.launch.py
```

**Terminal 2 — Start Nav2 with RViz:**
```bash
source install/setup.bash
ros2 launch rover_navigation navigation.launch.py
```

### How to Navigate in RViz:
1. Click the **"2D Pose Estimate"** tool in the RViz top toolbar.
   - Click and drag near `(-2.0, -1.0)` facing forward (+X direction) to set the initial pose (if AMCL hasn't auto-converged).
2. Click the **"Nav2 Goal"** tool in the top toolbar.
   - Click anywhere in free space (e.g. Room B near the pipes) and drag to set the desired arrival heading.
3. **Watch the rover drive:**
   - Nav2 draws the green **global plan** through the corridor.
   - The yellow/cyan trajectory shows the **DWB local plan** steering the rover.
   - The rover autonomously accelerates, navigates doorways, avoids obstacles, slows down, and aligns to the goal heading.

---

## What You See in RViz During Nav2

- **Black & White Map:** The loaded `facility.pgm` static map.
- **Color Gradients around walls:** The costmap inflation zones (purple/blue/yellow).
- **Green Particle Cloud:** AMCL's uncertainty cloud. As the rover moves, particles tightly converge around the chassis.
- **Red Footprint Box:** Rover's physical dimensions projected onto the ground.
- **Green Curve:** The global path generated by Navfn.
- **Blue/Yellow Vector:** Real-time velocity and trajectory chosen by DWB.
