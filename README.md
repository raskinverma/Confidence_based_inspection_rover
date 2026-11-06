# Confidence-Based Active Inspection Rover (ASCEND-GROUND)

> **Robotics and Intelligent Autonomous Systems — Course Project**  
> **Institute:** IIIT Dharwad | **Faculty:** Dr. Sunil C K  
> **Author:** Raskin Verma (Team Lead / Foundation Owner)  
> **Branch:** `active-inspection` | **ROS 2:** Jazzy Jalisco | **Simulator:** Gazebo Harmonic (gz-sim 8)

An autonomous ground rover designed for industrial surface defect inspection in complex, multi-room facilities. It combines 2D LiDAR SLAM, Nav2 autonomous navigation, monocular RGB perception (HSV color blob detection and ArUco fiducials), and a **Confidence-Driven Next-Best-View (NBV) Active Inspection Planner** with **Probabilistic Multi-View Defect Mapping**.

For full theoretical derivations, mathematical formulations, and the complete 18-run experimental benchmark, see the [Technical Report (REPORT.md)](file:///home/raskin/College/robotics_main_proj/ros2_ws/REPORT.md).

---

## Key Features

1. **Floor-Plane vs. Wall-Ray 3D Projection & Silhouette Filtering (`pose_projector.py`):**
   - Eliminates LiDAR-camera vertical parallax by performing geometric ray-floor plane intersections for downward camera rays ($z_{\text{floor}} = 0.02$ m).
   - Rejects depth-bleeding edge artifacts at obstacle silhouettes ($|\Delta d| > 0.50$ m).

2. **Probabilistic Multi-View Defect Belief Map (`defect_map.py`):**
   - Tracks hypotheses in log-odds confidence space with Bayesian belief updates.
   - Fuses 3D coordinates using inverse-variance spatial weights ($w \propto 1/d^2 \cdot \alpha_{\text{view}}$) and coordinate-wise weighted median with Median Absolute Deviation (MAD) outlier rejection.
   - Implements multi-angle parallax bonuses ($+0.20$) and negative-evidence rejection ($-0.30 \cdot dt$) for 3D physical distractors (warning signs, barrels).

3. **Frustum-Clipped Raycasting Coverage Grid (`coverage_map.py`):**
   - Maintains a 2D visibility grid ($0.25$ m resolution) clipped in real time by 2D LiDAR beams at 2 Hz (no seeing through walls).

4. **Confidence-Driven Next-Best-View (NBV) Active Planner (`mission_manager.py`):**
   - Combines a topological room sweep with targeted close-range standoff ($1.15$ m) re-inspections and focal dwell maneuvers to confirm true defects or reject false alarms.

---

## System Requirements & Dependencies

- **OS:** Ubuntu 24.04 LTS (Noble Numbat)
- **ROS Distribution:** ROS 2 Jazzy Jalisco
- **Gazebo Simulator:** Gazebo Harmonic (gz-sim 8)
- **Python Packages:** `numpy`, `matplotlib`, `pyyaml`, `psutil`, `opencv-python`, `pytest`

### Install Dependencies

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
  python3-colcon-common-extensions python3-rosdep python3-opencv \
  python3-matplotlib python3-numpy python3-yaml python3-psutil
```

---

## Build Workspace

```bash
cd ~/College/robotics_main_proj/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 1. One-Command Live Visual Demo (Gazebo GUI + RViz2 Dashboard)

Watch the rover navigate, scan, explore, and re-inspect defects live with the Gazebo Harmonic physical simulation and custom RViz2 dashboard running simultaneously:

```bash
# Default active demo in distractor environment (Seed 1)
./tools/run_live_demo.sh

# Or customize condition, seed, or inspection method:
./tools/run_live_demo.sh --condition distractors --seed 2 --method active
./tools/run_live_demo.sh --condition clean --seed 1 --method active
./tools/run_live_demo.sh --condition hard --seed 3 --method baseline
```

### RViz2 Dashboard Displays:
- **`Static Map` (`/map`):** 2D occupancy grid of the arena.
- **`Coverage Grid` (`/inspection/coverage_grid`):** Live frustum visibility filling in as the rover scans.
- **`Defect Hypotheses` (`/inspection/markers`):** 3D floating markers color-coded by belief status:
  - ⚪ **Gray:** `CANDIDATE` ($c < 0.50$)
  - 🟠 **Orange:** `UNCERTAIN` ($0.50 \le c < 0.78$)
  - 🟢 **Green:** `CONFIRMED` ($c \ge 0.78$, $n \ge 7$)
- **`Live Camera (Annotated)` (`/inspection/annotated_image`):** Front camera feed with bounding boxes, labels, and quality scores.
- **`Nav2 Global Plan` (`/plan`):** Active trajectory path planned by Nav2.

---

## 2. Automated Headless Batch Evaluation

To run headless multi-seed batch simulations across seeds and conditions:

```bash
# Run the active method across clean, distractors, and hard conditions
python3 tools/run_batch.py --method active --seeds 1 2 3 --conditions clean distractors hard --force-clean

# Run the passive baseline for comparison
python3 tools/run_batch.py --method baseline --seeds 1 2 3 --conditions clean distractors hard --force-clean
```

All results are evaluated against ground truth via [tools/rover_eval.py](file:///home/raskin/College/robotics_main_proj/ros2_ws/tools/rover_eval.py) and appended to `results/runs.csv`.

---

## 3. Generate Publication Plots & Metric Tables

```bash
# Generate 300 DPI publication plots and summary markdown/CSV tables
python3 tools/plot_results.py --input-csv results/runs.csv --plots-dir results/plots
```

Outputs saved to `results/plots/`:
- `f1_precision_recall_by_condition.png` (Grouped bar chart with Mean ± Std)
- `position_error_comparison.png` (3D position error in meters and strict $R=0.50$m F1)
- `efficiency_and_reinspections.png` (Simulation duration, path length, and re-inspections)
- `summary_metrics.md` and `results/summary_table.csv`

---

## 4. Run Unit Test Suite

```bash
# Run unit tests via unittest
python3 -m unittest discover -v tests

# Or via pytest
pytest tests/
```

All 12 unit tests across `test_active_planner.py`, `test_defect_map.py`, and `test_rover_eval.py` execute in $<0.2$s.

---

## Package Structure

| Package | Purpose |
|---|---|
| `rover_description` | URDF/Xacro mobile robot model, collision boxes, camera and LiDAR frames |
| `rover_gazebo` | Gazebo Harmonic SDF worlds, plugin bridges, differential drive controller |
| `rover_navigation` | Nav2 configuration, AMCL localization, custom RViz dashboard, coverage map |
| `rover_perception` | Defect detector (HSV & ArUco), 3D silhouette projector, probabilistic defect map |
| `rover_mission` | Active inspection mission manager FSM, NBV goal selector, telemetry logging |
| `rover_interfaces` | Custom ROS 2 message definitions (`Defect.msg`) |
| `rover_bringup` | Master orchestrator launch files (`full_sim.launch.py`) |
| `tools/` | World generator (`generate_world.py`), evaluator (`rover_eval.py`), batch runner (`run_batch.py`), plotting (`plot_results.py`), and live demo runner (`run_live_demo.sh`) |

---

## Summary Performance Benchmark (18-Run Matrix)

| Condition | Method | Runs ($N$) | Precision | Recall | F1 Score | Mean 3D Pos Error (m) | Re-inspections | Coverage (%) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Clean** | Baseline | 3 | $1.000 \pm 0.000$ | $0.889 \pm 0.157$ | $0.933 \pm 0.094$ | $0.207 \pm 0.007$ | $0.0 \pm 0.0$ | $32.1 \pm 45.4$ |
| **Clean** | **Active (Ours)** | 3 | **$0.952 \pm 0.067$** | **$1.000 \pm 0.000$** | **$0.974 \pm 0.036$** | **$0.157 \pm 0.007$** | $4.7 \pm 1.2$ | **$99.1 \pm 0.2$** |
| **Distractors** | Baseline | 3 | $0.644 \pm 0.031$ | $1.000 \pm 0.000$ | $0.783 \pm 0.024$ | $0.247 \pm 0.008$ | $0.0 \pm 0.0$ | $96.1 \pm 0.0$ |
| **Distractors** | **Active (Ours)** | 3 | **$0.750 \pm 0.000$** | **$1.000 \pm 0.000$** | **$0.857 \pm 0.000$** | **$0.239 \pm 0.031$** | $2.3 \pm 0.5$ | **$98.5 \pm 0.2$** |
| **Hard** | Baseline | 3 | $0.564 \pm 0.026$ | $1.000 \pm 0.000$ | $0.721 \pm 0.021$ | $0.236 \pm 0.022$ | $0.0 \pm 0.0$ | $96.2 \pm 0.1$ |
| **Hard** | **Active (Ours)** | 3 | $0.500 \pm 0.000$ | **$1.000 \pm 0.000$** | $0.667 \pm 0.000$ | **$0.172 \pm 0.002$** | $5.7 \pm 3.1$ | **$99.2 \pm 0.1$** |

---

## License

MIT License.
