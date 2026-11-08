#!/usr/bin/env bash
# ==============================================================================
# One-Command Live Visual Demo Runner for Autonomous Inspection Rover
# Launches:
#   - Gazebo Harmonic 3D Physical Simulation GUI (headless=false)
#   - Full Nav2 Navigation & AMCL Localization Stack
#   - Real-Time Defect Perception Stack & Belief Map
#   - Autonomous Next-Best-View (NBV) Active Mission Manager
#   - Pre-configured RViz2 Live Dashboard with Coverage Grid & Detection Overlays
# ==============================================================================

set -e

# Default demo arguments
SEED=1
CONDITION="distractors"
METHOD="active"
HEADLESS="false"
USE_RVIZ="true"

# Parse CLI options
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --seed) SEED="$2"; shift ;;
        --condition) CONDITION="$2"; shift ;;
        --method) METHOD="$2"; shift ;;
        --headless) HEADLESS="$2"; shift ;;
        --no-rviz) USE_RVIZ="false" ;;
        -h|--help)
            echo "Usage: ./tools/run_live_demo.sh [OPTIONS]"
            echo "Options:"
            echo "  --seed <int>          World random seed (default: 1)"
            echo "  --condition <str>     Environment condition: clean, distractors, hard (default: distractors)"
            echo "  --method <str>        Inspection method: active, baseline (default: active)"
            echo "  --headless <bool>     Run Gazebo without GUI (default: false)"
            echo "  --no-rviz             Disable RViz2 launch"
            exit 0
            ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$WS_ROOT"

echo "================================================================================"
echo "🚀 LAUNCHING AUTONOMOUS INSPECTION ROVER LIVE VISUAL DEMO"
echo "   Condition : $CONDITION (Seed $SEED)"
echo "   Method    : $METHOD (Confidence-Driven NBV Active Inspection)"
echo "   GUI Mode  : Gazebo GUI (headless=$HEADLESS) + RViz2 Dashboard ($USE_RVIZ)"
echo "================================================================================"

# 1. Clean up any residual simulation processes
echo "[1/4] Checking and cleaning residual processes..."
pkill -9 -x rviz2 2>/dev/null || true
pkill -9 -x bt_navigator 2>/dev/null || true
pkill -9 -x controller_server 2>/dev/null || true
pkill -9 -x planner_server 2>/dev/null || true
pkill -9 -x behavior_server 2>/dev/null || true
pkill -9 -x smoother_server 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "rover_perception" 2>/dev/null || true
pkill -9 -f "rover_mission" 2>/dev/null || true
pkill -9 -f "ros2 launch rover_bringup" 2>/dev/null || true
sleep 1

# 2. Source ROS 2 environment & check display
echo "[2/4] Sourcing ROS 2 Jazzy workspace..."
export DISPLAY="${DISPLAY:-:0}"
if [ -n "$WAYLAND_DISPLAY" ]; then
    export WAYLAND_DISPLAY="$WAYLAND_DISPLAY"
fi
source /opt/ros/jazzy/setup.bash
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "Building workspace..."
    colcon build --symlink-install
    source install/setup.bash
fi

# 3. Generate simulation world & ground truth
echo "[3/4] Generating arena world ($CONDITION, seed=$SEED)..."
NUM_DISTRACTORS=0
CAMERA_NOISE=0.007
if [ "$CONDITION" == "distractors" ]; then
    NUM_DISTRACTORS=2
elif [ "$CONDITION" == "hard" ]; then
    NUM_DISTRACTORS=4
    CAMERA_NOISE=0.02
fi

python3 tools/generate_world.py \
    --seed "$SEED" \
    --num-distractors "$NUM_DISTRACTORS" \
    --camera-noise "$CAMERA_NOISE" \
    --output-sdf "src/rover_gazebo/worlds/run_${CONDITION}_seed${SEED}.sdf" \
    --output-gt "src/rover_gazebo/worlds/run_${CONDITION}_seed${SEED}_ground_truth.yaml"

WORLD_PATH="$WS_ROOT/src/rover_gazebo/worlds/run_${CONDITION}_seed${SEED}.sdf"
DEMO_OUT_DIR="$WS_ROOT/results/live_demo"
mkdir -p "$DEMO_OUT_DIR"

# Cleanup trap on exit
cleanup() {
    echo ""
    echo "================================================================================"
    echo "🛑 Shutting down live demo and cleaning process group..."
    pkill -9 -x rviz2 2>/dev/null || true
    pkill -9 -x bt_navigator 2>/dev/null || true
    pkill -9 -x controller_server 2>/dev/null || true
    pkill -9 -x planner_server 2>/dev/null || true
    pkill -9 -x behavior_server 2>/dev/null || true
    pkill -9 -x smoother_server 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "rover_perception" 2>/dev/null || true
    pkill -9 -f "rover_mission" 2>/dev/null || true
    pkill -9 -f "ros2 launch rover_bringup" 2>/dev/null || true
    echo "✅ Shutdown complete."
}
trap cleanup EXIT INT TERM

# 4. Launch full system stack with live visual dashboards
echo "[4/4] Launching Gazebo + Nav2 + Perception + Mission Manager + RViz2..."
ros2 launch rover_bringup full_sim.launch.py \
    world:="$WORLD_PATH" \
    headless:="$HEADLESS" \
    use_rviz:="$USE_RVIZ" \
    run_mission:=true \
    inspection_mode:="$METHOD" \
    output_dir:="$DEMO_OUT_DIR"
