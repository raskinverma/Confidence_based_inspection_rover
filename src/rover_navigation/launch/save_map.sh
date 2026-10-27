#!/usr/bin/env bash
# Save the current SLAM map to the maps/ directory
# Usage: bash save_map.sh [map_name]
MAP_NAME="${1:-inspection_map}"
MAPS_DIR="$(ros2 pkg prefix rover_navigation)/share/rover_navigation/maps"
mkdir -p "$MAPS_DIR"
echo "[save_map] Saving map to: ${MAPS_DIR}/${MAP_NAME}"
ros2 run nav2_map_server map_saver_cli -f "${MAPS_DIR}/${MAP_NAME}" \
  --ros-args -p use_sim_time:=true
echo "[save_map] Done. Files: ${MAP_NAME}.pgm  ${MAP_NAME}.yaml"
