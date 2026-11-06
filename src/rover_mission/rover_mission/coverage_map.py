#!/usr/bin/env python3
"""
Frustum-Clipped Inspection Coverage Map (Phase 3)
Tracks 2D spatial inspection coverage across the 10m x 10m facility.
Updates coverage using camera frustum raycasting clipped by 2D LiDAR ranges,
evaluates information gain for candidate viewpoints, and publishes
an OccupancyGrid on /inspection/coverage_grid for RViz2 visualization.
"""

import math
from typing import List, Tuple, Optional
import numpy as np
from nav_msgs.msg import OccupancyGrid


def normalize_angle(theta: float) -> float:
    """Normalizes angle to [-pi, pi]."""
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta < -math.pi:
        theta += 2.0 * math.pi
    return theta


class CoverageMap:
    """
    Represents the 10m x 10m facility as a 2D grid at 0.25m resolution (40x40 cells).
    Origin: (-5.0, -5.0).
    """

    def __init__(self, x_min: float = -5.0, x_max: float = 5.0,
                 y_min: float = -5.0, y_max: float = 5.0, resolution: float = 0.25):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.resolution = resolution
        self.cols = int(round((x_max - x_min) / resolution))  # 40
        self.rows = int(round((y_max - y_min) / resolution))  # 40

        # Coverage values in [0.0, 1.0]
        self.grid = np.zeros((self.rows, self.cols), dtype=np.float32)

        # Static obstacle mask (True = wall/out of bounds, False = free inspectable)
        self.obstacle_mask = np.zeros((self.rows, self.cols), dtype=bool)
        self._init_static_obstacles()

        # Cache of inspectable cell indices
        self.inspectable_indices = np.where(~self.obstacle_mask)
        self.num_inspectable = len(self.inspectable_indices[0])

    def _init_static_obstacles(self):
        """Initializes facility wall bounds and internal walls."""
        for r in range(self.rows):
            for c in range(self.cols):
                cx, cy = self.cell_to_world(r, c)

                # Outer walls: facility interior is [-4.85, 4.85] x [-3.85, 3.85]
                if abs(cx) > 4.85 or abs(cy) > 3.85:
                    self.obstacle_mask[r, c] = True
                    continue

                # Inner North Wall: x in [-0.20, 0.20], y in [0.35, 4.0]
                if -0.20 <= cx <= 0.20 and 0.35 <= cy <= 4.0:
                    self.obstacle_mask[r, c] = True
                    continue

                # Inner South Wall: x in [-0.20, 0.20], y in [-2.85, -0.35]
                if -0.20 <= cx <= 0.20 and -2.85 <= cy <= -0.35:
                    self.obstacle_mask[r, c] = True
                    continue

                # Room A Sub-wall: x in [-4.05, -0.95], y in [0.35, 0.65]
                if -4.05 <= cx <= -0.95 and 0.35 <= cy <= 0.65:
                    self.obstacle_mask[r, c] = True
                    continue

                # Corridor Divider Wall: x in [0.05, 4.95], y in [-2.15, -1.85]
                if 0.05 <= cx <= 4.95 and -2.15 <= cy <= -1.85:
                    self.obstacle_mask[r, c] = True
                    continue

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        c = int((x - self.x_min) / self.resolution)
        r = int((y - self.y_min) / self.resolution)
        return r, c

    def cell_to_world(self, r: int, c: int) -> Tuple[float, float]:
        cx = self.x_min + (c + 0.5) * self.resolution
        cy = self.y_min + (r + 0.5) * self.resolution
        return cx, cy

    def is_in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.rows and 0 <= c < self.cols

    def is_navigable(self, x: float, y: float, clearance: float = 0.40) -> bool:
        """Returns True if (x, y) is in bounds and at least `clearance` m clear of obstacles."""
        if x < -4.5 or x > 4.5 or y < -3.5 or y > 3.5:
            return False

        r, c = self.world_to_cell(x, y)
        if not self.is_in_bounds(r, c) or self.obstacle_mask[r, c]:
            return False

        cell_radius = int(math.ceil(clearance / self.resolution))
        for dr in range(-cell_radius, cell_radius + 1):
            for dc in range(-cell_radius, cell_radius + 1):
                nr, nc = r + dr, c + dc
                if not self.is_in_bounds(nr, nc) or self.obstacle_mask[nr, nc]:
                    dist = math.hypot(dr * self.resolution, dc * self.resolution)
                    if dist <= clearance:
                        return False
        return True

    def update_coverage(self, robot_x: float, robot_y: float, robot_yaw: float, scan=None):
        """
        Updates coverage grid by casting rays across camera horizontal FOV (±35 deg).
        Clipped by LiDAR range to prevent seeing through walls.
        """
        fov_half = math.radians(35.0)
        num_rays = 40
        ray_angles = np.linspace(-fov_half, fov_half, num_rays)
        max_inspect_dist = 3.0

        for rel_angle in ray_angles:
            abs_angle = normalize_angle(robot_yaw + rel_angle)

            # Determine clipping distance from LiDAR
            clip_dist = max_inspect_dist
            if scan is not None:
                beam_idx = int((rel_angle - scan.angle_min) / scan.angle_increment)
                if 0 <= beam_idx < len(scan.ranges):
                    r_lidar = scan.ranges[beam_idx]
                    if not (math.isnan(r_lidar) or math.isinf(r_lidar)) and r_lidar > 0.15:
                        clip_dist = min(max_inspect_dist, max(0.20, r_lidar - 0.08))

            # Step along the ray
            dist = 0.25
            step_size = 0.15
            while dist <= clip_dist:
                px = robot_x + dist * math.cos(abs_angle)
                py = robot_y + dist * math.sin(abs_angle)
                r, c = self.world_to_cell(px, py)

                if not self.is_in_bounds(r, c):
                    break
                if self.obstacle_mask[r, c]:
                    # Hit wall obstacle: ray cannot pass through
                    break

                # Inspectable cell observed
                delta_cov = 0.40 * max(0.25, 1.0 - (dist / 3.5))
                self.grid[r, c] = min(1.0, self.grid[r, c] + delta_cov)
                dist += step_size

    def get_coverage_ratio(self) -> float:
        """Returns fraction of reachable inspectable cells with cov >= 0.50."""
        if self.num_inspectable == 0:
            return 0.0
        covered_count = np.sum(self.grid[self.inspectable_indices] >= 0.50)
        return float(covered_count / self.num_inspectable)

    def get_unseen_frustum_gain(self, vx: float, vy: float, vyaw: float,
                                max_range: float = 3.0) -> float:
        """
        Estimates the expected unseen coverage gain from candidate viewpoint (vx, vy, vyaw).
        Returns sum of (1.0 - current_cov) for inspectable cells in view frustum.
        """
        fov_half = math.radians(35.0)
        num_rays = 20
        ray_angles = np.linspace(-fov_half, fov_half, num_rays)
        seen_cells = set()
        gain = 0.0

        for rel_angle in ray_angles:
            abs_angle = normalize_angle(vyaw + rel_angle)
            dist = 0.35
            step = 0.25
            while dist <= max_range:
                px = vx + dist * math.cos(abs_angle)
                py = vy + dist * math.sin(abs_angle)
                r, c = self.world_to_cell(px, py)
                if not self.is_in_bounds(r, c) or self.obstacle_mask[r, c]:
                    break
                if (r, c) not in seen_cells:
                    seen_cells.add((r, c))
                    gain += max(0.0, 1.0 - float(self.grid[r, c]))
                dist += step

        return float(gain)

    def to_occupancy_grid(self, frame_id: str = "map", stamp=None) -> OccupancyGrid:
        """Converts coverage grid to nav_msgs/OccupancyGrid for RViz2."""
        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        if stamp is not None:
            msg.header.stamp = stamp

        msg.info.resolution = float(self.resolution)
        msg.info.width = int(self.cols)
        msg.info.height = int(self.rows)
        msg.info.origin.position.x = float(self.x_min)
        msg.info.origin.position.y = float(self.y_min)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        data = []
        for r in range(self.rows):
            for c in range(self.cols):
                if self.obstacle_mask[r, c]:
                    data.append(-1)
                else:
                    cov_pct = int(round(self.grid[r, c] * 100.0))
                    data.append(int(np.clip(cov_pct, 0, 100)))

        msg.data = data
        return msg
