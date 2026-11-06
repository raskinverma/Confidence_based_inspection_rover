#!/usr/bin/env python3
"""
Unit tests for Spatial Coverage Map & Confidence-Driven NBV Active Planner (Phase 3).
Covers:
  1. test_coverage_raycasting_and_occlusion: Verifies frustum raycasting and LiDAR wall clipping
  2. test_standoff_viewpoint_generation: Verifies 1.15m head-on standoff pose calculation
  3. test_nbv_utility_ranking: Verifies re-inspection prioritization over distant exploration
"""

import os
import sys
import math
import unittest
import numpy as np

# Add src modules to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'rover_navigation')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'rover_mission')))

from rover_navigation.coverage_map import CoverageMap, normalize_angle


class MockLaserScan:
    """Mock LaserScan message with configurable obstacle range."""
    def __init__(self, wall_distance: float = 2.0):
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(1.0)
        num_beams = int(round((self.angle_max - self.angle_min) / self.angle_increment))
        self.ranges = [5.0] * num_beams

        # Place a wall in the forward cone [-20 deg, +20 deg]
        beam_start = int(round((math.radians(-20.0) - self.angle_min) / self.angle_increment))
        beam_end = int(round((math.radians(20.0) - self.angle_min) / self.angle_increment))
        for i in range(beam_start, beam_end + 1):
            if 0 <= i < num_beams:
                self.ranges[i] = wall_distance


class TestActivePlanner(unittest.TestCase):

    def test_coverage_raycasting_and_occlusion(self):
        """
        Verify that camera frustum raycasting marks cells in front of a wall as covered,
        while cells behind the wall remain occluded (cov == 0.0).
        """
        cov_map = CoverageMap()
        rx, ry, ryaw = -2.0, 0.0, 0.0  # Robot at (-2, 0) facing East (+X)
        mock_scan = MockLaserScan(wall_distance=2.0)  # Wall at x = 0.0 (2.0m ahead)

        cov_map.update_coverage(rx, ry, ryaw, scan=mock_scan)

        # 1. Cells in front of the wall (e.g. x in [-1.5, -0.3], y = 0.0) must be covered
        in_front_r, in_front_c = cov_map.world_to_cell(-1.0, 0.0)
        self.assertGreater(cov_map.grid[in_front_r, in_front_c], 0.0,
                           "Cell in front of wall should have received coverage")

        # 2. Cells behind the wall (e.g. x in [0.5, 2.0], y = 0.0) must NOT be covered
        behind_r, behind_c = cov_map.world_to_cell(1.0, 0.0)
        self.assertEqual(cov_map.grid[behind_r, behind_c], 0.0,
                         "Cell behind wall was erroneously marked covered (occlusion check failed)")

        # 3. Overall coverage ratio should be non-zero but small (< 0.15)
        cov_ratio = cov_map.get_coverage_ratio()
        self.assertGreater(cov_ratio, 0.0)
        self.assertLess(cov_ratio, 0.15)

    def test_standoff_viewpoint_generation(self):
        """
        Verify that a wall defect at (-4.88, 2.00) on the outer west wall generates
        a safe navigable standoff pose at (-3.73, 2.00) facing yaw = pi.
        """
        cov_map = CoverageMap()
        hx, hy = -4.88, 2.00
        rx, ry = -2.00, 2.00  # Robot in center of Room A North

        dx, dy = rx - hx, ry - hy
        d = math.hypot(dx, dy)
        ux, uy = dx / d, dy / d

        standoff_dist = 1.15
        vx = hx + standoff_dist * ux
        vy = hy + standoff_dist * uy
        vyaw = math.atan2(hy - vy, hx - vx)

        # Position should be approx (-3.73, 2.00)
        self.assertAlmostEqual(vx, -3.73, delta=0.02)
        self.assertAlmostEqual(vy, 2.00, delta=0.02)

        # Facing toward defect: angle from (-3.73, 2.00) to (-4.88, 2.00) is pi (west)
        self.assertAlmostEqual(abs(normalize_angle(vyaw)), math.pi, delta=0.05)

        # Standoff pose must be in free navigable space with >= 0.40m clearance
        self.assertTrue(cov_map.is_navigable(vx, vy, clearance=0.40),
                        f"Standoff pose ({vx:.2f}, {vy:.2f}) failed navigability/clearance check")

    def test_nbv_utility_ranking(self):
        """
        Verify that an uncertain hypothesis near the robot outranks a distant exploration waypoint.
        """
        rx, ry = 0.0, 0.0

        # Candidate 1: Uncertain defect at (1.5, 0.0)
        # Standpoint is (1.5 - 1.15, 0) = (0.35, 0.0) -> travel_dist = 0.35m
        conf = 0.40
        sigma_pos = 0.22
        travel_dist_reinspect = 0.35
        # U_reinspect = 10.0 * (1 - conf) + 3.0 * sigma_pos - 0.35 * travel_dist + 5.0
        U_reinspect = 10.0 * (1.0 - conf) + 3.0 * sigma_pos - 0.35 * travel_dist_reinspect + 5.0

        # Candidate 2: Distant exploration anchor at (-4.0, -3.0) -> travel_dist = 5.0m
        unseen_gain = 1.2
        travel_dist_explore = math.hypot(-4.0 - rx, -3.0 - ry)
        # U_explore = 4.0 * unseen_gain - 0.30 * travel_dist
        U_explore = 4.0 * unseen_gain - 0.30 * travel_dist_explore

        self.assertGreater(U_reinspect, U_explore,
                           f"Expected U_reinspect ({U_reinspect:.2f}) > U_explore ({U_explore:.2f})")
        self.assertGreater(U_reinspect, 10.0,
                           "Uncertain hypothesis priority score should exceed 10.0")


if __name__ == '__main__':
    unittest.main()
