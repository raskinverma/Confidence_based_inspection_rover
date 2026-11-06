#!/usr/bin/env python3
"""
Unit tests for Probabilistic Defect Map with Outlier-Robust Fusion (Phase 2).
Covers:
  1. test_outlier_rejection: Robustness against distant grazing/noisy projections
  2. test_spurious_fp_rejection: Gating prevents 1-2 frame false positives from entering CSV
  3. test_cluster_merging: Merging multi-view streams for the same physical defect
  4. test_log_odds_belief_dynamics: Confidence accumulation and capping
"""

import os
import sys
import tempfile
import unittest
import numpy as np

# Add src/rover_perception to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'rover_perception')))
from rover_perception.defect_map import DefectMap, DefectHypothesis


class TestDefectMap(unittest.TestCase):

    def test_outlier_rejection(self):
        """
        Feed 2 distant/bad projections at d=3.2m with high noise (x + 0.9, y - 0.7),
        followed by 15 close-range observations at d=1.2m.
        Verify fused position error < 0.08m from true location.
        """
        true_pos = np.array([2.0, 1.0, 0.02])
        dmap = DefectMap(association_radius=1.20, merge_radius=0.90)

        # 2 distant outlier projections
        outlier_pos = true_pos + np.array([0.9, -0.7, 0.0])
        d_out = 3.2
        w_out = 1.0 / (0.15 + d_out ** 2)  # ~0.096
        for _ in range(2):
            dmap.add_observation(
                p_obs=outlier_pos,
                defect_type="leak",
                w_obs=w_out,
                d=d_out,
                alpha_view=0.5,
                q_det=0.6,
                p_cam=np.array([true_pos[0] - 2.5, true_pos[1] - 2.0, 0.3])
            )

        # 15 close-range inlier observations with slight Gaussian jitter
        np.random.seed(42)
        d_in = 1.2
        w_in = 1.0 / (0.15 + d_in ** 2)  # ~0.629
        for i in range(15):
            noise = np.random.normal(0.0, 0.02, size=3)
            noise[2] = 0.0
            inlier_pos = true_pos + noise
            # Sweep camera position slightly as the rover moves past
            p_cam = np.array([true_pos[0] - 1.2 + 0.08 * i, true_pos[1] - 0.3 + 0.04 * i, 0.3])
            dmap.add_observation(
                p_obs=inlier_pos,
                defect_type="leak",
                w_obs=w_in,
                d=d_in,
                alpha_view=0.9,
                q_det=0.85,
                p_cam=p_cam
            )

        # There should be exactly 1 consolidated hypothesis
        confirmed = dmap.get_confirmed_hypotheses()
        self.assertEqual(len(confirmed), 1)

        hyp = confirmed[0]
        pos_err = np.linalg.norm(hyp.pos[:2] - true_pos[:2])
        self.assertLess(pos_err, 0.08, f"Fused position error {pos_err:.4f}m exceeds 0.08m")
        self.assertEqual(hyp.obs_count, 17)
        self.assertEqual(hyp.status, "CONFIRMED")
        self.assertGreater(hyp.confidence, 0.85)

    def test_spurious_fp_rejection(self):
        """
        Feed 2 transient observations (simulating Det IDs 4, 9, 10).
        Verify status is CANDIDATE or UNCERTAIN, obs_count < 4,
        and it is NOT included in the exported inspection_defects.csv.
        """
        dmap = DefectMap(association_radius=1.10, merge_radius=0.90)

        # Observation 1
        dmap.add_observation(
            p_obs=np.array([-1.2, 3.4, 0.02]),
            defect_type="leak",
            w_obs=0.10,
            d=3.0,
            alpha_view=0.3,
            q_det=0.45
        )
        # Observation 2
        dmap.add_observation(
            p_obs=np.array([-1.22, 3.42, 0.02]),
            defect_type="leak",
            w_obs=0.10,
            d=3.0,
            alpha_view=0.3,
            q_det=0.45
        )

        self.assertEqual(len(dmap.hypotheses), 1)
        hyp = dmap.hypotheses[0]
        self.assertIn(hyp.status, ["CANDIDATE", "UNCERTAIN"])
        self.assertLess(hyp.obs_count, 4)
        self.assertLess(hyp.confidence, 0.55)

        # Test CSV export: must be empty (only headers, 0 data rows)
        with tempfile.NamedTemporaryFile(suffix='.csv', delete=False) as tf:
            csv_path = tf.name

        try:
            dmap.export_csv(csv_path)
            with open(csv_path, 'r') as f:
                lines = [l.strip() for l in f if l.strip()]
            # Line 0 is header, no data rows
            self.assertEqual(len(lines), 1, f"Expected only header line, got {len(lines)} lines")
            self.assertIn("id,timestamp,defect_type", lines[0])
        finally:
            if os.path.exists(csv_path):
                os.remove(csv_path)

    def test_cluster_merging(self):
        """
        Create two separate observation streams starting 1.0m apart,
        let them converge to (-2.50, 0.38) within 0.90m.
        Verify that defect_map.merge_clusters() merges them into exactly 1 hypothesis
        with consolidated obs_count.
        """
        dmap = DefectMap(association_radius=0.60, merge_radius=0.90)

        target_pos = np.array([-2.50, 0.38, 0.02])

        # Stream 1: approaching from hallway A and converging to target_pos
        for i in range(8):
            fraction = (7 - i) / 7.0  # goes from 1.0 down to 0.0
            pos = target_pos + np.array([-0.35 * fraction, 0.25 * fraction, 0.0])
            dmap.add_observation(
                p_obs=pos,
                defect_type="leak",
                w_obs=0.3 + 0.4 * (1.0 - fraction),
                d=2.0 - 0.8 * (1.0 - fraction),
                alpha_view=0.7 + 0.2 * (1.0 - fraction),
                q_det=0.8,
                p_cam=np.array([target_pos[0] - 1.5, target_pos[1], 0.3])
            )

        # Stream 2: approaching from hallway B and converging to target_pos
        for i in range(8):
            fraction = (7 - i) / 7.0  # goes from 1.0 down to 0.0
            pos = target_pos + np.array([0.35 * fraction, -0.25 * fraction, 0.0])
            dmap.add_observation(
                p_obs=pos,
                defect_type="leak",
                w_obs=0.3 + 0.4 * (1.0 - fraction),
                d=2.0 - 0.8 * (1.0 - fraction),
                alpha_view=0.7 + 0.2 * (1.0 - fraction),
                q_det=0.8,
                p_cam=np.array([target_pos[0], target_pos[1] - 1.5, 0.3])
            )

        # The two streams are ~0.72m apart (<= merge_radius 0.90m)
        # merge_clusters() should have automatically merged them
        self.assertEqual(len(dmap.hypotheses), 1)
        hyp = dmap.hypotheses[0]
        self.assertEqual(hyp.obs_count, 16)
        self.assertEqual(hyp.status, "CONFIRMED")
        err = np.linalg.norm(hyp.pos[:2] - target_pos[:2])
        self.assertLess(err, 0.20)

    def test_log_odds_belief_dynamics(self):
        """
        Verify that log-odds increments correctly with distance and viewing angle,
        and caps at L_max = 4.5 (P ~= 0.989).
        """
        hyp = DefectHypothesis(1, "rust", np.array([1.0, 2.0, 0.5]))
        self.assertAlmostEqual(hyp.log_odds, -1.2, places=2)
        initial_conf = hyp.confidence
        self.assertAlmostEqual(initial_conf, 1.0 / (1.0 + np.exp(1.2)), places=2)

        # Add single observation at d=1.0m, alpha=0.8, q=0.8
        hyp.add_observation(
            p_obs=np.array([1.0, 2.0, 0.5]),
            w_obs=0.5,
            d=1.0,
            alpha_view=0.8,
            q_det=0.8,
            p_cam=np.array([0.0, 2.0, 0.5])
        )
        self.assertGreater(hyp.log_odds, -1.2)
        self.assertGreater(hyp.confidence, initial_conf)

        # Add 40 strong multi-view observations to test ceiling
        for i in range(40):
            angle = i * (np.pi / 6.0)
            p_cam = np.array([1.0 + np.cos(angle), 2.0 + np.sin(angle), 0.5])
            hyp.add_observation(
                p_obs=np.array([1.0, 2.0, 0.5]),
                w_obs=0.8,
                d=1.0,
                alpha_view=0.9,
                q_det=0.9,
                p_cam=p_cam
            )

        self.assertAlmostEqual(hyp.log_odds, 4.5, places=2)
        self.assertGreater(hyp.confidence, 0.985)
        self.assertEqual(hyp.status, "CONFIRMED")

    def test_hypothesis_ghost_rejection(self):
        """
        Verify that a candidate hypothesis subjected to >= 2.5s of direct,
        unobstructed close-range viewing without positive detections is marked REJECTED.
        """
        dmap = DefectMap(association_radius=1.10, merge_radius=0.90)

        # 1. Single transient observation generates a CANDIDATE
        target_pos = np.array([1.0, 0.0, 0.3])
        dmap.add_observation(
            p_obs=target_pos,
            defect_type="rust",
            w_obs=0.10,
            d=3.0,
            alpha_view=0.3,
            q_det=0.45
        )
        self.assertEqual(len(dmap.hypotheses), 1)
        hyp = dmap.hypotheses[0]
        self.assertIn(hyp.status, ["CANDIDATE", "UNCERTAIN"])

        # 2. Robot moves to (0.0, 0.0) facing East (yaw = 0.0), looking directly at (1.0, 0.0)
        # Perform dwell checks over 3.0 seconds (6 steps of dt=0.5s)
        # Pretend last positive observation was 2.0s ago
        import time
        hyp.last_positive_obs_time = time.time() - 2.0

        for _ in range(6):
            dmap.check_negative_evidence(
                robot_x=0.0,
                robot_y=0.0,
                robot_yaw=0.0,
                scan=None,
                dt=0.5
            )

        self.assertEqual(hyp.status, "REJECTED")
        self.assertGreaterEqual(hyp.negative_dwell_time, 2.5)

        # REJECTED hypotheses must not be returned in confirmed or uncertain lists
        self.assertEqual(len(dmap.get_confirmed_hypotheses()), 0)
        self.assertEqual(len(dmap.get_uncertain_hypotheses()), 0)


if __name__ == '__main__':
    unittest.main()
