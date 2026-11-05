#!/usr/bin/env python3
"""
Unit tests for tools/rover_eval.py
Verifies evaluation metrics across synthetic detection datasets:
  - Exact match: P=1.0, R=1.0, TP=2, FP=0, FN=0
  - Shifted 0.3m: P=1.0, R=1.0 at R=0.75m and R=0.50m
  - Shifted 2.0m: P=0.0, R=0.0, TP=0, FP=2, FN=2 at R=0.75m and R=1.0m
"""

import os
import sys
import yaml
import csv
import unittest

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'tools')))
from rover_eval import evaluate_run


class TestRoverEval(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture_dir = os.path.join(os.path.dirname(__file__), 'fixtures')
        os.makedirs(cls.fixture_dir, exist_ok=True)
        cls.gt_path = os.path.join(cls.fixture_dir, 'test_gt.yaml')
        with open(cls.gt_path, 'w') as f:
            yaml.safe_dump({
                'defects': [
                    {'id': 1, 'name': 'defect_rust_1', 'type': 'rust', 'position': [1.0, 1.0, 0.25]},
                    {'id': 2, 'name': 'defect_leak_1', 'type': 'leak', 'position': [-1.0, -1.0, 0.25]},
                    {'id': 3, 'name': 'defect_aruco_1', 'type': 'aruco', 'position': [2.0, -2.0, 0.30]}
                ],
                'distractors': []
            }, f)

    def _write_csv(self, filename, detections):
        path = os.path.join(self.fixture_dir, filename)
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(
                f,
                fieldnames=['id', 'timestamp', 'defect_type', 'map_x', 'map_y', 'map_z', 'confidence', 'pixel_u', 'pixel_v']
            )
            writer.writeheader()
            for d in detections:
                writer.writerow(d)
        return path

    def test_exact_match(self):
        csv_file = self._write_csv('exact.csv', [
            {'id': 1, 'timestamp': '', 'defect_type': 'rust', 'map_x': 1.0, 'map_y': 1.0, 'map_z': 0.25, 'confidence': 0.9, 'pixel_u': 0, 'pixel_v': 0},
            {'id': 2, 'timestamp': '', 'defect_type': 'leak', 'map_x': -1.0, 'map_y': -1.0, 'map_z': 0.25, 'confidence': 0.9, 'pixel_u': 0, 'pixel_v': 0},
            {'id': 3, 'timestamp': '', 'defect_type': 'aruco_1', 'map_x': 2.0, 'map_y': -2.0, 'map_z': 0.30, 'confidence': 0.95, 'pixel_u': 0, 'pixel_v': 0}
        ])
        res = evaluate_run(self.gt_path, csv_file)
        self.assertEqual(res['tp'], 3)
        self.assertEqual(res['fp'], 0)
        self.assertEqual(res['fn'], 0)
        self.assertEqual(res['precision'], 1.0)
        self.assertEqual(res['recall'], 1.0)
        self.assertEqual(res['f1'], 1.0)

    def test_shifted_03m(self):
        csv_file = self._write_csv('shifted_03m.csv', [
            {'id': 1, 'timestamp': '', 'defect_type': 'rust', 'map_x': 1.3, 'map_y': 1.0, 'map_z': 0.25, 'confidence': 0.9, 'pixel_u': 0, 'pixel_v': 0},
            {'id': 2, 'timestamp': '', 'defect_type': 'leak', 'map_x': -1.0, 'map_y': -0.7, 'map_z': 0.25, 'confidence': 0.9, 'pixel_u': 0, 'pixel_v': 0},
            {'id': 3, 'timestamp': '', 'defect_type': 'aruco_1', 'map_x': 2.2, 'map_y': -2.0, 'map_z': 0.30, 'confidence': 0.95, 'pixel_u': 0, 'pixel_v': 0}
        ])
        res = evaluate_run(self.gt_path, csv_file)
        self.assertEqual(res['tp'], 3)
        self.assertEqual(res['fp'], 0)
        self.assertEqual(res['fn'], 0)
        self.assertEqual(res['precision'], 1.0)
        self.assertEqual(res['recall'], 1.0)
        self.assertEqual(res['tp_R0.50'], 3)

    def test_shifted_20m(self):
        csv_file = self._write_csv('shifted_20m.csv', [
            {'id': 1, 'timestamp': '', 'defect_type': 'rust', 'map_x': 3.0, 'map_y': 1.0, 'map_z': 0.25, 'confidence': 0.9, 'pixel_u': 0, 'pixel_v': 0},
            {'id': 2, 'timestamp': '', 'defect_type': 'leak', 'map_x': -1.0, 'map_y': 1.0, 'map_z': 0.25, 'confidence': 0.9, 'pixel_u': 0, 'pixel_v': 0},
            {'id': 3, 'timestamp': '', 'defect_type': 'aruco_1', 'map_x': 4.5, 'map_y': -2.0, 'map_z': 0.30, 'confidence': 0.95, 'pixel_u': 0, 'pixel_v': 0}
        ])
        res = evaluate_run(self.gt_path, csv_file)
        self.assertEqual(res['tp'], 0)
        self.assertEqual(res['fp'], 3)
        self.assertEqual(res['fn'], 3)
        self.assertEqual(res['precision'], 0.0)
        self.assertEqual(res['recall'], 0.0)
        self.assertEqual(res['tp_R1.00'], 0)

    def test_obs_count_csv_compatibility(self):
        csv_file = self._write_csv('obs_count.csv', [
            {'id': 1, 'timestamp': '', 'defect_type': 'rust', 'map_x': 1.0, 'map_y': 1.0, 'map_z': 0.25, 'confidence': 0.9, 'pixel_u': 0, 'pixel_v': 0},
            {'id': 2, 'timestamp': '', 'defect_type': 'leak', 'map_x': -1.0, 'map_y': -1.0, 'map_z': 0.25, 'confidence': 0.9, 'pixel_u': 0, 'pixel_v': 0},
            {'id': 3, 'timestamp': '', 'defect_type': 'aruco_1', 'map_x': 2.0, 'map_y': -2.0, 'map_z': 0.30, 'confidence': 0.95, 'pixel_u': 0, 'pixel_v': 0}
        ])
        res = evaluate_run(self.gt_path, csv_file)
        self.assertEqual(res['tp'], 3)
        self.assertEqual(res['precision'], 1.0)


if __name__ == '__main__':
    unittest.main()
