#!/usr/bin/env python3
"""
Rover Evaluation Tool (rover_eval)
Evaluates defect detections against ground-truth manifest:
  - Associates detections to GT defects of matching type within radius R
  - Greedy nearest matching (one GT matched per detection set)
  - Computes TP, FP, FN, precision, recall, F1
  - Tracks duplicates (extra detections of already matched GT)
  - Tracks distractor_FP (detections within R of a distractor)
  - Computes mean and median position errors
  - Computes sensitivity checks at R=0.5m, R=0.75m (primary), and R=1.0m
  - Integrates telemetry (mission duration, path length, min LiDAR clearance)
"""

import os
import sys
import csv
import math
import yaml
import argparse
import numpy as np


def load_ground_truth(gt_path):
    with open(gt_path, 'r') as f:
        data = yaml.safe_load(f)
    defects = data.get('defects', [])
    distractors = data.get('distractors', [])
    metadata = data.get('metadata', {})
    return defects, distractors, metadata


def load_detections(csv_path):
    if not os.path.exists(csv_path):
        return []
    detections = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                detections.append({
                    'id': int(row.get('id', 0)),
                    'timestamp': row.get('timestamp', ''),
                    'defect_type': row.get('defect_type', '').strip().lower(),
                    'x': float(row.get('map_x', 0.0)),
                    'y': float(row.get('map_y', 0.0)),
                    'z': float(row.get('map_z', 0.0)),
                    'confidence': float(row.get('confidence', 0.0)),
                    'pixel_u': int(row.get('pixel_u', 0)),
                    'pixel_v': int(row.get('pixel_v', 0)),
                })
            except (ValueError, KeyError):
                continue
    return detections


def load_telemetry(telemetry_path):
    if not os.path.exists(telemetry_path):
        return {
            'status': 'UNKNOWN',
            'mission_duration_sec': 0.0,
            'path_length_m': 0.0,
            'min_clearance_m': 0.0,
            'waypoints_completed': 0,
            'waypoints_total': 0
        }
    with open(telemetry_path, 'r') as f:
        return yaml.safe_load(f) or {}


def evaluate_at_radius(detections, gt_defects, distractors, R):
    """
    Greedy nearest matching at radius R:
      1. Candidate pairs (det, gt) with matching type and dist <= R.
      2. Sorted by distance ascending.
      3. First detection matching an unmatched GT becomes TP.
      4. Subsequent detections matching an already-matched GT become Duplicates.
      5. Remaining detections within R of any distractor become distractor_FP.
      6. All other remaining detections become other_FP.
    """
    candidate_pairs = []
    for d_idx, det in enumerate(detections):
        for g_idx, gt in enumerate(gt_defects):
            gt_type = gt.get('type', '').strip().lower()
            if det['defect_type'] == gt_type:
                gx, gy = gt['position'][0], gt['position'][1]
                dist = math.hypot(det['x'] - gx, det['y'] - gy)
                if dist <= R:
                    candidate_pairs.append((dist, d_idx, g_idx))

    candidate_pairs.sort(key=lambda x: x[0])

    matched_gts = set()
    matched_dets = set()
    duplicate_dets = set()
    errors = []

    for dist, d_idx, g_idx in candidate_pairs:
        if d_idx in matched_dets or d_idx in duplicate_dets:
            continue
        if g_idx not in matched_gts:
            matched_gts.add(g_idx)
            matched_dets.add(d_idx)
            errors.append(dist)
        else:
            duplicate_dets.add(d_idx)

    # Remaining detections
    unmatched_dets = set(range(len(detections))) - matched_dets - duplicate_dets
    distractor_fps = set()
    other_fps = set()

    for d_idx in unmatched_dets:
        det = detections[d_idx]
        min_dist_distractor = float('inf')
        for distractor in distractors:
            pos = distractor.get('position', [0, 0, 0])
            d = math.hypot(det['x'] - pos[0], det['y'] - pos[1])
            if d < min_dist_distractor:
                min_dist_distractor = d

        if min_dist_distractor <= R:
            distractor_fps.add(d_idx)
        else:
            other_fps.add(d_idx)

    tp = len(matched_gts)
    fn = len(gt_defects) - tp
    duplicates = len(duplicate_dets)
    distractor_fp = len(distractor_fps)
    other_fp = len(other_fps)
    fp = distractor_fp + other_fp

    # Precision: TP / total detections made (including duplicates and FPs)
    total_detections = len(detections)
    precision = (tp / total_detections) if total_detections > 0 else 0.0
    recall = (tp / len(gt_defects)) if len(gt_defects) > 0 else 0.0
    f1 = (2 * precision * recall / (precision + recall)) if (precision + recall) > 0 else 0.0

    mean_err = float(np.mean(errors)) if errors else 0.0
    median_err = float(np.median(errors)) if errors else 0.0

    return {
        f'tp_R{R:.2f}': tp,
        f'fp_R{R:.2f}': fp,
        f'fn_R{R:.2f}': fn,
        f'duplicates_R{R:.2f}': duplicates,
        f'distractor_fp_R{R:.2f}': distractor_fp,
        f'other_fp_R{R:.2f}': other_fp,
        f'precision_R{R:.2f}': round(precision, 4),
        f'recall_R{R:.2f}': round(recall, 4),
        f'f1_R{R:.2f}': round(f1, 4),
        f'mean_err_R{R:.2f}': round(mean_err, 4),
        f'median_err_R{R:.2f}': round(median_err, 4),
    }


def evaluate_run(gt_file, csv_file, telemetry_file=None):
    gt_defects, distractors, meta = load_ground_truth(gt_file)
    detections = load_detections(csv_file)
    telemetry = load_telemetry(telemetry_file) if telemetry_file else {}

    # Primary radius R=0.75m
    res_075 = evaluate_at_radius(detections, gt_defects, distractors, 0.75)
    # Sensitivity checks R=0.5m and R=1.0m
    res_050 = evaluate_at_radius(detections, gt_defects, distractors, 0.50)
    res_100 = evaluate_at_radius(detections, gt_defects, distractors, 1.00)

    # Combine into a single metrics row
    row = {
        'seed': meta.get('seed', 0),
        'num_gt_defects': len(gt_defects),
        'num_distractors': len(distractors),
        'camera_noise': meta.get('camera_noise', 0.0),
        'total_detections': len(detections),
        # Primary metrics (R=0.75m)
        'tp': res_075['tp_R0.75'],
        'fp': res_075['fp_R0.75'],
        'fn': res_075['fn_R0.75'],
        'duplicates': res_075['duplicates_R0.75'],
        'distractor_fp': res_075['distractor_fp_R0.75'],
        'other_fp': res_075['other_fp_R0.75'],
        'precision': res_075['precision_R0.75'],
        'recall': res_075['recall_R0.75'],
        'f1': res_075['f1_R0.75'],
        'mean_pos_error': res_075['mean_err_R0.75'],
        'median_pos_error': res_075['median_err_R0.75'],
        # Sensitivity R=0.50m
        'f1_R0.50': res_050['f1_R0.50'],
        'tp_R0.50': res_050['tp_R0.50'],
        'precision_R0.50': res_050['precision_R0.50'],
        'recall_R0.50': res_050['recall_R0.50'],
        # Sensitivity R=1.00m
        'f1_R1.00': res_100['f1_R1.00'],
        'tp_R1.00': res_100['tp_R1.00'],
        'precision_R1.00': res_100['precision_R1.00'],
        'recall_R1.00': res_100['recall_R1.00'],
        # Telemetry
        'mission_status': telemetry.get('status', 'UNKNOWN'),
        'mission_duration_sec': telemetry.get('mission_duration_sec', 0.0),
        'path_length_m': telemetry.get('path_length_m', 0.0),
        'min_clearance_m': telemetry.get('min_clearance_m', 0.0),
        'waypoints_completed': telemetry.get('waypoints_completed', 0),
        'waypoints_total': telemetry.get('waypoints_total', 0),
    }
    return row


def main():
    parser = argparse.ArgumentParser(description="Evaluate Rover inspection detections against ground truth.")
    parser.add_argument('--ground-truth', '-g', required=True, help="Path to ground_truth.yaml")
    parser.add_argument('--defects-csv', '-d', required=True, help="Path to inspection_defects.csv")
    parser.add_argument('--telemetry', '-t', default=None, help="Path to run_telemetry.yaml")
    parser.add_argument('--output-csv', '-o', default=None, help="Append row to CSV file")
    args = parser.parse_args()

    metrics = evaluate_run(args.ground_truth, args.defects_csv, args.telemetry)

    print("\n=== Inspection Rover Evaluation Results ===")
    for k, v in metrics.items():
        print(f"  {k:22s}: {v}")

    if args.output_csv:
        file_exists = os.path.exists(args.output_csv)
        os.makedirs(os.path.dirname(os.path.abspath(args.output_csv)), exist_ok=True)
        with open(args.output_csv, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(metrics.keys()))
            if not file_exists:
                writer.writeheader()
            writer.writerow(metrics)
        print(f"\nAppended metrics to {args.output_csv}")


if __name__ == '__main__':
    main()
