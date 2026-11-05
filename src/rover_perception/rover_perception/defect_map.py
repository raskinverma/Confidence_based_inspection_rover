#!/usr/bin/env python3
"""
Probabilistic Multi-View Defect Map with Outlier-Robust Fusion (Phase 2)
Tracks per-hypothesis log-odds confidence, fuses 3D observations using
observation-quality weights, prunes spatial outliers, dynamically merges
converging clusters, and exports verified defect manifests.
"""

import os
import math
import json
import csv
from datetime import datetime
from typing import List, Dict, Optional, Tuple, Set
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray


def weighted_median_1d(values: np.ndarray, weights: np.ndarray) -> float:
    """Computes the weighted median of a 1D array."""
    if len(values) == 0:
        return 0.0
    if len(values) == 1:
        return float(values[0])
    idx = np.argsort(values)
    sorted_v = values[idx]
    sorted_w = weights[idx]
    cumsum_w = np.cumsum(sorted_w)
    cutoff = cumsum_w[-1] / 2.0
    k = np.searchsorted(cumsum_w, cutoff)
    k = min(k, len(sorted_v) - 1)
    return float(sorted_v[k])


def is_compatible_defect_type(t1: str, t2: str) -> bool:
    """Checks whether two defect type labels represent the same class/defect."""
    s1 = t1.strip().lower()
    s2 = t2.strip().lower()
    if s1 == s2:
        return True
    if s1.startswith('aruco') and s2.startswith('aruco'):
        if '_' in s1 and '_' in s2:
            return s1 == s2
        return True
    return False


class DefectHypothesis:
    """Represents a single physical defect hypothesis with probabilistic confidence."""

    def __init__(self, hyp_id: int, defect_type: str, initial_pos: np.ndarray,
                 initial_timestamp: str = "", frame_id: str = "map"):
        self.id: int = hyp_id
        self.defect_type: str = defect_type
        self.frame_id: str = frame_id
        self.pos: np.ndarray = np.array(initial_pos, dtype=float)
        self.sigma_pos: float = 0.35

        # Log-odds confidence tracking: L0 = -1.2 -> P ~= 0.231 (CANDIDATE)
        self.log_odds: float = -1.2
        self.confidence: float = 1.0 / (1.0 + math.exp(-self.log_odds))
        self.obs_count: int = 0
        self.status: str = "CANDIDATE"

        # Multi-view angular and range bins
        self.distinct_view_bins: Set[Tuple[int, int]] = set()

        # Observation history buffer (up to 150 observations)
        self.observations: List[Dict] = []

        self.first_timestamp: str = initial_timestamp or datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self.last_timestamp: str = self.first_timestamp
        self.pixel_u: int = 0
        self.pixel_v: int = 0

    def add_observation(self, p_obs: np.ndarray, w_obs: float, d: float,
                        alpha_view: float, q_det: float, p_cam: Optional[np.ndarray] = None,
                        timestamp: str = "", pixel_u: int = 0, pixel_v: int = 0):
        """Adds a single observation to the hypothesis buffer and updates belief."""
        ts = timestamp or datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self.last_timestamp = ts
        self.pixel_u = pixel_u
        self.pixel_v = pixel_v
        self.obs_count += 1

        cam_pt = np.array(p_cam, dtype=float) if p_cam is not None else self.pos

        # Discretize viewing azimuth (30-deg bins) and range (0.5m bins)
        dx = cam_pt[0] - self.pos[0]
        dy = cam_pt[1] - self.pos[1]
        azimuth = math.atan2(dy, dx)
        angle_bin = int(math.floor(azimuth / (math.pi / 6.0)))
        dist_bin = int(d / 0.5)
        view_bin = (angle_bin, dist_bin)

        is_new_angle = angle_bin not in {b[0] for b in self.distinct_view_bins}
        self.distinct_view_bins.add(view_bin)

        # Buffer observation
        obs_entry = {
            'pos': np.array(p_obs, dtype=float),
            'weight': float(max(1e-4, w_obs)),
            'dist': float(d),
            'alpha': float(alpha_view),
            'quality': float(q_det),
            'cam': cam_pt,
            'timestamp': ts
        }
        self.observations.append(obs_entry)
        if len(self.observations) > 150:
            self.observations.pop(0)

        # Log-odds update: delta_L = clamp(0.22 * q * alpha * (1.8 / (0.4 + d)), 0.04, 0.35)
        base_delta = 0.22 * q_det * alpha_view * (1.8 / (0.4 + max(0.1, d)))
        multi_view_bonus = 0.08 if is_new_angle else 0.0
        delta_L = max(0.04, min(0.35, base_delta + multi_view_bonus))

        self.log_odds = min(4.5, self.log_odds + delta_L)
        self.confidence = float(1.0 / (1.0 + math.exp(-self.log_odds)))

        # Recompute fused position and uncertainty
        self.recompute_fused_position()

        # Update status
        self.update_status()

    def recompute_fused_position(self):
        """Computes outlier-robust weighted position and uncertainty from observation buffer."""
        n = len(self.observations)
        if n == 0:
            return

        pts = np.array([o['pos'] for o in self.observations])
        weights = np.array([o['weight'] for o in self.observations])

        if n <= 3:
            total_w = float(np.sum(weights))
            if total_w > 0:
                self.pos = np.sum(pts * weights[:, None], axis=0) / total_w
            self.sigma_pos = 0.25
            return

        # N >= 4: Coordinate-wise weighted median
        m_x = weighted_median_1d(pts[:, 0], weights)
        m_y = weighted_median_1d(pts[:, 1], weights)
        m_z = weighted_median_1d(pts[:, 2], weights)
        median_xy = np.array([m_x, m_y])

        # Distances to weighted median in XY plane
        xy_dists = np.linalg.norm(pts[:, :2] - median_xy, axis=1)
        med_dist = float(np.median(xy_dists))
        mad = float(np.median(np.abs(xy_dists - med_dist)))
        r_thresh = max(0.35, 2.0 * mad)

        # Inlier filter
        inlier_mask = (xy_dists <= r_thresh)
        if np.sum(inlier_mask) < 2:
            # Fallback to closest 3 observations if threshold is overly aggressive
            top_idx = np.argsort(xy_dists)[:3]
            inlier_mask = np.zeros(n, dtype=bool)
            inlier_mask[top_idx] = True

        inlier_pts = pts[inlier_mask]
        inlier_weights = weights[inlier_mask]
        total_inlier_w = float(np.sum(inlier_weights))

        if total_inlier_w > 0:
            self.pos = np.sum(inlier_pts * inlier_weights[:, None], axis=0) / total_inlier_w

            # Compute weighted standard deviation in XY as position uncertainty
            errs_xy = np.linalg.norm(inlier_pts[:, :2] - self.pos[:2], axis=1)
            var_xy = np.sum(inlier_weights * (errs_xy ** 2)) / total_inlier_w
            self.sigma_pos = float(max(0.04, math.sqrt(var_xy)))

    def update_status(self):
        """Updates hypothesis status based on confirmation thresholds."""
        is_aruco = self.defect_type.lower().startswith('aruco')
        min_obs = 3 if is_aruco else 5

        if self.confidence >= 0.75 and self.obs_count >= min_obs:
            self.status = "CONFIRMED"
        elif self.confidence >= 0.25 or self.obs_count >= 2:
            self.status = "UNCERTAIN"
        else:
            self.status = "CANDIDATE"

    def merge_with(self, other: 'DefectHypothesis'):
        """Merges another hypothesis into this one, combining observations and belief."""
        # Combine observation buffers and keep up to 150 newest
        self.observations.extend(other.observations)
        self.observations.sort(key=lambda o: o['timestamp'])
        if len(self.observations) > 150:
            self.observations = self.observations[-150:]

        self.obs_count += other.obs_count
        self.distinct_view_bins |= other.distinct_view_bins
        self.first_timestamp = min(self.first_timestamp, other.first_timestamp)
        self.last_timestamp = max(self.last_timestamp, other.last_timestamp)

        # Combine log-odds using Bayesian evidence pooling
        delta_L_other = max(0.0, other.log_odds - (-1.2))
        self.log_odds = min(4.5, self.log_odds + delta_L_other)
        self.confidence = float(1.0 / (1.0 + math.exp(-self.log_odds)))

        self.recompute_fused_position()
        self.update_status()

    def to_dict(self) -> Dict:
        return {
            'id': self.id,
            'defect_type': self.defect_type,
            'x': round(float(self.pos[0]), 3),
            'y': round(float(self.pos[1]), 3),
            'z': round(float(self.pos[2]), 3),
            'confidence': round(float(self.confidence), 3),
            'log_odds': round(float(self.log_odds), 3),
            'sigma_pos': round(float(self.sigma_pos), 3),
            'obs_count': int(self.obs_count),
            'status': self.status,
            'num_view_bins': len(self.distinct_view_bins),
            'pixel_u': int(self.pixel_u),
            'pixel_v': int(self.pixel_v),
            'timestamp': self.first_timestamp,
            'last_timestamp': self.last_timestamp
        }


class DefectMap:
    """Manages the full collection of defect hypotheses."""

    def __init__(self, association_radius: float = 1.10, merge_radius: float = 0.90,
                 confirmation_confidence: float = 0.75, confirmation_min_obs: int = 4):
        self.association_radius: float = association_radius
        self.merge_radius: float = merge_radius
        self.confirmation_confidence: float = confirmation_confidence
        self.confirmation_min_obs: int = confirmation_min_obs
        self.hypotheses: List[DefectHypothesis] = []
        self.next_id: int = 1

    def add_observation(self, p_obs: np.ndarray, defect_type: str, w_obs: float,
                        d: float, alpha_view: float, q_det: float,
                        p_cam: Optional[np.ndarray] = None, timestamp: str = "",
                        pixel_u: int = 0, pixel_v: int = 0, frame_id: str = "map") -> DefectHypothesis:
        """Associates observation to nearest compatible hypothesis or creates a new one."""
        p_arr = np.array(p_obs, dtype=float)
        best_hyp = None
        min_dist = float('inf')

        for hyp in self.hypotheses:
            if is_compatible_defect_type(hyp.defect_type, defect_type):
                dist = float(np.linalg.norm(hyp.pos[:2] - p_arr[:2]))
                if dist <= self.association_radius and dist < min_dist:
                    min_dist = dist
                    best_hyp = hyp

        if best_hyp is not None:
            best_hyp.add_observation(p_arr, w_obs, d, alpha_view, q_det, p_cam, timestamp, pixel_u, pixel_v)
            target_hyp = best_hyp
        else:
            new_hyp = DefectHypothesis(self.next_id, defect_type, p_arr, timestamp, frame_id)
            self.next_id += 1
            new_hyp.add_observation(p_arr, w_obs, d, alpha_view, q_det, p_cam, timestamp, pixel_u, pixel_v)
            self.hypotheses.append(new_hyp)
            target_hyp = new_hyp

        # Periodically run cluster merge pass to prevent split clusters
        self.merge_clusters()

        return target_hyp

    def merge_clusters(self):
        """Pass over active hypotheses to merge clusters within merge_radius."""
        merged = True
        while merged:
            merged = False
            n = len(self.hypotheses)
            for i in range(n):
                for j in range(i + 1, n):
                    h1 = self.hypotheses[i]
                    h2 = self.hypotheses[j]
                    if is_compatible_defect_type(h1.defect_type, h2.defect_type):
                        dist_xy = float(np.linalg.norm(h1.pos[:2] - h2.pos[:2]))
                        if dist_xy <= self.merge_radius:
                            # Merge into the hypothesis with more observations
                            if h2.obs_count > h1.obs_count:
                                h2.merge_with(h1)
                                self.hypotheses[i] = h2
                                self.hypotheses.pop(j)
                            else:
                                h1.merge_with(h2)
                                self.hypotheses.pop(j)
                            merged = True
                            break
                if merged:
                    break

    def get_confirmed_hypotheses(self) -> List[DefectHypothesis]:
        """Returns verified hypotheses meeting confirmation criteria."""
        confirmed = []
        for h in self.hypotheses:
            is_aruco = h.defect_type.lower().startswith('aruco')
            if is_aruco:
                # ArUco has high fidelity: confirmed if >= 3 detections with confidence >= 0.50
                if h.obs_count >= 3 and h.confidence >= 0.50:
                    confirmed.append(h)
            else:
                # Color defects (leak, rust): confirmed if status is CONFIRMED or obs_count >= 4 and confidence >= 0.55
                if h.status == "CONFIRMED" or (h.obs_count >= self.confirmation_min_obs and h.confidence >= 0.55):
                    confirmed.append(h)
        return confirmed

    def get_uncertain_hypotheses(self) -> List[DefectHypothesis]:
        """Returns non-confirmed candidate and uncertain hypotheses for active inspection."""
        return [h for h in self.hypotheses if h.status in ("UNCERTAIN", "CANDIDATE")]

    def to_json(self) -> str:
        """Serializes all hypotheses to JSON for /inspection/hypotheses."""
        return json.dumps([h.to_dict() for h in self.hypotheses])

    def to_markers(self, frame_id: str = "map", stamp=None) -> MarkerArray:
        """Generates visualization markers for RViz2."""
        marker_array = MarkerArray()

        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        marker_array.markers.append(del_marker)

        for h in self.hypotheses:
            # 1. Sphere marker
            sphere = Marker()
            sphere.header.frame_id = frame_id
            if stamp is not None:
                sphere.header.stamp = stamp
            sphere.ns = "defect_hypotheses_spheres"
            sphere.id = h.id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD

            sphere.pose.position.x = float(h.pos[0])
            sphere.pose.position.y = float(h.pos[1])
            sphere.pose.position.z = float(max(0.12, h.pos[2]))
            sphere.pose.orientation.w = 1.0

            scale = max(0.15, min(0.45, h.sigma_pos * 2.0))
            sphere.scale.x = scale
            sphere.scale.y = scale
            sphere.scale.z = scale

            if h.status == "CONFIRMED":
                sphere.color.r = 0.1
                sphere.color.g = 0.9
                sphere.color.b = 0.1
                sphere.color.a = 0.90
            elif h.status == "UNCERTAIN":
                sphere.color.r = 1.0
                sphere.color.g = 0.55
                sphere.color.b = 0.0
                sphere.color.a = 0.80
            else:  # CANDIDATE
                sphere.color.r = 0.6
                sphere.color.g = 0.6
                sphere.color.b = 0.6
                sphere.color.a = 0.40

            marker_array.markers.append(sphere)

            # 2. Text label marker
            label = Marker()
            label.header.frame_id = frame_id
            if stamp is not None:
                label.header.stamp = stamp
            label.ns = "defect_hypotheses_labels"
            label.id = h.id + 10000
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD

            label.pose.position.x = float(h.pos[0])
            label.pose.position.y = float(h.pos[1])
            label.pose.position.z = float(max(0.12, h.pos[2])) + 0.25
            label.pose.orientation.w = 1.0

            label.scale.z = 0.12
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.95
            label.text = f"{h.defect_type} C={h.confidence:.2f} (n={h.obs_count})"

            marker_array.markers.append(label)

        return marker_array

    def export_csv(self, file_path: str):
        """Exports confirmed non-spurious hypotheses to CSV format compatible with rover_eval."""
        os.makedirs(os.path.dirname(os.path.abspath(file_path)), exist_ok=True)
        temp_file = file_path + ".tmp"

        confirmed_hypotheses = self.get_confirmed_hypotheses()

        with open(temp_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'id',
                'timestamp',
                'defect_type',
                'map_x',
                'map_y',
                'map_z',
                'confidence',
                'pixel_u',
                'pixel_v',
                'obs_count',
                'status',
                'sigma_pos'
            ])
            for idx, h in enumerate(confirmed_hypotheses, start=1):
                writer.writerow([
                    idx,
                    h.first_timestamp,
                    h.defect_type,
                    f"{h.pos[0]:.3f}",
                    f"{h.pos[1]:.3f}",
                    f"{h.pos[2]:.3f}",
                    f"{h.confidence:.2f}",
                    h.pixel_u,
                    h.pixel_v,
                    h.obs_count,
                    h.status,
                    f"{h.sigma_pos:.3f}"
                ])
        os.replace(temp_file, file_path)
