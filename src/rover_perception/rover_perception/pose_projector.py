#!/usr/bin/env python3
"""
Pose Projector Node for Inspection Rover
Projects 2D image pixel coordinates into 3D world coordinates:
  1. Uses pinhole camera model + LiDAR depth correlation
  2. Transforms 3D point in camera_optical_frame to map frame via tf2
  3. Publishes /defect_events with filled map_position

Fix (2026-10-27): use rclpy.time.Time() (latest available transform) instead of
  msg.header.stamp to avoid "TF extrapolation into the future" warnings that occur
  because the 30 Hz camera timestamp frequently outpaces the TF buffer.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CameraInfo, LaserScan
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import tf2_ros
import tf2_geometry_msgs
from rover_interfaces.msg import Defect


class PoseProjector(Node):
    def __init__(self):
        super().__init__('pose_projector')

        self.camera_info = None
        self.latest_scan = None
        self.latest_map = None
        self.rejected_count = 0

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Target world frame
        self.target_frame = 'map'

        # Subscriptions
        self.sub_cam_info = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.cam_info_cb,
            10
        )
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            10
        )
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_cb,
            map_qos
        )
        self.sub_defect = self.create_subscription(
            Defect,
            '/defect_detections',
            self.defect_cb,
            10
        )

        # Publisher for geo-tagged defects
        self.pub_defect_event = self.create_publisher(
            Defect,
            '/defect_events',
            10
        )

        self.get_logger().info('PoseProjector initialized: Pixel -> camera_optical_frame -> map via TF2 with bounds validation')

    def cam_info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def map_cb(self, msg: OccupancyGrid):
        self.latest_map = msg

    def get_lidar_range_for_bearing(self, bearing: float):
        """
        Looks up 2D LiDAR range along angle_lidar = -bearing.
        Returns float distance in meters, or None if unavailable/invalid.
        """
        if self.latest_scan is None:
            return None

        scan = self.latest_scan
        angle_lidar = -bearing
        if not (scan.angle_min <= angle_lidar <= scan.angle_max):
            return None

        idx = int((angle_lidar - scan.angle_min) / scan.angle_increment)
        samples = []
        for delta in (-1, 0, 1):
            i = idx + delta
            if 0 <= i < len(scan.ranges):
                r = scan.ranges[i]
                if not math.isnan(r) and not math.isinf(r) and scan.range_min < r < scan.range_max:
                    samples.append(r)

        if not samples:
            return None

        # Check for sharp depth discontinuity across adjacent beams (e.g. obstacle silhouette edges)
        if len(samples) > 1 and (max(samples) - min(samples) > 0.50):
            if 0 <= idx < len(scan.ranges):
                r_center = scan.ranges[idx]
                if not math.isnan(r_center) and not math.isinf(r_center) and scan.range_min < r_center < scan.range_max:
                    if abs(r_center - min(samples)) < 0.15:
                        return float(r_center)
                    elif abs(r_center - max(samples)) < 0.15:
                        return float(r_center)
            return None

        median_r = float(np.median(samples))
        if math.isnan(median_r) or math.isinf(median_r) or median_r <= scan.range_min or median_r >= scan.range_max:
            return None

        return median_r

    def compute_wall_incidence(self, bearing: float, r_ray: np.ndarray) -> float:
        """
        Estimates local wall normal from adjacent LiDAR scan beams (+/-3 beams)
        and computes incidence cosine in [0.2, 1.0].
        """
        if self.latest_scan is None:
            return 0.7

        scan = self.latest_scan
        angle_lidar = -bearing
        if not (scan.angle_min <= angle_lidar <= scan.angle_max):
            return 0.7

        idx = int((angle_lidar - scan.angle_min) / scan.angle_increment)
        i1 = max(0, idx - 3)
        i2 = min(len(scan.ranges) - 1, idx + 3)
        if i1 == i2:
            return 0.7

        r1 = scan.ranges[i1]
        r2 = scan.ranges[i2]
        if math.isnan(r1) or math.isinf(r1) or math.isnan(r2) or math.isinf(r2):
            return 0.7
        if not (scan.range_min < r1 < scan.range_max and scan.range_min < r2 < scan.range_max):
            return 0.7

        th1 = scan.angle_min + i1 * scan.angle_increment
        th2 = scan.angle_min + i2 * scan.angle_increment
        p1 = np.array([r1 * math.cos(th1), r1 * math.sin(th1)])
        p2 = np.array([r2 * math.cos(th2), r2 * math.sin(th2)])
        tangent = p2 - p1
        tan_len = np.linalg.norm(tangent)
        if tan_len < 0.04 or tan_len > 1.2:
            return 0.7

        # 2D normal perpendicular to wall tangent
        normal = np.array([-tangent[1], tangent[0]]) / tan_len
        r_mid = np.array([math.cos(angle_lidar), math.sin(angle_lidar)])
        cos_inc = abs(float(np.dot(r_mid, normal)))
        r_xy = math.hypot(r_ray[0], r_ray[1])
        return float(np.clip(cos_inc * r_xy, 0.2, 1.0))

    def defect_cb(self, msg: Defect):
        if self.camera_info is None:
            self.get_logger().warn('No CameraInfo received yet, skipping projection')
            return

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        # Avoid divide by zero (fallback to 640x480 @60deg HFOV intrinsics)
        if fx == 0.0 or fy == 0.0:
            fx = 554.25
            fy = 554.25
            cx = 320.0
            cy = 240.0

        u = msg.pixel_u
        v = msg.pixel_v

        # 1. Optical frame ray vector for pixel (u, v)
        dx = float(u - cx) / fx
        dy = float(v - cy) / fy
        dz = 1.0
        norm_opt = math.sqrt(dx * dx + dy * dy + dz * dz)
        dir_opt = [dx / norm_opt, dy / norm_opt, dz / norm_opt]

        # Points in camera_optical_frame: origin (camera center) and step along ray
        pt_origin = PointStamped()
        pt_origin.header.stamp = rclpy.time.Time().to_msg()
        pt_origin.header.frame_id = 'camera_optical_frame'
        pt_origin.point.x = 0.0
        pt_origin.point.y = 0.0
        pt_origin.point.z = 0.0

        pt_ray = PointStamped()
        pt_ray.header.stamp = rclpy.time.Time().to_msg()
        pt_ray.header.frame_id = 'camera_optical_frame'
        pt_ray.point.x = dir_opt[0]
        pt_ray.point.y = dir_opt[1]
        pt_ray.point.z = dir_opt[2]

        if not self.tf_buffer.can_transform(
            self.target_frame, 'camera_optical_frame',
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.05)
        ):
            self.rejected_count += 1
            self.get_logger().info(
                f"Rejected observation #{self.rejected_count} (TF to {self.target_frame} unavailable): "
                f"{msg.defect_type} at ({u},{v})"
            )
            return

        try:
            tf_origin = self.tf_buffer.transform(
                pt_origin, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.1)
            )
            tf_ray = self.tf_buffer.transform(
                pt_ray, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.rejected_count += 1
            self.get_logger().info(
                f"Rejected observation #{self.rejected_count} (TF transform failed): {e}"
            )
            return

        p_cam = np.array([tf_origin.point.x, tf_origin.point.y, tf_origin.point.z], dtype=float)
        p_ray_pt = np.array([tf_ray.point.x, tf_ray.point.y, tf_ray.point.z], dtype=float)
        ray_vec = p_ray_pt - p_cam
        norm_ray = np.linalg.norm(ray_vec)
        if norm_ray < 1e-6:
            self.rejected_count += 1
            self.get_logger().info(
                f"Rejected observation #{self.rejected_count} (Degenerate ray vector): "
                f"{msg.defect_type} at ({u},{v})"
            )
            return
        r_ray = ray_vec / norm_ray

        # 2. Lookup LiDAR range along azimuth
        bearing = math.atan2(float(u - cx), fx)
        d_lidar = self.get_lidar_range_for_bearing(bearing)

        # 3. Floor intersection check vs. Wall hit
        is_floor = False
        t_hit = None

        if r_ray[2] < -0.03:
            z_floor = 0.02
            t_floor = (z_floor - p_cam[2]) / r_ray[2]
            if 0.25 <= t_floor <= 3.5:
                if d_lidar is None or t_floor < (d_lidar - 0.15):
                    is_floor = True
                    t_hit = t_floor

        if is_floor:
            p_hit = p_cam + t_hit * r_ray
            obs_dist = t_hit
        else:
            if d_lidar is None:
                self.rejected_count += 1
                self.get_logger().info(
                    f"Rejected observation #{self.rejected_count} (invalid LiDAR range for wall): "
                    f"{msg.defect_type} at ({u},{v})"
                )
                return

            r_xy = math.hypot(r_ray[0], r_ray[1])
            if r_xy < 1e-4:
                self.rejected_count += 1
                self.get_logger().info(
                    f"Rejected observation #{self.rejected_count} (vertical camera ray): "
                    f"{msg.defect_type} at ({u},{v})"
                )
                return

            t_wall = d_lidar / r_xy
            p_hit = p_cam + t_wall * r_ray
            obs_dist = float(np.linalg.norm(p_hit - p_cam))

        # 4. Enforce strict Z-bounds, observation distance, and map bounds
        if obs_dist < 0.20 or obs_dist > 3.85:
            self.rejected_count += 1
            self.get_logger().info(
                f"Rejected observation #{self.rejected_count} (observation distance {obs_dist:.2f}m outside [0.20, 3.85]m): "
                f"{msg.defect_type} at ({u},{v})"
            )
            return

        if p_hit[2] < -0.05 or p_hit[2] > 1.20:
            self.rejected_count += 1
            self.get_logger().info(
                f"Rejected observation #{self.rejected_count} (z={p_hit[2]:.2f} out of bounds [-0.05, 1.20]): "
                f"{msg.defect_type} at ({u},{v})"
            )
            return

        if p_hit[0] < -5.15 or p_hit[0] > 5.15 or p_hit[1] < -4.15 or p_hit[1] > 4.15:
            self.rejected_count += 1
            self.get_logger().info(
                f"Rejected observation #{self.rejected_count} (xy=({p_hit[0]:.2f}, {p_hit[1]:.2f}) outside [-5.15, 5.15] x [-4.15, 4.15]): "
                f"{msg.defect_type} at ({u},{v})"
            )
            return

        if self.latest_map is None:
            self.rejected_count += 1
            self.get_logger().info(
                f"Rejected observation #{self.rejected_count} (/map not yet received): "
                f"{msg.defect_type} at ({u},{v})"
            )
            return

        info = self.latest_map.info
        res = info.resolution
        col = int((p_hit[0] - info.origin.position.x) / res)
        row = int((p_hit[1] - info.origin.position.y) / res)

        if col < 0 or col >= info.width or row < 0 or row >= info.height:
            self.rejected_count += 1
            self.get_logger().info(
                f"Rejected observation #{self.rejected_count} (outside map bounds): "
                f"{msg.defect_type} at ({p_hit[0]:.2f}, {p_hit[1]:.2f}) [col={col}, row={row}]"
            )
            return

        cell_idx = row * info.width + col
        grid_val = self.latest_map.data[cell_idx]
        if grid_val == -1:  # unknown space
            self.rejected_count += 1
            self.get_logger().info(
                f"Rejected observation #{self.rejected_count} (in unknown space): "
                f"{msg.defect_type} at ({p_hit[0]:.2f}, {p_hit[1]:.2f}) [val=-1]"
            )
            return

        # Compute Task 1 observation quality factors & spatial fusion weight
        d = float(np.clip(obs_dist, 0.25, 3.50))
        if is_floor:
            alpha_view = float(np.clip(abs(r_ray[2]), 0.2, 1.0))
        else:
            alpha_view = self.compute_wall_incidence(bearing, r_ray)

        w_dist = 1.0 / (0.15 + d * d)
        q_det = float(np.clip(msg.confidence, 0.40, 1.0))
        w_obs = float(w_dist * alpha_view * q_det)

        event = Defect()
        event.header = msg.header
        event.header.frame_id = self.target_frame
        event.defect_type = msg.defect_type
        event.confidence = q_det
        event.pixel_u = msg.pixel_u
        event.pixel_v = msg.pixel_v
        event.map_position.x = float(p_hit[0])
        event.map_position.y = float(p_hit[1])
        event.map_position.z = float(p_hit[2])
        event.weight = w_obs
        event.observation_distance = d
        event.quality = q_det
        event.camera_position.x = float(p_cam[0])
        event.camera_position.y = float(p_cam[1])
        event.camera_position.z = float(p_cam[2])

        self.pub_defect_event.publish(event)
        self.get_logger().info(
            f"Projected {msg.defect_type} ({u},{v}) [mode={'floor' if is_floor else 'wall'}, d={d:.2f}m, w={w_obs:.3f}, alpha={alpha_view:.2f}, q={q_det:.2f}] -> [{self.target_frame}] "
            f"({p_hit[0]:.2f}, {p_hit[1]:.2f}, {p_hit[2]:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseProjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
