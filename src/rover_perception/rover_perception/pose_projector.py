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

    def get_depth_for_pixel(self, u: int, v: int, fx: float, cx: float):
        """
        Estimates depth Z using LiDAR scan correlation.
        Returns float depth Z in meters, or None if invalid range (inf, NaN, out of range).
        """
        if self.latest_scan is None:
            return None

        # Optical frame: angle around vertical axis is approximately:
        # tan(theta) = (u - cx) / fx
        bearing = math.atan2(float(u - cx), fx)

        # In robot body coordinates, camera faces forward (+x), angles are roughly -pi to +pi
        # Look up ray in scan around -bearing (since in optical frame x is right, while in lidar +angle is left)
        scan = self.latest_scan
        angle_lidar = -bearing
        if not (scan.angle_min <= angle_lidar <= scan.angle_max):
            return None

        idx = int((angle_lidar - scan.angle_min) / scan.angle_increment)
        # Sample a narrow window of 3 rays to eliminate noise / zero readings
        samples = []
        for delta in (-1, 0, 1):
            i = idx + delta
            if 0 <= i < len(scan.ranges):
                r = scan.ranges[i]
                if not math.isnan(r) and not math.isinf(r) and scan.range_min < r < scan.range_max:
                    samples.append(r)

        if not samples:
            return None

        median_r = float(np.median(samples))
        if math.isnan(median_r) or math.isinf(median_r) or median_r <= scan.range_min or median_r >= scan.range_max:
            return None

        depth_z = median_r * math.cos(bearing)
        if depth_z < 0.2:
            return None
        return depth_z

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

        # Estimate depth Z (reject observations with invalid LiDAR range)
        Z = self.get_depth_for_pixel(u, v, fx, cx)
        if Z is None:
            self.rejected_count += 1
            self.get_logger().info(
                f"Rejected observation #{self.rejected_count} (invalid LiDAR range): "
                f"{msg.defect_type} at pixel ({u},{v})"
            )
            return

        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

        # Create PointStamped in camera_optical_frame.
        pt_cam = PointStamped()
        pt_cam.header.stamp = rclpy.time.Time().to_msg()   # Time(0) = latest transform
        pt_cam.header.frame_id = 'camera_optical_frame'
        pt_cam.point.x = X
        pt_cam.point.y = Y
        pt_cam.point.z = Z

        # Attempt transform to target_frame (map)
        if not self.tf_buffer.can_transform(
            self.target_frame, pt_cam.header.frame_id,
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
            pt_transformed = self.tf_buffer.transform(
                pt_cam, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.rejected_count += 1
            self.get_logger().info(
                f"Rejected observation #{self.rejected_count} (TF transform failed): {e}"
            )
            return

        # Occupancy grid check: reject if outside occupancy-grid bounds or in unknown space
        if self.latest_map is None:
            self.rejected_count += 1
            self.get_logger().info(
                f"Rejected observation #{self.rejected_count} (/map not yet received): "
                f"{msg.defect_type} at ({u},{v})"
            )
            return

        info = self.latest_map.info
        res = info.resolution
        col = int((pt_transformed.point.x - info.origin.position.x) / res)
        row = int((pt_transformed.point.y - info.origin.position.y) / res)

        if col < 0 or col >= info.width or row < 0 or row >= info.height:
            self.rejected_count += 1
            self.get_logger().info(
                f"Rejected observation #{self.rejected_count} (outside map bounds): "
                f"{msg.defect_type} at ({pt_transformed.point.x:.2f}, {pt_transformed.point.y:.2f}) [col={col}, row={row}]"
            )
            return

        cell_idx = row * info.width + col
        grid_val = self.latest_map.data[cell_idx]
        if grid_val == -1:  # unknown space
            self.rejected_count += 1
            self.get_logger().info(
                f"Rejected observation #{self.rejected_count} (in unknown space): "
                f"{msg.defect_type} at ({pt_transformed.point.x:.2f}, {pt_transformed.point.y:.2f}) [val=-1]"
            )
            return

        event = Defect()
        event.header = msg.header
        event.header.frame_id = self.target_frame
        event.defect_type = msg.defect_type
        event.confidence = msg.confidence
        event.pixel_u = msg.pixel_u
        event.pixel_v = msg.pixel_v
        event.map_position = pt_transformed.point

        self.pub_defect_event.publish(event)
        self.get_logger().info(
            f"Projected {msg.defect_type} ({u},{v}) -> [{self.target_frame}] "
            f"({pt_transformed.point.x:.2f}, {pt_transformed.point.y:.2f}, {pt_transformed.point.z:.2f})"
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
