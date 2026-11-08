#!/usr/bin/env python3
"""
Defect Logger Node for Inspection Rover (Phase 2)
Functions:
  1. Subscribes to /defect_events
  2. Updates Probabilistic Multi-View DefectMap with outlier-robust fusion
  3. Periodically exports verified non-spurious defects to inspection_defects.csv
  4. Publishes /inspection/hypotheses (std_msgs/String JSON) at 2 Hz for active planner
  5. Publishes /inspection/markers and /defects (visualization_msgs/MarkerArray) for RViz2
"""

import os
import csv
import math
import json
from datetime import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from rover_interfaces.msg import Defect
from rover_perception.defect_map import DefectMap


class DefectLogger(Node):
    def __init__(self):
        super().__init__('defect_logger')

        # --- Output directory ---
        self.declare_parameter('output_dir', 'results')
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        output_dir = os.path.expanduser(output_dir)
        if not os.path.isabs(output_dir):
            output_dir = os.path.join(os.getcwd(), output_dir)
        os.makedirs(output_dir, exist_ok=True)

        self.csv_file = os.path.join(output_dir, 'inspection_defects.csv')

        self.declare_parameter('spatial_threshold', 1.10)
        self.declare_parameter('merge_radius', 0.90)
        self.declare_parameter('inspection_mode', 'baseline')
        spatial_thresh = float(self.get_parameter('spatial_threshold').value)
        merge_rad = float(self.get_parameter('merge_radius').value)
        self.inspection_mode = self.get_parameter('inspection_mode').get_parameter_value().string_value

        # Probabilistic Defect Map manager
        self.defect_map = DefectMap(
            association_radius=spatial_thresh,
            merge_radius=merge_rad,
            confirmation_confidence=0.75,
            confirmation_min_obs=4
        )

        # Ensure empty CSV is initialized
        self.flush_csv()

        self._last_logged_status = {}
        self.latest_robot_pos = None
        self.latest_robot_yaw = 0.0
        self.latest_scan = None

        # Subscriptions
        self.sub_event = self.create_subscription(
            Defect,
            '/defect_events',
            self.defect_event_cb,
            10
        )
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_cb,
            10
        )
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            10
        )

        # Publishers
        self.pub_markers = self.create_publisher(
            MarkerArray,
            '/defects',
            10
        )
        self.pub_inspection_markers = self.create_publisher(
            MarkerArray,
            '/inspection/markers',
            10
        )
        self.pub_hypotheses = self.create_publisher(
            String,
            '/inspection/hypotheses',
            10
        )

        # Periodic timer for RViz markers and active planner hypotheses (2 Hz)
        self.timer = self.create_timer(0.5, self.timer_cb)

        self.get_logger().info(f'DefectLogger initialized with Probabilistic DefectMap. Logging to: {self.csv_file}')

    @property
    def registered_defects(self):
        """Provides backward compatibility for code inspecting logger.registered_defects."""
        return [h.to_dict() for h in self.defect_map.get_confirmed_hypotheses()]

    def flush_csv(self):
        try:
            strict = (getattr(self, 'inspection_mode', 'baseline') == 'active')
            self.defect_map.export_csv(self.csv_file, strict_confirmed_only=strict)
        except Exception as e:
            self.get_logger().error(f"Failed to flush defects CSV: {e}")

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.latest_robot_pos = (p.x, p.y)
        self.latest_robot_yaw = 2.0 * math.atan2(o.z, o.w)

    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def timer_cb(self):
        # 1. Check negative evidence for close-range dwell
        if self.latest_robot_pos is not None:
            self.defect_map.check_negative_evidence(
                self.latest_robot_pos[0],
                self.latest_robot_pos[1],
                self.latest_robot_yaw,
                self.latest_scan,
                dt=0.5
            )

        # 2. Publish hypotheses JSON for active planner
        try:
            json_str = self.defect_map.to_json()
            msg = String()
            msg.data = json_str
            self.pub_hypotheses.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing hypotheses: {e}")

        # 3. Publish markers for RViz2
        self.publish_markers()

    def publish_markers(self):
        try:
            stamp = self.get_clock().now().to_msg()
            marker_array = self.defect_map.to_markers(frame_id='map', stamp=stamp)
            self.pub_inspection_markers.publish(marker_array)
            self.pub_markers.publish(marker_array)
        except Exception as e:
            self.get_logger().error(f"Error publishing markers: {e}")

    def defect_event_cb(self, msg: Defect):
        p_obs = np.array([msg.map_position.x, msg.map_position.y, msg.map_position.z], dtype=float)
        p_cam = np.array([msg.camera_position.x, msg.camera_position.y, msg.camera_position.z], dtype=float)

        d = float(msg.observation_distance)
        if d <= 0.0:
            d = float(np.linalg.norm(p_obs - p_cam)) if np.linalg.norm(p_cam) > 0.0 else 1.5

        q_det = float(msg.quality)
        if q_det <= 0.0:
            q_det = float(np.clip(msg.confidence, 0.40, 1.0))

        w_obs = float(msg.weight)
        if w_obs <= 0.0:
            w_dist = 1.0 / (0.15 + max(0.1, d) ** 2)
            w_obs = w_dist * 0.7 * q_det

        w_dist = 1.0 / (0.15 + max(0.1, d) ** 2)
        alpha_view = float(np.clip(w_obs / (w_dist * max(0.1, q_det)), 0.2, 1.0))

        now_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        frame_id = msg.header.frame_id or 'map'

        target_hyp = self.defect_map.add_observation(
            p_obs=p_obs,
            defect_type=msg.defect_type,
            w_obs=w_obs,
            d=d,
            alpha_view=alpha_view,
            q_det=q_det,
            p_cam=p_cam,
            timestamp=now_str,
            pixel_u=msg.pixel_u,
            pixel_v=msg.pixel_v,
            frame_id=frame_id
        )

        prev_status = self._last_logged_status.get(target_hyp.id, None)
        status_changed = (prev_status != target_hyp.status)
        is_first = (target_hyp.obs_count == 1)
        periodic_sample = (target_hyp.obs_count % 25 == 0 and target_hyp.obs_count <= 100)

        should_log = is_first or status_changed or periodic_sample

        log_str = (
            f"OBSERVED DEFECT: {target_hyp.defect_type} (Hyp ID #{target_hyp.id}) at "
            f"({target_hyp.pos[0]:.2f}, {target_hyp.pos[1]:.2f}, {target_hyp.pos[2]:.2f}) "
            f"[conf={target_hyp.confidence:.2f}, obs_count={target_hyp.obs_count}, "
            f"status={target_hyp.status}, sigma={target_hyp.sigma_pos:.2f}m]"
        )
        self.get_logger().debug(log_str)
        if should_log:
            self.get_logger().info(log_str)
            self._last_logged_status[target_hyp.id] = target_hyp.status

        # Flush updated consolidated CSV
        self.flush_csv()
        self.publish_markers()


def main(args=None):
    rclpy.init(args=args)
    node = DefectLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.flush_csv()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
