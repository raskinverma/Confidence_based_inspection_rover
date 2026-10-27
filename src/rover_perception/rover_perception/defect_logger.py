#!/usr/bin/env python3
"""
Defect Logger Node for Inspection Rover
Functions:
  1. Subscribes to /defect_events
  2. Spatial deduplication (< 0.35 m radius per defect class)
  3. Logs unique defects to CSV file
  4. Publishes /defects (visualization_msgs/MarkerArray) for persistent RViz rendering

Fix (2026-10-27): output directory is now a ROS parameter 'output_dir' instead of
  a hardcoded ~/College/... path. Default is a 'results/' folder relative to the
  current working directory. The directory is created automatically if missing.
"""

import os
import csv
import math
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from visualization_msgs.msg import Marker, MarkerArray
from rover_interfaces.msg import Defect


class DefectLogger(Node):
    def __init__(self):
        super().__init__('defect_logger')

        # --- Output directory (ROS parameter, not hardcoded path) ---
        self.declare_parameter('output_dir', 'results')
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value

        # Expand ~ and make the directory if it doesn't exist
        output_dir = os.path.expanduser(output_dir)
        if not os.path.isabs(output_dir):
            # relative path: anchor to cwd at launch time
            output_dir = os.path.join(os.getcwd(), output_dir)
        os.makedirs(output_dir, exist_ok=True)

        self.csv_file = os.path.join(output_dir, 'inspection_defects.csv')
        self.init_csv()

        # In-memory database of registered defects
        # Each item: {'id': int, 'type': str, 'x': float, 'y': float, 'z': float,
        #              'conf': float, 'count': int, 'frame_id': str}
        self.registered_defects = []
        self.spatial_threshold = 0.35  # metres

        # Subscriptions
        self.sub_event = self.create_subscription(
            Defect,
            '/defect_events',
            self.defect_event_cb,
            10
        )

        # Publisher for RViz visualization
        self.pub_markers = self.create_publisher(
            MarkerArray,
            '/defects',
            10
        )

        # Periodic timer to republish persistent markers (every 1.0 s)
        self.timer = self.create_timer(1.0, self.publish_markers)

        self.get_logger().info(f'DefectLogger initialized. Logging to: {self.csv_file}')

    def init_csv(self):
        with open(self.csv_file, mode='w', newline='') as f:
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
                'pixel_v'
            ])

    def defect_event_cb(self, msg: Defect):
        x = msg.map_position.x
        y = msg.map_position.y
        z = msg.map_position.z

        # Check for proximity match (deduplicate by type + location)
        matched = False
        for d in self.registered_defects:
            dist = math.hypot(d['x'] - x, d['y'] - y)
            if dist < self.spatial_threshold and d['type'] == msg.defect_type:
                # Update moving average position and max confidence
                d['count'] += 1
                d['conf'] = max(d['conf'], msg.confidence)
                d['x'] = 0.8 * d['x'] + 0.2 * x
                d['y'] = 0.8 * d['y'] + 0.2 * y
                matched = True
                break

        if not matched:
            new_id = len(self.registered_defects) + 1
            record = {
                'id': new_id,
                'type': msg.defect_type,
                'x': x,
                'y': y,
                'z': z,
                'conf': msg.confidence,
                'count': 1,
                'frame_id': msg.header.frame_id or 'map'
            }
            self.registered_defects.append(record)

            # Write to CSV
            now_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            with open(self.csv_file, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    new_id,
                    now_str,
                    msg.defect_type,
                    f'{x:.3f}',
                    f'{y:.3f}',
                    f'{z:.3f}',
                    f'{msg.confidence:.2f}',
                    msg.pixel_u,
                    msg.pixel_v
                ])

            self.get_logger().info(
                f"NEW DEFECT LOGGED #{new_id}: {msg.defect_type} at ({x:.2f}, {y:.2f}) "
                f"[conf={msg.confidence:.2f}] -> {self.csv_file}"
            )

            # Immediately update RViz markers
            self.publish_markers()

    def publish_markers(self):
        if not self.registered_defects:
            return

        marker_array = MarkerArray()

        for d in self.registered_defects:
            # 1. Sphere marker
            marker = Marker()
            marker.header.frame_id = d['frame_id']
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'defect_spheres'
            marker.id = d['id']
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = d['x']
            marker.pose.position.y = d['y']
            marker.pose.position.z = max(0.15, d['z'])
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25

            # Colour by defect type
            if 'rust' in d['type']:
                marker.color.r = 1.0
                marker.color.g = 0.1
                marker.color.b = 0.1
                marker.color.a = 0.9
            elif 'leak' in d['type']:
                marker.color.r = 1.0
                marker.color.g = 0.9
                marker.color.b = 0.0
                marker.color.a = 0.9
            else:  # ArUco
                marker.color.r = 0.1
                marker.color.g = 0.5
                marker.color.b = 1.0
                marker.color.a = 0.9

            marker_array.markers.append(marker)

            # 2. Text label marker above the sphere
            text_marker = Marker()
            text_marker.header.frame_id = d['frame_id']
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'defect_labels'
            text_marker.id = d['id'] + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = d['x']
            text_marker.pose.position.y = d['y']
            text_marker.pose.position.z = max(0.15, d['z']) + 0.25
            text_marker.pose.orientation.w = 1.0

            text_marker.scale.z = 0.12  # text height
            text_marker.text = f"{d['type']} ({d['conf']:.2f})"
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            marker_array.markers.append(text_marker)

        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = DefectLogger()
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
