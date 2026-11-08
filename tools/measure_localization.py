#!/usr/bin/env python3
"""
Measure Localization Error: AMCL vs Gazebo Ground Truth
Subscribes to:
  1. ROS 2 topic /amcl_pose (geometry_msgs/msg/PoseWithCovarianceStamped)
  2. Gazebo topic /world/<world_name>/dynamic_pose/info (via `gz topic -e`)
Extracts ground-truth position of model "inspection_rover",
aligns in time, and reports:
  - Initial pose offset
  - Mean localization error (m)
  - Max localization error (m)
"""

import sys
import os
import re
import math
import time
import argparse
import subprocess
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class LocErrorNode(Node):
    def __init__(self, world_name: str, duration_sec: float = 60.0):
        super().__init__('loc_error_measure')
        self.world_name = world_name
        self.duration_sec = duration_sec

        self.latest_amcl = None
        self.latest_gt = None
        self.errors = []
        self.initial_offset = None
        self.running = True

        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_cb,
            10
        )

        # Thread to stream gz topic
        self.gz_thread = threading.Thread(target=self.stream_gz_poses, daemon=True)
        self.gz_thread.start()

        self.timer = self.create_timer(0.2, self.record_error)

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self.latest_amcl = (p.x, p.y)

    def stream_gz_poses(self):
        cmd = ['gz', 'topic', '-e', '-t', f'/world/{self.world_name}/dynamic_pose/info']
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

        buf = ''
        while self.running and proc.poll() is None:
            line = proc.stdout.readline()
            if not line:
                break
            buf += line
            if 'pose {' in line:
                # Keep buffer reasonable
                if len(buf) > 10000:
                    buf = buf[-5000:]
            # Look for rover pose block
            if 'name: "rover"' in buf or 'name: "inspection_rover"' in buf:
                m = re.search(r'name: "(?:rover|inspection_rover)".*?position \{\s*x:\s*([^\s]+)\s*y:\s*([^\s]+)', buf, re.DOTALL)
                if m:
                    try:
                        gx = float(m.group(1))
                        gy = float(m.group(2))
                        self.latest_gt = (gx, gy)
                    except ValueError:
                        pass
        try:
            proc.terminate()
        except Exception:
            pass

    def record_error(self):
        if self.latest_amcl is not None and self.latest_gt is not None:
            dx = self.latest_amcl[0] - self.latest_gt[0]
            dy = self.latest_amcl[1] - self.latest_gt[1]
            dist = math.hypot(dx, dy)
            if self.initial_offset is None:
                self.initial_offset = dist
                self.get_logger().info(f"Initial pose offset recorded: {dist:.4f} m (AMCL={self.latest_amcl}, GT={self.latest_gt})")
            self.errors.append(dist)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', default='run_clean_seed1')
    parser.add_argument('--duration', type=float, default=45.0)
    args = parser.parse_args()

    rclpy.init()
    node = LocErrorNode(args.world, args.duration)

    start = time.time()
    try:
        while time.time() - start < args.duration and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

    print("\n=== LOCALIZATION ACCURACY REPORT ===")
    if node.errors:
        print(f"Sample count        : {len(node.errors)}")
        print(f"Initial Pose Offset : {node.initial_offset:.4f} m")
        print(f"Mean Localization Err: {np.mean(node.errors):.4f} m")
        print(f"Median Error        : {np.median(node.errors):.4f} m")
        print(f"Max Localization Err : {np.max(node.errors):.4f} m")
        print(f"Std Dev             : {np.std(node.errors):.4f} m")
    else:
        print("No paired samples recorded.")


if __name__ == '__main__':
    main()
