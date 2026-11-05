#!/usr/bin/env python3
"""
Mission Manager Node for Inspection Rover
Implements Finite State Machine (FSM):
  IDLE -> WAITING_NAV2 -> LOCALIZING -> NAVIGATING -> SCANNING -> NEXT_WP -> COMPLETED / FAILED
Interacts with Nav2 NavigateToPose action server.

Publishes:
  /mission_status (std_msgs/msg/String: RUNNING, DONE, FAILED)
  /initialpose (geometry_msgs/msg/PoseWithCovarianceStamped) to automatically localize Nav2
  /cmd_vel (geometry_msgs/msg/Twist) for in-place 360-deg scanning sweeps

Tracks Telemetry:
  - Total path length integrated from /odom
  - Minimum LiDAR clearance from /scan
  - Mission duration from start to finish
  - Writes run_telemetry.yaml to output_dir
"""

import os
import math
import time
import re
import subprocess
import threading
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped, Pose
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from ament_index_python.packages import get_package_share_directory


class MissionState:
    IDLE = "IDLE"
    WAITING_NAV2 = "WAITING_NAV2"
    LOCALIZING = "LOCALIZING"
    NAVIGATING = "NAVIGATING"
    RETRY_WAIT = "RETRY_WAIT"
    SCANNING = "SCANNING"
    NEXT_WP = "NEXT_WP"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # Load parameters
        pkg_share = get_package_share_directory('rover_mission')
        default_wp_path = os.path.join(pkg_share, 'config', 'inspection_waypoints.yaml')
        self.declare_parameter('waypoints_file', default_wp_path)
        self.declare_parameter('output_dir', 'results')
        self.declare_parameter('initial_x', -2.0)
        self.declare_parameter('initial_y', -1.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('world_name', '')

        wp_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        out_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.initial_x = self.get_parameter('initial_x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('initial_y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('initial_yaw').get_parameter_value().double_value

        self.output_dir = os.path.expanduser(out_dir)
        if not os.path.isabs(self.output_dir):
            self.output_dir = os.path.join(os.getcwd(), self.output_dir)
        os.makedirs(self.output_dir, exist_ok=True)

        self.waypoints = self.load_waypoints(wp_file)
        self.current_wp_idx = 0
        self.state = MissionState.IDLE
        self.retry_count = 0
        self.max_retries = 2
        self.successful_wps = 0

        # Telemetry variables
        self.mission_start_time = None
        self.mission_end_time = None
        self.last_odom_pos = None
        self.path_length = 0.0
        self.min_clearance = float('inf')

        # Localization telemetry
        self.latest_amcl_pos = None
        self.latest_gz_pos = None
        self.initial_loc_err = None
        self.loc_errors = []
        self.max_loc_err = 0.0
        self._last_telemetry_save = 0.0
        self.running = True

        # Thread to stream Gazebo dynamic pose ground truth
        self.gz_thread = threading.Thread(target=self.stream_gz_poses, daemon=True)
        self.gz_thread.start()

        # Publishers
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.pub_status = self.create_publisher(String, '/mission_status', latched_qos)
        self.pub_initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Nav2 Action Client & Lifecycle Service Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.lifecycle_client = self.create_client(GetState, '/bt_navigator/get_state')

        # Subscriptions for telemetry
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.sub_amcl_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_cb, 10)
        self.sub_gz_pose = self.create_subscription(Pose, '/gz/robot_pose', self.gz_pose_cb, 10)

        # Control loop timer (10 Hz)
        self.timer_fsm = self.create_timer(0.1, self.fsm_step)

        self.goal_handle = None
        self.scan_start_time = None
        self.localize_start_time = None
        self.retry_wait_start = None
        self.lifecycle_req_future = None
        self.last_poll_time = 0.0

        self.get_logger().info(f"MissionManager initialized with {len(self.waypoints)} waypoints.")
        self.publish_status("RUNNING")

    def load_waypoints(self, path):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints from {path}: {e}")
            return []

    def publish_status(self, status_str: str):
        msg = String()
        msg.data = status_str
        self.pub_status.publish(msg)
        self.get_logger().info(f"[/mission_status] -> {status_str}")

    def odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        if self.state in (MissionState.NAVIGATING, MissionState.SCANNING, MissionState.NEXT_WP):
            if self.last_odom_pos is not None:
                dx = pos.x - self.last_odom_pos[0]
                dy = pos.y - self.last_odom_pos[1]
                dist = math.hypot(dx, dy)
                if dist < 2.0:  # ignore odometry resets
                    self.path_length += dist
            self.last_odom_pos = (pos.x, pos.y)

    def scan_cb(self, msg: LaserScan):
        if self.state in (MissionState.NAVIGATING, MissionState.SCANNING):
            valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max and not math.isnan(r) and not math.isinf(r)]
            if valid:
                curr_min = min(valid)
                if curr_min < self.min_clearance:
                    self.min_clearance = float(curr_min)

    def amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        self.latest_amcl_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.compute_localization_error()

    def gz_pose_cb(self, msg: Pose):
        self.latest_gz_pos = (msg.position.x, msg.position.y)
        self.compute_localization_error()

    def compute_localization_error(self):
        if self.latest_amcl_pos is not None and self.latest_gz_pos is not None:
            err = math.hypot(
                self.latest_amcl_pos[0] - self.latest_gz_pos[0],
                self.latest_amcl_pos[1] - self.latest_gz_pos[1]
            )
            if self.initial_loc_err is None:
                self.initial_loc_err = err
                self.get_logger().info(
                    f"Initial localization error: {err:.4f} m (AMCL={self.latest_amcl_pos}, GT={self.latest_gz_pos})"
                )
            if self.state in (MissionState.LOCALIZING, MissionState.NAVIGATING, MissionState.SCANNING, MissionState.NEXT_WP):
                self.loc_errors.append(err)
                if err > self.max_loc_err:
                    self.max_loc_err = err

    def stream_gz_poses(self):
        # Determine candidate dynamic_pose topics
        world_arg = self.get_parameter('world_name').get_parameter_value().string_value
        candidate_topics = []
        if world_arg:
            world_base = os.path.basename(world_arg).replace('.sdf', '')
            candidate_topics.append(f"/world/{world_base}/dynamic_pose/info")
        candidate_topics.extend([
            "/world/run_clean_seed1/dynamic_pose/info",
            "/world/run_clean_seed2/dynamic_pose/info",
            "/world/run_clean_seed3/dynamic_pose/info",
            "/world/inspection_facility/dynamic_pose/info",
            "/world/default/dynamic_pose/info"
        ])

        topic = candidate_topics[0] if candidate_topics else None
        time.sleep(2.0)

        # Confirm or discover topic if needed
        for attempt in range(10):
            if not self.running:
                return
            try:
                res = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True, timeout=5.0)
                for line in res.stdout.splitlines():
                    if '/dynamic_pose/info' in line:
                        topic = line.strip()
                        break
            except Exception:
                pass
            if topic:
                break
            time.sleep(1.0)

        if not topic:
            self.get_logger().warn("Could not find dynamic_pose/info topic in Gazebo")
            return

        self.get_logger().info(f"Streaming ground truth rover poses from Gazebo topic: {topic}")
        cmd = ['gz', 'topic', '-e', '-t', topic]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

        buf = ''
        while self.running and proc.poll() is None:
            line = proc.stdout.readline()
            if not line:
                break
            buf += line
            if len(buf) > 10000:
                buf = buf[-5000:]
            if 'name: "rover"' in buf or 'name: "inspection_rover"' in buf:
                m = re.search(r'name: "(?:rover|inspection_rover)".*?position \{\s*x:\s*([^\s]+)\s*y:\s*([^\s]+)', buf, re.DOTALL)
                if m:
                    try:
                        gx = float(m.group(1))
                        gy = float(m.group(2))
                        self.latest_gz_pos = (gx, gy)
                        self.compute_localization_error()
                        buf = buf[m.end():]
                    except ValueError:
                        pass
        try:
            proc.terminate()
        except Exception:
            pass

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.initial_x
        msg.pose.pose.position.y = self.initial_y
        msg.pose.pose.position.z = 0.0

        q = self.yaw_to_quaternion(self.initial_yaw)
        msg.pose.pose.orientation.x = q['x']
        msg.pose.pose.orientation.y = q['y']
        msg.pose.pose.orientation.z = q['z']
        msg.pose.pose.orientation.w = q['w']

        cov = [0.0] * 36
        cov[0] = 0.25   # x var
        cov[7] = 0.25   # y var
        cov[35] = 0.068 # yaw var
        msg.pose.covariance = cov

        self.pub_initial_pose.publish(msg)
        self.get_logger().info(f"Published /initialpose -> ({self.initial_x}, {self.initial_y}, yaw={self.initial_yaw})")

    def yaw_to_quaternion(self, yaw):
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)
        }

    def save_telemetry(self, status: str):
        telemetry_file = os.path.join(self.output_dir, 'run_telemetry.yaml')
        duration = 0.0
        if self.mission_start_time is not None:
            end = self.mission_end_time or self.get_clock().now()
            duration = (end - self.mission_start_time).nanoseconds / 1e9

        min_c = self.min_clearance if self.min_clearance != float('inf') else 0.0
        mean_loc_err = float(np.mean(self.loc_errors)) if self.loc_errors else 0.0

        data = {
            'status': status,
            'mission_duration_sec': round(float(duration), 2),
            'sim_time_sec': round(float(duration), 2),
            'path_length_m': round(float(self.path_length), 2),
            'min_clearance_m': round(float(min_c), 3),
            'waypoints_completed': int(self.successful_wps),
            'waypoints_total': len(self.waypoints),
            'initial_loc_error_m': round(float(self.initial_loc_err), 4) if self.initial_loc_err is not None else 0.0,
            'mean_loc_error_m': round(mean_loc_err, 4),
            'max_loc_error_m': round(float(self.max_loc_err), 4),
        }
        try:
            with open(telemetry_file, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=False)
            self.get_logger().debug(f"Saved run telemetry to {telemetry_file}: {data}")
        except Exception as e:
            self.get_logger().error(f"Failed to write telemetry: {e}")

    def fsm_step(self):
        now_sec = time.time()

        # Periodically flush telemetry every 2 seconds during active mission
        if self.mission_start_time is not None and (now_sec - self._last_telemetry_save >= 2.0):
            st = "DONE" if self.state == MissionState.COMPLETED else self.state
            self.save_telemetry(st)
            self._last_telemetry_save = now_sec

        if self.state == MissionState.IDLE:
            # Check if Nav2 action server is available
            if self.nav_client.wait_for_server(timeout_sec=0.01):
                # Also check if bt_navigator is active via lifecycle service
                if self.lifecycle_client.service_is_ready():
                    self.transition_to(MissionState.WAITING_NAV2)
                else:
                    if now_sec - self.last_poll_time > 3.0:
                        self.get_logger().info("Waiting for /bt_navigator/get_state service...")
                        self.last_poll_time = now_sec
            else:
                if now_sec - self.last_poll_time > 3.0:
                    self.get_logger().info("Waiting for Nav2 'navigate_to_pose' action server...")
                    self.last_poll_time = now_sec

        elif self.state == MissionState.WAITING_NAV2:
            if self.lifecycle_req_future is None:
                req = GetState.Request()
                self.lifecycle_req_future = self.lifecycle_client.call_async(req)
            elif self.lifecycle_req_future.done():
                try:
                    resp = self.lifecycle_req_future.result()
                    state_label = resp.current_state.label
                    self.get_logger().info(f"bt_navigator lifecycle state: '{state_label}'")
                    if state_label == 'active':
                        self.get_logger().info("Nav2 bt_navigator is ACTIVE. Publishing /initialpose...")
                        self.publish_initial_pose()
                        self.localize_start_time = self.get_clock().now()
                        self.transition_to(MissionState.LOCALIZING)
                    else:
                        self.lifecycle_req_future = None
                except Exception as e:
                    self.get_logger().warn(f"Failed to get lifecycle state: {e}")
                    self.lifecycle_req_future = None

        elif self.state == MissionState.LOCALIZING:
            # Allow 2.0 seconds for AMCL particle filter to settle after initial pose
            elapsed = (self.get_clock().now() - self.localize_start_time).nanoseconds / 1e9
            if elapsed < 2.0:
                # Re-publish initial pose once more during settling
                if elapsed > 1.0 and getattr(self, '_repub_done', False) is False:
                    self.publish_initial_pose()
                    self._repub_done = True
            else:
                self.get_logger().info("Localization established. Starting autonomous inspection mission!")
                self.mission_start_time = self.get_clock().now()
                self.transition_to(MissionState.NAVIGATING)
                self.send_current_goal()

        elif self.state == MissionState.RETRY_WAIT:
            # Wait 3.0s before retrying to let Nav2 clear its internal state
            elapsed = (self.get_clock().now() - self.retry_wait_start).nanoseconds / 1e9
            if elapsed >= 3.0:
                self.transition_to(MissionState.NAVIGATING)
                self.send_current_goal()

        elif self.state == MissionState.SCANNING:
            now = self.get_clock().now()
            elapsed = (now - self.scan_start_time).nanoseconds / 1e9
            wp = self.waypoints[self.current_wp_idx]
            duration = float(wp.get('scan_duration', 8.0))

            if elapsed < duration:
                # Rotate smoothly in place at 0.9 rad/s (approx 386 deg in 7.5s)
                cmd = Twist()
                cmd.angular.z = 0.9
                self.pub_cmd_vel.publish(cmd)
            else:
                # Stop rotation
                self.pub_cmd_vel.publish(Twist())
                self.get_logger().info(f"Inspection scan completed for WP #{wp['id']} ({wp['name']})")
                self.transition_to(MissionState.NEXT_WP)

        elif self.state == MissionState.NEXT_WP:
            self.current_wp_idx += 1
            if self.current_wp_idx < len(self.waypoints):
                self.retry_count = 0
                self.transition_to(MissionState.NAVIGATING)
                self.send_current_goal()
            else:
                self.mission_end_time = self.get_clock().now()
                if self.successful_wps > 0:
                    self.transition_to(MissionState.COMPLETED)
                    self.publish_status("DONE")
                    self.save_telemetry("DONE")
                    self.get_logger().info("🎉 MISSION COMPLETE! All waypoints visited and inspected.")
                else:
                    self.transition_to(MissionState.FAILED)
                    self.publish_status("FAILED")
                    self.save_telemetry("FAILED")
                    self.get_logger().error("Mission finished with 0 successful waypoints. Status: FAILED.")

    def transition_to(self, new_state):
        self.get_logger().info(f"[FSM TRANSITION] {self.state} -> {new_state}")
        self.state = new_state
        if new_state == MissionState.FAILED:
            self.mission_end_time = self.get_clock().now()
            self.publish_status("FAILED")
            self.save_telemetry("FAILED")

    def send_current_goal(self):
        wp = self.waypoints[self.current_wp_idx]
        self.get_logger().info(
            f"Navigating to WP #{wp['id']}/{len(self.waypoints)} ({wp['name']}): "
            f"({wp['x']}, {wp['y']}, yaw={wp['yaw']})"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(wp['x'])
        goal_msg.pose.pose.position.y = float(wp['y'])
        goal_msg.pose.pose.position.z = 0.0

        q = self.yaw_to_quaternion(float(wp['yaw']))
        goal_msg.pose.pose.orientation.x = q['x']
        goal_msg.pose.pose.orientation.y = q['y']
        goal_msg.pose.pose.orientation.z = q['z']
        goal_msg.pose.pose.orientation.w = q['w']

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to send goal: {e}")
            self.handle_nav_failure(reject=True)
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected by Nav2!")
            self.handle_nav_failure(reject=True)
            return

        self.get_logger().info(f"Goal accepted by Nav2 (WP #{self.waypoints[self.current_wp_idx]['id']}). Moving...")
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f"Error getting goal result: {e}")
            self.handle_nav_failure(reject=False)
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            wp_id = self.waypoints[self.current_wp_idx]['id']
            self.successful_wps += 1
            self.get_logger().info(f"Reached Waypoint #{wp_id} successfully.")
            self.scan_start_time = self.get_clock().now()
            self.transition_to(MissionState.SCANNING)
        else:
            self.get_logger().warn(f"Goal failed with Nav2 status: {status}")
            self.handle_nav_failure(reject=False)

    def handle_nav_failure(self, reject: bool = False):
        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.get_logger().info(f"Retrying waypoint ({self.retry_count}/{self.max_retries}) after backoff...")
            self.retry_wait_start = self.get_clock().now()
            self.transition_to(MissionState.RETRY_WAIT)
        else:
            self.get_logger().warn(f"Max retries exceeded for WP #{self.waypoints[self.current_wp_idx]['id']}. Skipping to next.")
            self.transition_to(MissionState.NEXT_WP)

    def destroy_node(self):
        self.running = False
        try:
            st = "DONE" if self.state == MissionState.COMPLETED else self.state
            self.save_telemetry(st)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
