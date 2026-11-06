#!/usr/bin/env python3
"""
Mission Manager Node for Inspection Rover (Phase 3)
Supports two operational modes:
  1. baseline: Fixed 7-waypoint full-coverage sweep with 360-degree scans
  2. active: Confidence-Driven Next-Best-View (NBV) Active Planner with
             frustum-clipped coverage map, targeted re-inspections for
             uncertain hypotheses, and adaptive scanning.

Finite State Machine (FSM):
  IDLE -> WAITING_NAV2 -> LOCALIZING -> NAVIGATING -> SCANNING -> NEXT_WP -> COMPLETED / FAILED

Publishes:
  /mission_status (std_msgs/msg/String: RUNNING, DONE, FAILED)
  /initialpose (geometry_msgs/msg/PoseWithCovarianceStamped)
  /cmd_vel (geometry_msgs/msg/Twist)
  /inspection/coverage_grid (nav_msgs/msg/OccupancyGrid)

Tracks Telemetry in run_telemetry.yaml:
  - Path length, mission duration, clearance, localization errors
  - reinspections_performed, final_coverage_ratio
"""

import os
import math
import time
import re
import json
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
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped, Pose
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from ament_index_python.packages import get_package_share_directory
from rover_mission.coverage_map import CoverageMap, normalize_angle


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
        self.declare_parameter('inspection_mode', 'baseline')

        wp_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        out_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.initial_x = self.get_parameter('initial_x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('initial_y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('initial_yaw').get_parameter_value().double_value
        self.inspection_mode = self.get_parameter('inspection_mode').get_parameter_value().string_value

        self.output_dir = os.path.expanduser(out_dir)
        if not os.path.isabs(self.output_dir):
            self.output_dir = os.path.join(os.getcwd(), self.output_dir)
        os.makedirs(self.output_dir, exist_ok=True)

        self.waypoints = self.load_waypoints(wp_file)
        self.current_wp_idx = 0
        self.state = MissionState.IDLE
        self.retry_count = 0
        self.max_retries = 1 if self.inspection_mode == 'active' else 2
        self.successful_wps = 0

        # Telemetry variables
        self.mission_start_time = None
        self.mission_end_time = None
        self.last_odom_pos = None
        self.path_length = 0.0
        self.min_clearance = float('inf')

        # Localization telemetry
        self.latest_amcl_pos = None
        self.latest_amcl_yaw = 0.0
        self.latest_gz_pos = None
        self.latest_scan = None
        self.initial_loc_err = None
        self.loc_errors = []
        self.max_loc_err = 0.0
        self._last_telemetry_save = 0.0
        self.running = True

        # Phase 3 Active Inspection components
        self.coverage_map = CoverageMap()
        self.active_hypotheses = []
        self.reinspection_attempts = {}
        self.reinspections_performed = 0
        self.visited_anchors = set()
        self.current_active_goal = None
        self.active_goal_type = None  # 'REINSPECT' or 'EXPLORE'
        self.last_coverage_pub_time = 0.0

        # Room exploration anchors for active inspection mode (navigable poses forming continuous topological sweep)
        self.room_anchors = [
            {'id': 1, 'name': 'Room_A_South', 'x': -2.0, 'y': -1.5, 'yaw': 0.0},
            {'id': 2, 'name': 'Corridor_Entrance', 'x': 0.8, 'y': -2.8, 'yaw': 0.0},
            {'id': 3, 'name': 'Corridor_South_Mid', 'x': 1.6, 'y': -2.8, 'yaw': 0.0},
            {'id': 4, 'name': 'Corridor_Exit', 'x': -1.0, 'y': -1.0, 'yaw': 1.5708},
            {'id': 5, 'name': 'Room_A_North', 'x': -2.0, 'y': 2.0, 'yaw': 1.5708},
            {'id': 6, 'name': 'Room_B_North', 'x': 2.2, 'y': 2.2, 'yaw': 1.5708},
            {'id': 7, 'name': 'Room_B_East', 'x': 3.2, 'y': 1.8, 'yaw': 0.0},
            {'id': 8, 'name': 'Room_B_South', 'x': 2.8, 'y': -1.4, 'yaw': 0.0},
        ]

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
        self.pub_coverage_grid = self.create_publisher(OccupancyGrid, '/inspection/coverage_grid', 1)

        # Nav2 Action Client & Lifecycle Service Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.lifecycle_client = self.create_client(GetState, '/bt_navigator/get_state')

        # Subscriptions
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.sub_amcl_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_cb, 10)
        self.sub_gz_pose = self.create_subscription(Pose, '/gz/robot_pose', self.gz_pose_cb, 10)
        self.sub_hypotheses = self.create_subscription(String, '/inspection/hypotheses', self.hypotheses_cb, 10)

        # Control loop timer (10 Hz)
        self.timer_fsm = self.create_timer(0.1, self.fsm_step)

        self.goal_handle = None
        self.scan_start_time = None
        self.localize_start_time = None
        self.retry_wait_start = None
        self.lifecycle_req_future = None
        self.last_poll_time = 0.0

        self.get_logger().info(
            f"MissionManager initialized in '{self.inspection_mode}' mode "
            f"({len(self.waypoints)} baseline waypoints, {len(self.room_anchors)} active anchors)."
        )
        self.publish_status("RUNNING")

    def load_waypoints(self, path):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints from {path}: {e}")
            return []

    def hypotheses_cb(self, msg: String):
        try:
            self.active_hypotheses = json.loads(msg.data)
        except Exception:
            pass

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
        self.latest_scan = msg
        if self.state in (MissionState.NAVIGATING, MissionState.SCANNING):
            valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max and not math.isnan(r) and not math.isinf(r)]
            if valid:
                curr_min = min(valid)
                if curr_min < self.min_clearance:
                    self.min_clearance = float(curr_min)
        if self.latest_amcl_pos is not None:
            self.coverage_map.update_coverage(self.latest_amcl_pos[0], self.latest_amcl_pos[1], self.latest_amcl_yaw, msg)

    def amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.latest_amcl_pos = (p.x, p.y)
        self.latest_amcl_yaw = 2.0 * math.atan2(o.z, o.w)
        self.compute_localization_error()
        if self.latest_scan is not None:
            self.coverage_map.update_coverage(p.x, p.y, self.latest_amcl_yaw, self.latest_scan)

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
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.068
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
            'waypoints_total': len(self.waypoints) if self.inspection_mode == 'baseline' else len(self.room_anchors),
            'reinspections_performed': int(self.reinspections_performed),
            'final_coverage_ratio': round(float(self.coverage_map.get_coverage_ratio()), 3),
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

    def select_next_active_goal(self):
        """
        Selects next viewpoint for active inspection mode using NBV utility.
        Priority 1: Re-inspecting UNCERTAIN or CANDIDATE hypotheses.
        Priority 2: Exploring unvisited room anchors / high unseen coverage gain.
        """
        rx, ry = self.latest_amcl_pos or (self.initial_x, self.initial_y)

        # 1. Candidate re-inspections
        best_reinspect = None
        best_reinspect_score = -float('inf')

        for h in self.active_hypotheses:
            if h.get('status') in ('CANDIDATE', 'UNCERTAIN'):
                hid = h['id']
                if self.reinspection_attempts.get(hid, 0) >= 2:
                    continue
                hx, hy = float(h['x']), float(h['y'])
                dx, dy = rx - hx, ry - hy
                d = math.hypot(dx, dy)
                if d < 0.15:
                    dx, dy = -hx, -hy
                    d = math.hypot(dx, dy)
                if d < 0.01:
                    dx, dy = 1.0, 0.0
                    d = 1.0
                ux, uy = dx / d, dy / d

                best_v = None
                for standoff_dist in (1.15, 1.30, 1.00):
                    vx = float(np.clip(hx + standoff_dist * ux, -4.5, 4.5))
                    vy = float(np.clip(hy + standoff_dist * uy, -3.5, 3.5))
                    if self.coverage_map.is_navigable(vx, vy, clearance=0.35):
                        best_v = (vx, vy)
                        break

                if best_v is None:
                    continue

                vx, vy = best_v
                vyaw = math.atan2(hy - vy, hx - vx)

                travel_dist = math.hypot(vx - rx, vy - ry)
                conf = float(h.get('confidence', 0.5))
                sigma = float(h.get('sigma_pos', 0.25))

                score = 10.0 * (1.0 - conf) + 3.0 * sigma - 0.35 * travel_dist + 5.0
                if score > best_reinspect_score:
                    best_reinspect_score = score
                    best_reinspect = {
                        'type': 'REINSPECT',
                        'hyp_id': hid,
                        'name': f"Reinspect_{h.get('defect_type')}_#{hid}",
                        'x': vx,
                        'y': vy,
                        'yaw': vyaw,
                        'score': score
                    }

        # 2. Candidate exploration anchors (in topological sweep order)
        best_explore = None
        for a in self.room_anchors:
            if a['name'] not in self.visited_anchors:
                gain = self.coverage_map.get_unseen_frustum_gain(a['x'], a['y'], a['yaw'])
                best_explore = {
                    'type': 'EXPLORE',
                    'name': a['name'],
                    'x': a['x'],
                    'y': a['y'],
                    'yaw': a['yaw'],
                    'score': float(gain)
                }
                break

        # Priority 1: Complete structured exploration sweep across all rooms and corridors
        if best_explore is not None:
            return best_explore

        # Priority 2: Targeted close-range re-inspection for any remaining uncertain hypotheses
        if best_reinspect is not None:
            return best_reinspect

        return None

    def fsm_step(self):
        now_sec = time.time()

        # Periodically flush telemetry every 2 seconds during active mission
        if self.mission_start_time is not None and (now_sec - self._last_telemetry_save >= 2.0):
            st = "DONE" if self.state == MissionState.COMPLETED else self.state
            self.save_telemetry(st)
            self._last_telemetry_save = now_sec

        # Periodically publish coverage grid (1 Hz) in active mode
        if self.inspection_mode == 'active' and (now_sec - self.last_coverage_pub_time >= 1.0):
            stamp = self.get_clock().now().to_msg()
            grid_msg = self.coverage_map.to_occupancy_grid(frame_id='map', stamp=stamp)
            self.pub_coverage_grid.publish(grid_msg)
            self.last_coverage_pub_time = now_sec

        if self.state == MissionState.IDLE:
            if self.nav_client.wait_for_server(timeout_sec=0.01):
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
            elapsed = (self.get_clock().now() - self.localize_start_time).nanoseconds / 1e9
            if elapsed < 2.0:
                if elapsed > 1.0 and getattr(self, '_repub_done', False) is False:
                    self.publish_initial_pose()
                    self._repub_done = True
            else:
                self.get_logger().info(f"Localization established. Starting {self.inspection_mode} inspection mission!")
                self.mission_start_time = self.get_clock().now()
                if self.inspection_mode == 'active':
                    self.transition_to(MissionState.NEXT_WP)
                else:
                    self.transition_to(MissionState.NAVIGATING)
                    self.send_current_goal()

        elif self.state == MissionState.RETRY_WAIT:
            elapsed = (self.get_clock().now() - self.retry_wait_start).nanoseconds / 1e9
            if elapsed >= 3.0:
                self.transition_to(MissionState.NAVIGATING)
                self.send_current_goal()

        elif self.state == MissionState.SCANNING:
            now = self.get_clock().now()
            elapsed = (now - self.scan_start_time).nanoseconds / 1e9

            if self.inspection_mode == 'active' and self.active_goal_type == 'REINSPECT':
                # Focal dwell (2.5s total)
                if elapsed < 1.0:
                    self.pub_cmd_vel.publish(Twist())
                elif elapsed < 1.7:
                    cmd = Twist()
                    cmd.angular.z = 0.3
                    self.pub_cmd_vel.publish(cmd)
                elif elapsed < 2.5:
                    cmd = Twist()
                    cmd.angular.z = -0.3
                    self.pub_cmd_vel.publish(cmd)
                else:
                    self.pub_cmd_vel.publish(Twist())
                    self.get_logger().info(f"Targeted re-inspection dwell completed for {self.current_active_goal['name']}")
                    self.transition_to(MissionState.NEXT_WP)
            else:
                # Exploration scan
                if self.inspection_mode == 'active':
                    if self.current_active_goal and self.current_active_goal.get('name') == 'Corridor_Exit':
                        duration = 0.5
                        angular_speed = 0.0
                    else:
                        duration = 6.2
                        angular_speed = 1.05
                else:
                    wp = self.waypoints[self.current_wp_idx]
                    duration = float(wp.get('scan_duration', 8.0))
                    angular_speed = 0.9

                if elapsed < duration:
                    cmd = Twist()
                    cmd.angular.z = angular_speed
                    self.pub_cmd_vel.publish(cmd)
                else:
                    self.pub_cmd_vel.publish(Twist())
                    self.get_logger().info("Inspection sweep completed.")
                    self.transition_to(MissionState.NEXT_WP)

        elif self.state == MissionState.NEXT_WP:
            if self.inspection_mode == 'active':
                sim_dur = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9 if self.mission_start_time else 0.0
                if sim_dur >= 320.0:
                    self.get_logger().info(f"Active inspection mission time limit reached ({sim_dur:.1f}s >= 320s). Completing.")
                    self.mission_end_time = self.get_clock().now()
                    self.transition_to(MissionState.COMPLETED)
                    self.publish_status("DONE")
                    self.save_telemetry("DONE")
                    return

                next_goal = self.select_next_active_goal()
                if next_goal is not None:
                    self.current_active_goal = next_goal
                    self.active_goal_type = next_goal['type']
                    if next_goal['type'] == 'REINSPECT':
                        self.reinspection_attempts[next_goal['hyp_id']] = self.reinspection_attempts.get(next_goal['hyp_id'], 0) + 1
                        self.reinspections_performed += 1
                        self.get_logger().info(f"[ACTIVE NBV] Selected RE-INSPECTION goal: {next_goal['name']} (score={next_goal['score']:.2f})")
                    else:
                        self.visited_anchors.add(next_goal['name'])
                        self.get_logger().info(f"[ACTIVE NBV] Selected EXPLORATION goal: {next_goal['name']} (score={next_goal['score']:.2f})")

                    self.retry_count = 0
                    self.transition_to(MissionState.NAVIGATING)
                    self.send_current_goal()
                else:
                    self.mission_end_time = self.get_clock().now()
                    self.transition_to(MissionState.COMPLETED)
                    self.publish_status("DONE")
                    self.save_telemetry("DONE")
                    self.get_logger().info("🎉 ACTIVE INSPECTION COMPLETE! Coverage achieved and all hypotheses verified.")
            else:
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
        if self.inspection_mode == 'active' and self.current_active_goal is not None:
            gx = float(self.current_active_goal['x'])
            gy = float(self.current_active_goal['y'])
            gyaw = float(self.current_active_goal['yaw'])
            gname = self.current_active_goal['name']
            self.get_logger().info(f"Navigating to Active Goal ({gname}): ({gx:.2f}, {gy:.2f}, yaw={gyaw:.2f})")
        else:
            wp = self.waypoints[self.current_wp_idx]
            gx = float(wp['x'])
            gy = float(wp['y'])
            gyaw = float(wp['yaw'])
            gname = wp['name']
            self.get_logger().info(
                f"Navigating to WP #{wp['id']}/{len(self.waypoints)} ({gname}): "
                f"({gx}, {gy}, yaw={gyaw})"
            )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = gx
        goal_msg.pose.pose.position.y = gy
        goal_msg.pose.pose.position.z = 0.0

        q = self.yaw_to_quaternion(gyaw)
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

        target_name = self.current_active_goal['name'] if (self.inspection_mode == 'active' and self.current_active_goal) else f"WP #{self.waypoints[self.current_wp_idx]['id']}"
        self.get_logger().info(f"Goal accepted by Nav2 ({target_name}). Moving...")
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
            self.successful_wps += 1
            target_name = self.current_active_goal['name'] if (self.inspection_mode == 'active' and self.current_active_goal) else f"WP #{self.waypoints[self.current_wp_idx]['id']}"
            self.get_logger().info(f"Reached goal successfully: {target_name}")
            self.scan_start_time = self.get_clock().now()
            self.transition_to(MissionState.SCANNING)
        else:
            self.get_logger().warn(f"Goal failed with Nav2 status: {status}")
            self.handle_nav_failure(reject=False)

    def handle_nav_failure(self, reject: bool = False):
        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.get_logger().info(f"Retrying goal ({self.retry_count}/{self.max_retries}) after backoff...")
            self.retry_wait_start = self.get_clock().now()
            self.transition_to(MissionState.RETRY_WAIT)
        else:
            target_name = self.current_active_goal['name'] if (self.inspection_mode == 'active' and self.current_active_goal) else f"WP #{self.waypoints[self.current_wp_idx]['id']}"
            self.get_logger().warn(f"Max retries exceeded for {target_name}. Skipping to next.")
            if self.inspection_mode == 'active' and self.current_active_goal:
                if self.current_active_goal['type'] == 'EXPLORE':
                    self.visited_anchors.add(self.current_active_goal['name'])
                elif self.current_active_goal['type'] == 'REINSPECT':
                    hid = self.current_active_goal.get('hyp_id')
                    if hid is not None:
                        self.reinspection_attempts[hid] = 2
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
