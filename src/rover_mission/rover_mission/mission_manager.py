#!/usr/bin/env python3
"""
Mission Manager Node for Inspection Rover
Implements Finite State Machine (FSM):
  IDLE -> NAV_TO_WP -> SCAN_INSPECT -> NEXT_WP -> RETURN_HOME -> COMPLETED
Interacts with Nav2 NavigateToPose action server.
"""

import os
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from ament_index_python.packages import get_package_share_directory


class MissionState:
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    SCANNING = "SCANNING"
    NEXT_WP = "NEXT_WP"
    RETURNING_HOME = "RETURNING_HOME"
    COMPLETED = "COMPLETED"


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # Load waypoints
        pkg_share = get_package_share_directory('rover_mission')
        default_wp_path = os.path.join(pkg_share, 'config', 'inspection_waypoints.yaml')
        self.declare_parameter('waypoints_file', default_wp_path)
        wp_file = self.get_parameter('waypoints_file').get_parameter_value().string_value

        self.waypoints = self.load_waypoints(wp_file)
        self.current_wp_idx = 0
        self.state = MissionState.IDLE
        self.retry_count = 0
        self.max_retries = 1

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Velocity publisher for in-place inspection scanning rotation
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # FSM execution loop timer (1.0 Hz)
        self.timer = self.create_timer(1.0, self.fsm_step)

        self.goal_handle = None
        self.scan_start_time = None
        self.active_goal_future = None

        self.get_logger().info(f"MissionManager initialized with {len(self.waypoints)} waypoints.")

    def load_waypoints(self, path):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints from {path}: {e}")
            return []

    def yaw_to_quaternion(self, yaw):
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)
        }

    def fsm_step(self):
        if self.state == MissionState.IDLE:
            if not self.waypoints:
                self.get_logger().warn("No waypoints loaded. Standing by.")
                return

            self.get_logger().info("Waiting for Nav2 action server 'navigate_to_pose'...")
            if self.nav_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().info("Nav2 server ready. Starting autonomous mission.")
                self.transition_to(MissionState.NAVIGATING)
                self.send_current_goal()
            else:
                self.get_logger().warn("Waiting for navigate_to_pose action server...")

        elif self.state == MissionState.SCANNING:
            now = self.get_clock().now()
            elapsed = (now - self.scan_start_time).nanoseconds / 1e9
            wp = self.waypoints[self.current_wp_idx]
            duration = wp.get('scan_duration', 3.0)

            if elapsed < duration:
                # Rotate gently in place to sweep the camera across the defect area
                cmd = Twist()
                cmd.angular.z = 0.4
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
                self.transition_to(MissionState.COMPLETED)
                self.get_logger().info("🎉 MISSION COMPLETE! All waypoints visited and inspected.")

    def transition_to(self, new_state):
        self.get_logger().info(f"[FSM TRANSITION] {self.state} -> {new_state}")
        self.state = new_state

    def send_current_goal(self):
        wp = self.waypoints[self.current_wp_idx]
        self.get_logger().info(f"Navigating to WP #{wp['id']} ({wp['name']}): ({wp['x']}, {wp['y']}, yaw={wp['yaw']})")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(wp['x'])
        goal_msg.pose.pose.position.y = float(wp['y'])
        goal_msg.pose.pose.position.z = 0.0

        q = self.yaw_to_quaternion(wp['yaw'])
        goal_msg.pose.pose.orientation.x = q['x']
        goal_msg.pose.pose.orientation.y = q['y']
        goal_msg.pose.pose.orientation.z = q['z']
        goal_msg.pose.pose.orientation.w = q['w']

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected by Nav2!")
            self.handle_nav_failure()
            return

        self.get_logger().info("Goal accepted by Nav2. Moving...")
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Reached Waypoint #{self.waypoints[self.current_wp_idx]['id']} successfully.")
            self.scan_start_time = self.get_clock().now()
            self.transition_to(MissionState.SCANNING)
        else:
            self.get_logger().warn(f"Goal failed with status: {status}")
            self.handle_nav_failure()

    def handle_nav_failure(self):
        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.get_logger().info(f"Retrying waypoint ({self.retry_count}/{self.max_retries})...")
            self.send_current_goal()
        else:
            self.get_logger().warn("Max retries exceeded. Skipping to next waypoint.")
            self.transition_to(MissionState.NEXT_WP)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
