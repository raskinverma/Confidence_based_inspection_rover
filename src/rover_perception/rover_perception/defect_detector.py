#!/usr/bin/env python3
"""
Defect Detector Node for Inspection Rover
Detects:
  1. Red defects (rust) via dual-band HSV thresholding
  2. Yellow defects (leaks) via HSV thresholding
  3. ArUco markers (asset tags) via cv2.aruco
Publishes:
  /defect_detections: custom rover_interfaces/msg/Defect
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from rover_interfaces.msg import Defect


class DefectDetector(Node):
    def __init__(self):
        super().__init__('defect_detector')

        self.bridge = CvBridge()
        self.camera_info = None

        # ArUco detector setup (OpenCV 4.x compatible)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.aruco_detector = None

        # Min contour area to avoid noise
        self.min_contour_area = 500

        # Subscriptions
        self.sub_cam_info = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        self.sub_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for defect detections
        self.pub_defect = self.create_publisher(
            Defect,
            '/defect_detections',
            10
        )

        # Debug image publisher
        self.pub_debug_img = self.create_publisher(
            Image,
            '/camera/detections_debug',
            10
        )

        self.get_logger().info('DefectDetector initialized (HSV Red/Yellow + ArUco 4x4)')

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge conversion error: {e}')
            return

        debug_image = cv_image.copy()
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 1. Detect Red (Rust) - Two ranges across the hue boundary
        lower_red1 = np.array([0, 100, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 70])
        upper_red2 = np.array([180, 255, 255])

        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        self.process_color_mask(mask_red, "rust", (0, 0, 255), msg.header, debug_image)

        # 2. Detect Yellow (Leak)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        self.process_color_mask(mask_yellow, "leak", (0, 255, 255), msg.header, debug_image)

        # 3. Detect ArUco Markers
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            for i in range(len(ids)):
                c = corners[i][0]
                u = int(np.mean(c[:, 0]))
                v = int(np.mean(c[:, 1]))
                marker_id = int(ids[i][0])
                defect_type = f"aruco_{marker_id}"

                cv2.polylines(debug_image, [c.astype(np.int32)], True, (255, 0, 0), 2)
                cv2.circle(debug_image, (u, v), 4, (255, 0, 0), -1)
                cv2.putText(debug_image, defect_type, (u - 20, v - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                self.publish_defect(defect_type, 0.95, u, v, msg.header)

        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        debug_msg.header = msg.header
        self.pub_debug_img.publish(debug_msg)

    def process_color_mask(self, mask, defect_type, color_bgr, header, debug_image):
        # Morphological opening and closing to clean noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= self.min_contour_area:
                x, y, w, h = cv2.boundingRect(cnt)
                u = int(x + w / 2)
                v = int(y + h / 2)
                confidence = min(1.0, area / 20000.0 + 0.5)

                cv2.rectangle(debug_image, (x, y), (x + w, y + h), color_bgr, 2)
                cv2.circle(debug_image, (u, v), 4, color_bgr, -1)
                cv2.putText(debug_image, f"{defect_type} ({confidence:.2f})", (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)

                self.publish_defect(defect_type, confidence, u, v, header)

    def publish_defect(self, defect_type: str, confidence: float, u: int, v: int, header):
        defect = Defect()
        defect.header = header
        defect.defect_type = defect_type
        defect.confidence = float(confidence)
        defect.pixel_u = int(u)
        defect.pixel_v = int(v)
        self.pub_defect.publish(defect)


from rclpy.executors import ExternalShutdownException


def main(args=None):
    rclpy.init(args=args)
    node = DefectDetector()
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
