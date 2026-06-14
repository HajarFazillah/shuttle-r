#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
import cv2
import numpy as np


class ShuttlecockDetector(Node):
    def __init__(self):
        super().__init__('shuttlecock_detector')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.det_pub = self.create_publisher(
            Detection2DArray, '/shuttlecock_detections', 10)
        self.dbg_pub = self.create_publisher(
            Image, '/shuttlecock_detection/debug_image', 10)

        # Shuttlecock skirt: bright orange (1.0, 0.4, 0.0) — unique in scene
        # OpenCV HSV: H≈10, S≈255, V≈255
        self.lower_orange = np.array([5,  120,  80])
        self.upper_orange = np.array([20, 255, 255])

        self.kernel = np.ones((3, 3), np.uint8)
        self.get_logger().info('Shuttlecock detector started')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = Detection2DArray()
        detections.header = msg.header
        annotated = frame.copy()

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (5 < area < 2000):
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w // 2, y + h // 2

            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = float(cx)
            det.bbox.center.position.y = float(cy)
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)
            detections.detections.append(det)

            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(annotated, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(annotated, f'SC ({cx},{cy})', (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        if detections.detections:
            self.get_logger().info(
                f'Detected {len(detections.detections)} shuttlecock(s)',
                throttle_duration_sec=2.0)

        self.det_pub.publish(detections)
        self.dbg_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = ShuttlecockDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
