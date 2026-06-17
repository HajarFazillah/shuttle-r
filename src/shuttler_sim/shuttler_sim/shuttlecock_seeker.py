#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose, Spin
from std_msgs.msg import Int32

from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped transform support

DEPTH_PATCH = 5          # px — window around bbox center to sample depth
MIN_DEPTH = 0.1          # m
MAX_DEPTH = 5.0          # m
GOAL_UPDATE_DIST = 0.3   # m — ignore new target if within this of current goal
DETECTION_TIMEOUT = 1.0  # s

BATCH_SIZE = 3    # shuttlecocks to collect before heading to a gather point
TOTAL_TARGET = 3  # stop after this many have been deposited (demo: single collect-and-deposit run)

# Ignore detections near gather points - these are already-deposited
# shuttlecocks, not new ones to collect.
GATHER_EXCLUSION_RADIUS = 1.0  # m

# Ignore detection-derived targets already within the collector's pickup
# radius - if a real shuttlecock were there, it would already be collected,
# so this is a false detection (e.g. robot's own body) that would otherwise
# cause an instant "reached" / re-send loop.
SELF_DETECTION_RADIUS = 0.35  # m

# Ignore targets outside the court walls (walls at x=+-8, y=+-5) - these are
# false detections (e.g. depth noise off a wall) that nav2 can never reach,
# which would otherwise cause an infinite send/abort loop.
COURT_X_LIMIT = 7.5  # m
COURT_Y_LIMIT = 4.5  # m

# If this many consecutive cycles produce no usable target (e.g. only the
# robot's own body is in view), head back to the search anchor for a fresh
# vantage point instead of sitting idle indefinitely.
NO_TARGET_RECOVERY_CYCLES = 5

SPIN_ANGLE = 1.0  # rad (~57 deg) per recovery spin cycle
MAX_SPINS_BEFORE_RELOCATE = 3

SEARCH_POINTS = [
    (3.0, 0.5),
    (5.0, 0.5),
    (4.0, 1.5),
    (2.0, -1.0),
]

# Gather/drop-off points — one at each corner of the court. Must match
# shuttlecock_collector.GATHER_POINTS. The robot heads to whichever is nearest.
GATHER_POINTS = [
    (7.3, 4.3),    # NE corner — single deposit zone for demo
]


class ShuttlecockSeeker(Node):
    def __init__(self):
        super().__init__('shuttlecock_seeker')
        self.bridge = CvBridge()
        self.cam_model = None
        self.depth_image = None
        self.last_detections = None
        self.last_detections_time = self.get_clock().now()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, '/spin')
        self.goal_handle = None
        self.spin_in_progress = False
        self.current_target = None  # (x, y) in map frame
        self.heading_to_dropoff = False

        self.collected_count = 0
        self.deposited_count = 0
        self.target_reached_logged = False
        self.returning_to_search = False
        self.no_target_count = 0
        self.spin_count = 0
        self.search_point_index = 0

        self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(
            Detection2DArray, '/shuttlecock_detections', self.detection_callback, 10)
        self.create_subscription(
            Int32, '/shuttlecocks_collected', self.collected_callback, 10)
        self.create_subscription(
            Int32, '/shuttlecocks_deposited', self.deposited_callback, 10)

        self.create_timer(1.0, self.control_loop)
        self.get_logger().info('Shuttlecock seeker started')

    def camera_info_callback(self, msg):
        if self.cam_model is None:
            self.cam_model = PinholeCameraModel()
            self.cam_model.fromCameraInfo(msg)
            self.get_logger().info('Camera intrinsics received')

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def detection_callback(self, msg):
        self.last_detections = msg
        self.last_detections_time = self.get_clock().now()

    def collected_callback(self, msg):
        self.collected_count = msg.data

    def deposited_callback(self, msg):
        self.deposited_count = msg.data

    def sample_depth(self, cx, cy):
        h, w = self.depth_image.shape
        x0, x1 = max(0, cx - DEPTH_PATCH // 2), min(w, cx + DEPTH_PATCH // 2 + 1)
        y0, y1 = max(0, cy - DEPTH_PATCH // 2), min(h, cy + DEPTH_PATCH // 2 + 1)
        patch = self.depth_image[y0:y1, x0:x1]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size == 0:
            return None
        return float(np.min(valid))

    def control_loop(self):
        if self.goal_handle is not None or self.spin_in_progress:
            return  # navigation or recovery spin already in progress

        if self.returning_to_search:
            self.returning_to_search = False
            self.no_target_count = 0
            self.send_spin(SPIN_ANGLE)
            return

        if self.deposited_count >= TOTAL_TARGET:
            if not self.target_reached_logged:
                self.get_logger().info(
                    f'Target of {TOTAL_TARGET} shuttlecocks deposited - stopping.')
                self.target_reached_logged = True
            return

        onboard = self.collected_count - self.deposited_count
        if onboard >= BATCH_SIZE:
            gather_point = self.nearest_gather_point()
            if gather_point is not None:
                self.send_goal(gather_point, is_dropoff=True)
            return

        if self.cam_model is None or self.depth_image is None:
            return

        stale = (self.get_clock().now() - self.last_detections_time) > Duration(seconds=DETECTION_TIMEOUT)
        if stale or self.last_detections is None or not self.last_detections.detections:
            if onboard > 0:
                self.get_logger().info(
                    f'No more shuttlecocks visible with {onboard} onboard '
                    f'(batch target {BATCH_SIZE}) - depositing what we have first')
                gather_point = self.nearest_gather_point()
                if gather_point is not None:
                    self.send_goal(gather_point, is_dropoff=True)
            else:
                # Nothing in view at all (not just an unusable detection) -
                # still count toward the search-anchor recovery, otherwise
                # the seeker idles forever if no shuttlecock is ever visible
                # from the current vantage point.
                self.no_target_recovery()
            return

        robot_xy = None
        try:
            robot_tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time(), timeout=Duration(seconds=2.0))
            robot_xy = (robot_tf.transform.translation.x, robot_tf.transform.translation.y)
        except tf2_ros.TransformException:
            pass

        detections_by_size = sorted(
            self.last_detections.detections,
            key=lambda d: d.bbox.size_x * d.bbox.size_y, reverse=True)

        target = None
        for det in detections_by_size:
            cx = int(det.bbox.center.position.x)
            cy = int(det.bbox.center.position.y)

            depth = self.sample_depth(cx, cy)
            if depth is None or not (MIN_DEPTH < depth < MAX_DEPTH):
                continue

            ray = self.cam_model.projectPixelTo3dRay((cx, cy))
            scale = depth / ray[2]
            point_cam = PointStamped()
            point_cam.header = self.last_detections.header
            point_cam.point.x = ray[0] * scale
            point_cam.point.y = ray[1] * scale
            point_cam.point.z = ray[2] * scale

            try:
                point_map = self.tf_buffer.transform(
                    point_cam, 'map', timeout=Duration(seconds=2.0))
            except tf2_ros.TransformException as e:
                self.get_logger().warn(f'TF transform failed: {e}', throttle_duration_sec=2.0)
                continue

            candidate = (point_map.point.x, point_map.point.y)

            if abs(candidate[0]) > COURT_X_LIMIT or abs(candidate[1]) > COURT_Y_LIMIT:
                continue  # outside the court walls - unreachable false detection

            if any(math.hypot(candidate[0] - gx, candidate[1] - gy) < GATHER_EXCLUSION_RADIUS
                   for gx, gy in GATHER_POINTS):
                continue  # likely an already-deposited shuttlecock at a gather point

            if robot_xy is not None and math.hypot(
                    candidate[0] - robot_xy[0], candidate[1] - robot_xy[1]) < SELF_DETECTION_RADIUS:
                continue  # already within pickup radius - likely a false detection

            target = candidate
            break

        if target is None:
            self.no_target_recovery()
            return

        self.no_target_count = 0
        self.spin_count = 0

        if self.current_target is not None:
            d = math.hypot(target[0] - self.current_target[0],
                            target[1] - self.current_target[1])
            if d < GOAL_UPDATE_DIST:
                return  # already heading to (essentially) the same shuttlecock

        self.send_goal(target)

    def no_target_recovery(self):
        self.no_target_count += 1
        if self.no_target_count < NO_TARGET_RECOVERY_CYCLES:
            return

        self.no_target_count = 0
        self.spin_count += 1

        if self.spin_count <= MAX_SPINS_BEFORE_RELOCATE:
            self.get_logger().info(
                f'No usable target in view - spinning to scan '
                f'({self.spin_count}/{MAX_SPINS_BEFORE_RELOCATE} before relocating)')
            self.send_spin(SPIN_ANGLE)
        else:
            point = SEARCH_POINTS[self.search_point_index % len(SEARCH_POINTS)]
            self.search_point_index += 1
            self.spin_count = 0
            self.get_logger().info(
                f'Still no target after {MAX_SPINS_BEFORE_RELOCATE} spins - '
                f'relocating to search point ({point[0]:.1f}, {point[1]:.1f})')
            self.send_goal(point)

    def send_spin(self, angle):
        if not self.spin_client.server_is_ready():
            self.get_logger().warn('Spin server not ready', throttle_duration_sec=5.0)
            return
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = angle
        self.spin_in_progress = True
        send_future = self.spin_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.spin_response_callback)

    def spin_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Spin goal rejected')
            self.spin_in_progress = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.spin_result_callback)

    def spin_result_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Spin finished with status: {result.status}')
        self.spin_in_progress = False

    def nearest_gather_point(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time(), timeout=Duration(seconds=2.0))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF transform failed: {e}', throttle_duration_sec=2.0)
            return None

        rx = t.transform.translation.x
        ry = t.transform.translation.y
        return min(GATHER_POINTS, key=lambda p: math.hypot(p[0] - rx, p[1] - ry))

    def send_goal(self, target, is_dropoff=False):
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('NavigateToPose server not ready', throttle_duration_sec=5.0)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target[0]
        goal_msg.pose.pose.position.y = target[1]
        goal_msg.pose.pose.orientation.w = 1.0

        self.current_target = target
        self.heading_to_dropoff = is_dropoff
        label = 'dropoff zone' if is_dropoff else 'shuttlecock'
        self.get_logger().info(f'Sending NavigateToPose goal to {label}: ({target[0]:.2f}, {target[1]:.2f})')

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.current_target = None
            self.heading_to_dropoff = False
            return
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Navigation finished with status: {result.status}')
        if self.heading_to_dropoff:
            self.returning_to_search = True
        self.goal_handle = None
        self.heading_to_dropoff = False
        self.current_target = None


def main(args=None):
    rclpy.init(args=args)
    node = ShuttlecockSeeker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
