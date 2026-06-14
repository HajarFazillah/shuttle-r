#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist

IMAGE_CENTER_X = 160.0   # half of 320px camera width
DEADZONE       = 25.0    # px — no turn if error smaller than this
COLLECT_AREA   = 800.0   # px² — stop when bbox this large (shuttlecock is close)

LINEAR_SPEED   = 0.25    # m/s forward
ANGULAR_GAIN   = 0.005   # rad/s per pixel of error (proportional)
SEARCH_SPEED   = 0.45    # rad/s rotate while searching


class ShuttlecockChaser(Node):
    def __init__(self):
        super().__init__('shuttlecock_chaser')
        self.sub = self.create_subscription(
            Detection2DArray, '/shuttlecock_detections', self.det_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_msg = None
        self.last_msg_time = self.get_clock().now()

        self.create_timer(0.1, self.control_loop)   # 10 Hz control
        self.get_logger().info('Shuttlecock chaser started')

    def det_callback(self, msg):
        self.last_msg = msg
        self.last_msg_time = self.get_clock().now()

    def control_loop(self):
        twist = Twist()
        stale = (self.get_clock().now() - self.last_msg_time) > Duration(seconds=0.5)

        if stale or self.last_msg is None or not self.last_msg.detections:
            # No fresh detection — rotate slowly to search
            twist.angular.z = SEARCH_SPEED
            self.pub.publish(twist)
            return

        # Pick the largest detection (biggest bbox = closest shuttlecock)
        best = max(self.last_msg.detections,
                   key=lambda d: d.bbox.size_x * d.bbox.size_y)

        area  = best.bbox.size_x * best.bbox.size_y
        error = best.bbox.center.position.x - IMAGE_CENTER_X   # +ve = right of center

        if area >= COLLECT_AREA:
            self.get_logger().info('Shuttlecock reached — stopping.', throttle_duration_sec=2.0)
            self.pub.publish(Twist())   # stop
            return

        if abs(error) > DEADZONE:
            # Turn proportionally toward shuttlecock
            twist.angular.z = -error * ANGULAR_GAIN
        else:
            # Centered — drive straight forward
            twist.linear.x = LINEAR_SPEED

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ShuttlecockChaser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
