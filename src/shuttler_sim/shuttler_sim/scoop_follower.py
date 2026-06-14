#!/usr/bin/env python3
import math
import subprocess

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped

from shuttler_sim.shuttlecock_collector import (
    WORLD_NAME, SCOOP_OFFSET, HOPPER_OFFSET, yaw_from_quaternion)

UPDATE_PERIOD = 0.2  # s (5 Hz)


def set_pose(name, x, y, z, yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    subprocess.run(
        [
            'ign', 'service', '-s', f'/world/{WORLD_NAME}/set_pose',
            '--reqtype', 'ignition.msgs.Pose',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '500',
            '-r', f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
                  f'orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}',
        ],
        capture_output=True, text=True,
    )


class ScoopFollower(Node):
    """Keeps the scoop_assembly and hopper_bin models rigidly attached to
    the robot by teleporting them to robot_pose + a fixed local offset every
    cycle. Passive - no joints involved."""

    def __init__(self):
        super().__init__('scoop_follower')
        self.robot_pose = None  # (x, y, yaw)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.create_timer(UPDATE_PERIOD, self.update)
        self.get_logger().info('Scoop follower started')

    def pose_callback(self, msg):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def update(self):
        if self.robot_pose is None:
            return
        rx, ry, yaw = self.robot_pose
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        for name, (dx, dy, dz) in (('scoop_assembly', SCOOP_OFFSET),
                                    ('hopper_bin', HOPPER_OFFSET)):
            wx = rx + dx * cos_y - dy * sin_y
            wy = ry + dx * sin_y + dy * cos_y
            set_pose(name, wx, wy, dz, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = ScoopFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
