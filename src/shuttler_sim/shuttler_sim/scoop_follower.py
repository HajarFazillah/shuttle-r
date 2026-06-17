#!/usr/bin/env python3
import math
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

import tf2_ros

from shuttler_sim.shuttlecock_collector import (
    WORLD_NAME, SCOOP_OFFSET, HOPPER_OFFSET, yaw_from_quaternion)

UPDATE_PERIOD = 1.0  # s (1 Hz) - slower to avoid overloading ign service


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
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._busy = False
        self.create_timer(UPDATE_PERIOD, self.update)
        self.get_logger().info('Scoop follower started')

    def update(self):
        if self._busy:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time(), timeout=Duration(seconds=2.0))
        except tf2_ros.TransformException:
            return
        yaw = yaw_from_quaternion(tf.transform.rotation)
        rx, ry = tf.transform.translation.x, tf.transform.translation.y
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        self._busy = True

        def _do_updates():
            for name, (dx, dy, dz) in (('scoop_assembly', SCOOP_OFFSET),
                                        ('hopper_bin', HOPPER_OFFSET)):
                wx = rx + dx * cos_y - dy * sin_y
                wy = ry + dx * sin_y + dy * cos_y
                set_pose(name, wx, wy, dz, yaw)
            self._busy = False

        threading.Thread(target=_do_updates, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = ScoopFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
