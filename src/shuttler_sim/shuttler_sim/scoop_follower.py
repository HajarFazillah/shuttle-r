#!/usr/bin/env python3
import math
import re
import subprocess
import threading

import rclpy
from rclpy.node import Node

from shuttler_sim.shuttlecock_collector import (
    WORLD_NAME, SCOOP_OFFSET, HOPPER_OFFSET)

UPDATE_PERIOD = 1.0


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


def get_gazebo_pose():
    try:
        result = subprocess.run(
            ['ign', 'model', '-m', 'turtlebot4', '-p'],
            capture_output=True, text=True, timeout=5,
        )
    except subprocess.TimeoutExpired:
        return None
    match = re.search(
        r'\[(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\]\s*\n\s*\[(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\]',
        result.stdout)
    if not match:
        return None
    x, y = float(match.group(1)), float(match.group(2))
    yaw = float(match.group(6))
    return (x, y, yaw)


class ScoopFollower(Node):

    def __init__(self):
        super().__init__('scoop_follower')
        self._busy = False
        self.create_timer(UPDATE_PERIOD, self.update)
        self.get_logger().info('Scoop follower started')

    def update(self):
        if self._busy:
            return
        pose = get_gazebo_pose()
        if pose is None:
            return
        rx, ry, yaw = pose
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
