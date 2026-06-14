#!/usr/bin/env python3
import math
import subprocess

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32

COLLECTION_RADIUS = 0.8  # m — distance from robot center to "pick up" a shuttlecock
STORAGE_Z = -2.0         # m — hide onboard (collected, not yet deposited) shuttlecocks
WORLD_NAME = 'empty_court'

# Gather/drop-off points — one at each corner of the court, matching the
# dropoff_zone_* markers in worlds/empty_court.sdf. The robot deposits at
# whichever corner it's currently nearest to.
GATHER_POINTS = [
    (7.3, 4.3),    # NE corner
    (7.3, -4.3),   # SE corner
    (-7.3, 4.3),   # NW corner
    (-7.3, -4.3),  # SW corner
]
DROPOFF_RADIUS = 0.6      # m
# Deposit grid is centered on the gather point. dropoff_zone_* markers are
# 1.2x1.2m squares (0.6m half-size), so a 5x5 grid at 0.2m spacing spans 0.8m
# (0.4m half-span), keeping every deposited shuttlecock inside the blue zone.
DEPOSIT_GRID_SPACING = 0.2  # m
DEPOSIT_GRID_COLS = 5

# Shuttlecock positions as placed in worlds/empty_court.sdf
SHUTTLECOCKS = {
    'shuttlecock_1': (3.0, 0.8),
    'shuttlecock_2': (4.5, -1.2),
    'shuttlecock_3': (5.5, 0.3),
    'shuttlecock_4': (2.0, -1.8),
    'shuttlecock_5': (4.0, 1.5),
    'shuttlecock_6': (-2.0, 1.0),
    'shuttlecock_7': (-3.5, -0.8),
    'shuttlecock_8': (-4.5, 1.2),
    'shuttlecock_9': (-1.5, -1.5),
    'shuttlecock_10': (-5.5, -0.3),
    'shuttlecock_11': (-5.7, -2.4),
    'shuttlecock_12': (-4.9, -2.8),
    'shuttlecock_13': (-2.9, -2.4),
    'shuttlecock_14': (-2.1, -2.8),
    'shuttlecock_15': (-0.3, -2.4),
    'shuttlecock_16': (0.3, -2.8),
    'shuttlecock_17': (2.1, -2.4),
    'shuttlecock_18': (2.9, -2.8),
    'shuttlecock_19': (4.9, -2.4),
    'shuttlecock_20': (5.7, -2.8),
    'shuttlecock_21': (-6.3, -0.8),
    'shuttlecock_22': (-4.3, -1.2),
    'shuttlecock_23': (-3.1, -1.0),
    'shuttlecock_24': (-1.5, -1.2),
    'shuttlecock_25': (-0.9, -0.8),
    'shuttlecock_26': (0.9, -1.2),
    'shuttlecock_27': (1.5, -0.8),
    'shuttlecock_28': (3.5, -1.2),
    'shuttlecock_29': (4.3, -0.8),
    'shuttlecock_30': (6.3, -1.2),
    'shuttlecock_31': (-5.7, 1.2),
    'shuttlecock_32': (-4.9, 0.8),
    'shuttlecock_33': (-2.9, 1.2),
    'shuttlecock_34': (-2.1, 0.8),
    'shuttlecock_35': (-0.3, 1.2),
    'shuttlecock_36': (0.3, 0.8),
    'shuttlecock_37': (2.1, 1.2),
    'shuttlecock_38': (2.9, 0.8),
    'shuttlecock_39': (4.9, 1.2),
    'shuttlecock_40': (5.7, 0.8),
    'shuttlecock_41': (-6.3, 2.8),
    'shuttlecock_42': (-4.3, 2.4),
    'shuttlecock_43': (-3.5, 2.8),
    'shuttlecock_44': (-1.5, 2.4),
    'shuttlecock_45': (-0.9, 2.8),
    'shuttlecock_46': (0.9, 2.4),
    'shuttlecock_47': (1.5, 2.8),
    'shuttlecock_48': (3.5, 2.4),
    'shuttlecock_49': (4.3, 2.8),
    'shuttlecock_50': (6.3, 2.4),
}


class ShuttlecockCollector(Node):

    def __init__(self):
        super().__init__('shuttlecock_collector')
        self.robot_pose = None
        self.collected = set()  # all shuttlecocks ever picked up
        self.onboard = []       # collected but not yet deposited
        self.deposited = 0      # total deposited across all corners
        self.deposited_per_corner = {p: 0 for p in GATHER_POINTS}

        self.count_pub = self.create_publisher(Int32, '/shuttlecocks_collected', 10)
        self.deposited_pub = self.create_publisher(Int32, '/shuttlecocks_deposited', 10)

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        self.create_timer(0.5, self.check_collection)
        self.create_timer(0.5, self.check_dropoff)
        self.get_logger().info(
            f'Shuttlecock collector started, tracking {len(SHUTTLECOCKS)} shuttlecocks')

    def pose_callback(self, msg):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def check_collection(self):
        if self.robot_pose is None:
            return

        rx, ry = self.robot_pose
        for name, (sx, sy) in SHUTTLECOCKS.items():
            if name in self.collected:
                continue
            if math.hypot(sx - rx, sy - ry) <= COLLECTION_RADIUS:
                self.collect(name)

    def check_dropoff(self):
        if self.robot_pose is None or not self.onboard:
            return

        rx, ry = self.robot_pose
        for gx, gy in GATHER_POINTS:
            if math.hypot(gx - rx, gy - ry) <= DROPOFF_RADIUS:
                self.deposit_all((gx, gy))
                return

    def collect(self, name):
        sx, sy = SHUTTLECOCKS[name]
        result = subprocess.run(
            [
                'ign', 'service', '-s', f'/world/{WORLD_NAME}/set_pose',
                '--reqtype', 'ignition.msgs.Pose',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '2000',
                '-r', f'name: "{name}", position: {{x: {sx}, y: {sy}, z: {STORAGE_Z}}}',
            ],
            capture_output=True, text=True,
        )

        if 'true' not in result.stdout:
            self.get_logger().warn(f'Failed to collect {name}: {result.stdout} {result.stderr}')
            return

        self.collected.add(name)
        self.onboard.append(name)
        self.get_logger().info(
            f'Collected {name} ({len(self.collected)}/{len(SHUTTLECOCKS)})')

        msg = Int32()
        msg.data = len(self.collected)
        self.count_pub.publish(msg)

        if len(self.collected) == len(SHUTTLECOCKS):
            self.get_logger().info('All shuttlecocks collected!')

    def deposit_all(self, gather_point):
        gx, gy = gather_point
        grid_half_span = (DEPOSIT_GRID_COLS - 1) * DEPOSIT_GRID_SPACING / 2.0
        origin_x = gx - grid_half_span
        origin_y = gy - grid_half_span
        slot_start = self.deposited_per_corner[gather_point]

        for i, name in enumerate(self.onboard):
            slot = slot_start + i
            row, col = divmod(slot, DEPOSIT_GRID_COLS)
            x = origin_x + col * DEPOSIT_GRID_SPACING
            y = origin_y + row * DEPOSIT_GRID_SPACING

            result = subprocess.run(
                [
                    'ign', 'service', '-s', f'/world/{WORLD_NAME}/set_pose',
                    '--reqtype', 'ignition.msgs.Pose',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '2000',
                    '-r', f'name: "{name}", position: {{x: {x}, y: {y}, z: 0.0}}',
                ],
                capture_output=True, text=True,
            )

            if 'true' not in result.stdout:
                self.get_logger().warn(f'Failed to deposit {name}: {result.stdout} {result.stderr}')
                continue

            self.deposited_per_corner[gather_point] += 1
            self.deposited += 1
            self.get_logger().info(f'Deposited {name} at dropoff zone {gather_point} ({self.deposited} total)')

        self.onboard = []

        msg = Int32()
        msg.data = self.deposited
        self.deposited_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ShuttlecockCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
