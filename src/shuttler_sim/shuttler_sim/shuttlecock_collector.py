#!/usr/bin/env python3
import math
import subprocess

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32

WORLD_NAME = 'empty_court'

# Passive scoop + hopper bin, tracked every cycle by scoop_follower.py to
# stay rigidly attached to the robot (local offsets in the base_link frame:
# x = forward, y = left, z = up). Shared with scoop_follower.py.
SCOOP_OFFSET = (0.28, 0.0, 0.05)
HOPPER_OFFSET = (-0.05, 0.0, 0.30)

# A shuttlecock is captured once it's within this radius of the scoop's
# tracked position.
CAPTURE_RADIUS = 0.40  # m

# Slots (local x, y offsets from HOPPER_OFFSET) where onboard shuttlecocks
# are placed inside the hopper bin. Must have >= BATCH_SIZE entries. z offset
# drops them onto the hopper floor (HOPPER_OFFSET z is the bin's center).
HOPPER_SLOTS = [(-0.05, -0.05), (-0.05, 0.05), (0.0, -0.05), (0.0, 0.05), (0.05, 0.0)]
HOPPER_SLOT_Z = -0.02

DROPOFF_RADIUS = 0.6      # m
# Deposit grid is centered on the gather point. dropoff_zone_* markers are
# 1.2x1.2m squares (0.6m half-size), so a 5x5 grid at 0.2m spacing spans 0.8m
# (0.4m half-span), keeping every deposited shuttlecock inside the blue zone.
DEPOSIT_GRID_SPACING = 0.2  # m
DEPOSIT_GRID_COLS = 5

# Gather/drop-off points — one at each corner of the court, matching the
# dropoff_zone_* markers in worlds/empty_court.sdf. The robot deposits at
# whichever corner it's currently nearest to.
GATHER_POINTS = [
    (7.3, 4.3),    # NE corner
    (7.3, -4.3),   # SE corner
    (-7.3, 4.3),   # NW corner
    (-7.3, -4.3),  # SW corner
]

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
}


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ShuttlecockCollector(Node):

    def __init__(self):
        super().__init__('shuttlecock_collector')
        self.robot_pose = None  # (x, y, yaw)
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
        self.create_timer(0.1, self.track_onboard)
        self.get_logger().info(
            f'Shuttlecock collector started, tracking {len(SHUTTLECOCKS)} shuttlecocks')

    def pose_callback(self, msg):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def teleport(self, name, x, y, z):
        result = subprocess.run(
            [
                'ign', 'service', '-s', f'/world/{WORLD_NAME}/set_pose',
                '--reqtype', 'ignition.msgs.Pose',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '2000',
                '-r', f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}',
            ],
            capture_output=True, text=True,
        )
        return 'true' in result.stdout, result

    def scoop_position(self, rx, ry, yaw):
        dx, dy, _ = SCOOP_OFFSET
        return (rx + dx * math.cos(yaw) - dy * math.sin(yaw),
                ry + dx * math.sin(yaw) + dy * math.cos(yaw))

    def hopper_slot_position(self, rx, ry, yaw, slot_index):
        hx, hy, hz = HOPPER_OFFSET
        sx, sy = HOPPER_SLOTS[slot_index % len(HOPPER_SLOTS)]
        local_x, local_y = hx + sx, hy + sy
        wx = rx + local_x * math.cos(yaw) - local_y * math.sin(yaw)
        wy = ry + local_x * math.sin(yaw) + local_y * math.cos(yaw)
        return wx, wy, hz + HOPPER_SLOT_Z

    def check_collection(self):
        if self.robot_pose is None:
            return

        rx, ry, yaw = self.robot_pose
        scoop_x, scoop_y = self.scoop_position(rx, ry, yaw)
        for name, (sx, sy) in SHUTTLECOCKS.items():
            if name in self.collected:
                continue
            if math.hypot(sx - scoop_x, sy - scoop_y) <= CAPTURE_RADIUS:
                self.collect(name)

    def check_dropoff(self):
        if self.robot_pose is None or not self.onboard:
            return

        rx, ry, _ = self.robot_pose
        for gx, gy in GATHER_POINTS:
            if math.hypot(gx - rx, gy - ry) <= DROPOFF_RADIUS:
                self.deposit_all((gx, gy))
                return

    def collect(self, name):
        rx, ry, yaw = self.robot_pose
        x, y, z = self.hopper_slot_position(rx, ry, yaw, len(self.onboard))
        ok, result = self.teleport(name, x, y, z)

        if not ok:
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

    def track_onboard(self):
        """Keep onboard shuttlecocks riding in the hopper as the robot moves."""
        if self.robot_pose is None or not self.onboard:
            return

        rx, ry, yaw = self.robot_pose
        for i, name in enumerate(self.onboard):
            x, y, z = self.hopper_slot_position(rx, ry, yaw, i)
            self.teleport(name, x, y, z)

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

            ok, result = self.teleport(name, x, y, 0.0)
            if not ok:
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
