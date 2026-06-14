#!/usr/bin/env python3
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# (linear_x, angular_z)
WASD = {
    'w': ( 1,  0),
    's': (-1,  0),
    'a': ( 0,  1),
    'd': ( 0, -1),
}

# Arrow escape sequences: ESC [ <code>
ARROWS = {
    'A': ( 1,  0),  # up
    'B': (-1,  0),  # down
    'D': ( 0,  1),  # left
    'C': ( 0, -1),  # right
}

LINEAR_SPEED  = 0.3   # m/s
ANGULAR_SPEED = 1.0   # rad/s

MSG = """
Controls:
  W / Up    : forward
  S / Down  : backward
  A / Left  : turn left
  D / Right : turn right
  Space     : stop
  Ctrl+C    : quit
"""


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            sys.stdin.read(1)          # discard '['
            return ('arrow', sys.stdin.read(1))
        return ('char', ch)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def run(self):
        print(MSG)
        while rclpy.ok():
            kind, key = get_key()
            twist = Twist()

            if kind == 'char':
                if key == '\x03':      # Ctrl+C
                    break
                if key in WASD:
                    lin, ang = WASD[key]
                    twist.linear.x  = float(lin) * LINEAR_SPEED
                    twist.angular.z = float(ang) * ANGULAR_SPEED
                # space or unrecognised → zero twist (stop)
            elif kind == 'arrow' and key in ARROWS:
                lin, ang = ARROWS[key]
                twist.linear.x  = float(lin) * LINEAR_SPEED
                twist.angular.z = float(ang) * ANGULAR_SPEED

            self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())   # stop robot on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
