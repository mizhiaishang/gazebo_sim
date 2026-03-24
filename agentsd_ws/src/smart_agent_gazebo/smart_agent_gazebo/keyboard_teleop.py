#!/usr/bin/env python3
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
Keyboard Teleop (smart_agent)
  w/s : forward / backward
  a/d : left / right (lateral)
  q/e : yaw left / yaw right
  space or x : stop
  Ctrl+C : quit
"""


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, '/model/smart_agent/cmd_vel', 10)
        self.linear_speed = self.declare_parameter('linear_speed', 1.2).value
        self.angular_speed = self.declare_parameter('angular_speed', 1.6).value

        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        self.current = Twist()
        self.last_key_time = self.get_clock().now().nanoseconds
        self.stop_timeout_ns = int(0.3 * 1e9)

        print(HELP)
        print(
            f'Using linear_speed={self.linear_speed:.2f}, angular_speed={self.angular_speed:.2f}, '
            f'topic=/model/smart_agent/cmd_vel'
        )

        self.create_timer(0.05, self._tick)

    def _read_key(self) -> str:
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        key = sys.stdin.read(1) if rlist else ''
        return key

    def _publish_zero(self) -> None:
        t = Twist()
        self.publisher.publish(t)
        self.current = t

    def _tick(self) -> None:
        key = self._read_key()
        updated = False
        t = Twist()

        if key == 'w':
            t.linear.x = float(self.linear_speed)
            updated = True
        elif key == 's':
            t.linear.x = -float(self.linear_speed)
            updated = True
        elif key == 'a':
            t.linear.y = float(self.linear_speed)
            updated = True
        elif key == 'd':
            t.linear.y = -float(self.linear_speed)
            updated = True
        elif key == 'q':
            t.angular.z = float(self.angular_speed)
            updated = True
        elif key == 'e':
            t.angular.z = -float(self.angular_speed)
            updated = True
        elif key in (' ', 'x'):
            self._publish_zero()
            self.last_key_time = self.get_clock().now().nanoseconds
            return

        now_ns = self.get_clock().now().nanoseconds

        if updated:
            self.current = t
            self.last_key_time = now_ns
        elif now_ns - self.last_key_time > self.stop_timeout_ns:
            if (
                self.current.linear.x != 0.0
                or self.current.linear.y != 0.0
                or self.current.angular.z != 0.0
            ):
                self._publish_zero()
                return

        # Keep publishing current command for smoother control and reliability.
        self.publisher.publish(self.current)

    def close(self) -> None:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
