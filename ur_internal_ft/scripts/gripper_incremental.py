#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
import select

class GripperIncremental(Node):
    def __init__(self):
        super().__init__('gripper_incremental')
        self.pub = self.create_publisher(Float64, '/gripper/command', 10)
        self.sub = self.create_subscription(Float64, '/gripper/position', self.position_callback, 10)
        self.current_position = 0.0
        self.min_position = 0.0      # fully closed
        self.max_position = 75.0     # fully open (mm)
        self.step_size = 10         # mm per key press
        self.get_logger().info("Gripper incremental control started.")
        self.get_logger().info("Hold 'o' to open, hold 'c' to close, 'q' to quit.")

    def position_callback(self, msg):
        self.current_position = msg.data

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([fd], [], [], 0.02)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            if key is None:
                continue
            if key == 'o':
                new_pos = min(self.current_position + self.step_size, self.max_position)
                self.publish_position(new_pos)
            elif key == 'c':
                new_pos = max(self.current_position - self.step_size, self.min_position)
                self.publish_position(new_pos)
            elif key == 'q':
                break
            # Small delay to avoid flooding
            rclpy.spin_once(self, timeout_sec=0)

    def publish_position(self, pos):
        msg = Float64()
        msg.data = pos
        self.pub.publish(msg)
        self.get_logger().info(f"Position: {pos:.1f} mm")

def main(args=None):
    rclpy.init(args=args)
    node = GripperIncremental()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
