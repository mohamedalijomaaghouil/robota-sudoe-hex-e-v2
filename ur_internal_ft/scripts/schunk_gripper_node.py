#!/usr/bin/env python3
"""
SCHUNK EGK 50 gripper ROS2 node
Controls gripper via TCP socket to port 55050 on the robot controller
Subscribes to /gripper/command (std_msgs/Float64) for position commands
Publishes /gripper/position (std_msgs/Float64) for current position
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
import socket
import time
import threading

ROBOT_IP = '192.168.0.210'
SCHUNK_PORT = 55050
MAX_POSITION = 75.0  # mm safety limit
MIN_POSITION = 0.0
DEFAULT_SPEED = 20.0  # mm/s

class SchunkGripperNode(Node):
    def __init__(self):
        super().__init__('schunk_gripper_node')
        
        # Publishers
        self.pos_publisher = self.create_publisher(Float64, '/gripper/position', 10)
        
        # Subscribers
        self.cmd_subscriber = self.create_subscription(
            Float64, '/gripper/command', self.command_callback, 10)
        self.str_subscriber = self.create_subscription(
            String, '/gripper/action', self.action_callback, 10)
        
        # Timer to publish current position at 10Hz
        self.timer = self.create_timer(0.1, self.publish_position)
        
        self.get_logger().info('SCHUNK gripper node started')
        self.get_logger().info(f'Connecting to {ROBOT_IP}:{SCHUNK_PORT}')
        
        # Acknowledge any startup errors
        self.schunk_command('acknowledge(0)')
        self.get_logger().info('Gripper ready')

    def schunk_command(self, cmd):
        try:
            s = socket.socket()
            s.settimeout(5.0)
            s.connect((ROBOT_IP, SCHUNK_PORT))
            s.send((cmd + '\n').encode())
            time.sleep(0.1)
            response = s.recv(1024).decode().strip()
            s.close()
            return response
        except Exception as e:
            self.get_logger().error(f'Socket error: {e}')
            return None

    def command_callback(self, msg):
        position = float(msg.data)
        position = max(MIN_POSITION, min(MAX_POSITION, position))
        self.get_logger().info(f'Moving gripper to {position:.1f}mm')
        self.schunk_command(f'absolute(0, {position:.1f}, {DEFAULT_SPEED})')

    def action_callback(self, msg):
        action = msg.data.lower()
        if action == 'open':
            self.get_logger().info('Opening gripper')
            self.schunk_command(f'absolute(0, {MAX_POSITION}, {DEFAULT_SPEED})')
        elif action == 'close':
            self.get_logger().info('Closing gripper')
            self.schunk_command(f'absolute(0, 30.0, {DEFAULT_SPEED})')
        elif action == 'grip':
            self.get_logger().info('Gripping')
            self.schunk_command(f'grip(0, true, 50.0, 50.0, {DEFAULT_SPEED})')
        elif action == 'release':
            self.get_logger().info('Releasing')
            self.schunk_command(f'release(0)')
        elif action == 'acknowledge':
            self.schunk_command('acknowledge(0)')
        else:
            self.get_logger().warn(f'Unknown action: {action}')

    def publish_position(self):
        response = self.schunk_command('getPosition(0)')
        if response and response.startswith('float,'):
            try:
                pos = float(response.split(',')[1])
                msg = Float64()
                msg.data = pos
                self.pos_publisher.publish(msg)
            except:
                pass

def main():
    rclpy.init()
    node = SchunkGripperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
