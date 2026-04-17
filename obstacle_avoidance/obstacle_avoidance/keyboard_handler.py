#!/usr/bin/env python3
"""
keyboard_handler.py
A non-blocking keyboard listener to control the iRobot Create3.

Key Commands:
  - SPACE : Toggles between AUTO (Exploration) and TELEOP (Manual).
  - Arrows: Moves the robot manually (only in TELEOP mode).
  - Q     : Stops the robot and shuts down the node.
"""

import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Safe speeds for manual teleop
TELEOP_LINEAR  = 0.2
TELEOP_ANGULAR = 0.8

class KeyboardHandler(Node):
    def __init__(self):
        super().__init__('keyboard_handler')

        # Standard Create3 Topics
        self.mode_pub = self.create_publisher(String, 'mode', 10)
        self.cmd_pub  = self.create_publisher(Twist, 'cmd_vel', 10)

        self.teleop_active = False
        self.running = True

        self.get_logger().info(
            'Create3 Keyboard Handler Ready.\n'
            '  SPACE      — Toggle AUTO / TELEOP\n'
            '  Arrow keys — Manual Drive (Teleop mode only)\n'
            '  Q          — Quit')

        # Run keyboard reading in a separate thread so it doesn't block ROS spin
        self.key_thread = threading.Thread(target=self.key_loop, daemon=True)
        self.key_thread.start()

    def key_loop(self):
        """Captures raw keystrokes from the terminal."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self.running:
                key = self.read_key(fd)
                if key == ' ': # SPACE
                    self.toggle_mode()
                elif key == 'q':
                    self.running = False
                    break
                elif self.teleop_active:
                    self.handle_teleop_key(key)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    @staticmethod
    def read_key(fd):
        """Reads characters and handles multi-byte arrow key sequences."""
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                return '\x1b[' + ch3
        return ch

    def toggle_mode(self):
        """Publishes the mode string to inform the create3_controller."""
        self.teleop_active = not self.teleop_active
        mode_str = 'TELEOP' if self.teleop_active else 'AUTO'
        
        msg = String()
        msg.data = mode_str
        self.mode_pub.publish(msg)
        
        self.get_logger().info(f'Switching to {mode_str} Mode')
        # Safety: stop robot on toggle
        self.cmd_pub.publish(Twist())

    def handle_teleop_key(self, key):
        """Translates arrow keys to Twist commands."""
        cmd = Twist()
        if key == '\x1b[A': # Up
            cmd.linear.x = TELEOP_LINEAR
        elif key == '\x1b[B': # Down
            cmd.linear.x = -TELEOP_LINEAR
        elif key == '\x1b[D': # Left
            cmd.angular.z = TELEOP_ANGULAR
        elif key == '\x1b[C': # Right
            cmd.angular.z = -TELEOP_ANGULAR
        else:
            return 
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()