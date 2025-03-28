#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import termios, tty, sys, select
import numpy as np

# Global variable to store terminal settings for restoration
settings = None
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        w     
   a    s    d
              
↑ : up (+z)
↓ : down (-z)
→ : rotate (+z axis)
← : rotate (-z axis)

x : stop
z : disable control momentarily

CTRL-C twice to quit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    # Use select to check if input is available (non-blocking)
    select.select([sys.stdin], [], [], 0.0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    # Handle arrow keys (which are sent as escape sequences)
    if key == '\x1b':  # Escape character
        key2 = sys.stdin.read(1)
        if key2 == '[':
            key3 = sys.stdin.read(1)
            if key3 == 'A':
                key = 'UP'
            elif key3 == 'B':
                key = 'DOWN'
            elif key3 == 'C':
                key = 'RIGHT'
            elif key3 == 'D':
                key = 'LEFT'
    return key

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        # Publishers for Twist commands and control state
        self.keyboard_cmd_pub = self.create_publisher(Twist, '/keyboard_interface', 10)
        self.control_state_pub = self.create_publisher(Bool, '/keyboard_control_state', 5)

        # Timer callback at 100Hz
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Command parameters
        self.cmd_vel = 0.1
        self.cmd_omega = 0.5
        self.control_state = True

    def timer_callback(self):
        key = getKey()
        twist = Twist()

        vel_x = 0.0
        vel_y = 0.0
        vel_z = 0.0
        omega_z = 0.0
        
        # Default zero velocities
        if key == 'w':  # Move in +x direction
            vel_x = self.cmd_vel
        elif key == 's':  # Move in -x direction
            vel_x = -self.cmd_vel
        elif key == 'a':  # Move in +y direction
            vel_y = self.cmd_vel
        elif key == 'd':  # Move in -y direction
            vel_y = -self.cmd_vel
        elif key == 'UP':  # Move in +z direction
            vel_z = self.cmd_vel
        elif key == 'DOWN':  # Move in -z direction
            vel_z = -self.cmd_vel
        elif key == 'RIGHT':  # Rotate positively about z
            omega_z = self.cmd_omega
        elif key == 'LEFT':  # Rotate negatively about z
            omega_z = -self.cmd_omega
        elif key == 'x':  # Stop movement
            vel_x = 0.0
            vel_y = 0.0
            vel_z = 0.0
            omega_z = 0.0
        elif key == 'z':  # Disable control momentarily
            self.control_state = False
        elif key == '\x03':  # Ctrl-C pressed: shutdown
            rclpy.shutdown()
            return
        
        twist.linear.x = vel_x
        twist.linear.y = vel_y
        twist.linear.z = vel_z
        twist.angular.z = omega_z

        # Publish the twist command
        self.keyboard_cmd_pub.publish(twist)
        
        # Publish the control state
        bool_msg = Bool()
        bool_msg.data = self.control_state
        self.control_state_pub.publish(bool_msg)

        # Reset the control state if it was disabled
        if not self.control_state:
            self.control_state = True

def main(args=None):
    global settings
    # Save current terminal settings so they can be restored later
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        print(msg)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal settings on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
