#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys, termios, tty, select
import time 

class ArmTeleopNode(Node):
    def __init__(self):
        super().__init__('arm_teleop_node')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_state = JointState()
        # Defining the joint names, must be accurate to URDF file
        self.joint_names = ['base_link_revolute-1', 'Motor-2_Revolute-21', 'Outer-Segment-1_Revolute-25']

        # Set initial joint positions to zero
        self.joint_positions = [0.0,0.0,0.0]
        self.joint_velocities = [0.0,0.0,0.0]

        # timer to publish at a certain interval
        self.timer = self.create_timer(0.1, self.publish_joint_states)     
    
        self.get_logger().info(
            'Press: \n' \
            'z/x to rotate entire arm, \n' \
            'a/s to rotate shoulder joint, \n' \
            'q/w to move elbow joint, \n' \
            'CCW/CW \n' \
        )
        

    def get_key(self):
        # Reads a single keypress from keyboard

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

        if rlist: 
            key = sys.stdin.read(1)
        else: 
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
    def publish_joint_states(self):

        key = self.get_key()

        if key == 'z':
            self.joint_positions[0] += 0.1 # Rotate base ccw
        elif key == 'x':
            self.joint_positions[0] -= 0.1 # Rotate base cw

        elif key == 'a':
            self.joint_positions[1] += 0.1 # Rotate shoulder ccw
        elif key == 's':
            self.joint_positions[1] -= 0.1 # Rotate shoulder cw

        elif key == 'q':
            self.joint_positions[2] += 0.1 # Rotate elbow ccw
        elif key == 'w':
            self.joint_positions[2] -= 0.1 # Rotate elbow cw

        elif key == '\x03': # Ctrl-C to quit
            self.get_logger().info('Exiting...')
            rclpy.shutdown()
            return
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        self.publisher.publish(msg)

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)
    settings[3] = settings[3] & ~termios.ECHO # Disable echo
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    rclpy.init(args=args)
    node = ArmTeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

