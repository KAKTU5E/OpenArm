#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time



class JointStateListenerNode(Node):
    def __init__(self):
        super().__init__('joint_state_listener_node')
        self.subscription = self.create_subscription(JointState, 'joint_states', self.subscription_callback, qos_profile=10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.get_logger().info('Joint State Listener Node has been started.')


    def subscription_callback(self, msg):
        # Process the received joint state message
        self.get_logger().info(f'Received joint state: {msg}')
        # Example: Log the joint names and positions
        for name, position in zip(msg.name, msg.position):
            self.get_logger().info(f'\nJoint: {name}, Position: {position:.2f}')
            
        time.sleep(0.05)
        
        # Add any additional processing logic here

    def timer_callback(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        t = now - self.start_time
        self.get_logger().info(f'Timer callback at time: {t:.2f} seconds')


def main(args=None):
    rclpy.init(args=args)
    node = JointStateListenerNode()     
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()
    finally:
        pass



if __name__ == '__main__':
    main()
# This code defines a ROS 2 node that listens to joint state messages and logs the received data.
# It subscribes to the 'joint_states' topic and processes incoming messages in the listener_callback method.    