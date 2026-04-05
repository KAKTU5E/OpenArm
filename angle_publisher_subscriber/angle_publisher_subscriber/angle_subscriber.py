#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class AngleSubscriber(Node):
    def __init__(self):
        super().__init__('angle_subscriber')
        
        # Define the expected angle names in order
        self.angle_names = ['base_Theta', 'blue_Theta1', 'white_Theta2', 'wrist_Theta3', 'open/close']
        
        # Subscriber for the angles
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Angle Subscriber Node has been started')
        self.get_logger().info(f'Waiting for angles on topic: joint_angles')
        self.get_logger().info('=' * 50)

    def listener_callback(self, msg):
        angles = msg.data
        
        # Check if we received the expected number of angles
        if len(angles) == 5:
            self.get_logger().info('=' * 50)
            self.get_logger().info('RECEIVED ANGLES:')
            
            # Display each angle with proper formatting
            for i, (name, value) in enumerate(zip(self.angle_names, angles)):
                if i < 4:
                    self.get_logger().info(f'  {name}: {value:.2f}°')
                else:
                    state = "OPEN" if value == 1 else "CLOSED"
                    self.get_logger().info(f'  {name}: {state} ({int(value)})')
            
            # Also show as a 5x1 array
            five_by_one = [[value] for value in angles]
            self.get_logger().info(f'\n5x1 Array: {five_by_one}')
            
            # Store the angles for potential later use
            self.current_angles = {
                'base_Theta': angles[0],
                'blue_Theta1': angles[1],
                'white_Theta2': angles[2],
                'wrist_Theta3': angles[3],
                'open_close': angles[4]
            }
            
            self.get_logger().info('=' * 50)
            
        else:
            self.get_logger().warn(f'Expected 5 angles, but received {len(angles)}')

def main(args=None):
    rclpy.init(args=args)
    node = AngleSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()