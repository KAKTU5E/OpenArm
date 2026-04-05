#!/usr/bin/env python3
import angles
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import sys

class AnglePublisher(Node):
    def __init__(self, angles):
        super().__init__('angle_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_state = JointState()

        self.joint_positions = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.joint_velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]


        # Unpack the 5 logical inputs
        base_theta   = math.radians(angles[0])
        blue_theta1  = math.radians(angles[1])
        white_theta2 = math.radians(angles[2])
        wrist_theta3 = math.radians(angles[3])
        finger_a1    = math.radians(angles[4])  # L1 / L2 group A
        finger_a2    = math.radians(angles[5])  # L1 / L2 group B  ← was being shadowed
        open_close   = angles[6]                # ← now valid index

        # Map open/close to finger joint positions (radians)
        # 1 = OPEN, 0 = CLOSED — tune these limits to match your hardware
        finger_open_pos   =  0.0
        finger_closed_pos =  1.0  # ~57 degrees closed
        finger_pos = finger_open_pos if open_close == 1 else finger_closed_pos

        # URDF joint names (continuous joints only — fixed joints are not published)
        # matched to your 5 logical inputs
        self.joint_names = ['Cycloidal_Simplified_for_FEA-1_Revolute-28', 'Cycloidal_Simplified_for_FEA-2_Revolute-30'
                            , 'Secondary-Joint-Cartilage_Revolute-11', 'Openhand1.0_Hand_simplifiedmain-v1_Revolute-33', 
                            'Openhand1.0_Hand_simplifiedmain-v1_Revolute-21', 'L1-v1_Revolute-23', 'L1-v1-1_Revolute-24', 'L2-v1_Revolute-25', 'L2-v1-1_Revolute-26']
        
        finger_scale = 0.0 if open_close == 1 else 1.0  # 1=OPEN→no deflection, 0=CLOSED→full deflection

        # Map logical inputs to all URDF joints
        self.joint_positions = [
        base_theta,                    # Cycloidal_Simplified_for_FEA-1_Revolute-28
        blue_theta1,                   # Cycloidal_Simplified_for_FEA-2_Revolute-30
        white_theta2,                  # Secondary-Joint-Cartilage_Revolute-11
        wrist_theta3,                  # Openhand1.0_Hand_simplifiedmain-v1_Revolute-33
        wrist_theta3,                  # Openhand1.0_Hand_simplifiedmain-v1_Revolute-21
        finger_a1 * finger_scale,      # L1-v1_Revolute-23
        -finger_a1 * finger_scale,     # L1-v1-1_Revolute-24  (mirrored)
        finger_a2 * finger_scale,      # L2-v1_Revolute-25
        -finger_a2 * finger_scale,     # L2-v1-1_Revolute-26  (mirrored)
        ]

        # Human-readable labels for logging
        self.angle_names = [
            'Cycloidal_Simplified_for_FEA-1_Revolute-28', 'Cycloidal_Simplified_for_FEA-2_Revolute-30', 'Secondary-Joint-Cartilage_Revolute-11',
            'Openhand1.0_Hand_simplifiedmain-v1_Revolute-33', 'Openhand1.0_Hand_simplifiedmain-v1_Revolute-21', 'L1-v1_Revolute-23', 'L1-v1-1_Revolute-24', 'L2-v1_Revolute-25', 'L2-v1-1_Revolute-26'
        ]
        self.angles_input = angles

        self.get_logger().info('=' * 50)
        self.get_logger().info('Angle Publisher initialized with:')
        for i, (name, value) in enumerate(zip(self.angle_names, self.angles_input)):
            if i < 4:
                self.get_logger().info(f'  {name}: {value}° ({math.radians(value):.4f} rad)')
            else:
                state = "OPEN" if value == 1 else "CLOSED"
                self.get_logger().info(f'  {name}: {state} ({value})')
        self.get_logger().info('=' * 50)

        self.timer = self.create_timer(0.1, self.publish_once)

    def publish_once(self):
        msg = JointState()

        # Header timestamp is required for robot_state_publisher
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name     = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = []   # empty = unspecified
        msg.effort   = []   # empty = unspecified

        self.publisher_.publish(msg)

        self.get_logger().info('Published JointState:')
        for name, pos in zip(self.joint_names, self.joint_positions):
            self.get_logger().info(f'  {name}: {pos:.4f} rad')

        self.get_logger().info('Temporary node — shutting down after publishing')
        #self.timer.cancel()
        


def parse_angles():
    if len(sys.argv) != 8:  # script name + 7 values
        print("Error: Please provide exactly 7 values")
        print("Usage: ros2 run <pkg> angle_publisher <base> <shoulder> <elbow> <wrist> <finger_angle_1> <finger_angle_2> <open_close>")
        print("Example: ros2 run <pkg> angle_publisher 45 90 135 25 25 25 1")
        print("  Angles in degrees. open_close: 0 = CLOSED, 1 = OPEN")
        sys.exit(1)

    angle_names = ['Cycloidal_Simplified_for_FEA-1_Revolute-28', 'Cycloidal_Simplified_for_FEA-2_Revolute-30', 'Secondary-Joint-Cartilage_Revolute-11',
                    'Openhand1.0_Hand_simplifiedmain-v1_Revolute-33', 'Openhand1.0_Hand_simplifiedmain-v1_Revolute-21', 'L1-v1_Revolute-23', 'L1-v1-1_Revolute-24', 'L2-v1_Revolute-25', 'L2-v1-1_Revolute-26']
    angles = []

    for i, arg in enumerate(sys.argv[1:]):
        try:
            value = float(arg)
            if i == 6 and value not in [0.0, 1.0]:
                print(f"Error: open/close must be 0 (CLOSED) or 1 (OPEN), got {value}")
                sys.exit(1)
            angles.append(value)
        except ValueError:
            print(f"Error: Value for {angle_names[i]} must be a number, got '{arg}'")
            sys.exit(1)

    return angles


def main(args=None):
    angles = parse_angles()
    rclpy.init(args=args)
    node = AnglePublisher(angles)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
    
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()