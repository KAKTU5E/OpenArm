import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointMoverNode(Node):
    def __init__(self):
        super().__init__('joint_mover_node')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

    def timer_callback(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        t = now - self.start_time
        # Oscillate angle between 0 and pi
        angle = math.pi * (0.5 * (1 + math.sin(t)))
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # List all joint names as used in your URDF
        msg.name = ['base_link_revolute-1', 'Motor-2_Revolute-21', 'Outer-Segment-1_Revolute-25']
        msg.position = [angle, angle, angle]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing angle: {angle:.2f} rad')

def main(args=None):
    rclpy.init(args=args)
    node = JointMoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()