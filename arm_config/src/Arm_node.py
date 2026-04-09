import math
import rclpy
from rclpy.node import Node
import time

from controller_manager_msgs.srv import ListControllers

class Arm_Node(Node):
    def __init__(self):
        super().__init__('Arm_Node')

        self.cli = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.send_request()

    def send_request(self):
        req = ListControllers.Request()
        self.future = self.cli.call_async(req)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self,future):
        try:
            response = future.result()
            self.get_logger().info(f'Found {len(response.controller)} controllers:')
            for controller in response.controller:
                self.get_logger().info(f'- Name: {controller.name}, State: {controller.state}')
        except Exception as e:
            self.get_logger(f'Service call failed: {e}')
        time.sleep(5)



def main(args=None):
    rclpy.init(args=args)
    node = Arm_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()
