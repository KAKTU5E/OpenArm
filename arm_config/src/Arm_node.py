import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import ListControllers


class ArmNode(Node):
    def __init__(self):
        super().__init__("arm_node")

        self.cli = self.create_client(
            ListControllers, "/controller_manager/list_controllers")
        self.timer = self.create_timer(1.0, self.try_list_controllers)
        self.request_in_flight = False # prevents timer from sending new ListControllers call while previous is pending

    def try_list_controllers(self):
        if not self.cli.service_is_ready():
            self.get_logger().info("Service /controller_manager/list_controllers not available, waiting...")
            return

        if self.request_in_flight:
            return

        self.request_in_flight = True
        req = ListControllers.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        self.request_in_flight = False
        try:
            response = future.result()
            self.get_logger().info(f"Found {len(response.controller)} controllers:")
            for controller in response.controller:
                self.get_logger().info(f"- Name: {controller.name}, State: {controller.state}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
