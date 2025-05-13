"""Simple node use to transfer normal trigger service (standard) to a dynamixel trigger."""

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from std_msgs.msg import Header

from dynamixel_interfaces.srv import RebootDxl


class DxlTriggerServer(Node):
    def __init__(self):
        super().__init__("gripper_reboot_server")
        self.cli = self.create_client(
            RebootDxl, "dynamixel_hardware_interface/reboot_dxl"
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for reboot_dxl service...")

        self.srv = self.create_service(Trigger, "reboot_gripper", self.trigger_callback)
        self.get_logger().info("Gripper reboot server initialized.")

    def trigger_callback(self, request, response):
        req = RebootDxl.Request()
        req.header = Header()
        req.header.stamp = self.get_clock().now().to_msg()
        req.header.frame_id = "left_finger_joint"

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response.success = future.result().result
            response.message = "Reboot command sent"
        else:
            response.success = False
            response.message = "Failed to call RebootDxl service"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DxlTriggerServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
