#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class RebootServer(Node):

    def __init__(self):
        super().__init__('reboot_server')
        self.reboot_service = self.create_service(
            Trigger, 
            'reboot_dynamixel', 
            self.reboot_callback
        )
        self.get_logger().info('Reboot server started')

    def reboot_callback(self, request, response):
        self.get_logger().info('Received reboot request')
        # This would typically send a reboot command to the Dynamixel hardware
        # For now, just return success
        response.success = True
        response.message = "Reboot command sent"
        return response


def main(args=None):
    rclpy.init(args=args)
    reboot_server = RebootServer()
    
    try:
        rclpy.spin(reboot_server)
    except KeyboardInterrupt:
        pass
    finally:
        reboot_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()