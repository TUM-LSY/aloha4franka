"""Simple script to test the position controller."""

from math import cos, pi, sin

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class PositionCommander(Node):
    def __init__(self):
        super().__init__("effort_commander")
        self.effort_val = self.declare_parameter("effort", value=500.0).value
        self.publisher_ = self.create_publisher(
            # Float64MultiArray, "gripper/gripper_effort_controller/commands", 10
            Float64MultiArray, "gripper/trigger_effort_controller/commands", 10
        )
        self.timer_frequency = 50.0
        self.timer_period = 1.0 / self.timer_frequency
        self.timer = self.create_timer(self.timer_period, self.publish_command)

        # Sine wave parameters

    def publish_command(self):
        msg = Float64MultiArray()

        msg.data = [self.effort_val]

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = PositionCommander()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
