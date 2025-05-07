"""Simple script to test the position controller."""

from math import cos, pi, sin

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class PositionCommander(Node):
    def __init__(self):
        super().__init__("position_commander")
        self.publisher_ = self.create_publisher(
            Float64MultiArray, "gripper_position_controller/commands", 10
        )
        self.min_value = 0.0
        # self.max_value = 0.041
        self.max_value = 2.0

        self.timer_frequency = 50.0
        self.timer_period = 1.0 / self.timer_frequency
        self.timer = self.create_timer(self.timer_period, self.publish_command)

        # Sine wave parameters
        self.t = 0.0
        self.sine_frequency = 0.25
        self.sine_amplitude = (self.max_value + self.min_value) / 2.0

        self.get_logger().info("Position Commander Node Initialized.")

    def publish_command(self):
        msg = Float64MultiArray()

        gripper_val = (
            self.sine_amplitude * sin(2.0 * pi * self.sine_frequency * self.t)
            + self.sine_amplitude
        )

        msg.data = [gripper_val]

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}", throttle_duration_sec=2.0)

        self.t += self.timer_period


def main(args=None):
    rclpy.init(args=args)
    node = PositionCommander()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
