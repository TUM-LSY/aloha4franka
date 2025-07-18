"""Simple node use to transfer normal trigger service (standard) to a dynamixel trigger."""

import rclpy
from rclpy.node import Node

from rclpy.time import Time
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from dynamixel_interfaces.srv import RebootDxl
from dynamixel_interfaces.msg import DynamixelState


class DiagnosticsPublisher(Node):
    def __init__(self):
        super().__init__("dynamixel_diagnostics")
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            "/diagnostics",
            10,
        )

        self.create_subscription(
            DynamixelState,
            "dynamixel_hardware_interface/dxl_state",
            self._callback_dynamixel_state,
            10,
        )

        self.create_timer(1.0, self._callback_publish_diagnostics)

        self._state = None

        self.get_logger().info("Diagnostics Publisher active.")

    def _callback_publish_diagnostics(self):
        """Publish diagnostics based on the current state."""
        if self._state is None:
            diagnostics_msg = DiagnosticArray()
            diagnostics_msg.header = Header()
            diagnostics_msg.header.stamp = self.get_clock().now().to_msg()
            diagnostics_msg.status = []
            diagnostics_msg.status.append(
                DiagnosticStatus(
                    level=DiagnosticStatus.STALE,
                    name=f"{self.get_namespace()} Gripper Dynamixel Diagnostics",
                    message="No state received yet.",
                )
            )
            self._diagnostics_publisher.publish(diagnostics_msg)
            return

        now = self.get_clock().now()
        if now - Time.from_msg(self._state.header.stamp) > rclpy.duration.Duration(
            seconds=1.0
        ):
            diagnostics_msg = DiagnosticArray()
            diagnostics_msg.header = Header()
            diagnostics_msg.header.stamp = self.get_clock().now().to_msg()
            diagnostics_msg.status = []
            diagnostics_msg.status.append(
                DiagnosticStatus(
                    level=DiagnosticStatus.STALE,
                    name=f"{self.get_namespace()} Gripper Dynamixel Diagnostics",
                    message="Sensor is stale. USB probably disconnected.",
                )
            )
            self._diagnostics_publisher.publish(diagnostics_msg)
            return
        diagnostics_msg = DiagnosticArray()
        diagnostics_msg.header = Header()
        diagnostics_msg.header.stamp = self.get_clock().now().to_msg()
        diagnostics_msg.status = []
        diagnostics_msg.status.append(
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name=f"{self.get_namespace()} Gripper Dynamixel Diagnostics",
                message="Receiving dynamixel state.",
            )
        )
        self._diagnostics_publisher.publish(diagnostics_msg)

        diagnostics_msg = DiagnosticArray()
        diagnostics_msg.header = Header()
        diagnostics_msg.header.stamp = self.get_clock().now().to_msg()
        diagnostics_msg.status = []
        state = (
            DiagnosticStatus.OK
            if self._state.comm_state == DynamixelState.COMM_STATE_OK
            and not any(self._state.dxl_hw_state)
            else DiagnosticStatus.ERROR
        )
        status = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name=f"{self.get_namespace()} Gripper Dynamixel State",
            message="Dynamixel is operational."
            if state == DiagnosticStatus.OK
            else "Dynamixel has an error. Check values.",
        )
        status.values = []
        status.values.append(
            KeyValue(
                key="ID(s)",
                value=", ".join([str(id) for id in self._state.id]),
            )
        )
        status.values.append(
            KeyValue(
                key="hardware state",
                value=", ".join(
                    [str(hw_state) for hw_state in self._state.dxl_hw_state]
                ),
            )
        )
        status.values.append(
            KeyValue(
                key="communication state",
                value=str(self._state.comm_state),
            )
        )
        status.values.append(
            KeyValue(
                key="Torque enabled",
                value=str(self._state.torque_state),
            )
        )
        diagnostics_msg.status.append(status)

        self._diagnostics_publisher.publish(diagnostics_msg)

    def _callback_dynamixel_state(self, msg: DynamixelState):
        """Callback for Dynamixel state messages."""
        self._state = msg


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
