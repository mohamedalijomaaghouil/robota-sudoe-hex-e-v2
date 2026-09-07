#!/usr/bin/env python3

import time

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Bool


INPUT_TOPIC = "/hex_e/compensated_wrench"
OUTPUT_TOPIC = "/hex_e/conditioned_wrench"
READY_TOPIC = "/hex_e/conditioned_wrench_ready"

ALPHA = 0.20
DEADBAND = np.asarray((0.90, 0.12, 0.85), dtype=float)

MAX_COMPONENT_FORCE = 15.0
MAX_FORCE_NORM = 20.0
STALE_TIMEOUT = 0.50


class SignalConditioner(Node):
    def __init__(self):
        super().__init__("hex_e_signal_conditioning_diagnostic")

        self.filtered = np.zeros(3, dtype=float)
        self.initialized = False
        self.last_message_time = None
        self.was_ready = False

        self.subscription = self.create_subscription(
            WrenchStamped,
            INPUT_TOPIC,
            self.wrench_callback,
            10,
        )

        self.publisher = self.create_publisher(
            WrenchStamped,
            OUTPUT_TOPIC,
            10,
        )

        self.ready_publisher = self.create_publisher(
            Bool,
            READY_TOPIC,
            10,
        )

        self.watchdog = self.create_timer(
            0.10,
            self.watchdog_callback,
        )

        self.get_logger().info("HEX-E signal conditioner started.")
        self.get_logger().info(f"Input:  {INPUT_TOPIC}")
        self.get_logger().info(f"Output: {OUTPUT_TOPIC}")
        self.get_logger().info(
            f"EMA alpha={ALPHA:.2f}; deadband={DEADBAND} N"
        )
        self.get_logger().warning(
            "Diagnostic only: this node publishes no robot-motion commands."
        )

    def publish_ready(self, value):
        message = Bool()
        message.data = bool(value)
        self.ready_publisher.publish(message)
        self.was_ready = bool(value)

    def wrench_callback(self, message):
        force = np.asarray(
            (
                message.wrench.force.x,
                message.wrench.force.y,
                message.wrench.force.z,
            ),
            dtype=float,
        )

        if not np.all(np.isfinite(force)):
            self.get_logger().error(
                "Non-finite compensated force received; output disabled."
            )
            self.initialized = False
            self.publish_ready(False)
            return

        self.last_message_time = time.monotonic()

        if not self.initialized:
            self.filtered = force.copy()
            self.initialized = True
        else:
            self.filtered = (
                ALPHA * force
                + (1.0 - ALPHA) * self.filtered
            )

        # Continuous soft deadband:
        # zero inside the deadband, and subtract the threshold outside it.
        conditioned = (
            np.sign(self.filtered)
            * np.maximum(np.abs(self.filtered) - DEADBAND, 0.0)
        )

        conditioned = np.clip(
            conditioned,
            -MAX_COMPONENT_FORCE,
            MAX_COMPONENT_FORCE,
        )

        force_norm = float(np.linalg.norm(conditioned))
        if force_norm > MAX_FORCE_NORM:
            conditioned *= MAX_FORCE_NORM / force_norm

        output = WrenchStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = message.header.frame_id or "tool0"

        output.wrench.force.x = float(conditioned[0])
        output.wrench.force.y = float(conditioned[1])
        output.wrench.force.z = float(conditioned[2])

        # Torque conditioning has not yet been validated.
        output.wrench.torque.x = 0.0
        output.wrench.torque.y = 0.0
        output.wrench.torque.z = 0.0

        self.publisher.publish(output)

        if not self.was_ready:
            self.get_logger().info(
                "Conditioned wrench is valid and ready for diagnostics."
            )
        self.publish_ready(True)

    def watchdog_callback(self):
        if self.last_message_time is None:
            self.publish_ready(False)
            return

        age = time.monotonic() - self.last_message_time

        if age > STALE_TIMEOUT:
            if self.was_ready:
                self.get_logger().error(
                    f"Input became stale ({age:.3f} s); readiness cleared."
                )

            self.initialized = False
            self.publish_ready(False)


def main():
    rclpy.init()
    node = SignalConditioner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_ready(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
