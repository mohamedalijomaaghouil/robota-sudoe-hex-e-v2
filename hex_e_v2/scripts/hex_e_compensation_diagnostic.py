#!/usr/bin/env python3

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


RAW_TOPIC = "/bus0/ft_sensor0/ft_sensor_readings/wrench"
OUTPUT_TOPIC = "/hex_e/compensated_wrench"
READY_TOPIC = "/hex_e/compensation_ready"

BASE_FRAME = "base_link"
TOOL_FRAME = "tool0"

GRAVITY_BASE = np.array([0.0, 0.0, -9.80665], dtype=float)

# Provisional 625 g-derived force calibration:
#
#     calibrated_force_tool = A @ raw_force
#
# This matrix was independently validated using the 1051 g assembly.
A_FORCE = np.array(
    [
        [1.25553198, -0.0316532263, -0.0637730031],
        [0.000731990577, 1.12968389, -0.0459014772],
        [-0.116084707, 0.0696001054, 1.41995740],
    ],
    dtype=float,
)

# Provisional effective tool/handle mass estimate.
# This remains subject to unloaded multi-orientation validation.
TOOL_MASS_KG = 2.233316

# The HEX-E publishes at 10 Hz, so 50 samples require about five seconds.
TARE_SAMPLE_COUNT = 50


def quaternion_to_rotation_matrix(x, y, z, w):
    """Return the 3x3 rotation matrix represented by a quaternion."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)

    if norm < 1.0e-12:
        raise ValueError("Quaternion norm is zero")

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=float,
    )


class HexECompensationDiagnostic(Node):
    def __init__(self):
        super().__init__("hex_e_compensation_diagnostic")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        self.subscription = self.create_subscription(
            WrenchStamped,
            RAW_TOPIC,
            self.wrench_callback,
            10,
        )

        self.status_timer = self.create_timer(1.0, self.status_callback)

        self.tare_samples = []
        self.bias_tool = None
        self.latest_external_force = None
        self.last_message_monotonic = None
        self.last_tf_warning_monotonic = 0.0

        self.get_logger().info(
            "HEX-E diagnostic compensation node started."
        )
        self.get_logger().info(
            f"Input:  {RAW_TOPIC}"
        )
        self.get_logger().info(
            f"Output: {OUTPUT_TOPIC}"
        )
        self.get_logger().info(
            "This node publishes diagnostic wrench data only."
        )
        self.get_logger().info(
            "It does not publish any robot motion commands."
        )
        self.get_logger().warning(
            "Keep the tool unloaded and do not touch the robot "
            f"while collecting {TARE_SAMPLE_COUNT} tare samples."
        )

    def gravity_in_tool(self):
        transform = self.tf_buffer.lookup_transform(
            BASE_FRAME,
            TOOL_FRAME,
            Time(),
        )

        quaternion = transform.transform.rotation

        # lookup_transform(base, tool) provides the tool-to-base rotation.
        rotation_base_from_tool = quaternion_to_rotation_matrix(
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )

        # Express the base-frame gravity vector in tool coordinates.
        return rotation_base_from_tool.T @ GRAVITY_BASE

    def publish_ready(self, ready):
        message = Bool()
        message.data = bool(ready)
        self.ready_publisher.publish(message)

    def wrench_callback(self, message):
        self.last_message_monotonic = time.monotonic()

        raw_force = np.array(
            [
                message.wrench.force.x,
                message.wrench.force.y,
                message.wrench.force.z,
            ],
            dtype=float,
        )

        if not np.all(np.isfinite(raw_force)):
            self.get_logger().warning(
                "Discarding a non-finite HEX-E force sample."
            )
            return

        try:
            gravity_tool = self.gravity_in_tool()
        except (TransformException, ValueError) as error:
            now = time.monotonic()

            if now - self.last_tf_warning_monotonic >= 2.0:
                self.get_logger().warning(
                    f"Waiting for {BASE_FRAME} -> {TOOL_FRAME} TF: {error}"
                )
                self.last_tf_warning_monotonic = now

            self.publish_ready(False)
            return

        calibrated_force = A_FORCE @ raw_force
        gravity_force = TOOL_MASS_KG * gravity_tool

        # This contains the remaining sensor/tool bias plus any externally
        # applied force.
        pre_tare_force = calibrated_force - gravity_force

        if self.bias_tool is None:
            self.tare_samples.append(pre_tare_force)

            count = len(self.tare_samples)

            if count == 1 or count % 10 == 0:
                self.get_logger().info(
                    f"Collecting stationary tare: "
                    f"{count}/{TARE_SAMPLE_COUNT}"
                )

            if count < TARE_SAMPLE_COUNT:
                self.publish_ready(False)
                return

            samples = np.vstack(self.tare_samples)
            self.bias_tool = np.mean(samples, axis=0)
            tare_std = np.std(samples, axis=0, ddof=1)

            self.get_logger().info(
                "Tare complete."
            )
            self.get_logger().info(
                "Estimated tool-frame bias [N]: "
                + np.array2string(
                    self.bias_tool,
                    precision=6,
                    suppress_small=False,
                )
            )
            self.get_logger().info(
                "Tare sample standard deviation [N]: "
                + np.array2string(
                    tare_std,
                    precision=6,
                    suppress_small=False,
                )
            )
            self.get_logger().warning(
                "Compensation is diagnostic and provisional; "
                "do not connect it to a motion controller yet."
            )

        external_force = pre_tare_force - self.bias_tool
        self.latest_external_force = external_force

        output = WrenchStamped()
        output.header.stamp = message.header.stamp
        output.header.frame_id = TOOL_FRAME

        output.wrench.force.x = float(external_force[0])
        output.wrench.force.y = float(external_force[1])
        output.wrench.force.z = float(external_force[2])

        # Torque calibration has not yet been accepted for control use.
        output.wrench.torque.x = 0.0
        output.wrench.torque.y = 0.0
        output.wrench.torque.z = 0.0

        self.publisher.publish(output)
        self.publish_ready(True)

    def status_callback(self):
        if self.last_message_monotonic is None:
            self.get_logger().warning(
                f"No wrench messages received yet from {RAW_TOPIC}."
            )
            self.publish_ready(False)
            return

        age = time.monotonic() - self.last_message_monotonic

        if age > 0.5:
            self.get_logger().warning(
                f"HEX-E wrench stream is stale: age={age:.3f} s"
            )
            self.publish_ready(False)
            return

        if self.bias_tool is not None and self.latest_external_force is not None:
            norm = np.linalg.norm(self.latest_external_force)
            self.get_logger().info(
                "Compensated force in tool0 [N]: "
                + np.array2string(
                    self.latest_external_force,
                    precision=4,
                    suppress_small=False,
                )
                + f"; norm={norm:.4f} N"
            )


def main(args=None):
    rclpy.init(args=args)
    node = HexECompensationDiagnostic()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
