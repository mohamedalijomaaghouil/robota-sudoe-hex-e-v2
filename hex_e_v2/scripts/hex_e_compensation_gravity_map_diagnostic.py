#!/usr/bin/env python3

import time

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


RAW_TOPIC = "/bus0/ft_sensor0/ft_sensor_readings/wrench"
OUTPUT_TOPIC = "/hex_e/gravity_map_compensated_wrench"
READY_TOPIC = "/hex_e/gravity_map_compensation_ready"

BASE_FRAME = "base_link"
TOOL_FRAME = "tool0"

TARE_SAMPLES = 50
STALE_TIMEOUT = 0.50

GRAVITY_BASE = np.asarray((0.0, 0.0, -9.80665), dtype=float)

# Force-only calibration identified using the 625 g experiments.
A_FORCE = np.asarray(
    (
        (1.25553198, -0.0316532263, -0.0637730031),
        (0.000731990577, 1.12968389, -0.0459014772),
        (-0.116084707, 0.0696001054, 1.41995740),
    ),
    dtype=float,
)

# Unloaded orientation-dependent gravity map:
#
# delta calibrated force = H_GRAVITY_MAP @ delta gravity_tool
H_GRAVITY_MAP = np.asarray(
    (
        (2.36141775, -0.09427079, -0.08677186),
        (0.05265703, 2.07757729, -0.08895589),
        (-0.38057919, -0.15119476, 2.25313453),
    ),
    dtype=float,
)


def quaternion_rotation_matrix(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    norm = np.sqrt(x*x + y*y + z*z + w*w)

    if not np.isfinite(norm) or norm < 1.0e-12:
        raise ValueError("Invalid TF quaternion")

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    return np.asarray(
        (
            (
                1.0 - 2.0*(y*y + z*z),
                2.0*(x*y - z*w),
                2.0*(x*z + y*w),
            ),
            (
                2.0*(x*y + z*w),
                1.0 - 2.0*(x*x + z*z),
                2.0*(y*z - x*w),
            ),
            (
                2.0*(x*z - y*w),
                2.0*(y*z + x*w),
                1.0 - 2.0*(x*x + y*y),
            ),
        ),
        dtype=float,
    )


class GravityMapCompensation(Node):
    def __init__(self):
        super().__init__("hex_e_compensation_gravity_map_diagnostic")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
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

        self.subscription = self.create_subscription(
            WrenchStamped,
            RAW_TOPIC,
            self.wrench_callback,
            10,
        )

        self.status_timer = self.create_timer(
            0.25,
            self.status_callback,
        )

        self.tare_forces = []
        self.tare_gravities = []

        self.reference_force = None
        self.reference_gravity = None

        self.last_input_time = None
        self.latest_external_force = None
        self.last_report_time = 0.0
        self.last_tf_warning_time = 0.0
        self.ready_state = False

        self.publish_ready(False)

        self.get_logger().info(
            "HEX-E gravity-map compensation diagnostic started."
        )
        self.get_logger().info(f"Input:  {RAW_TOPIC}")
        self.get_logger().info(f"Output: {OUTPUT_TOPIC}")
        self.get_logger().warning(
            "Keep the complete tool unloaded and do not touch the robot "
            "during the 50-sample startup tare."
        )
        self.get_logger().warning(
            "Diagnostic only: this node publishes no robot-motion commands."
        )

    def publish_ready(self, state):
        message = Bool()
        message.data = bool(state)
        self.ready_publisher.publish(message)
        self.ready_state = bool(state)

    def gravity_in_tool(self):
        transform = self.tf_buffer.lookup_transform(
            BASE_FRAME,
            TOOL_FRAME,
            Time(),
        )

        rotation_tool_to_base = quaternion_rotation_matrix(
            transform.transform.rotation
        )

        return rotation_tool_to_base.T @ GRAVITY_BASE

    def wrench_callback(self, message):
        raw_force = np.asarray(
            (
                message.wrench.force.x,
                message.wrench.force.y,
                message.wrench.force.z,
            ),
            dtype=float,
        )

        if not np.all(np.isfinite(raw_force)):
            self.get_logger().warning(
                "Discarding non-finite raw force sample."
            )
            return

        try:
            gravity_tool = self.gravity_in_tool()
        except (TransformException, ValueError) as error:
            now = time.monotonic()

            if now - self.last_tf_warning_time >= 2.0:
                self.get_logger().warning(
                    f"Waiting for {BASE_FRAME} -> {TOOL_FRAME} TF: {error}"
                )
                self.last_tf_warning_time = now

            return

        calibrated_force = A_FORCE @ raw_force
        self.last_input_time = time.monotonic()

        if self.reference_force is None:
            self.tare_forces.append(calibrated_force)
            self.tare_gravities.append(gravity_tool)

            count = len(self.tare_forces)

            if count == 1 or count % 10 == 0:
                self.get_logger().info(
                    f"Collecting stationary tare: "
                    f"{count}/{TARE_SAMPLES}"
                )

            if count >= TARE_SAMPLES:
                forces = np.vstack(self.tare_forces)
                gravities = np.vstack(self.tare_gravities)

                self.reference_force = np.mean(forces, axis=0)
                self.reference_gravity = np.mean(gravities, axis=0)

                self.get_logger().info("Tare complete.")
                self.get_logger().info(
                    "Reference calibrated force [N]: "
                    + np.array2string(
                        self.reference_force,
                        precision=6,
                    )
                )
                self.get_logger().info(
                    "Reference gravity in tool0 [m/s^2]: "
                    + np.array2string(
                        self.reference_gravity,
                        precision=6,
                    )
                )
                self.get_logger().info(
                    "Tare force standard deviation [N]: "
                    + np.array2string(
                        np.std(forces, axis=0),
                        precision=6,
                    )
                )

                self.publish_ready(True)

            return

        gravity_change = gravity_tool - self.reference_gravity

        predicted_unloaded_change = (
            H_GRAVITY_MAP @ gravity_change
        )

        external_force = (
            calibrated_force
            - self.reference_force
            - predicted_unloaded_change
        )

        if not np.all(np.isfinite(external_force)):
            self.publish_ready(False)
            return

        output = WrenchStamped()
        output.header.stamp = message.header.stamp
        output.header.frame_id = TOOL_FRAME

        output.wrench.force.x = float(external_force[0])
        output.wrench.force.y = float(external_force[1])
        output.wrench.force.z = float(external_force[2])

        output.wrench.torque.x = 0.0
        output.wrench.torque.y = 0.0
        output.wrench.torque.z = 0.0

        self.publisher.publish(output)
        self.latest_external_force = external_force
        self.publish_ready(True)

    def status_callback(self):
        now = time.monotonic()

        if self.last_input_time is None:
            self.publish_ready(False)
            return

        age = now - self.last_input_time

        if age > STALE_TIMEOUT:
            if self.ready_state:
                self.get_logger().warning(
                    f"Raw wrench input is stale: age={age:.3f} seconds"
                )

            self.publish_ready(False)
            return

        if (
            self.latest_external_force is not None
            and now - self.last_report_time >= 1.0
        ):
            force = self.latest_external_force
            norm = np.linalg.norm(force)

            self.get_logger().info(
                "Gravity-map compensated force in tool0 [N]: "
                f"{np.array2string(force, precision=4)}; "
                f"norm={norm:.4f} N"
            )

            self.last_report_time = now


def main():
    rclpy.init()
    node = GravityMapCompensation()

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
