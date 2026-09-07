#!/usr/bin/env python3

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.time import Time

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener


WRENCH_TOPIC = "/bus0/ft_sensor0/ft_sensor_readings/wrench"
ACTION_NAME = (
    "/scaled_joint_trajectory_controller/follow_joint_trajectory"
)

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

REFERENCE = np.array([
    0.0,
    -math.pi / 2.0,
    math.pi / 2.0,
    -math.pi / 2.0,
    -math.pi / 2.0,
    -math.pi / 2.0,
])

# Ordered to avoid unnecessary large jumps.
POSES_DEG = [
    ("reference_start", -90.0, -90.0),
    ("w1_m75_w2_m90",  -75.0, -90.0),
    ("w1_m75_w2_m75",  -75.0, -75.0),
    ("w1_m90_w2_m75",  -90.0, -75.0),
    ("w1_m105_w2_m75", -105.0, -75.0),
    ("w1_m105_w2_m90", -105.0, -90.0),
    ("w1_m105_w2_m105", -105.0, -105.0),
    ("w1_m90_w2_m105",  -90.0, -105.0),
    ("w1_m75_w2_m105",  -75.0, -105.0),
    ("reference_return", -90.0, -90.0),
]


def quaternion_matrix(x, y, z, w):
    q = np.array([x, y, z, w], dtype=float)
    norm = np.linalg.norm(q)
    if norm == 0.0:
        raise RuntimeError("Zero-length TF quaternion")
    x, y, z, w = q / norm

    return np.array([
        [
            1.0 - 2.0 * (y*y + z*z),
            2.0 * (x*y - z*w),
            2.0 * (x*z + y*w),
        ],
        [
            2.0 * (x*y + z*w),
            1.0 - 2.0 * (x*x + z*z),
            2.0 * (y*z - x*w),
        ],
        [
            2.0 * (x*z - y*w),
            2.0 * (y*z + x*w),
            1.0 - 2.0 * (x*x + y*y),
        ],
    ])


class Collector:
    def __init__(self):
        self.node = rclpy.create_node("hex_e_orientation_collector")

        self.samples = []
        self.collecting = False
        self.latest_joint = None

        self.wrench_sub = self.node.create_subscription(
            WrenchStamped,
            WRENCH_TOPIC,
            self.wrench_callback,
            50,
        )

        self.joint_sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            20,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self.node,
            spin_thread=False,
        )

        self.action = ActionClient(
            self.node,
            FollowJointTrajectory,
            ACTION_NAME,
        )

    def wrench_callback(self, msg):
        if self.collecting:
            self.samples.append([
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z,
            ])

    def joint_callback(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(name in values for name in JOINTS):
            self.latest_joint = np.array(
                [values[name] for name in JOINTS],
                dtype=float,
            )

    def spin_until_ready(self, timeout=15.0):
        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            tf_ready = self.tf_buffer.can_transform(
                "base",
                "tool0",
                Time(),
            )

            if self.latest_joint is not None and tf_ready:
                return

        raise RuntimeError(
            "Timed out waiting for /joint_states or base->tool0 TF"
        )

    def gravity_in_tool0(self):
        transform = self.tf_buffer.lookup_transform(
            "base",
            "tool0",
            Time(),
        )

        q = transform.transform.rotation
        rotation_base_from_tool = quaternion_matrix(
            q.x, q.y, q.z, q.w
        )

        gravity_base = np.array([0.0, 0.0, -9.80665])
        gravity_tool = rotation_base_from_tool.T @ gravity_base

        return gravity_tool, np.array([q.x, q.y, q.z, q.w])

    def move(self, target):
        if not self.action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Trajectory action server unavailable")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        point = JointTrajectoryPoint()
        point.positions = target.tolist()
        point.time_from_start.sec = 8
        goal.trajectory.points = [point]

        send_future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self.node,
            send_future,
            timeout_sec=15.0,
        )

        if not send_future.done():
            raise RuntimeError("Timed out sending trajectory goal")

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Trajectory goal was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node,
            result_future,
            timeout_sec=60.0,
        )

        if not result_future.done():
            raise RuntimeError("Trajectory result timed out")

        wrapped = result_future.result()

        if wrapped is None:
            raise RuntimeError("Trajectory returned no result")

        result = wrapped.result

        if result.error_code != 0:
            raise RuntimeError(
                f"Trajectory failed: code={result.error_code}, "
                f"message={result.error_string!r}"
            )

    def capture(self, number=100, timeout=20.0):
        self.samples = []
        self.collecting = True
        deadline = time.monotonic() + timeout

        while (
            rclpy.ok()
            and len(self.samples) < number
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self.node, timeout_sec=0.2)

        self.collecting = False

        if len(self.samples) < number:
            raise RuntimeError(
                f"Received only {len(self.samples)} wrench samples"
            )

        data = np.asarray(self.samples[:number], dtype=float)
        return data.mean(axis=0), data.std(axis=0), len(data)


def write_rows(path, rows):
    fields = [
        "pose",
        "wrist1_deg",
        "wrist2_deg",
        "sample_count",
        *[f"target_{name}" for name in JOINTS],
        *[f"actual_{name}" for name in JOINTS],
        "gravity_tool_x",
        "gravity_tool_y",
        "gravity_tool_z",
        "Fx_mean", "Fy_mean", "Fz_mean",
        "Tx_mean", "Ty_mean", "Tz_mean",
        "Fx_std", "Fy_std", "Fz_std",
        "Tx_std", "Ty_std", "Tz_std",
        "tf_qx", "tf_qy", "tf_qz", "tf_qw",
    ]

    with path.open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path.home() / "hex_e_orientation_data.csv",
    )
    args = parser.parse_args()

    rclpy.init()
    collector = Collector()

    try:
        collector.spin_until_ready()

        current = collector.latest_joint.copy()
        error = np.max(np.abs(current - REFERENCE))

        print("Current joints:", current)
        print("Reference joints:", REFERENCE)
        print(f"Maximum reference error: {error:.6f} rad")

        gravity, quaternion = collector.gravity_in_tool0()
        print("Gravity in tool0:", gravity)
        print("base->tool0 quaternion:", quaternion)

        if error > math.radians(2.0):
            raise RuntimeError(
                "Robot is more than 2 degrees from the reference pose"
            )

        if not args.execute:
            print(
                "CHECK ONLY: inputs and reference pose are valid. "
                "No motion was commanded."
            )
            return

        print("Execution starts in 5 seconds...")
        time.sleep(5.0)

        rows = []

        for index, (label, wrist1_deg, wrist2_deg) in enumerate(
            POSES_DEG,
            start=1,
        ):
            target = REFERENCE.copy()
            target[3] = math.radians(wrist1_deg)
            target[4] = math.radians(wrist2_deg)

            print(
                f"\n[{index}/{len(POSES_DEG)}] {label}: "
                f"Wrist1={wrist1_deg:.1f} deg, "
                f"Wrist2={wrist2_deg:.1f} deg"
            )

            collector.move(target)
            print("  Trajectory succeeded; settling for 10 seconds...")
            time.sleep(10.0)

            # Refresh joint and TF messages before capture.
            for _ in range(10):
                rclpy.spin_once(collector.node, timeout_sec=0.1)

            mean, std, sample_count = collector.capture()
            actual = collector.latest_joint.copy()
            gravity, quaternion = collector.gravity_in_tool0()

            row = {
                "pose": label,
                "wrist1_deg": wrist1_deg,
                "wrist2_deg": wrist2_deg,
                "sample_count": sample_count,
            }

            for name, value in zip(JOINTS, target):
                row[f"target_{name}"] = value

            for name, value in zip(JOINTS, actual):
                row[f"actual_{name}"] = value

            for axis, value in zip("xyz", gravity):
                row[f"gravity_tool_{axis}"] = value

            for name, value in zip(
                ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"],
                mean,
            ):
                row[f"{name}_mean"] = value

            for name, value in zip(
                ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"],
                std,
            ):
                row[f"{name}_std"] = value

            for name, value in zip(
                ["tf_qx", "tf_qy", "tf_qz", "tf_qw"],
                quaternion,
            ):
                row[name] = value

            rows.append(row)
            write_rows(args.output, rows)

            print("  Mean wrench:", mean)
            print("  Std wrench: ", std)
            print("  Gravity:    ", gravity)
            print(f"  Saved: {args.output}")

        print("\nCollection completed successfully.")
        print(f"Final file: {args.output}")

    finally:
        collector.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
