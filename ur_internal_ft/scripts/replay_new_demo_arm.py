#!/usr/bin/env python3

import argparse
import math
import sys

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    StorageOptions,
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def duration_message(seconds):
    seconds = max(0.0, float(seconds))
    whole = int(seconds)
    nanoseconds = int(round((seconds - whole) * 1e9))

    if nanoseconds >= 1_000_000_000:
        whole += 1
        nanoseconds -= 1_000_000_000

    return Duration(sec=whole, nanosec=nanoseconds)


def load_trajectory(bag_path, sample_period):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id="mcap"),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    raw_samples = []

    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()

        if topic != "/joint_states":
            continue

        message = deserialize_message(serialized, JointState)
        index = {name: i for i, name in enumerate(message.name)}

        if not all(name in index for name in JOINT_ORDER):
            continue

        position = [
            float(message.position[index[name]])
            for name in JOINT_ORDER
        ]

        if all(math.isfinite(value) for value in position):
            raw_samples.append((timestamp / 1e9, position))

    if not raw_samples:
        raise RuntimeError("No valid /joint_states samples were found")

    start_time = raw_samples[0][0]
    selected = []
    next_sample_time = 0.0

    for timestamp, position in raw_samples:
        relative_time = timestamp - start_time

        if relative_time + 1e-9 >= next_sample_time:
            selected.append((relative_time, position))
            next_sample_time += sample_period

    last_relative = raw_samples[-1][0] - start_time

    if selected[-1][0] < last_relative - 1e-6:
        selected.append((last_relative, raw_samples[-1][1]))

    return raw_samples, selected


class ReplayNode(Node):
    def __init__(self):
        super().__init__("accurate_arm_replay")
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--sample-period", type=float, default=0.10)
    args = parser.parse_args()

    if args.speed <= 0.0:
        raise SystemExit("ERROR: --speed must be positive")

    raw_samples, selected = load_trajectory(
        args.bag,
        args.sample_period,
    )

    duration = selected[-1][0]
    maximum_delta = 0.0

    for previous, current in zip(raw_samples, raw_samples[1:]):
        delta = max(
            abs(a - b)
            for a, b in zip(previous[1], current[1])
        )
        maximum_delta = max(maximum_delta, delta)

    print(f"Raw joint samples: {len(raw_samples)}")
    print(f"Trajectory points: {len(selected)}")
    print(f"Recorded duration: {duration:.3f} s")
    print(f"Requested speed:   {args.speed:.3f}x")
    print(f"Command duration:  {duration / args.speed:.3f} s")
    print(f"Maximum jump:      {maximum_delta:.6f} rad")
    print("Initial position:")
    print("[" + ", ".join(f"{x:.12f}" for x in selected[0][1]) + "]")
    print("Final position:")
    print("[" + ", ".join(f"{x:.12f}" for x in selected[-1][1]) + "]")

    if not args.execute:
        print("DRY RUN ONLY: no trajectory was sent.")
        return

    rclpy.init()
    node = ReplayNode()

    print("Waiting for trajectory action server...")

    if not node.client.wait_for_server(timeout_sec=10.0):
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit("ERROR: trajectory action server unavailable")

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINT_ORDER

    # The robot is moved to the initial position separately.
    # A 1-second offset gives the controller time to accept the goal.
    for relative_time, position in selected:
        point = JointTrajectoryPoint()
        point.positions = position
        point.time_from_start = duration_message(
            1.0 + relative_time / args.speed
        )
        goal.trajectory.points.append(point)

    print(f"Sending {len(goal.trajectory.points)} trajectory points...")

    send_future = node.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)

    goal_handle = send_future.result()

    if goal_handle is None or not goal_handle.accepted:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit("ERROR: trajectory goal was rejected")

    print("Trajectory accepted.")
    print("Robot is replaying the recorded arm path.")
    print("Keep clear of the robot and keep the emergency stop accessible.")

    result_future = goal_handle.get_result_async()

    try:
        rclpy.spin_until_future_complete(node, result_future)
    except KeyboardInterrupt:
        print("\nInterrupt received. Cancelling trajectory goal...")
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(
            node,
            cancel_future,
            timeout_sec=5.0,
        )

        if cancel_future.done():
            response = cancel_future.result()
            print(
                "Cancellation response received; "
                f"goals_canceling={len(response.goals_canceling)}"
            )
        else:
            print(
                "WARNING: cancellation response timed out. "
                "Stop the robot from PolyScope."
            )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        return

    wrapped_result = result_future.result()

    if wrapped_result is None:
        print("ERROR: no trajectory result was received")
        return_code = 1
    else:
        print(f"Action status: {wrapped_result.status}")
        print(f"Controller error code: {wrapped_result.result.error_code}")
        print(f"Controller message: {wrapped_result.result.error_string}")

        return_code = (
            0 if wrapped_result.result.error_code == 0 else 1
        )

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(return_code)


if __name__ == "__main__":
    main()
