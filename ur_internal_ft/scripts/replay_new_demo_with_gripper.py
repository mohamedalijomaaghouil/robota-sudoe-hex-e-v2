#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
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
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def duration_message(seconds):
    whole = int(seconds)
    nanoseconds = int(round((seconds - whole) * 1e9))

    if nanoseconds >= 1_000_000_000:
        whole += 1
        nanoseconds -= 1_000_000_000

    return whole, nanoseconds


def load_recording(bag_path, gripper_step):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id="mcap"),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    joints = []
    gripper = []

    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()

        if topic == "/joint_states":
            message = deserialize_message(serialized, JointState)
            index = {
                name: position
                for position, name in enumerate(message.name)
            }

            if not all(name in index for name in JOINT_ORDER):
                continue

            values = [
                float(message.position[index[name]])
                for name in JOINT_ORDER
            ]

            if all(math.isfinite(value) for value in values):
                joints.append((timestamp / 1e9, values))

        elif topic == "/gripper/position":
            message = deserialize_message(serialized, Float64)

            if math.isfinite(message.data):
                gripper.append(
                    (timestamp / 1e9, float(message.data))
                )

    if not joints:
        raise RuntimeError("No valid /joint_states samples found")

    if not gripper:
        raise RuntimeError("No valid /gripper/position samples found")

    recording_start = joints[0][0]

    joint_samples = [
        (timestamp - recording_start, position)
        for timestamp, position in joints
    ]

    gripper_samples = [
        (timestamp - recording_start, position)
        for timestamp, position in gripper
        if timestamp >= recording_start
    ]

    # Preserve approximately 10 Hz arm points.
    selected_joints = [joint_samples[0]]
    last_selected_time = joint_samples[0][0]

    for sample in joint_samples[1:-1]:
        if sample[0] - last_selected_time >= 0.099:
            selected_joints.append(sample)
            last_selected_time = sample[0]

    if selected_joints[-1] != joint_samples[-1]:
        selected_joints.append(joint_samples[-1])

    # Suppress sensor noise and redundant gripper commands.
    selected_gripper = [gripper_samples[0]]
    last_selected_position = gripper_samples[0][1]

    for sample in gripper_samples[1:-1]:
        timestamp, position = sample

        if abs(position - last_selected_position) >= gripper_step:
            selected_gripper.append(sample)
            last_selected_position = position

    if selected_gripper[-1] != gripper_samples[-1]:
        selected_gripper.append(gripper_samples[-1])

    return (
        joint_samples,
        selected_joints,
        gripper_samples,
        selected_gripper,
    )


class SynchronizedReplay(Node):
    def __init__(self):
        super().__init__("synchronized_demo_replay")

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/"
            "follow_joint_trajectory",
        )

        self.gripper_publisher = self.create_publisher(
            Float64,
            "/gripper/command",
            10,
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--gripper-step", type=float, default=0.25)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    if args.speed <= 0.0:
        raise SystemExit("ERROR: --speed must be positive")

    if args.gripper_step <= 0.0:
        raise SystemExit("ERROR: --gripper-step must be positive")

    (
        raw_joints,
        selected_joints,
        raw_gripper,
        selected_gripper,
    ) = load_recording(args.bag, args.gripper_step)

    recorded_duration = raw_joints[-1][0]
    command_duration = recorded_duration / args.speed

    maximum_jump = 0.0
    for (_, previous), (_, current) in zip(
        selected_joints,
        selected_joints[1:],
    ):
        maximum_jump = max(
            maximum_jump,
            max(
                abs(a - b)
                for a, b in zip(previous, current)
            ),
        )

    print(f"Raw joint samples:       {len(raw_joints)}")
    print(f"Arm trajectory points:   {len(selected_joints)}")
    print(f"Raw gripper samples:     {len(raw_gripper)}")
    print(f"Gripper replay commands: {len(selected_gripper)}")
    print(f"Recorded duration:       {recorded_duration:.3f} s")
    print(f"Requested replay speed:  {args.speed:.3f}x")
    print(f"Command duration:        {command_duration:.3f} s")
    print(f"Maximum arm jump:        {maximum_jump:.6f} rad")
    print(
        "Initial gripper:         "
        f"{raw_gripper[0][1]:.3f} mm"
    )
    print(
        "Minimum gripper:         "
        f"{min(value for _, value in raw_gripper):.3f} mm"
    )
    print(
        "Maximum gripper:         "
        f"{max(value for _, value in raw_gripper):.3f} mm"
    )
    print(
        "Final gripper:           "
        f"{raw_gripper[-1][1]:.3f} mm"
    )

    if maximum_jump > 0.02:
        raise SystemExit(
            "ABORT: an unexpectedly large arm jump was found"
        )

    if not args.execute:
        print("DRY RUN ONLY: no arm or gripper command was sent.")
        return

    rclpy.init()
    node = SynchronizedReplay()

    trajectory = JointTrajectory()
    trajectory.joint_names = JOINT_ORDER

    # Give the controller one second before the first movement.
    start_delay = 1.0

    for timestamp, position in selected_joints:
        point = JointTrajectoryPoint()
        point.positions = position

        scheduled_time = start_delay + timestamp / args.speed
        sec, nanosec = duration_message(scheduled_time)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec

        trajectory.points.append(point)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory

    print("Waiting for trajectory action server...")

    if not node.action_client.wait_for_server(timeout_sec=10.0):
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit("ABORT: trajectory action server unavailable")

    discovery_deadline = time.monotonic() + 5.0
    while (
        node.gripper_publisher.get_subscription_count() < 1
        and time.monotonic() < discovery_deadline
    ):
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.gripper_publisher.get_subscription_count() < 1:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit("ABORT: gripper command subscriber unavailable")

    print(
        f"Sending {len(trajectory.points)} arm points and "
        f"{len(selected_gripper)} gripper commands..."
    )

    send_future = node.action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)

    goal_handle = send_future.result()

    if goal_handle is None or not goal_handle.accepted:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit("ABORT: trajectory goal was rejected")

    print("Trajectory accepted.")
    print("Synchronized arm-and-gripper replay started.")
    print("Expected duration: approximately "
          f"{command_duration:.1f} seconds.")

    result_future = goal_handle.get_result_async()
    replay_start = time.monotonic()
    gripper_index = 0

    try:
        while not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.01)
            elapsed = time.monotonic() - replay_start

            while gripper_index < len(selected_gripper):
                event_time, position = selected_gripper[gripper_index]
                scheduled_time = start_delay + event_time / args.speed

                if scheduled_time > elapsed:
                    break

                command = Float64()
                command.data = position
                node.gripper_publisher.publish(command)

                print(
                    f"Gripper command: {position:7.3f} mm "
                    f"at replay t={elapsed:8.3f} s"
                )

                gripper_index += 1

        wrapped_result = result_future.result()
        status = wrapped_result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            print("SUCCESS: synchronized replay completed.")
        else:
            print(f"Replay ended with action status: {status}")
            raise SystemExit(1)

    except KeyboardInterrupt:
        print("\nInterrupt received. Cancelling trajectory goal...")

        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(
            node,
            cancel_future,
            timeout_sec=5.0,
        )

        print("Trajectory cancellation requested.")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(130)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
