#!/usr/bin/env python3
"""Replay accuracy of a synchronized arm + gripper replay bag (MCAP).

Joint tracking error comes from the error field of
/scaled_joint_trajectory_controller/controller_state. The gripper error is the
difference between the last /gripper/command and the last /gripper/position.

Usage: python3 analyze_replay_tracking_error.py <bag_0.mcap>
Requires: pip install mcap mcap-ros2-support numpy
"""
import sys

import numpy as np
from mcap_ros2.reader import read_ros2_messages

bag = sys.argv[1]
errors, stamps, joint_names = [], [], []
gripper_commands, gripper_positions = [], []

for m in read_ros2_messages(bag, topics=[
        "/scaled_joint_trajectory_controller/controller_state",
        "/gripper/command",
        "/gripper/position"]):
    topic = m.channel.topic
    if topic.endswith("controller_state"):
        if len(m.ros_msg.error.positions):
            errors.append(list(m.ros_msg.error.positions))
            stamps.append(m.log_time_ns * 1e-9)
            joint_names = list(m.ros_msg.joint_names)
    elif topic == "/gripper/command":
        gripper_commands.append(m.ros_msg.data)
    else:
        gripper_positions.append(m.ros_msg.data)

E = np.array(errors)
print("Bag:", bag)
print("Joints:", ", ".join(joint_names))
print("Controller state samples:", len(E))
print("Duration: %.1f s" % (stamps[-1] - stamps[0]))
print("RMS tracking error per joint (mrad):",
      " ".join("%.3f" % v for v in np.sqrt((E ** 2).mean(axis=0)) * 1e3))
print("RMS tracking error, all joints: %.3f mrad" % (np.sqrt((E ** 2).mean()) * 1e3))
print("Maximum absolute tracking error: %.3f mrad" % (np.abs(E).max() * 1e3))
if gripper_commands and gripper_positions:
    print("Gripper commands: %d" % len(gripper_commands))
    print("Final gripper command: %.2f mm, final position: %.2f mm, error: %.2f mm"
          % (gripper_commands[-1], gripper_positions[-1],
             abs(gripper_commands[-1] - gripper_positions[-1])))
