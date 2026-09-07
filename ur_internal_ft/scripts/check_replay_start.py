#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

order = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

target = [
    -0.14146787325014287,
    -1.7301913700499476,
    1.3671920935260218,
    -1.2560893160155793,
    -1.4974034468280237,
    -1.5992963949786585,
]


class Check(Node):
    def __init__(self):
        super().__init__("check_replay_start")
        self.create_subscription(
            JointState,
            "/joint_states",
            self.callback,
            10,
        )

    def callback(self, message):
        index = {name: i for i, name in enumerate(message.name)}

        if not all(name in index for name in order):
            return

        actual = [
            message.position[index[name]]
            for name in order
        ]

        errors = [
            abs(actual_value - target_value)
            for actual_value, target_value in zip(actual, target)
        ]

        print("Actual position:")
        print(actual)
        print("Absolute errors:")
        print(errors)
        print(f"Maximum error: {max(errors):.6f} rad")

        if max(errors) <= 0.02:
            print("PASS: robot is at the recording start position.")
        else:
            print("FAIL: initial-position error is too large.")

        raise SystemExit


rclpy.init()
node = Check()

try:
    rclpy.spin(node)
except SystemExit:
    pass

node.destroy_node()
rclpy.shutdown()
