# Experimentally Validated Results

## UR5e internal force/torque sensor

Status: **operational and experimentally validated**

The UR5e internal force/torque sensor was used successfully for force-guided
teaching through a custom ROS 2 admittance controller.

Validated capabilities:

- acquisition of the internal wrench on `/ft_sensor_wrench`;
- software zeroing and publication on `/force_sensor_ur_zeroed`;
- translational force-guided teaching;
- joint-velocity control through `forward_velocity_controller`;
- recording of joint states, wrench data, velocity commands, and gripper data;
- restoration of the robot to the first recorded joint position;
- arm trajectory replay through `scaled_joint_trajectory_controller`;
- synchronized replay of the arm and SCHUNK gripper;
- recording of replay feedback for later accuracy analysis.

### Replay accuracy

Synchronized arm + gripper replay of the 242 s demonstration
(bag `synchronized_replay_20260907_175919`, manifest in
`ur_internal_ft/data_manifests/demo_a_synchronized_replay_evaluation_bag_info.txt`):

| Metric | Value |
|--------|-------|
| RMS joint tracking error, all joints | **0.367 mrad** |
| Maximum absolute joint tracking error | 6.455 mrad |
| Final gripper position error | **0.06 mm** (65.03 mm commanded, 64.97 mm reached) |

Computed with `ur_internal_ft/scripts/analyze_replay_tracking_error.py` from the
`error` field of `/scaled_joint_trajectory_controller/controller_state`. Full output:
`ur_internal_ft/results/synchronized_replay_tracking_error.txt`.

## Validated legacy controller

The successful controller source has SHA-256:

```text
9b4436350361804dc8ee34b811a7c1a9f8e9db2ff36d4c56226ba955fdd02536

