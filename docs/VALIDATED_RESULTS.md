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

## Validated legacy controller

The successful controller source has SHA-256:

```text
9b4436350361804dc8ee34b811a7c1a9f8e9db2ff36d4c56226ba955fdd02536

