# UR5e Internal F/T Demonstration — Quick Start

## Purpose

This checklist summarizes the experimentally validated procedure for recording
and replaying a force-guided UR5e demonstration using the UR5e internal
force/torque sensor and the SCHUNK gripper.

For detailed explanations, troubleshooting, safety requirements, and expected
outputs, read:

- `docs/UR_INTERNAL_FT_RUNBOOK.md`

## Validated controller

The validated legacy admittance-controller source is:

```text
ur_internal_ft/legacy_admittance_control_ros2/src/admittance_control_ros2.cpp
```

Expected SHA-256:

```text
9b4436350361804dc8ee34b811a7c1a9f8e9db2ff36d4c56226ba955fdd02536
```

Verify it with:

```bash
sha256sum \
  ur_internal_ft/legacy_admittance_control_ros2/src/admittance_control_ros2.cpp
```

## 1. Start the UR5e driver

In Terminal A:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kairos_ws/install/setup.bash

ros2 launch \
  cartesian_controllers_universal_robots \
  robot_ur5e.launch.py \
  robot_ip:=192.168.0.210
```

Leave this terminal running.

## 2. Load and play External Control

In Terminal B:

```bash
python3 ~/send_dashboard.py "load ext_control.urp"
python3 ~/send_dashboard.py "play"
```

Wait for the driver terminal to report:

```text
Robot connected to reverse interface. Ready to receive control commands.
```

Verify:

```bash
timeout 3s ros2 topic echo \
  /io_and_status_controller/robot_program_running --once
```

Expected:

```text
data: true
```

## 3. Start the SCHUNK gripper interface

In Terminal C:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kairos_ws/install/setup.bash

python3 \
  ~/reference_repos/robota-sudoe-ur5e-ros2/schunk_gripper_node.py
```

Expected:

```text
SCHUNK gripper node started
Gripper ready
```

Leave the terminal running.

## 4. Zero the UR internal force/torque sensor

Keep the complete tool assembly stationary and unloaded.

In Terminal D:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kairos_ws/install/setup.bash

python3 \
  ~/reference_repos/robota-sudoe-ur5e-ros2/ur_force_zeroed.py
```

Wait until the zero offset is reported.

Verify in another terminal:

```bash
timeout 3s ros2 topic echo /force_sensor_ur_zeroed --once
```

Leave the zeroing process running.

## 5. Start the validated legacy admittance node

In Terminal E:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur_internal_sensor_demo_ws/install/setup.bash

~/ur_internal_sensor_demo_ws/install/admittance_control_ros2/lib/admittance_control_ros2/admittance_control_node \
  --ros-args \
  -p sensor:=UR \
  -p M_trans:=15.0 \
  -p M_rot:=1.0 \
  -p c:=30.0 \
  -p F_alpha:=0.03 \
  -p V_alpha:=0.05 \
  -p dead_cart:=3.0 \
  -p dead_rot:=100.0 \
  -p freq:=125.0 \
  -r /ft_sensor_wrench:=/force_sensor_ur_zeroed
```

Expected:

```text
Admittance ready. sensor=UR freq=125 Hz
```

Do not guide the robot yet.

## 6. Start recording

In Terminal F:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kairos_ws/install/setup.bash

mkdir -p ~/ur_demo_bags
BAG=~/ur_demo_bags/new_demo_$(date +%Y%m%d_%H%M%S)
echo "$BAG" | tee ~/latest_ur_demo_bag.txt

ros2 bag record \
  -o "$BAG" \
  --topics \
  /joint_states \
  /force_sensor_ur_zeroed \
  /forward_velocity_controller/commands \
  /gripper/command \
  /gripper/action \
  /gripper/position
```

Wait until the available topics have been subscribed.

## 7. Activate admittance control

Verify that the robot program is running and that the workspace is clear.

Then execute:

```bash
ros2 control switch_controllers \
  --deactivate scaled_joint_trajectory_controller \
  --activate forward_velocity_controller \
  --strict
```

Expected:

```text
Successfully switched controllers!
```

Verify:

```bash
ros2 control list_controllers | grep -E \
  'scaled_joint_trajectory_controller|forward_velocity_controller'
```

Required live configuration:

```text
scaled_joint_trajectory_controller ... inactive
forward_velocity_controller        ... active
```

Guide the robot and operate the gripper as required.

## 8. Stop recording and restore position control

First, stop the bag recorder with `Ctrl+C`.

Then immediately restore the safe controller configuration:

```bash
ros2 control switch_controllers \
  --deactivate forward_velocity_controller \
  --activate scaled_joint_trajectory_controller \
  --strict
```

Stop the admittance node with `Ctrl+C`.

Verify:

```bash
ros2 control list_controllers | grep -E \
  'scaled_joint_trajectory_controller|forward_velocity_controller'

ros2 topic info /forward_velocity_controller/commands
```

Required state:

```text
scaled_joint_trajectory_controller ... active
forward_velocity_controller        ... inactive
Publisher count: 0
```

## 9. Inspect the recording

```bash
BAG=$(cat ~/latest_ur_demo_bag.txt)

ros2 bag info "$BAG"

python3 ~/replay_new_demo_with_gripper.py \
  "$BAG" \
  --speed 1.0
```

The second command is a dry run. It must finish with:

```text
DRY RUN ONLY: no arm or gripper command was sent.
```

## 10. Restore the initial pose

The robot must be moved to the first joint position of the recording before
replay.

Use the procedure in Section 18 of:

```text
docs/UR_INTERNAL_FT_RUNBOOK.md
```

Verify the initial pose with:

```bash
python3 \
  ~/robota_sudoe_final_delivery/ur_internal_ft/scripts/check_replay_start.py
```

Do not start replay unless the script reports:

```text
PASS: robot is at the recording start position.
```

Also set the gripper to the recording's initial position.

## 11. Replay the arm and gripper

Keep the workspace clear and the emergency stop accessible.

```bash
BAG=$(cat ~/latest_ur_demo_bag.txt)

python3 ~/replay_new_demo_with_gripper.py \
  "$BAG" \
  --execute \
  --speed 1.0
```

Expected final message:

```text
SUCCESS: synchronized replay completed.
```

Do not use `ros2 bag play` to command the robot.

## 12. Final safe-state verification

```bash
ros2 control list_controllers | grep -E \
  'scaled_joint_trajectory_controller|forward_velocity_controller'

timeout 3s ros2 topic echo /joint_states --once |
  sed -n '/velocity:/,/effort:/p'

ros2 topic info /forward_velocity_controller/commands
```

Required final conditions:

- `scaled_joint_trajectory_controller` is active;
- `forward_velocity_controller` is inactive;
- all arm joint velocities are zero;
- `/forward_velocity_controller/commands` has zero publishers.

## Protective-stop recovery

If a protective stop occurs:

1. stop physical interaction immediately;
2. cancel or stop the active motion process;
3. restore position control if ROS communication remains available;
4. inspect the robot and workspace;
5. use PolyScope to unlock or enable the robot;
6. reload and play `ext_control.urp`;
7. wait for the reverse-interface connection message;
8. verify robot and controller states before resuming.

Do not automatically continue a recording after a protective stop without
documenting the interruption.

