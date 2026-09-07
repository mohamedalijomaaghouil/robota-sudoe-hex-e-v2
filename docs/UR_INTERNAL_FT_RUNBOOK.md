# UR5e Internal F/T Sensor — Demonstration Recording and Replay Runbook

## 1. Purpose

This document describes the experimentally validated procedure for:

1. starting the UR5e ROS 2 driver;
2. starting the SCHUNK gripper interface;
3. zeroing the UR5e internal force/torque signal;
4. running the legacy admittance controller;
5. recording a new arm-and-gripper demonstration;
6. safely stopping admittance control;
7. returning the robot to the recording's initial configuration;
8. replaying the recorded arm and gripper motion synchronously.

This workflow was validated on September 7, 2026.

---

## 2. Important safety information

This procedure can generate real robot motion.

Before enabling motion:

- keep the emergency stop accessible;
- keep people outside the robot workspace;
- remove unnecessary objects from the workspace;
- confirm that the mounted tool and payload configuration are correct;
- start with a reduced speed slider;
- verify the controller states before every controller switch;
- never activate two controllers that claim the same joint command interfaces;
- stop immediately if unexpected motion, oscillation, or a protective stop occurs.

The legacy admittance controller directly publishes joint-velocity commands and does not contain the newer execution interlocks.

---

## 3. Validated software versions

### GitHub reference

Repository:

```text
https://github.com/mohamedalijomaaghouil/robota-sudoe-ur5e-ros2
```

Validated commit:

```text
3ebdcbf
```

Commit description:

```text
Final demo: pick-and-place with admittance, replay, and automated gripper.
Added critical startup order and working configuration.
```

### Legacy admittance-controller identity

Validated SHA-256:

```text
9b4436350361804dc8ee34b811a7c1a9f8e9db2ff36d4c56226ba955fdd02536
```

Reference source:

```text
~/reference_repos/robota-sudoe-ur5e-ros2/admittance_control_ros2.cpp
```

Isolated workspace:

```text
~/ur_internal_sensor_demo_ws
```

Installed executable:

```text
~/ur_internal_sensor_demo_ws/install/admittance_control_ros2/lib/admittance_control_ros2/admittance_control_node
```

Verify the source before rebuilding:

```bash
sha256sum \
  ~/ur_internal_sensor_demo_ws/src/admittance_control_ros2/src/admittance_control_ros2.cpp
```

Expected checksum:

```text
9b4436350361804dc8ee34b811a7c1a9f8e9db2ff36d4c56226ba955fdd02536
```

---

## 4. ROS 2 interfaces

### Sensor topics

Raw UR internal F/T sensor:

```text
/ft_sensor_wrench
```

Software-zeroed UR F/T sensor:

```text
/force_sensor_ur_zeroed
```

### Robot state

```text
/joint_states
```

### Admittance output

```text
/forward_velocity_controller/commands
```

Type:

```text
std_msgs/msg/Float64MultiArray
```

### Arm replay interface

```text
/scaled_joint_trajectory_controller/follow_joint_trajectory
```

### Gripper interfaces

Command:

```text
/gripper/command
```

Position feedback:

```text
/gripper/position
```

The gripper command and position topics use:

```text
std_msgs/msg/Float64
```

---

## 5. Controller configurations

### Safe position-control configuration

Before starting or after stopping admittance:

```text
scaled_joint_trajectory_controller    active
forward_velocity_controller           inactive
```

### Live admittance configuration

During force-guided teaching:

```text
scaled_joint_trajectory_controller    inactive
forward_velocity_controller           active
```

Check the states with:

```bash
ros2 control list_controllers | grep -E \
  'scaled_joint_trajectory_controller|forward_velocity_controller'
```

---

## 6. Terminal A — Start the UR5e driver

On the teach pendant:

1. power on the robot;
2. release the emergency stop;
3. enable the robot;
4. select Remote Control;
5. verify that the safety state is Normal;
6. verify the configured payload and center of gravity.

On the lab computer:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kairos_ws/install/setup.bash

ros2 launch \
  cartesian_controllers_universal_robots \
  robot_ur5e.launch.py \
  robot_ip:=192.168.0.210
```

Leave this terminal running.

Expected result:

- the controller manager becomes available;
- `/joint_states` is published;
- `/ft_sensor_wrench` is published;
- `scaled_joint_trajectory_controller` is active;
- `forward_velocity_controller` is inactive.

---

## 7. Terminal B — Start External Control

Run:

```bash
python3 ~/send_dashboard.py \
  "load ext_control.urp" \
  "play"
```

In the driver terminal, wait for a message equivalent to:

```text
Robot connected to reverse interface. Ready to receive control commands.
```

Verify:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kairos_ws/install/setup.bash 2>/dev/null || true

timeout 3s ros2 topic echo \
  /io_and_status_controller/robot_program_running --once
```

Expected result:

```yaml
data: true
```

---

## 8. Set the robot speed slider

For the first test, use 30%:

```bash
ros2 service call \
  /io_and_status_controller/set_speed_slider \
  ur_msgs/srv/SetSpeedSliderFraction \
  "{speed_slider_fraction: 0.30}"
```

Verify:

```bash
timeout 3s ros2 topic echo \
  /speed_scaling_state_broadcaster/speed_scaling --once
```

Expected result:

```yaml
data: 30.0
```

After the workflow has been verified and the workspace is clear, the slider may be increased deliberately.

Example for 100%:

```bash
ros2 service call \
  /io_and_status_controller/set_speed_slider \
  ur_msgs/srv/SetSpeedSliderFraction \
  "{speed_slider_fraction: 1.0}"
```

The speed slider does not replace the safety checks or controller-side velocity limits.

---

## 9. Terminal C — Start the SCHUNK gripper node

Run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kairos_ws/install/setup.bash 2>/dev/null || true

python3 \
  ~/reference_repos/robota-sudoe-ur5e-ros2/schunk_gripper_node.py
```

Leave this terminal running.

Expected messages:

```text
SCHUNK gripper node started
Connecting to 192.168.0.210:55050
Gripper ready
```

Verify from another terminal:

```bash
ros2 node list | grep -i gripper

timeout 3s ros2 topic echo \
  /gripper/position --once

ros2 topic info /gripper/command
```

Expected results:

- `/schunk_gripper_node` exists;
- `/gripper/position` contains a value in millimetres;
- `/gripper/command` has one subscriber.

---

## 10. Terminal D — Zero the internal F/T sensor

The complete tool must remain stationary during zeroing.

Run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kairos_ws/install/setup.bash 2>/dev/null || true

python3 \
  ~/reference_repos/robota-sudoe-ur5e-ros2/ur_force_zeroed.py
```

Leave this process running.

Expected message:

```text
Waiting for zero offset (keep robot still)...
```

followed by:

```text
Zero computed: F=[...] N
```

Verify:

```bash
timeout 3s ros2 topic echo \
  /force_sensor_ur_zeroed --once

timeout 4s ros2 topic hz \
  /force_sensor_ur_zeroed
```

Expected result:

- a valid `geometry_msgs/msg/WrenchStamped` message;
- approximately 100 Hz;
- small forces at rest immediately after zeroing.

If the tool, payload, or robot configuration changes, repeat the zeroing step.

---

## 11. Terminal E — Start the validated legacy admittance node

First verify that the position controller remains active:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur_internal_sensor_demo_ws/install/setup.bash

ros2 control list_controllers | grep -E \
  'scaled_joint_trajectory_controller|forward_velocity_controller'
```

Expected state:

```text
scaled_joint_trajectory_controller    active
forward_velocity_controller           inactive
```

Start the exact legacy controller:

```bash
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

Expected message:

```text
Admittance ready. sensor=UR freq=125 Hz
```

Important:

- the validated legacy source does not declare `max_joint_vel`;
- it does not declare `debug_forces`;
- adding those command-line parameters does not add their functionality;
- the node itself does not switch controllers.

Do not push the robot while the velocity controller is inactive.

---

## 12. Terminal F — Start recording a new demonstration

Create a new bag path:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kairos_ws/install/setup.bash 2>/dev/null || true

mkdir -p ~/ur_demo_bags

BAG=~/ur_demo_bags/ur_internal_demo_$(date +%Y%m%d_%H%M%S)

echo "$BAG" | tee ~/latest_ur_demo_bag.txt
```

Start recording:

```bash
ros2 bag record \
  -o "$BAG" \
  --topics \
  /joint_states \
  /gripper/command \
  /gripper/position \
  /force_sensor_ur_zeroed \
  /forward_velocity_controller/commands
```

Leave this terminal recording.

The `/gripper/command` topic is important because it preserves the intended gripper events directly.

---

## 13. Activate admittance control

Verify:

- External Control is playing;
- the workspace is clear;
- the zeroing node is running;
- the legacy admittance node is running;
- rosbag recording is running;
- the gripper node is running;
- the emergency stop is accessible.

Switch controllers:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kairos_ws/install/setup.bash 2>/dev/null || true

ros2 control switch_controllers \
  --deactivate scaled_joint_trajectory_controller \
  --activate forward_velocity_controller \
  --strict
```

Expected result:

```text
Successfully switched controllers!
```

Verify:

```bash
ros2 control list_controllers | grep -E \
  'scaled_joint_trajectory_controller|forward_velocity_controller'
```

Expected state:

```text
scaled_joint_trajectory_controller    inactive
forward_velocity_controller           active
```

The robot is now in the live force-guided teaching state.

Guide the arm and operate the gripper as required.

Avoid abrupt lateral forces. Lateral interaction previously caused a protective stop, whereas vertical interaction was more stable.

---

## 14. Protective-stop recovery

If a protective stop occurs:

1. stop touching the robot;
2. inspect the cause of the stop;
3. use PolyScope to recover and enable the robot;
4. reload and play `ext_control.urp`;
5. wait for the reverse-interface connection message;
6. verify the controller states;
7. resume only if the robot state is understood and safe.

Dashboard command:

```bash
python3 ~/send_dashboard.py \
  "load ext_control.urp" \
  "play"
```

A recording can continue across a short interruption, but the interruption must be documented because it may affect replay timing and trajectory continuity.

---

## 15. Stop the recording and restore the safe state

First restore the position controller:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kairos_ws/install/setup.bash 2>/dev/null || true

ros2 control switch_controllers \
  --deactivate forward_velocity_controller \
  --activate scaled_joint_trajectory_controller \
  --strict
```

Expected result:

```text
Successfully switched controllers!
```

Then stop the legacy admittance node with `Ctrl+C`.

Stop rosbag recording with `Ctrl+C`.

The zeroing node may also be stopped with `Ctrl+C` when it is no longer needed.

Verify:

```bash
ros2 control list_controllers | grep -E \
  'scaled_joint_trajectory_controller|forward_velocity_controller'

ros2 topic info \
  /forward_velocity_controller/commands
```

Expected state:

```text
scaled_joint_trajectory_controller    active
forward_velocity_controller           inactive
Publisher count: 0
```

---

## 16. Inspect the new bag before replay

Run:

```bash
BAG=$(cat ~/latest_ur_demo_bag.txt)

ros2 bag info "$BAG"
```

Confirm that the bag contains:

```text
/joint_states
/gripper/command
/gripper/position
/force_sensor_ur_zeroed
/forward_velocity_controller/commands
```

Check the replay without sending motion:

```bash
python3 ~/replay_new_demo_with_gripper.py \
  "$BAG" \
  --speed 1.0
```

Expected result:

```text
DRY RUN ONLY: no arm or gripper command was sent.
```

The dry run prints:

- number of joint samples;
- number of trajectory points;
- number of gripper samples and commands;
- recorded duration;
- initial arm configuration;
- final arm configuration;
- maximum consecutive trajectory jump.

Do not replay if the values are unexpected.

---

## 17. Important replay warning

Never use this command on the demonstration bag:

```bash
ros2 bag play "$BAG"
```

The bag contains:

```text
/forward_velocity_controller/commands
```

Playing the bag directly could republish recorded velocity commands.

Use only the dedicated trajectory replay script.

---

## 18. Move to the recorded initial pose

The robot must be moved to the first joint configuration of the recording before replay.

The dry run prints the required initial position in this order:

```text
shoulder_pan_joint
shoulder_lift_joint
elbow_joint
wrist_1_joint
wrist_2_joint
wrist_3_joint
```

Send the initial pose through the trajectory controller using a deliberately slow trajectory.

After movement, compare the current pose to the recorded initial position.

For the September 7 validated recording, the initial position was:

```text
shoulder_pan_joint  = -0.141467873250
shoulder_lift_joint = -1.730191370050
elbow_joint         =  1.367192093526
wrist_1_joint       = -1.256089316016
wrist_2_joint       = -1.497403446828
wrist_3_joint       = -1.599296394979
```

These lines are documentation, not shell commands.

Do not paste them directly into Bash.

The validated position check reported a maximum initial-position error of approximately:

```text
0.000053 rad
```

---

## 19. Set the initial gripper position

Before replay, command the gripper to the initial position reported by the dry run.

For the September 7 recording:

```text
Initial gripper position: 36.110 mm
```

Example:

```bash
ros2 topic pub --once \
  /gripper/command \
  std_msgs/msg/Float64 \
  "{data: 36.11}"
```

Verify:

```bash
timeout 3s ros2 topic echo \
  /gripper/position --once
```

---

## 20. Synchronized arm-and-gripper replay

Preconditions:

```text
scaled_joint_trajectory_controller    active
forward_velocity_controller           inactive
```

The SCHUNK gripper node must be running.

Execute:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kairos_ws/install/setup.bash 2>/dev/null || true

BAG=$(cat ~/latest_ur_demo_bag.txt)

python3 ~/replay_new_demo_with_gripper.py \
  "$BAG" \
  --execute \
  --speed 1.0
```

Expected messages:

```text
Waiting for trajectory action server...
Trajectory accepted.
Synchronized arm-and-gripper replay started.
```

At completion:

```text
SUCCESS: synchronized replay completed.
```

Keep clear of the robot throughout the complete recorded duration.

Pressing `Ctrl+C` should cancel the active trajectory goal.

---

## 21. Optional replay evaluation recording

Before starting replay, record the executed state in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/kairos_ws/install/setup.bash 2>/dev/null || true

mkdir -p ~/ur_demo_bags/replay_tests

EVAL_BAG=~/ur_demo_bags/replay_tests/synchronized_replay_$(date +%Y%m%d_%H%M%S)

echo "$EVAL_BAG" | tee \
  ~/latest_synchronized_replay_evaluation_bag.txt

ros2 bag record \
  -o "$EVAL_BAG" \
  --topics \
  /joint_states \
  /scaled_joint_trajectory_controller/controller_state \
  /gripper/command \
  /gripper/position
```

Stop this recorder after replay completes.

This evaluation bag can be used to compare:

- desired and actual joint positions;
- trajectory tracking error;
- gripper command timing;
- actual gripper position;
- final arm and gripper state.

---

## 22. Validated September 7 result

Demonstration bag:

```text
~/ur_demo_bags/new_demo_20260907_163432
```

Recorded duration:

```text
242.110 seconds
```

Joint samples:

```text
24,212
```

Maximum consecutive recorded joint delta:

```text
0.001381 rad
```

Synchronized replay:

```text
2,423 arm trajectory points
22 reconstructed gripper commands
speed = 1.0
```

Result:

```text
SUCCESS: synchronized replay completed.
```

Final gripper command:

```text
65.030 mm
```

Measured final gripper position:

```text
64.970 mm
```

Final controller state:

```text
scaled_joint_trajectory_controller    active
forward_velocity_controller           inactive
```

Final joint velocities were zero.

---

## 23. Known limitations

### Legacy admittance controller

The validated legacy controller:

- directly publishes velocity commands;
- has no deadman switch;
- has no stale-input timeout;
- has no execution interlock;
- does not switch controllers itself;
- does not implement `max_joint_vel`;
- does not implement `debug_forces`;
- has no explicit command saturation in the preserved source.

It should therefore be retained for reproducibility but not considered the final production-safe architecture.

### F/T zeroing

The software-zeroing process removes the initial offset but does not provide complete dynamic payload and gravity compensation for every robot orientation.

Changing the object, handle, payload, or orientation can change the measured wrench and admittance response.

### Protective stop

A protective stop occurred during lateral pushing in one recording session. Vertical force interaction was more stable. The root cause requires further investigation.

### Gripper recording

The September 7 demonstration bag recorded `/gripper/position`, but not `/gripper/command`.

The synchronized replay script reconstructed 22 gripper commands from the measured position trajectory.

Future demonstrations must record both:

```text
/gripper/command
/gripper/position
```

---

## 24. Final shutdown checklist

After any recording or replay:

```bash
ros2 control list_controllers | grep -E \
  'scaled_joint_trajectory_controller|forward_velocity_controller'
```

Expected:

```text
scaled_joint_trajectory_controller    active
forward_velocity_controller           inactive
```

Check velocity-command publishers:

```bash
ros2 topic info \
  /forward_velocity_controller/commands
```

Expected:

```text
Publisher count: 0
```

Check the robot is stationary:

```bash
for i in 1 2 3; do
  timeout 3s ros2 topic echo /joint_states --once |
    sed -n '/velocity:/,/effort:/p'
  sleep 1
done
```

Expected velocities should be zero or effectively zero.

Stop all temporary recording, admittance, replay, and zeroing processes before leaving the laboratory.

---

## 25. Summary

The UR5e internal F/T sensor workflow is operational and repeatable.

The validated system can:

- zero the UR internal F/T signal;
- provide force-guided arm teaching through admittance control;
- record arm, force, and gripper data;
- restore the robot to the recording's initial joint configuration;
- replay the arm trajectory;
- reconstruct and replay gripper motion;
- evaluate replay accuracy from recorded controller feedback.

The next development objective is to reproduce the same demonstration-recording and replay capability using the OnRobot HEX-E v2 external force/torque sensor.

