# ROBOTA-SUDOE — UR5e Force-Guided Teaching and Replay

This repository documents the ROS 2 migration and experimental development
performed during an engineering internship on a Universal Robots UR5e mounted
on a Robotnik KAIROS platform.

## Project objectives

The project investigates force-guided robot teaching and trajectory replay for
a collaborative doll demolding and assembly application.

The primary objectives were:

1. Integrate the UR5e and SCHUNK gripper with ROS 2 Jazzy.
2. Implement force-guided admittance control.
3. Record manually guided demonstrations.
4. Replay recorded arm and gripper motions.
5. Integrate a OnRobot HEX-E v2 external force/torque sensor.
6. Develop safer preview, interlock, and execution mechanisms.

## Current status

### UR internal force/torque sensor

Validated experimentally:

- UR internal F/T data acquisition
- software force/torque zeroing
- translational admittance control
- joint-state demonstration recording
- initial-pose restoration
- trajectory replay through the scaled trajectory controller
- synchronized reconstruction and replay of SCHUNK gripper motion

A complete recorded demonstration of approximately 242 seconds was replayed
successfully with 2,423 arm trajectory points and 22 reconstructed gripper
commands.

### HEX-E v2 external sensor

Partially validated:

- EtherCAT communication established
- ROS 2 wrench publication established
- sensor calibration and readiness investigated
- admittance preview and safety architecture implemented

Remaining work includes robust frame transformation, gravity/payload
compensation, stable zeroing, and final closed-loop robot validation.

## Repository structure

- `ur_internal_ft/` — validated UR internal-sensor workflow
- `hex_e_v2/` — external HEX-E v2 integration and development status
- `docs/` — architecture, safety, procedures, results, and troubleshooting
- `data_manifests/` — metadata and checksums for experimental data
- `report_assets/` — material for the internship report, CV, and presentations
- `evidence/` — selected read-only validation outputs

## Safety notice

This software controls an industrial collaborative robot. It is research
software and is not a certified safety system.

Before commanding motion:

- inspect the robot and its workspace;
- keep the emergency stop accessible;
- verify the active controller;
- verify the robot program and safety state;
- use conservative speed settings during initial tests;
- never push the robot while the trajectory controller is holding position;
- stop execution immediately if unexpected motion occurs.

See `docs/SAFETY.md` before operating the robot.

## Software environment

- Ubuntu 24.04
- ROS 2 Jazzy
- Universal Robots ROS 2 Driver
- ros2_control
- UR5e
- SCHUNK gripper
- OnRobot HEX-E v2 F/T sensor

## Project status

The UR internal-sensor record-and-replay workflow is operational. The HEX-E v2
workflow remains an active development task.
