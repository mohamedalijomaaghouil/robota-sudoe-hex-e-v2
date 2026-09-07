# HEX-E v2 External Force/Torque Sensor — Project Status

## Objective

The objective is to reproduce the demonstrated UR internal-sensor workflow
using the external HEX-E v2 force/torque sensor:

1. acquire stable six-axis wrench data;
2. zero or compensate the sensor correctly;
3. transform the wrench into the required robot frame;
4. drive the admittance controller;
5. guide the UR5e safely;
6. record a new arm-and-gripper demonstration;
7. replay the demonstration accurately.

## Current status

Status: **sensor communication achieved; complete physical admittance
demonstration not yet validated**

Work completed or partially completed includes:

- EtherCAT communication with the external sensor;
- ROS 2 driver adaptation;
- publication of force/torque data;
- zeroing and calibration experiments;
- inspection of sensor orientation and frame transformations;
- construction of a safer preview and execution architecture;
- controller-state interlocks;
- zero-command and inactive-controller validation;
- execution-bridge tests while the velocity controller remained inactive.

## Current evidence

The experimental safe architecture is preserved in:

```text
ur_internal_ft/experimental_safe_architecture/

