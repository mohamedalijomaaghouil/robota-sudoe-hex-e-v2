# OnRobot HEX-E v2 — Continuation Plan

## Objective

Reproduce the validated UR internal force/torque sensor teaching workflow using
the external OnRobot HEX-E v2 sensor.

The final target is:

1. stable compensated external wrench data;
2. force-guided admittance teaching;
3. synchronized arm-and-gripper recording;
4. restoration of the recorded initial pose;
5. synchronized replay.

## Current starting point

The EtherCAT sensor can publish force/torque data through ROS 2.

The current authoritative raw topic is expected to be:

```text
/bus0/ft_sensor0/ft_sensor_readings/wrench

