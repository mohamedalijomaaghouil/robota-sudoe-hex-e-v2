# OnRobot HEX-E v2 — Technical Findings

## 1. Physical sensor identity

The physical force/torque sensor used in this project is an OnRobot HEX-E v2.

References to Bota or Rokubimini in the source tree describe the ancestry of
the adapted EtherCAT software stack. They do not identify the physical sensor
manufacturer.

## 2. Integration objective

The objective is to reproduce the validated UR internal-sensor workflow using
the external OnRobot HEX-E v2 sensor:

1. acquire a stable six-axis wrench;
2. perform software zeroing;
3. compensate fixed offsets and tool loading;
4. transform the wrench consistently into the robot control frame;
5. drive the admittance controller;
6. record a new demonstration;
7. replay the recorded arm and gripper motion synchronously.

## 3. EtherCAT driver work

A Bota/Rokubimini EtherCAT driver was investigated and adapted to decode the
OnRobot sensor process data.

The modified driver was based on commit:

```text
a064c9be230d96539ff8733d16f2e4068875d649
```

Important modified source files and a Git patch are preserved under:

```text
hex_e_v2/src/
hex_e_v2/driver_patch/
hex_e_v2/reference/
```

The complete patch relative to the original driver repository is:

```text
hex_e_v2/driver_patch/onrobot_hex_e_v2_driver_changes.patch
```

## 4. Process-data layout identified during integration

The investigated transmit PDO has a total size of 56 bytes.

The interpreted layout is:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | Force X, REAL32 |
| 4 | 4 | Force Y, REAL32 |
| 8 | 4 | Force Z, REAL32 |
| 12 | 4 | Torque X, REAL32 |
| 16 | 4 | Torque Y, REAL32 |
| 20 | 4 | Torque Z, REAL32 |
| 24 | 24 | Six additional signed 32-bit fields |
| 48 | 2 | Divider, signed 16-bit |
| 50 | 2 | Sample counter, unsigned 16-bit |
| 52 | 4 | Status, unsigned 32-bit |

The receive PDO was treated as four bytes containing two unsigned 16-bit
control fields:

```text
FILTER
ZERO
```

The force and torque values decoded from the first six REAL32 fields were
interpreted in newtons and newton-metres.

This interpretation produced physically responsive force/torque data, but it
must still be considered an experimentally derived implementation rather than
an officially validated OnRobot driver.

## 5. ROS 2 sensor interface

The authoritative raw sensor wrench topic used during the investigation was:

```text
/bus0/ft_sensor0/ft_sensor_readings/wrench
```

The expected message type is:

```text
geometry_msgs/msg/WrenchStamped
```

Other topics such as `/force_sensor_eth` may exist only when a separate
processing or republishing node is running. They should not automatically be
treated as the authoritative raw sensor source.

A representative working acquisition rate during the present configuration
was approximately 10 Hz. This was associated with the configured driver time
step and is below the preferred rate for final admittance control.

## 6. Sensor frame and transform limitation

The wrench messages used the frame:

```text
ft_sensor0_wrench
```

A complete and experimentally validated TF connection between this frame and
the UR5e robot frames was not yet available.

Consequently, force directions observed in the sensor frame cannot yet be
assumed to correspond directly to:

```text
tool0
base_link
```

The estimated distance from the robot flange/tool reference to the sensor
centre was approximately 0.07 m, but this value remained provisional and
required confirmation from the mechanical assembly and CAD model.

The UR driver's standard internal force/torque frame must not be assumed to
represent the external OnRobot sensor.

## 7. Software zeroing and filtering

Software tare and filtering were investigated.

A low-pass filter coefficient of approximately:

```text
alpha = 0.20
```

was characterized.

A candidate per-axis translational deadband was also investigated:

```text
Fx: 0.90 N
Fy: 0.12 N
Fz: 0.85 N
```

These values reduced stationary noise in the tested pose. They are not a
substitute for valid gravity compensation and frame transformation.

No final hardware ZERO command through the receive PDO was adopted as the
validated operating procedure. The project used software-side offset
processing during the experiments.

## 8. Known-mass calibration experiments

### 8.1 625 g orientation experiment

Two runs using a known 625 g load were analyzed.

A candidate 3-by-3 force calibration matrix was estimated:

```text
[ 1.25553198  -0.03165323  -0.06377300 ]
[ 0.00073199   1.12968389  -0.04590148 ]
[-0.11608471   0.06960011   1.41995740 ]
```

This matrix improved consistency for the calibration dataset.

### 8.2 Independent 1051 g validation

The candidate 3-by-3 calibration was tested using an independent 1051 g load.

The reported force-vector error statistics were approximately:

```text
Mean error: 0.692225 N
RMS error:  0.728314 N
Maximum:    1.135015 N
```

This provided useful evidence that the raw sensor scaling and axis coupling
could be improved through calibration. However, the matrix was not deployed as
a final real-time admittance-control calibration.

### 8.3 Experimental 3-by-6 model

A more flexible 3-by-6 wrench model was also investigated.

Its independent validation produced approximately:

```text
Mean error: 0.478269 N
RMS error:  0.520995 N
Maximum:    0.959711 N
```

Although these independent-test values were promising, grouped
cross-validation exposed poor generalization:

```text
Grouped-validation maximum error: approximately 4.586 N
Cross-run maximum error:           approximately 5.031 N
```

The likely cause was insufficient variation in the calibration geometry and
strong correlation between force and moment components.

The 3-by-6 model was therefore not accepted for deployment.

## 9. Gravity-compensation investigation

An unloaded orientation-dependent force map was fitted.

The fitted matrix was approximately:

```text
[ 2.36141775  -0.09427079  -0.08677186 ]
[ 0.05265703   2.07757729  -0.08895589 ]
[-0.38057919  -0.15119476   2.25313453 ]
```

A later orientation test, including a wrist-1 rotation near -60 degrees,
produced a residual force of approximately:

```text
[-1.588, -0.074, 0.156] N
```

after application of the candidate compensation.

This result showed that the compensation was not orientation-invariant and was
not sufficiently reliable for closed-loop force-guided robot motion.

Possible causes include:

- missing or incorrect sensor-to-tool rotation;
- inaccurate payload centre-of-mass assumptions;
- unmodelled cable or mechanical loading;
- calibration data that did not sufficiently span the workspace;
- coupling between force and torque;
- insufficiently validated sign conventions.

## 10. Current validated status

The following achievements are supported by preserved evidence:

- the OnRobot HEX-E v2 was detected through EtherCAT;
- force/torque process data were decoded;
- a ROS 2 `WrenchStamped` topic was produced;
- known-mass and orientation datasets were collected;
- multiple calibration models were evaluated;
- filtering and deadband candidates were characterized;
- the driver modifications were preserved as source files and a Git patch;
- inactive-controller and zero-command software tests were performed during
  earlier development.

The following result has **not** yet been achieved:

> A validated, stable, closed-loop admittance demonstration using the external
> OnRobot HEX-E v2 sensor.

No documentation should claim that the HEX-E v2 admittance workflow is
complete until a real robot test has been performed with correct transforms,
compensation, filtering, command limits, and an operator deadman mechanism.

## 11. Main technical blockers

The remaining blockers are:

1. establish the exact sensor coordinate convention;
2. publish or configure the correct sensor-to-tool transform;
3. confirm the external sensor centre and payload centre of mass;
4. improve the acquisition rate beyond the current approximately 10 Hz;
5. implement reliable software tare;
6. implement gravity and payload compensation using robot orientation;
7. verify force and torque signs independently;
8. validate filtering and deadbands in several robot orientations;
9. introduce command saturation and stale-sensor protection;
10. require a deadman input before enabling robot-motion commands.

## 12. Recommended continuation sequence

The recommended continuation sequence is:

1. keep the forward velocity controller inactive;
2. start only the HEX-E v2 EtherCAT acquisition;
3. confirm the authoritative raw wrench topic and publication rate;
4. verify axis signs using small, manually applied forces;
5. determine the exact sensor-to-tool transform;
6. collect unloaded data in a structured set of robot orientations;
7. estimate and validate payload/gravity compensation;
8. validate the compensated wrench while the controller remains inactive;
9. run a zero-command admittance preview;
10. enable tightly limited motion only after all previous tests pass.

The first live motion test should use:

- a low robot speed slider;
- conservative joint-velocity limits;
- translation only;
- disabled rotational admittance;
- a physical or keyboard deadman input;
- an immediately available stop command;
- an operator clear of pinch and collision zones.

## 13. Preserved evidence

Calibration and compensation results are stored under:

```text
hex_e_v2/results/analysis/
hex_e_v2/results/datasets/
```

Driver provenance is stored under:

```text
hex_e_v2/reference/
```

The source patch and selected modified source files are stored under:

```text
hex_e_v2/driver_patch/
hex_e_v2/src/
```

These files preserve both successful findings and unsuccessful calibration
attempts so that future students can continue from experimentally supported
results rather than repeating the entire investigation.

