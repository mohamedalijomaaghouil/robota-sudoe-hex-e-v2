# HEX-E v2 Experimental Scripts

This directory preserves scripts used during the OnRobot HEX-E v2
integration, calibration, validation, compensation, and filtering work.

These scripts are experimental research artifacts. They must be reviewed
before they are used to command robot motion.

Inventory generated: 2026-09-07T21:17:22+02:00

## Preserved files

- `hex_e_1051g_step_test.py`
- `hex_e_625g_step_test.py`
- `hex_e_additivity.py`
- `hex_e_analyze_1051g_failing_pose_repeats.py`
- `hex_e_analyze_interleaved_30.py`
- `hex_e_analyze_interleaved_3runs.py`
- `hex_e_analyze_known_625g_2runs.py`
- `hex_e_analyze_orientations.py`
- `hex_e_characterize_compensation_filters.py`
- `hex_e_collect_interleaved_30.py`
- `hex_e_collect_known_1051g_continuation.py`
- `hex_e_collect_known_1051g_failing_pose_repeats.py`
- `hex_e_collect_known_1051g_validation.py`
- `hex_e_collect_known_625g_orientations.py`
- `hex_e_collect_orientations.py`
- `hex_e_compensation_diagnostic.py`
- `hex_e_compensation_gravity_map_diagnostic.py`
- `hex_e_fit_effective_tool_mass.py`
- `hex_e_fit_known_mass_wrench_3x6.py`
- `hex_e_fit_unloaded_gravity_map.py`
- `hex_e_repeatability.py`
- `hex_e_signal_conditioning_diagnostic.py`
- `hex_e_unloaded_drift.py`

## Safety classification

- Analysis-only scripts may be run on saved datasets.
- ROS data-collection scripts may subscribe to live topics.
- Any script that publishes velocity, trajectory, or controller commands
  must be inspected and tested with robot motion disabled first.

## Current technical status

The HEX-E v2 EtherCAT acquisition path and static calibration experiments
have been preserved. Full orientation-independent compensation and live
admittance-control validation are not yet complete.
