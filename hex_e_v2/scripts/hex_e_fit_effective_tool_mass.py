#!/usr/bin/env python3
import csv
import json
import sys
from pathlib import Path

import numpy as np

session = Path(sys.argv[1])

candidate_file = (
    session / "hex_e_known_625g_orientation_analysis_2runs.json"
)

training_files = [
    session / "hex_e_known_625g_orientations_run1_clean.csv",
    session / "hex_e_known_625g_orientations_run2_reverse.csv",
]

output_file = session / "hex_e_effective_tool_mass_fit.json"

A = np.asarray(
    json.loads(candidate_file.read_text())["combined"]["matrix_A"],
    dtype=float,
)

force_columns = ["Fx", "Fy", "Fz"]
gravity_columns = [
    "gravity_tool_x",
    "gravity_tool_y",
    "gravity_tool_z",
]

design_rows = []
observations = []

# Model:
#   A @ F_raw = bias_for_run + mass * gravity_in_tool0
#
# Unknowns:
#   [mass, run1_bias_x, run1_bias_y, run1_bias_z,
#          run2_bias_x, run2_bias_y, run2_bias_z]

for run_index, path in enumerate(training_files):
    with path.open(newline="") as f:
        reader = csv.DictReader(f)

        for row in reader:
            gravity = np.array(
                [float(row[name]) for name in gravity_columns],
                dtype=float,
            )

            for phase in ("unloaded_before", "unloaded_after"):
                raw_force = np.array(
                    [
                        float(row[f"{phase}_mean_{axis}"])
                        for axis in force_columns
                    ],
                    dtype=float,
                )

                calibrated_force = A @ raw_force

                for axis in range(3):
                    design = np.zeros(7, dtype=float)
                    design[0] = gravity[axis]
                    design[1 + 3 * run_index + axis] = 1.0

                    design_rows.append(design)
                    observations.append(calibrated_force[axis])

X = np.vstack(design_rows)
y = np.asarray(observations)

solution, _, rank, singular_values = np.linalg.lstsq(
    X, y, rcond=None
)

predicted = X @ solution
residual = predicted - y

mass_kg = float(solution[0])
bias_run1 = solution[1:4]
bias_run2 = solution[4:7]

component_rmse = float(np.sqrt(np.mean(residual ** 2)))

result = {
    "model": "A @ raw_force = run_bias + mass * gravity_tool0",
    "matrix_A": A.tolist(),
    "effective_mass_kg": mass_kg,
    "run1_bias_N": bias_run1.tolist(),
    "run2_bias_N": bias_run2.tolist(),
    "component_rmse_N": component_rmse,
    "rank": int(rank),
    "singular_values": singular_values.tolist(),
    "scalar_observations": int(len(y)),
}

output_file.write_text(json.dumps(result, indent=2) + "\n")

print("========== EFFECTIVE TOOL-MASS FIT ==========")
print(f"Scalar observations:       {len(y)}")
print(f"Design rank:              {rank}")
print(f"Effective mass:           {mass_kg:.6f} kg")
print(f"Run 1 calibrated bias:    {bias_run1} N")
print(f"Run 2 calibrated bias:    {bias_run2} N")
print(f"Component RMSE:           {component_rmse:.6f} N")
print()
print(f"Saved: {output_file}")
print("No calibration was applied and no robot motion was commanded.")
