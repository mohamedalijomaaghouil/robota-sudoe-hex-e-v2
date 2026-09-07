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

training_files = (
    (
        "run1",
        session / "hex_e_known_625g_orientations_run1_clean.csv",
    ),
    (
        "run2",
        session / "hex_e_known_625g_orientations_run2_reverse.csv",
    ),
)

output_file = session / "hex_e_unloaded_gravity_map_fit.json"

A = np.asarray(
    json.loads(candidate_file.read_text())["combined"]["matrix_A"],
    dtype=float,
)

gravity_columns = (
    "gravity_tool_x",
    "gravity_tool_y",
    "gravity_tool_z",
)

phases = ("unloaded_before", "unloaded_after")


def read_run(run_name, path):
    observations = []

    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames or []

        required = list(gravity_columns)

        for phase in phases:
            required.extend(
                (
                    f"{phase}_mean_Fx",
                    f"{phase}_mean_Fy",
                    f"{phase}_mean_Fz",
                )
            )

        missing = [name for name in required if name not in fields]

        if missing:
            raise RuntimeError(
                f"{path.name}: missing columns {missing}\n"
                f"Available columns: {fields}"
            )

        for row in reader:
            gravity = np.asarray(
                tuple(float(row[name]) for name in gravity_columns),
                dtype=float,
            )

            pose = row["pose"]

            for phase in phases:
                raw_force = np.asarray(
                    (
                        float(row[f"{phase}_mean_Fx"]),
                        float(row[f"{phase}_mean_Fy"]),
                        float(row[f"{phase}_mean_Fz"]),
                    ),
                    dtype=float,
                )

                calibrated_force = A @ raw_force

                observations.append(
                    {
                        "run": run_name,
                        "pose": pose,
                        "phase": phase,
                        "gravity": gravity,
                        "raw_force": raw_force,
                        "calibrated_force": calibrated_force,
                    }
                )

    return observations


observations = []

for run_name, path in training_files:
    observations.extend(read_run(run_name, path))


def reference_anchor(run_name):
    reference = [
        item
        for item in observations
        if item["run"] == run_name
        and item["pose"] == "reference"
    ]

    if not reference:
        raise RuntimeError(
            f"No reference observations were found for {run_name}"
        )

    gravity = np.mean(
        np.vstack(tuple(item["gravity"] for item in reference)),
        axis=0,
    )

    force = np.mean(
        np.vstack(
            tuple(item["calibrated_force"] for item in reference)
        ),
        axis=0,
    )

    return gravity, force


anchors = {
    run_name: reference_anchor(run_name)
    for run_name, _ in training_files
}


def make_relative_dataset(selected):
    delta_gravity = []
    delta_force = []
    metadata = []

    for item in selected:
        if item["pose"] == "reference":
            continue

        gravity_reference, force_reference = anchors[item["run"]]

        delta_gravity.append(
            item["gravity"] - gravity_reference
        )

        delta_force.append(
            item["calibrated_force"] - force_reference
        )

        metadata.append(
            {
                "run": item["run"],
                "pose": item["pose"],
                "phase": item["phase"],
            }
        )

    return (
        np.vstack(delta_gravity),
        np.vstack(delta_force),
        metadata,
    )


def fit_map(selected):
    delta_gravity, delta_force, metadata = (
        make_relative_dataset(selected)
    )

    coefficients, _, rank, singular_values = np.linalg.lstsq(
        delta_gravity,
        delta_force,
        rcond=None,
    )

    # NumPy solves:
    #
    #   delta_gravity @ coefficients = delta_force
    #
    # The online column-vector form is:
    #
    #   delta_force = H @ delta_gravity
    H = coefficients.T

    predicted = delta_gravity @ coefficients
    residual = predicted - delta_force
    errors = np.linalg.norm(residual, axis=1)

    condition = (
        float(singular_values[0] / singular_values[-1])
        if singular_values[-1] > 0.0
        else float("inf")
    )

    return {
        "H": H,
        "rank": int(rank),
        "singular_values": singular_values,
        "condition": condition,
        "predicted": predicted,
        "measured": delta_force,
        "residual": residual,
        "errors": errors,
        "metadata": metadata,
    }


combined = fit_map(observations)

print("========== UNLOADED GRAVITY-MAP FIT ==========")
print(f"Unloaded observations:       {len(observations)}")
print(f"Relative fit observations:   {len(combined['errors'])}")
print(f"Design rank:                 {combined['rank']}")
print(f"Design condition number:     {combined['condition']:.6f}")
print("Design singular values:")
print(combined["singular_values"])
print()
print("Calibrated-force gravity map H:")
print(combined["H"])
print()
print("Model:")
print("  delta calibrated force = H @ delta gravity_tool")
print()
print("Combined fit quality:")
print(
    "  Component RMSE:             "
    f"{np.sqrt(np.mean(combined['residual'] ** 2)):.6f} N"
)
print(
    "  Vector RMSE:                "
    f"{np.sqrt(np.mean(combined['errors'] ** 2)):.6f} N"
)
print(
    "  Mean vector error:          "
    f"{np.mean(combined['errors']):.6f} N"
)
print(
    "  Maximum vector error:       "
    f"{np.max(combined['errors']):.6f} N"
)

print()
print("========== INDIVIDUAL-RUN FITS ==========")

run_results = {}

for run_name, _ in training_files:
    selected = [
        item for item in observations
        if item["run"] == run_name
    ]

    result = fit_map(selected)
    run_results[run_name] = result

    print()
    print(run_name)
    print("H:")
    print(result["H"])
    print(f"Rank:              {result['rank']}")
    print(f"Condition number:  {result['condition']:.6f}")
    print(
        "Mean vector error: "
        f"{np.mean(result['errors']):.6f} N"
    )
    print(
        "Maximum error:     "
        f"{np.max(result['errors']):.6f} N"
    )

print()
print("========== CROSS-RUN VALIDATION ==========")

cross_run_results = {}

for train_name, test_name in (("run1", "run2"), ("run2", "run1")):
    H_train = run_results[train_name]["H"]

    test_observations = [
        item
        for item in observations
        if item["run"] == test_name
        and item["pose"] != "reference"
    ]

    gravity_reference, force_reference = anchors[test_name]

    errors = []

    for item in test_observations:
        delta_gravity = item["gravity"] - gravity_reference
        measured_delta = (
            item["calibrated_force"] - force_reference
        )
        predicted_delta = H_train @ delta_gravity
        error = predicted_delta - measured_delta
        errors.append(float(np.linalg.norm(error)))

    errors = np.asarray(errors, dtype=float)

    key = f"{train_name}_to_{test_name}"
    cross_run_results[key] = errors

    print()
    print(f"Train {train_name}, test {test_name}:")
    print(f"  Test records:       {len(errors)}")
    print(
        "  Mean vector error:  "
        f"{np.mean(errors):.6f} N"
    )
    print(
        "  Vector RMSE:        "
        f"{np.sqrt(np.mean(errors ** 2)):.6f} N"
    )
    print(
        "  Maximum error:      "
        f"{np.max(errors):.6f} N"
    )

print()
print("========== LEAVE-ONE-ORIENTATION-OUT ==========")

poses = sorted(
    {
        item["pose"]
        for item in observations
        if item["pose"] != "reference"
    }
)

lopo_errors = []

for omitted_pose in poses:
    training = [
        item
        for item in observations
        if item["pose"] != omitted_pose
    ]

    result = fit_map(training)
    H = result["H"]

    test_items = [
        item
        for item in observations
        if item["pose"] == omitted_pose
    ]

    pose_errors = []

    for item in test_items:
        gravity_reference, force_reference = anchors[item["run"]]

        delta_gravity = item["gravity"] - gravity_reference
        measured_delta = (
            item["calibrated_force"] - force_reference
        )
        predicted_delta = H @ delta_gravity

        pose_errors.append(
            float(
                np.linalg.norm(
                    predicted_delta - measured_delta
                )
            )
        )

    lopo_errors.extend(pose_errors)

    print(
        f"{omitted_pose}: "
        f"errors={pose_errors}, "
        f"mean={np.mean(pose_errors):.6f} N"
    )

lopo_errors = np.asarray(lopo_errors, dtype=float)

print()
print("Grouped cross-validation summary:")
print(
    "  Mean vector error:          "
    f"{np.mean(lopo_errors):.6f} N"
)
print(
    "  Vector RMSE:                "
    f"{np.sqrt(np.mean(lopo_errors ** 2)):.6f} N"
)
print(
    "  Maximum vector error:       "
    f"{np.max(lopo_errors):.6f} N"
)

serializable = {
    "matrix_A": A.tolist(),
    "gravity_map_H": combined["H"].tolist(),
    "fit_rank": combined["rank"],
    "fit_condition_number": combined["condition"],
    "fit_singular_values": (
        combined["singular_values"].tolist()
    ),
    "combined_component_rmse_N": float(
        np.sqrt(np.mean(combined["residual"] ** 2))
    ),
    "combined_vector_rmse_N": float(
        np.sqrt(np.mean(combined["errors"] ** 2))
    ),
    "combined_mean_vector_error_N": float(
        np.mean(combined["errors"])
    ),
    "combined_maximum_vector_error_N": float(
        np.max(combined["errors"])
    ),
    "run_maps": {
        name: result["H"].tolist()
        for name, result in run_results.items()
    },
    "cross_run": {
        name: {
            "mean_error_N": float(np.mean(errors)),
            "rms_error_N": float(
                np.sqrt(np.mean(errors ** 2))
            ),
            "maximum_error_N": float(np.max(errors)),
        }
        for name, errors in cross_run_results.items()
    },
    "leave_one_orientation_out": {
        "mean_error_N": float(np.mean(lopo_errors)),
        "rms_error_N": float(
            np.sqrt(np.mean(lopo_errors ** 2))
        ),
        "maximum_error_N": float(np.max(lopo_errors)),
    },
}

output_file.write_text(
    json.dumps(serializable, indent=2) + "\n"
)

print()
print(f"Saved: {output_file}")
print("No ROS node was started.")
print("No calibration was applied.")
print("No robot motion was commanded.")
