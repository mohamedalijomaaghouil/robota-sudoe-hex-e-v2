#!/usr/bin/env python3
import csv
import json
import sys
from itertools import combinations
from pathlib import Path

import numpy as np

session = Path(sys.argv[1])

paths = [
    session / "hex_e_known_625g_orientations_run1_clean.csv",
    session / "hex_e_known_625g_orientations_run2_reverse.csv",
]

RAW_COLUMNS = [
    "bracketed_raw_delta_Fx",
    "bracketed_raw_delta_Fy",
    "bracketed_raw_delta_Fz",
]

EXPECTED_COLUMNS = [
    "expected_force_delta_x",
    "expected_force_delta_y",
    "expected_force_delta_z",
]


def load(path, run_number):
    records = []

    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []

        missing = [
            name for name in RAW_COLUMNS + EXPECTED_COLUMNS
            if name not in fields
        ]
        if missing:
            raise RuntimeError(
                f"{path.name}: missing columns {missing}; available={fields}"
            )

        for row in reader:
            records.append({
                "run": run_number,
                "pose": row["pose"],
                "raw": np.array(
                    [float(row[name]) for name in RAW_COLUMNS],
                    dtype=float,
                ),
                "expected": np.array(
                    [float(row[name]) for name in EXPECTED_COLUMNS],
                    dtype=float,
                ),
            })

    return records


def fit(records):
    X = np.vstack([record["raw"] for record in records])
    Y = np.vstack([record["expected"] for record in records])

    # Y = X @ A.T
    solution, _, rank, singular_values = np.linalg.lstsq(
        X, Y, rcond=None
    )
    A = solution.T

    condition = (
        float(singular_values[0] / singular_values[-1])
        if singular_values[-1] > 0
        else float("inf")
    )

    return A, int(rank), condition, singular_values


def evaluate(A, records):
    errors = []

    for record in records:
        prediction = A @ record["raw"]
        errors.append(np.linalg.norm(prediction - record["expected"]))

    errors = np.asarray(errors, dtype=float)

    return {
        "vector_rmse": float(np.sqrt(np.mean(errors ** 2))),
        "mean_vector_error": float(np.mean(errors)),
        "maximum_vector_error": float(np.max(errors)),
        "errors": errors,
    }


runs = [load(path, index + 1) for index, path in enumerate(paths)]
combined = runs[0] + runs[1]

order1 = [record["pose"] for record in runs[0]]
order2 = [record["pose"] for record in runs[1]]

print("========== TWO-RUN KNOWN 625 g ANALYSIS ==========")
print(f"Session:                       {session}")
print(f"Run 1 records:                 {len(runs[0])}")
print(f"Run 2 records:                 {len(runs[1])}")
print(f"Combined records:              {len(combined)}")
print(f"Run 2 reverses Run 1:          {order2 == list(reversed(order1))}")

A_combined, rank, condition, singular_values = fit(combined)
combined_metrics = evaluate(A_combined, combined)

print("\nCombined known-mass force matrix A:")
print(A_combined)
print(f"Rank:                          {rank}")
print(f"Condition number:              {condition:.6f}")
print("Design singular values:")
print(singular_values)
print(
    f"Vector RMSE:                   "
    f"{combined_metrics['vector_rmse']:.6f} N"
)
print(
    f"Mean vector error:             "
    f"{combined_metrics['mean_vector_error']:.6f} N"
)
print(
    f"Maximum vector error:          "
    f"{combined_metrics['maximum_vector_error']:.6f} N"
)

per_run = []
matrices = []

print("\n========== PER-RUN FITS ==========")

for run_number, records in enumerate(runs, start=1):
    A, run_rank, run_condition, run_singular = fit(records)
    metrics = evaluate(A, records)
    matrices.append(A)

    print(f"\nRun {run_number}:")
    print(A)
    print(f"Rank:                          {run_rank}")
    print(f"Condition number:              {run_condition:.6f}")
    print(f"Vector RMSE:                   {metrics['vector_rmse']:.6f} N")
    print(
        f"Maximum vector error:          "
        f"{metrics['maximum_vector_error']:.6f} N"
    )

    per_run.append({
        "run": run_number,
        "matrix_A": A.tolist(),
        "rank": run_rank,
        "condition_number": run_condition,
        "singular_values": run_singular.tolist(),
        "vector_rmse": metrics["vector_rmse"],
        "mean_vector_error": metrics["mean_vector_error"],
        "maximum_vector_error": metrics["maximum_vector_error"],
    })

matrix_difference = matrices[1] - matrices[0]

print("\nRun 2 matrix minus Run 1 matrix:")
print(matrix_difference)
print(
    f"Matrix Frobenius difference:   "
    f"{np.linalg.norm(matrix_difference):.6f}"
)
print(f"Run 1 Azz:                     {matrices[0][2,2]:.6f}")
print(f"Run 2 Azz:                     {matrices[1][2,2]:.6f}")
print(
    f"Azz absolute difference:       "
    f"{abs(matrices[1][2,2] - matrices[0][2,2]):.6f}"
)

print("\n========== CROSS-RUN PREDICTION ==========")

cross_run_results = []

for training_index, testing_index in ((0, 1), (1, 0)):
    A_train, _, _, _ = fit(runs[training_index])
    metrics = evaluate(A_train, runs[testing_index])

    print(
        f"\nTrain Run {training_index + 1}, "
        f"test Run {testing_index + 1}:"
    )
    print(f"  Vector RMSE:                 {metrics['vector_rmse']:.6f} N")
    print(
        f"  Mean vector error:           "
        f"{metrics['mean_vector_error']:.6f} N"
    )
    print(
        f"  Maximum vector error:        "
        f"{metrics['maximum_vector_error']:.6f} N"
    )

    cross_run_results.append({
        "training_run": training_index + 1,
        "testing_run": testing_index + 1,
        "vector_rmse": metrics["vector_rmse"],
        "mean_vector_error": metrics["mean_vector_error"],
        "maximum_vector_error": metrics["maximum_vector_error"],
    })

print("\n========== RAW-DELTA REPEATABILITY ==========")

by_run = [
    {record["pose"]: record for record in records}
    for records in runs
]

repeatability = []
for pose in sorted(set(by_run[0]) & set(by_run[1])):
    difference = by_run[1][pose]["raw"] - by_run[0][pose]["raw"]
    norm = float(np.linalg.norm(difference))
    repeatability.append(norm)

    print(f"\n{pose}")
    print(f"  Run2 - Run1:                 {difference} N")
    print(f"  Difference norm:             {norm:.6f} N")

print("\nRepeatability summary:")
print(
    f"  Mean difference:             "
    f"{np.mean(repeatability):.6f} N"
)
print(
    f"  Maximum difference:          "
    f"{np.max(repeatability):.6f} N"
)

print("\n========== LEAVE-ONE-ORIENTATION-OUT ==========")

poses = sorted({record["pose"] for record in combined})
loo_errors = []
loo_by_pose = {}

for pose in poses:
    training = [
        record for record in combined
        if record["pose"] != pose
    ]
    testing = [
        record for record in combined
        if record["pose"] == pose
    ]

    A_loo, loo_rank, _, _ = fit(training)
    if loo_rank != 3:
        raise RuntimeError(
            f"Leave-one-orientation-out fit for {pose} has rank {loo_rank}"
        )

    errors = []
    for record in testing:
        error = float(np.linalg.norm(
            A_loo @ record["raw"] - record["expected"]
        ))
        errors.append(error)
        loo_errors.append(error)

    loo_by_pose[pose] = errors
    print(
        f"{pose}: errors={errors}, "
        f"mean={np.mean(errors):.6f} N"
    )

loo_errors = np.asarray(loo_errors, dtype=float)

print("\nLeave-one-orientation-out summary:")
print(f"  Mean error:                  {np.mean(loo_errors):.6f} N")
print(
    f"  RMS error:                   "
    f"{np.sqrt(np.mean(loo_errors ** 2)):.6f} N"
)
print(f"  Maximum error:               {np.max(loo_errors):.6f} N")

output = {
    "inputs": [str(path) for path in paths],
    "order_checks": {
        "run2_reverses_run1": order2 == list(reversed(order1)),
        "run1": order1,
        "run2": order2,
    },
    "combined": {
        "matrix_A": A_combined.tolist(),
        "rank": rank,
        "condition_number": condition,
        "singular_values": singular_values.tolist(),
        "vector_rmse": combined_metrics["vector_rmse"],
        "mean_vector_error": combined_metrics["mean_vector_error"],
        "maximum_vector_error": combined_metrics["maximum_vector_error"],
    },
    "per_run": per_run,
    "matrix_repeatability": {
        "run2_minus_run1": matrix_difference.tolist(),
        "frobenius_difference": float(np.linalg.norm(matrix_difference)),
        "Azz_absolute_difference": float(
            abs(matrices[1][2,2] - matrices[0][2,2])
        ),
    },
    "cross_run_prediction": cross_run_results,
    "raw_delta_repeatability": {
        "mean_difference": float(np.mean(repeatability)),
        "maximum_difference": float(np.max(repeatability)),
    },
    "leave_one_orientation_out": {
        "errors_by_pose": loo_by_pose,
        "mean_error": float(np.mean(loo_errors)),
        "rms_error": float(np.sqrt(np.mean(loo_errors ** 2))),
        "maximum_error": float(np.max(loo_errors)),
    },
}

output_path = (
    session / "hex_e_known_625g_orientation_analysis_2runs.json"
)
output_path.write_text(json.dumps(output, indent=2) + "\n")

print("\nNo calibration was applied.")
print(f"Saved: {output_path}")
