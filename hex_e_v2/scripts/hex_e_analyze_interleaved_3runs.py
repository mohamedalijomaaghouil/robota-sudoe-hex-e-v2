#!/usr/bin/env python3
import csv
import json
import sys
from itertools import combinations
from pathlib import Path

import numpy as np

session = Path(sys.argv[1])

paths = [
    session / "hex_e_interleaved_30_run1.csv",
    session / "hex_e_interleaved_30_run2.csv",
    session / "hex_e_interleaved_30_run3_true_reverse.csv",
]

RAW_COLUMNS = [
    "raw_force_delta_x",
    "raw_force_delta_y",
    "raw_force_delta_z",
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

        missing = [
            name for name in RAW_COLUMNS + EXPECTED_COLUMNS
            if name not in (reader.fieldnames or [])
        ]
        if missing:
            raise RuntimeError(f"{path}: missing columns: {missing}")

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

    coefficient, _, rank, singular_values = np.linalg.lstsq(
        X, Y, rcond=None
    )
    A = coefficient.T

    predicted = (A @ X.T).T
    errors = predicted - Y
    vector_errors = np.linalg.norm(errors, axis=1)

    return {
        "A": A,
        "rank": int(rank),
        "singular_values": singular_values,
        "condition": float(np.linalg.cond(X)),
        "component_rmse": float(np.sqrt(np.mean(errors ** 2))),
        "vector_rmse": float(np.sqrt(np.mean(vector_errors ** 2))),
        "mean_vector_error": float(np.mean(vector_errors)),
        "maximum_vector_error": float(np.max(vector_errors)),
        "errors": errors,
        "vector_errors": vector_errors,
    }


def evaluate(A, records):
    X = np.vstack([record["raw"] for record in records])
    Y = np.vstack([record["expected"] for record in records])

    errors = (A @ X.T).T - Y
    vector_errors = np.linalg.norm(errors, axis=1)

    return {
        "vector_rmse": float(np.sqrt(np.mean(vector_errors ** 2))),
        "mean_vector_error": float(np.mean(vector_errors)),
        "maximum_vector_error": float(np.max(vector_errors)),
        "per_record": [
            {
                "pose": record["pose"],
                "error_N": float(error),
            }
            for record, error in zip(records, vector_errors)
        ],
    }


runs = [
    load(path, number)
    for number, path in enumerate(paths, start=1)
]

all_records = [record for run in runs for record in run]

combined = fit(all_records)
forward = fit(runs[0] + runs[1])
reverse = fit(runs[2])
per_run = [fit(run) for run in runs]

print("========== THREE-RUN INTERLEAVED ANALYSIS ==========")
print(f"Session:                       {session}")
print(f"Total records:                 {len(all_records)}")
print(f"Run 1 records:                 {len(runs[0])}")
print(f"Run 2 records:                 {len(runs[1])}")
print(f"Run 3 records:                 {len(runs[2])}")

print("\nPose-order checks:")
order1 = [record["pose"] for record in runs[0]]
order2 = [record["pose"] for record in runs[1]]
order3 = [record["pose"] for record in runs[2]]

print(f"  Run 1 equals Run 2:          {order1 == order2}")
print(f"  Run 3 reverses Run 1:        {order3 == list(reversed(order1))}")

print("\nCombined three-run force matrix A:")
print(combined["A"])
print(f"Rank:                          {combined['rank']}")
print(f"Condition number:              {combined['condition']:.6f}")
print(f"Component RMSE:                {combined['component_rmse']:.6f} N")
print(f"Vector RMSE:                   {combined['vector_rmse']:.6f} N")
print(f"Mean vector error:             {combined['mean_vector_error']:.6f} N")
print(f"Maximum vector error:          {combined['maximum_vector_error']:.6f} N")

print("\n========== PER-RUN MATRICES ==========")
for index, result in enumerate(per_run, start=1):
    print(f"\nRun {index}:")
    print(result["A"])
    print(f"Rank:                          {result['rank']}")
    print(f"Condition number:              {result['condition']:.6f}")
    print(f"Vector RMSE:                   {result['vector_rmse']:.6f} N")
    print(f"Maximum vector error:          {result['maximum_vector_error']:.6f} N")

print("\n========== FORWARD VERSUS REVERSE ==========")
print("Forward-order matrix (Runs 1 + 2):")
print(forward["A"])

print("\nTrue-reverse matrix (Run 3):")
print(reverse["A"])

matrix_difference = reverse["A"] - forward["A"]
print("\nReverse minus forward:")
print(matrix_difference)
print(
    "Frobenius matrix difference:   "
    f"{np.linalg.norm(matrix_difference):.6f}"
)
print(f"Forward Azz:                   {forward['A'][2,2]:.6f}")
print(f"Reverse Azz:                   {reverse['A'][2,2]:.6f}")
print(
    "Azz absolute difference:       "
    f"{abs(reverse['A'][2,2] - forward['A'][2,2]):.6f}"
)

print("\n========== LEAVE-ONE-RUN-OUT PREDICTION ==========")
leave_one_run_out = []

for test_index in range(3):
    training = [
        record
        for run_index, run in enumerate(runs)
        if run_index != test_index
        for record in run
    ]
    test = runs[test_index]

    training_fit = fit(training)
    validation = evaluate(training_fit["A"], test)

    leave_one_run_out.append({
        "test_run": test_index + 1,
        **validation,
    })

    print(f"\nTest Run {test_index + 1}:")
    print(f"  Vector RMSE:                 {validation['vector_rmse']:.6f} N")
    print(f"  Mean vector error:           {validation['mean_vector_error']:.6f} N")
    print(f"  Maximum vector error:        {validation['maximum_vector_error']:.6f} N")

print("\n========== POSE REPEATABILITY ACROSS RUNS ==========")
poses = sorted(set(record["pose"] for record in all_records))
pose_repeatability = {}

all_pairwise_differences = []

for pose in poses:
    vectors = []
    for run in runs:
        match = next(record for record in run if record["pose"] == pose)
        vectors.append(match["raw"])

    pairwise = [
        float(np.linalg.norm(vectors[i] - vectors[j]))
        for i, j in combinations(range(3), 2)
    ]

    all_pairwise_differences.extend(pairwise)
    pose_repeatability[pose] = {
        "pairwise_difference_norms_N": pairwise,
        "mean_N": float(np.mean(pairwise)),
        "maximum_N": float(np.max(pairwise)),
    }

    print(f"\n{pose}")
    print(f"  Pairwise differences:        {pairwise}")
    print(f"  Mean:                        {np.mean(pairwise):.6f} N")
    print(f"  Maximum:                     {np.max(pairwise):.6f} N")

print("\nRepeatability summary:")
print(
    "  Mean pairwise difference:    "
    f"{np.mean(all_pairwise_differences):.6f} N"
)
print(
    "  Maximum pairwise difference: "
    f"{np.max(all_pairwise_differences):.6f} N"
)

print("\n========== LEAVE-ONE-POSE-OUT VALIDATION ==========")
leave_one_pose_out = {}
all_lopo_errors = []

for pose in poses:
    training = [
        record for record in all_records
        if record["pose"] != pose
    ]
    test = [
        record for record in all_records
        if record["pose"] == pose
    ]

    training_fit = fit(training)
    validation = evaluate(training_fit["A"], test)
    errors = [
        item["error_N"] for item in validation["per_record"]
    ]

    leave_one_pose_out[pose] = errors
    all_lopo_errors.extend(errors)

    print(
        f"{pose}: errors={errors}, "
        f"mean={np.mean(errors):.6f} N"
    )

print("\nLeave-one-pose-out summary:")
print(f"  Mean error:                  {np.mean(all_lopo_errors):.6f} N")
print(
    "  RMS error:                   "
    f"{np.sqrt(np.mean(np.square(all_lopo_errors))):.6f} N"
)
print(f"  Maximum error:               {np.max(all_lopo_errors):.6f} N")

output = {
    "status": "diagnostic_only_no_calibration_applied",
    "input_files": [str(path) for path in paths],
    "orders": {
        "run1": order1,
        "run2": order2,
        "run3": order3,
        "run1_equals_run2": order1 == order2,
        "run3_is_reverse_of_run1": order3 == list(reversed(order1)),
    },
    "combined": {
        "matrix_A": combined["A"].tolist(),
        "rank": combined["rank"],
        "condition_number": combined["condition"],
        "component_rmse_N": combined["component_rmse"],
        "vector_rmse_N": combined["vector_rmse"],
        "mean_vector_error_N": combined["mean_vector_error"],
        "maximum_vector_error_N": combined["maximum_vector_error"],
    },
    "per_run": [
        {
            "run": index,
            "matrix_A": result["A"].tolist(),
            "rank": result["rank"],
            "condition_number": result["condition"],
            "vector_rmse_N": result["vector_rmse"],
            "maximum_vector_error_N": result["maximum_vector_error"],
        }
        for index, result in enumerate(per_run, start=1)
    ],
    "forward_runs_1_and_2": {
        "matrix_A": forward["A"].tolist(),
        "vector_rmse_N": forward["vector_rmse"],
        "maximum_vector_error_N": forward["maximum_vector_error"],
    },
    "true_reverse_run_3": {
        "matrix_A": reverse["A"].tolist(),
        "vector_rmse_N": reverse["vector_rmse"],
        "maximum_vector_error_N": reverse["maximum_vector_error"],
    },
    "reverse_minus_forward_matrix": matrix_difference.tolist(),
    "matrix_frobenius_difference": float(
        np.linalg.norm(matrix_difference)
    ),
    "forward_Azz": float(forward["A"][2, 2]),
    "reverse_Azz": float(reverse["A"][2, 2]),
    "Azz_absolute_difference": float(
        abs(reverse["A"][2, 2] - forward["A"][2, 2])
    ),
    "leave_one_run_out": leave_one_run_out,
    "pose_repeatability": pose_repeatability,
    "mean_pairwise_repeatability_N": float(
        np.mean(all_pairwise_differences)
    ),
    "maximum_pairwise_repeatability_N": float(
        np.max(all_pairwise_differences)
    ),
    "leave_one_pose_out": leave_one_pose_out,
    "leave_one_pose_out_mean_N": float(np.mean(all_lopo_errors)),
    "leave_one_pose_out_rms_N": float(
        np.sqrt(np.mean(np.square(all_lopo_errors)))
    ),
    "leave_one_pose_out_maximum_N": float(np.max(all_lopo_errors)),
    "restrictions": {
        "apply_matrix": False,
        "modify_driver": False,
        "hardware_zero": False,
        "activate_force_controller": False,
        "activate_compliance_controller": False,
    },
}

output_path = session / "hex_e_interleaved_3run_analysis.json"
output_path.write_text(json.dumps(output, indent=2) + "\n")

print("\nNo calibration was applied.")
print(f"Saved: {output_path}")
