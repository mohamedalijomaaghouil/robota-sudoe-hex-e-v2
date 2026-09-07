#!/usr/bin/env python3
import csv
import json
import sys
from pathlib import Path

import numpy as np


session = Path(sys.argv[1])

training_files = [
    (
        1,
        session / "hex_e_known_625g_orientations_run1_clean.csv",
    ),
    (
        2,
        session / "hex_e_known_625g_orientations_run2_reverse.csv",
    ),
]

validation_files = [
    (
        "original",
        session / "hex_e_known_1051g_validation_combined_5poses.csv",
    ),
    (
        "repeat",
        session / "hex_e_known_1051g_failing_pose_repeats.csv",
    ),
]

candidate_3x3_file = (
    session / "hex_e_known_625g_orientation_analysis_2runs.json"
)

output_file = (
    session / "hex_e_known_mass_wrench_3x6_analysis.json"
)

wrench_columns = [
    "bracketed_raw_delta_Fx",
    "bracketed_raw_delta_Fy",
    "bracketed_raw_delta_Fz",
    "bracketed_raw_delta_Tx",
    "bracketed_raw_delta_Ty",
    "bracketed_raw_delta_Tz",
]

force_columns = [
    "expected_force_delta_x",
    "expected_force_delta_y",
    "expected_force_delta_z",
]


def load_csv(path, source, run=None):
    records = []

    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []

        missing = [
            name
            for name in wrench_columns + force_columns
            if name not in fields
        ]
        if missing:
            raise RuntimeError(
                f"{path.name}: missing columns {missing}\n"
                f"Available columns: {fields}"
            )

        for row in reader:
            records.append({
                "pose": row["pose"].strip(),
                "source": source,
                "run": run,
                "wrench": np.array(
                    [float(row[name]) for name in wrench_columns],
                    dtype=float,
                ),
                "expected": np.array(
                    [float(row[name]) for name in force_columns],
                    dtype=float,
                ),
            })

    return records


def arrays(records):
    X = np.vstack([record["wrench"] for record in records])
    Y = np.vstack([record["expected"] for record in records])
    return X, Y


def fit(records):
    X, Y = arrays(records)

    scales = np.sqrt(np.mean(X**2, axis=0))
    if np.any(scales <= 0.0):
        raise RuntimeError(f"Invalid design scales: {scales}")

    X_scaled = X / scales

    coefficient_scaled, _, rank, singular_values = np.linalg.lstsq(
        X_scaled,
        Y,
        rcond=None,
    )

    # Y = X @ coefficient_raw
    coefficient_raw = coefficient_scaled / scales[:, None]

    # Present as force = matrix_3x6 @ wrench.
    matrix = coefficient_raw.T

    prediction = X @ coefficient_raw
    residual = prediction - Y
    vector_errors = np.linalg.norm(residual, axis=1)

    return {
        "matrix": matrix,
        "scales": scales,
        "rank": int(rank),
        "singular_values": singular_values,
        "condition_number": float(
            singular_values[0] / singular_values[-1]
        ),
        "prediction": prediction,
        "residual": residual,
        "component_rmse": float(np.sqrt(np.mean(residual**2))),
        "vector_rmse": float(np.sqrt(np.mean(vector_errors**2))),
        "mean_vector_error": float(np.mean(vector_errors)),
        "maximum_vector_error": float(np.max(vector_errors)),
    }


def evaluate(matrix, records):
    X, Y = arrays(records)
    prediction = X @ matrix.T
    residual = prediction - Y
    errors = np.linalg.norm(residual, axis=1)

    return {
        "prediction": prediction,
        "residual": residual,
        "errors": errors,
        "component_rmse": float(np.sqrt(np.mean(residual**2))),
        "vector_rmse": float(np.sqrt(np.mean(errors**2))),
        "mean_vector_error": float(np.mean(errors)),
        "maximum_vector_error": float(np.max(errors)),
    }


training = []
for run, path in training_files:
    training.extend(
        load_csv(path, source=f"625g_run{run}", run=run)
    )

validation = []
for source, path in validation_files:
    validation.extend(
        load_csv(path, source=f"1051g_{source}")
    )

if len(training) != 18:
    raise RuntimeError(
        f"Expected 18 training records, found {len(training)}"
    )

if len(validation) != 8:
    raise RuntimeError(
        f"Expected 8 independent validation records, "
        f"found {len(validation)}"
    )

combined_fit = fit(training)
matrix_3x6 = combined_fit["matrix"]

print("========== KNOWN-MASS 3x6 WRENCH FIT ==========")
print(f"625 g training records:       {len(training)}")
print(f"1051 g validation records:    {len(validation)}")
print(f"Scaled design rank:           {combined_fit['rank']}")
print(
    f"Scaled condition number:      "
    f"{combined_fit['condition_number']:.6f}"
)
print("Scaled singular values:")
print(combined_fit["singular_values"])

print("\n3x6 force-from-wrench matrix:")
print(matrix_3x6)

print("\nColumn order:")
print("  [Fx, Fy, Fz, Tx, Ty, Tz]")

print("\nUnits:")
print("  First three columns: N output per N input")
print("  Last three columns:  N output per Nm input")

print("\n========== COMBINED 625 g TRAINING FIT ==========")
print(
    f"Component RMSE:               "
    f"{combined_fit['component_rmse']:.6f} N"
)
print(
    f"Vector RMSE:                  "
    f"{combined_fit['vector_rmse']:.6f} N"
)
print(
    f"Mean vector error:            "
    f"{combined_fit['mean_vector_error']:.6f} N"
)
print(
    f"Maximum vector error:         "
    f"{combined_fit['maximum_vector_error']:.6f} N"
)

print("\n========== CROSS-RUN 625 g PREDICTION ==========")

cross_run_results = {}

for train_run, test_run in ((1, 2), (2, 1)):
    train_records = [
        record for record in training
        if record["run"] == train_run
    ]
    test_records = [
        record for record in training
        if record["run"] == test_run
    ]

    run_fit = fit(train_records)
    result = evaluate(run_fit["matrix"], test_records)

    key = f"train_run_{train_run}_test_run_{test_run}"
    cross_run_results[key] = result

    print(f"\nTrain Run {train_run}, test Run {test_run}:")
    print(f"  Fit rank:                    {run_fit['rank']}")
    print(
        f"  Fit condition number:        "
        f"{run_fit['condition_number']:.6f}"
    )
    print(
        f"  Test vector RMSE:            "
        f"{result['vector_rmse']:.6f} N"
    )
    print(
        f"  Test mean error:             "
        f"{result['mean_vector_error']:.6f} N"
    )
    print(
        f"  Test maximum error:          "
        f"{result['maximum_vector_error']:.6f} N"
    )

print("\n========== GROUPED LEAVE-ONE-ORIENTATION-OUT ==========")

pose_names = sorted(set(record["pose"] for record in training))
lopo_errors = []

for pose in pose_names:
    train_records = [
        record for record in training
        if record["pose"] != pose
    ]
    test_records = [
        record for record in training
        if record["pose"] == pose
    ]

    pose_fit = fit(train_records)
    result = evaluate(pose_fit["matrix"], test_records)

    errors = result["errors"]
    lopo_errors.extend(errors.tolist())

    print(
        f"{pose}: errors={errors.tolist()}, "
        f"mean={np.mean(errors):.6f} N"
    )

lopo_errors = np.asarray(lopo_errors)

print("\nGrouped cross-validation summary:")
print(f"  Mean error:                  {np.mean(lopo_errors):.6f} N")
print(
    f"  RMS error:                   "
    f"{np.sqrt(np.mean(lopo_errors**2)):.6f} N"
)
print(f"  Maximum error:               {np.max(lopo_errors):.6f} N")

print("\n========== INDEPENDENT 1051 g VALIDATION ==========")

validation_3x6 = evaluate(matrix_3x6, validation)

for index, record in enumerate(validation):
    prediction = validation_3x6["prediction"][index]
    residual = validation_3x6["residual"][index]
    error = validation_3x6["errors"][index]

    raw_force = record["wrench"][:3]
    expected = record["expected"]

    print(f"\n{record['source']} / {record['pose']}")
    print(f"  Raw wrench:                  {record['wrench']}")
    print(f"  3x6 predicted force:         {prediction} N")
    print(f"  Expected force:              {expected} N")
    print(f"  Residual:                    {residual} N")
    print(f"  Error norm:                  {error:.6f} N")
    print(
        f"  Raw-force magnitude ratio:   "
        f"{np.linalg.norm(raw_force)/np.linalg.norm(expected):.6f}"
    )
    print(
        f"  Predicted magnitude ratio:   "
        f"{np.linalg.norm(prediction)/np.linalg.norm(expected):.6f}"
    )

print("\nIndependent 1051 g 3x6 summary:")
print(
    f"  Mean vector error:           "
    f"{validation_3x6['mean_vector_error']:.6f} N"
)
print(
    f"  Vector RMSE:                 "
    f"{validation_3x6['vector_rmse']:.6f} N"
)
print(
    f"  Maximum vector error:        "
    f"{validation_3x6['maximum_vector_error']:.6f} N"
)

candidate_json = json.loads(candidate_3x3_file.read_text())
matrix_3x3 = np.asarray(
    candidate_json["combined"]["matrix_A"],
    dtype=float,
)

validation_3x3_records = []
for record in validation:
    validation_3x3_records.append({
        **record,
        "wrench": record["wrench"][:3],
    })

validation_3x3 = evaluate(matrix_3x3, validation_3x3_records)

print("\n========== 3x3 VERSUS 3x6 ON ALL EIGHT 1051 g RECORDS ==========")
print("Existing 3x3 force-only candidate:")
print(
    f"  Mean vector error:           "
    f"{validation_3x3['mean_vector_error']:.6f} N"
)
print(
    f"  Vector RMSE:                 "
    f"{validation_3x3['vector_rmse']:.6f} N"
)
print(
    f"  Maximum vector error:        "
    f"{validation_3x3['maximum_vector_error']:.6f} N"
)

print("\nNew 3x6 wrench candidate:")
print(
    f"  Mean vector error:           "
    f"{validation_3x6['mean_vector_error']:.6f} N"
)
print(
    f"  Vector RMSE:                 "
    f"{validation_3x6['vector_rmse']:.6f} N"
)
print(
    f"  Maximum vector error:        "
    f"{validation_3x6['maximum_vector_error']:.6f} N"
)

mean_improvement = (
    validation_3x3["mean_vector_error"]
    - validation_3x6["mean_vector_error"]
)

print(
    f"\n3x6 mean-error improvement:    "
    f"{mean_improvement:.6f} N"
)

if mean_improvement > 0:
    print("RESULT: 3x6 improves mean error on the independent records.")
else:
    print("RESULT: 3x6 does not improve independent mean error.")

result_json = {
    "wrench_column_order": wrench_columns,
    "force_output_order": force_columns,
    "training_record_count": len(training),
    "validation_record_count": len(validation),
    "matrix_3x6": matrix_3x6.tolist(),
    "training": {
        "rank": combined_fit["rank"],
        "condition_number": combined_fit["condition_number"],
        "singular_values": combined_fit["singular_values"].tolist(),
        "component_rmse_N": combined_fit["component_rmse"],
        "vector_rmse_N": combined_fit["vector_rmse"],
        "mean_vector_error_N": combined_fit["mean_vector_error"],
        "maximum_vector_error_N": combined_fit["maximum_vector_error"],
    },
    "leave_one_orientation_out": {
        "mean_error_N": float(np.mean(lopo_errors)),
        "rms_error_N": float(np.sqrt(np.mean(lopo_errors**2))),
        "maximum_error_N": float(np.max(lopo_errors)),
    },
    "independent_1051g": {
        "force_only_3x3": {
            "mean_vector_error_N": validation_3x3[
                "mean_vector_error"
            ],
            "vector_rmse_N": validation_3x3["vector_rmse"],
            "maximum_vector_error_N": validation_3x3[
                "maximum_vector_error"
            ],
        },
        "wrench_3x6": {
            "mean_vector_error_N": validation_3x6[
                "mean_vector_error"
            ],
            "vector_rmse_N": validation_3x6["vector_rmse"],
            "maximum_vector_error_N": validation_3x6[
                "maximum_vector_error"
            ],
            "per_record_errors_N": validation_3x6[
                "errors"
            ].tolist(),
        },
        "mean_error_improvement_N": float(mean_improvement),
    },
}

output_file.write_text(
    json.dumps(result_json, indent=2) + "\n"
)

print(f"\nSaved: {output_file}")
print("No calibration was applied.")
