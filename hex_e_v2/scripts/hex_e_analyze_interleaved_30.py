#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def find_triplet(fieldnames, candidates, description):
    for triplet in candidates:
        if all(name in fieldnames for name in triplet):
            return triplet

    raise RuntimeError(
        f"Could not identify {description} columns.\n"
        f"Available columns:\n  " + "\n  ".join(fieldnames)
    )


def load_file(path, run_number):
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []

        raw_columns = find_triplet(
            fieldnames,
            [
                ("raw_force_delta_x", "raw_force_delta_y", "raw_force_delta_z"),
                ("raw_delta_Fx", "raw_delta_Fy", "raw_delta_Fz"),
                ("bracketed_delta_Fx", "bracketed_delta_Fy", "bracketed_delta_Fz"),
                ("delta_Fx", "delta_Fy", "delta_Fz"),
                ("raw_delta_x", "raw_delta_y", "raw_delta_z"),
            ],
            "raw force-delta",
        )

        expected_columns = find_triplet(
            fieldnames,
            [
                (
                    "expected_force_delta_x",
                    "expected_force_delta_y",
                    "expected_force_delta_z",
                ),
                (
                    "expected_delta_Fx",
                    "expected_delta_Fy",
                    "expected_delta_Fz",
                ),
                (
                    "expected_Fx",
                    "expected_Fy",
                    "expected_Fz",
                ),
                (
                    "expected_delta_x",
                    "expected_delta_y",
                    "expected_delta_z",
                ),
            ],
            "expected force-delta",
        )

        records = []
        for row in reader:
            records.append(
                {
                    "run": run_number,
                    "pose": row["pose"],
                    "raw": np.array(
                        [float(row[name]) for name in raw_columns],
                        dtype=float,
                    ),
                    "expected": np.array(
                        [float(row[name]) for name in expected_columns],
                        dtype=float,
                    ),
                }
            )

    return records, raw_columns, expected_columns


def fit_matrix(records):
    raw = np.vstack([record["raw"] for record in records])
    expected = np.vstack([record["expected"] for record in records])

    # raw @ coefficients = expected
    coefficients, _, rank, singular_values = np.linalg.lstsq(
        raw, expected, rcond=None
    )

    matrix = coefficients.T
    predicted = raw @ coefficients
    residual = predicted - expected
    vector_errors = np.linalg.norm(residual, axis=1)

    condition = (
        float(singular_values[0] / singular_values[-1])
        if singular_values[-1] > 0
        else float("inf")
    )

    return {
        "matrix": matrix,
        "rank": int(rank),
        "singular_values": singular_values,
        "condition": condition,
        "predicted": predicted,
        "residual": residual,
        "vector_errors": vector_errors,
        "rmse_components": float(np.sqrt(np.mean(residual**2))),
        "rmse_vectors": float(np.sqrt(np.mean(vector_errors**2))),
        "mean_vector_error": float(np.mean(vector_errors)),
        "maximum_vector_error": float(np.max(vector_errors)),
    }


def to_serializable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {key: to_serializable(item) for key, item in value.items()}
    if isinstance(value, list):
        return [to_serializable(item) for item in value]
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run1", type=Path)
    parser.add_argument("run2", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    run1, raw1, expected1 = load_file(args.run1, 1)
    run2, raw2, expected2 = load_file(args.run2, 2)

    if raw1 != raw2 or expected1 != expected2:
        raise RuntimeError("The two CSV files use different force columns.")

    order1 = [record["pose"] for record in run1]
    order2 = [record["pose"] for record in run2]

    if set(order1) != set(order2):
        raise RuntimeError("The two runs do not contain the same pose set.")

    combined = run1 + run2
    fit_all = fit_matrix(combined)
    fit1 = fit_matrix(run1)
    fit2 = fit_matrix(run2)

    print("========== INTERLEAVED ±30 DEGREE ANALYSIS ==========")
    print(f"Run 1 records:                 {len(run1)}")
    print(f"Run 2 records:                 {len(run2)}")
    print(f"Combined records:              {len(combined)}")
    print(f"Raw columns:                   {raw1}")
    print(f"Expected columns:              {expected1}")

    print("\nRun order check:")
    print(f"  Run 1 first pose:            {order1[0]}")
    print(f"  Run 2 first pose:            {order2[0]}")
    print(f"  Orders identical:            {order1 == order2}")
    print(f"  Run 2 is exact reverse:      {order2 == list(reversed(order1))}")

    print("\nCombined differential force matrix A:")
    print(fit_all["matrix"])
    print(f"Rank:                          {fit_all['rank']}")
    print(f"Condition number:              {fit_all['condition']:.6f}")
    print(f"Component RMSE:                {fit_all['rmse_components']:.6f} N")
    print(f"Vector RMSE:                   {fit_all['rmse_vectors']:.6f} N")
    print(f"Mean vector error:             {fit_all['mean_vector_error']:.6f} N")
    print(f"Maximum vector error:          {fit_all['maximum_vector_error']:.6f} N")

    print("\n========== PER-RUN MATRICES ==========")
    for number, result in ((1, fit1), (2, fit2)):
        print(f"\nRun {number}:")
        print(result["matrix"])
        print(f"Rank:                          {result['rank']}")
        print(f"Condition number:              {result['condition']:.6f}")
        print(f"Vector RMSE:                   {result['rmse_vectors']:.6f} N")
        print(f"Maximum vector error:          {result['maximum_vector_error']:.6f} N")

    matrix_difference = fit2["matrix"] - fit1["matrix"]

    print("\nRun 2 matrix minus Run 1 matrix:")
    print(matrix_difference)
    print(
        "Matrix Frobenius difference:   "
        f"{np.linalg.norm(matrix_difference):.6f}"
    )

    by_pose1 = {record["pose"]: record for record in run1}
    by_pose2 = {record["pose"]: record for record in run2}

    repeatability = []
    print("\n========== RUN-TO-RUN RAW-DELTA REPEATABILITY ==========")

    for pose in order1:
        difference = by_pose2[pose]["raw"] - by_pose1[pose]["raw"]
        norm = float(np.linalg.norm(difference))
        repeatability.append(norm)

        print(f"\n{pose}")
        print(f"  Run2 - Run1:                 {difference} N")
        print(f"  Difference norm:             {norm:.6f} N")

    print("\nRepeatability summary:")
    print(f"  Mean difference:             {np.mean(repeatability):.6f} N")
    print(f"  Maximum difference:          {np.max(repeatability):.6f} N")

    print("\n========== LEAVE-ONE-POSE-OUT VALIDATION ==========")
    validation_errors = []

    for pose in order1:
        training = [
            record for record in combined if record["pose"] != pose
        ]
        testing = [
            record for record in combined if record["pose"] == pose
        ]

        result = fit_matrix(training)
        errors = []

        for record in testing:
            predicted = result["matrix"] @ record["raw"]
            error = float(np.linalg.norm(predicted - record["expected"]))
            errors.append(error)
            validation_errors.append(error)

        print(
            f"{pose}: errors={errors}, "
            f"mean={np.mean(errors):.6f} N"
        )

    print("\nCross-validation summary:")
    print(f"  Mean error:                  {np.mean(validation_errors):.6f} N")
    print(
        f"  RMS error:                   "
        f"{np.sqrt(np.mean(np.square(validation_errors))):.6f} N"
    )
    print(f"  Maximum error:               {np.max(validation_errors):.6f} N")

    output = {
        "status": "diagnostic_only_do_not_apply",
        "model": "expected_force_delta = A * bracketed_raw_force_delta",
        "run1_file": str(args.run1),
        "run2_file": str(args.run2),
        "run1_order": order1,
        "run2_order": order2,
        "orders_identical": order1 == order2,
        "run2_is_exact_reverse": order2 == list(reversed(order1)),
        "raw_columns": list(raw1),
        "expected_columns": list(expected1),
        "combined": {
            key: value
            for key, value in fit_all.items()
            if key not in ("predicted", "residual")
        },
        "run1": {
            key: value
            for key, value in fit1.items()
            if key not in ("predicted", "residual")
        },
        "run2": {
            key: value
            for key, value in fit2.items()
            if key not in ("predicted", "residual")
        },
        "run2_minus_run1_matrix": matrix_difference,
        "repeatability_mean_N": float(np.mean(repeatability)),
        "repeatability_max_N": float(np.max(repeatability)),
        "leave_one_pose_out_mean_N": float(np.mean(validation_errors)),
        "leave_one_pose_out_rms_N": float(
            np.sqrt(np.mean(np.square(validation_errors)))
        ),
        "leave_one_pose_out_max_N": float(np.max(validation_errors)),
        "restrictions": {
            "apply_matrix": False,
            "modify_driver": False,
            "modify_6x6_calibration": False,
            "hardware_zero": False,
            "activate_force_controller": False,
            "activate_compliance_controller": False,
        },
    }

    args.output.write_text(
        json.dumps(to_serializable(output), indent=2) + "\n"
    )

    print("\nNo calibration was applied.")
    print(f"Saved: {args.output}")


if __name__ == "__main__":
    main()
