#!/usr/bin/env python3

import argparse
import csv
import json
from pathlib import Path

import numpy as np


MASS_KG = 1.890


def load_files(paths):
    records = []

    for path in paths:
        with path.open(newline="") as source:
            for row in csv.DictReader(source):
                records.append({
                    "file": path.name,
                    "pose": row["pose"],
                    "raw_force": np.array([
                        float(row["Fx_mean"]),
                        float(row["Fy_mean"]),
                        float(row["Fz_mean"]),
                    ]),
                    "raw_torque": np.array([
                        float(row["Tx_mean"]),
                        float(row["Ty_mean"]),
                        float(row["Tz_mean"]),
                    ]),
                    "gravity": np.array([
                        float(row["gravity_tool_x"]),
                        float(row["gravity_tool_y"]),
                        float(row["gravity_tool_z"]),
                    ]),
                })

    if not records:
        raise RuntimeError("No records loaded")

    return records


def fit_force_affine(raw_force, expected_force):
    center = raw_force.mean(axis=0)
    centered = raw_force - center

    design = np.column_stack([
        centered,
        np.ones(len(centered)),
    ])

    coefficients, _, rank, singular_values = np.linalg.lstsq(
        design,
        expected_force,
        rcond=None,
    )

    # expected = centered @ A.T + intercept
    A = coefficients[:3, :].T
    intercept_centered = coefficients[3, :]

    # expected = A @ raw + c
    c = intercept_centered - A @ center

    predicted = raw_force @ A.T + c
    residual = predicted - expected_force

    return {
        "A": A,
        "c": c,
        "predicted": predicted,
        "residual": residual,
        "rank": int(rank),
        "singular_values": singular_values,
        "center": center,
        "condition_number": float(np.linalg.cond(design)),
    }


def leave_one_out(raw_force, expected_force):
    errors = []

    for omitted in range(len(raw_force)):
        keep = np.arange(len(raw_force)) != omitted

        fit = fit_force_affine(
            raw_force[keep],
            expected_force[keep],
        )

        prediction = (
            fit["A"] @ raw_force[omitted] + fit["c"]
        )

        error = prediction - expected_force[omitted]
        errors.append(error)

    return np.asarray(errors)


def fit_torque_diagnostic(gravity, raw_torque):
    # Diagnostic model only:
    # raw_torque = K * gravity + bias
    design = np.column_stack([
        gravity,
        np.ones(len(gravity)),
    ])

    coefficients, _, rank, singular = np.linalg.lstsq(
        design,
        raw_torque,
        rcond=None,
    )

    K = coefficients[:3, :].T
    bias = coefficients[3, :]

    predicted = gravity @ K.T + bias
    residual = predicted - raw_torque

    skew_part = 0.5 * (K - K.T)
    symmetric_part = 0.5 * (K + K.T)

    # If K were exactly skew(h), this would recover h = mass * CoG.
    h = np.array([
        skew_part[2, 1],
        skew_part[0, 2],
        skew_part[1, 0],
    ])

    diagnostic_cog = h / MASS_KG

    return {
        "K": K,
        "bias": bias,
        "predicted": predicted,
        "residual": residual,
        "skew_part": skew_part,
        "symmetric_part": symmetric_part,
        "diagnostic_cog": diagnostic_cog,
        "rank": int(rank),
        "singular_values": singular,
    }


def native(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("files", nargs="+", type=Path)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path.home() / "hex_e_orientation_analysis.json",
    )
    args = parser.parse_args()

    records = load_files(args.files)

    raw_force = np.vstack([r["raw_force"] for r in records])
    raw_torque = np.vstack([r["raw_torque"] for r in records])
    gravity = np.vstack([r["gravity"] for r in records])

    expected_force = MASS_KG * gravity

    force_fit = fit_force_affine(raw_force, expected_force)
    force_error_norm = np.linalg.norm(
        force_fit["residual"],
        axis=1,
    )

    loo_error = leave_one_out(raw_force, expected_force)
    loo_error_norm = np.linalg.norm(loo_error, axis=1)

    try:
        raw_zero_offset = -np.linalg.solve(
            force_fit["A"],
            force_fit["c"],
        )
    except np.linalg.LinAlgError:
        raw_zero_offset = np.full(3, np.nan)

    torque_fit = fit_torque_diagnostic(
        gravity,
        raw_torque,
    )

    torque_error_norm = np.linalg.norm(
        torque_fit["residual"],
        axis=1,
    )

    print("========== MULTI-ORIENTATION FORCE ANALYSIS ==========")
    print(f"Input records:                  {len(records)}")
    print(f"Assumed downstream mass:        {MASS_KG:.6f} kg")
    print(f"Design rank:                    {force_fit['rank']}")
    print(
        "Design condition number:        "
        f"{force_fit['condition_number']:.6f}"
    )

    print("\nForce matrix A, tool-aligned output:")
    print(force_fit["A"])

    print("\nForce affine offset c [N]:")
    print(force_fit["c"])

    print("\nEquivalent raw zero offset:")
    print(raw_zero_offset)

    print(
        "\nTraining force RMSE:             "
        f"{np.sqrt(np.mean(force_error_norm**2)):.6f} N"
    )
    print(
        "Training force maximum error:    "
        f"{force_error_norm.max():.6f} N"
    )
    print(
        "Leave-one-out force RMSE:        "
        f"{np.sqrt(np.mean(loo_error_norm**2)):.6f} N"
    )
    print(
        "Leave-one-out maximum error:     "
        f"{loo_error_norm.max():.6f} N"
    )

    print("\nTorque orientation matrix K [kg*m equivalent if valid]:")
    print(torque_fit["K"])

    print("\nTorque bias [Nm]:")
    print(torque_fit["bias"])

    print("\nDiagnostic CoG from skew part [m]:")
    print(torque_fit["diagnostic_cog"])

    print(
        "\nTorque-model RMSE:               "
        f"{np.sqrt(np.mean(torque_error_norm**2)):.6f} Nm"
    )

    result = {
        "status": "diagnostic_only_do_not_apply_yet",
        "input_files": [str(p) for p in args.files],
        "input_records": len(records),
        "assumed_downstream_mass_kg": MASS_KG,
        "output_coordinate_convention": (
            "tool0-aligned axes at the external sensor center"
        ),
        "force_model": "F_tool = A * F_raw + c",
        "force_matrix_A": force_fit["A"],
        "force_affine_offset_c_N": force_fit["c"],
        "equivalent_raw_zero_offset": raw_zero_offset,
        "design_rank": force_fit["rank"],
        "design_singular_values": force_fit["singular_values"],
        "design_condition_number": force_fit["condition_number"],
        "training_force_rmse_N": float(
            np.sqrt(np.mean(force_error_norm**2))
        ),
        "training_force_max_error_N": float(
            force_error_norm.max()
        ),
        "leave_one_out_force_rmse_N": float(
            np.sqrt(np.mean(loo_error_norm**2))
        ),
        "leave_one_out_force_max_error_N": float(
            loo_error_norm.max()
        ),
        "torque_status": (
            "diagnostic only; torque calibration and sensor-origin "
            "geometry are not yet validated"
        ),
        "torque_orientation_matrix_K": torque_fit["K"],
        "torque_bias_Nm": torque_fit["bias"],
        "diagnostic_cog_m": torque_fit["diagnostic_cog"],
        "torque_model_rmse_Nm": float(
            np.sqrt(np.mean(torque_error_norm**2))
        ),
        "restrictions": {
            "apply_to_driver": False,
            "write_6x6_matrix": False,
            "hardware_zero": False,
            "activate_force_controller": False,
            "activate_compliance_controller": False,
        },
    }

    args.output.write_text(
        json.dumps(result, indent=2, default=native) + "\n"
    )

    text_output = args.output.with_suffix(".txt")
    text_output.write_text(
        "MULTI-ORIENTATION CALIBRATION CANDIDATE\n"
        "=======================================\n"
        f"Input records: {len(records)}\n"
        f"Assumed mass: {MASS_KG:.6f} kg\n"
        f"Design rank: {force_fit['rank']}\n"
        f"Condition number: "
        f"{force_fit['condition_number']:.6f}\n"
        f"Force matrix A:\n{force_fit['A']}\n"
        f"Force offset c:\n{force_fit['c']}\n"
        f"Equivalent raw zero offset:\n{raw_zero_offset}\n"
        f"Training RMSE: "
        f"{np.sqrt(np.mean(force_error_norm**2)):.6f} N\n"
        f"LOO RMSE: "
        f"{np.sqrt(np.mean(loo_error_norm**2)):.6f} N\n"
        f"LOO maximum: {loo_error_norm.max():.6f} N\n"
        f"Diagnostic CoG:\n{torque_fit['diagnostic_cog']}\n"
        f"Torque-model RMSE: "
        f"{np.sqrt(np.mean(torque_error_norm**2)):.6f} Nm\n"
        "\nDO NOT APPLY THIS CALIBRATION YET.\n"
    )

    print(f"\nSaved JSON: {args.output}")
    print(f"Saved text: {text_output}")


if __name__ == "__main__":
    main()
