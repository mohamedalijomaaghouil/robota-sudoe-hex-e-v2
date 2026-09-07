#!/usr/bin/env python3
import csv
import json
import sys
from itertools import combinations
from pathlib import Path

import numpy as np

session = Path(sys.argv[1])

candidate_file = session / "hex_e_known_625g_orientation_analysis_2runs.json"
original_file = session / "hex_e_known_1051g_validation_combined_5poses.csv"
repeat_file = session / "hex_e_known_1051g_failing_pose_repeats.csv"

A = np.asarray(
    json.loads(candidate_file.read_text())["combined"]["matrix_A"],
    dtype=float,
)

raw_names = [
    "bracketed_raw_delta_Fx",
    "bracketed_raw_delta_Fy",
    "bracketed_raw_delta_Fz",
]
expected_names = [
    "expected_force_delta_x",
    "expected_force_delta_y",
    "expected_force_delta_z",
]


def vectors(row):
    raw = np.array([float(row[name]) for name in raw_names])
    expected = np.array([float(row[name]) for name in expected_names])
    corrected = A @ raw
    residual = corrected - expected
    return raw, expected, corrected, residual


with original_file.open(newline="") as f:
    original_rows = [
        row for row in csv.DictReader(f)
        if row["pose"] == "w1_m90_w2_m120"
    ]

if len(original_rows) != 1:
    raise RuntimeError(
        f"Expected one original failing record, found {len(original_rows)}"
    )

with repeat_file.open(newline="") as f:
    repeat_rows = list(csv.DictReader(f))

if len(repeat_rows) != 3:
    raise RuntimeError(f"Expected three repeat records, found {len(repeat_rows)}")

print("========== CANDIDATE MATRIX ==========")
print(A)

all_repeat_raw = []
all_repeat_corrected = []
all_repeat_residuals = []
all_repeat_errors = []

print("\n========== ORIGINAL VALIDATION RECORD ==========")
raw, expected, corrected, residual = vectors(original_rows[0])
print("Raw delta:       ", raw, "N")
print("Corrected delta: ", corrected, "N")
print("Expected delta:  ", expected, "N")
print("Residual:        ", residual, "N")
print(f"Error norm:       {np.linalg.norm(residual):.6f} N")

print("\n========== THREE REPEATS ==========")
for index, row in enumerate(repeat_rows, start=1):
    raw, expected, corrected, residual = vectors(row)
    error = float(np.linalg.norm(residual))

    all_repeat_raw.append(raw)
    all_repeat_corrected.append(corrected)
    all_repeat_residuals.append(residual)
    all_repeat_errors.append(error)

    print(f"\nTrial {index}: {row['pose']}")
    print("  Raw delta:       ", raw, "N")
    print("  Corrected delta: ", corrected, "N")
    print("  Expected delta:  ", expected, "N")
    print("  Residual:        ", residual, "N")
    print(f"  Error norm:       {error:.6f} N")

raw_array = np.asarray(all_repeat_raw)
corrected_array = np.asarray(all_repeat_corrected)
residual_array = np.asarray(all_repeat_residuals)
error_array = np.asarray(all_repeat_errors)

print("\n========== REPEAT SUMMARY ==========")
print("Mean raw delta:             ", raw_array.mean(axis=0), "N")
print("Raw-delta sample std:       ", raw_array.std(axis=0, ddof=1), "N")
print("Mean corrected delta:       ", corrected_array.mean(axis=0), "N")
print("Mean corrected residual:    ", residual_array.mean(axis=0), "N")
print("Residual sample std:        ", residual_array.std(axis=0, ddof=1), "N")
print(f"Mean corrected error:        {error_array.mean():.6f} N")
print(f"RMS corrected error:         {np.sqrt(np.mean(error_array**2)):.6f} N")
print(f"Maximum corrected error:     {error_array.max():.6f} N")

print("\nRaw-delta pairwise differences:")
for first, second in combinations(range(len(raw_array)), 2):
    difference = raw_array[second] - raw_array[first]
    print(
        f"  Trial {second + 1} - Trial {first + 1}: "
        f"{difference} N; norm={np.linalg.norm(difference):.6f} N"
    )

print("\nNo calibration was applied.")
