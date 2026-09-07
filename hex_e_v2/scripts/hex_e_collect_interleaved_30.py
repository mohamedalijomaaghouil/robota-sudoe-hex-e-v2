#!/usr/bin/env python3
import argparse
import csv
import importlib.util
import time
from pathlib import Path

import numpy as np
import rclpy

HOME = Path.home()
BASE_SCRIPT = HOME / "hex_e_collect_orientations.py"
MASS = 1.890

spec = importlib.util.spec_from_file_location(
    "orientation_base", BASE_SCRIPT
)
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)

REFERENCE = np.array([
    0.0,
    -np.pi / 2,
    np.pi / 2,
    -np.pi / 2,
    -np.pi / 2,
    -np.pi / 2,
])

POSES = [
    ("w1_m60_w2_m90",  -60.0,  -90.0),
    ("w1_m120_w2_m90", -120.0, -90.0),
    ("w1_m90_w2_m60",  -90.0,  -60.0),
    ("w1_m90_w2_m120", -90.0,  -120.0),
    ("w1_m60_w2_m60",  -60.0,  -60.0),
    ("w1_m120_w2_m60", -120.0, -60.0),
    ("w1_m120_w2_m120", -120.0, -120.0),
    ("w1_m60_w2_m120", -60.0, -120.0),
]

def spin_for(collector, duration):
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(collector.node, timeout_sec=0.1)

def target_joints(wrist1_deg, wrist2_deg):
    q = REFERENCE.copy()
    q[3] = np.deg2rad(wrist1_deg)
    q[4] = np.deg2rad(wrist2_deg)
    return q

def add_capture(row, prefix, collector):
    mean, std, count = collector.capture()
    gravity, _ = collector.gravity_in_tool0()

    labels = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
    for label, value in zip(labels, mean):
        row[f"{prefix}_{label}_mean"] = float(value)
    for label, value in zip(labels, std):
        row[f"{prefix}_{label}_std"] = float(value)

    row[f"{prefix}_sample_count"] = int(count)
    row[f"{prefix}_gravity_x"] = float(gravity[0])
    row[f"{prefix}_gravity_y"] = float(gravity[1])
    row[f"{prefix}_gravity_z"] = float(gravity[2])

    print(f"  {prefix} wrench:  {mean}")
    print(f"  {prefix} gravity: {gravity}")

    return mean, gravity

def save_rows(path, rows):
    if not rows:
        return

    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--reverse", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    sequence = list(reversed(POSES)) if args.reverse else POSES

    if args.output is None:
        args.output = HOME / "hex_e_interleaved_30.csv"

    rclpy.init()
    collector = base.Collector()

    try:
        collector.spin_until_ready()

        current = collector.latest_joint.copy()
        error = np.max(np.abs(current - REFERENCE))

        print(f"Current joints: {current}")
        print(f"Reference joints: {REFERENCE}")
        print(f"Maximum reference error: {error:.6f} rad")
        print(f"Gravity in tool0: {collector.gravity_in_tool0()}")

        if error > np.deg2rad(2.0):
            raise RuntimeError(
                "Robot is not sufficiently close to the reference pose."
            )

        if not args.execute:
            print("\nCHECK ONLY: no movement was commanded.")
            print("Planned interleaved poses:")
            for name, w1, w2 in sequence:
                print(f"  reference -> {name} ({w1:.1f}, {w2:.1f}) -> reference")
            return

        print("\nExecution starts in 5 seconds...")
        spin_for(collector, 5.0)

        rows = []

        for index, (name, w1_deg, w2_deg) in enumerate(sequence, start=1):
            print(
                f"\n[{index}/{len(sequence)}] {name}: "
                f"Wrist1={w1_deg:.1f}, Wrist2={w2_deg:.1f}"
            )

            print("  Settling at reference...")
            spin_for(collector, 10.0)

            row = {
                "pose": name,
                "wrist1_deg": w1_deg,
                "wrist2_deg": w2_deg,
                "mass_kg": MASS,
            }

            before_force, before_gravity = add_capture(
                row, "reference_before", collector
            )

            print("  Moving to test orientation...")
            collector.move(target_joints(w1_deg, w2_deg))
            spin_for(collector, 10.0)

            target_force, target_gravity = add_capture(
                row, "target", collector
            )

            print("  Returning to reference...")
            collector.move(REFERENCE)
            spin_for(collector, 10.0)

            after_force, after_gravity = add_capture(
                row, "reference_after", collector
            )

            bracket_force = 0.5 * (before_force + after_force)
            bracket_gravity = 0.5 * (before_gravity + after_gravity)

            raw_delta = target_force[:3] - bracket_force[:3]
            expected_delta = MASS * (target_gravity - bracket_gravity)

            for axis, value in zip("xyz", raw_delta):
                row[f"raw_force_delta_{axis}"] = float(value)

            for axis, value in zip("xyz", expected_delta):
                row[f"expected_force_delta_{axis}"] = float(value)

            row["raw_force_delta_norm"] = float(
                np.linalg.norm(raw_delta)
            )
            row["expected_force_delta_norm"] = float(
                np.linalg.norm(expected_delta)
            )

            print(f"  Bracketed raw delta: {raw_delta} N")
            print(f"  Expected delta:      {expected_delta} N")

            rows.append(row)
            save_rows(args.output, rows)
            print(f"  Saved: {args.output}")

        print("\nInterleaved ±30-degree collection completed.")
        print(f"Final file: {args.output}")

    finally:
        collector.node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
