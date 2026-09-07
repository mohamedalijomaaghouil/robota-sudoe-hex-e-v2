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

MASS_KG = 1.051
SAMPLE_COUNT = 100
SETTLING_SECONDS = 10.0

REFERENCE = np.array([
    0.0,
    -np.pi / 2,
    np.pi / 2,
    -np.pi / 2,
    -np.pi / 2,
    -np.pi / 2,
], dtype=float)

# Wrist angles are in degrees. All other joints remain at REFERENCE.
POSES = list((
    ("reference",        -90.0,  -90.0),
    ("w1_m60_w2_m90",   -60.0,  -90.0),
    ("w1_m120_w2_m90", -120.0,  -90.0),
    ("w1_m90_w2_m60",   -90.0,  -60.0),
    ("w1_m90_w2_m120",  -90.0, -120.0),
))


spec = importlib.util.spec_from_file_location(
    "orientation_base", BASE_SCRIPT
)
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
Collector = base.Collector


def target_for(wrist1_deg, wrist2_deg):
    target = REFERENCE.copy()
    target[3] = np.deg2rad(wrist1_deg)
    target[4] = np.deg2rad(wrist2_deg)
    return target


def spin_for(collector, duration):
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(collector.node, timeout_sec=0.1)


def capture_wrench(collector):
    mean, std, sample_count = collector.capture(
        number=SAMPLE_COUNT,
        timeout=30.0,
    )
    if sample_count != SAMPLE_COUNT:
        raise RuntimeError(
            f"Expected {SAMPLE_COUNT} samples, received {sample_count}"
        )
    return np.asarray(mean, dtype=float), np.asarray(std, dtype=float)


def vector_fields(prefix, vector):
    labels = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
    return {
        f"{prefix}_{label}": float(value)
        for label, value in zip(labels, vector)
    }


def force_fields(prefix, vector):
    labels = ("x", "y", "z")
    return {
        f"{prefix}_{label}": float(value)
        for label, value in zip(labels, vector)
    }


def write_rows(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)

    if not rows:
        return

    temporary = path.with_suffix(path.suffix + ".tmp")

    with temporary.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    temporary.replace(path)


def print_plan(sequence):
    print("\nPlanned known-mass orientations:")
    for index, (name, wrist1, wrist2) in enumerate(sequence, start=1):
        print(
            f"  {index:2d}. {name}: "
            f"Wrist1={wrist1:.1f} deg, Wrist2={wrist2:.1f} deg"
        )

    print("\nAt every orientation:")
    print("  1. Confirm the complete 1051 g assembly is removed.")
    print("  2. Record unloaded_before.")
    print("  3. Attach the complete assembly by the same clip.")
    print("  4. Record loaded.")
    print("  5. Remove the complete assembly.")
    print("  6. Record unloaded_after.")
    print("\nThe robot moves only while the assembly is removed.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--reverse", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    sequence = list(reversed(POSES)) if args.reverse else list(POSES)

    if args.execute and args.output is None:
        parser.error("--output is required with --execute")

    rclpy.init()
    collector = Collector()

    try:
        collector.spin_until_ready(timeout=15.0)
        spin_for(collector, 1.0)

        current = np.asarray(collector.latest_joint, dtype=float)
        reference_error = float(np.max(np.abs(current - REFERENCE)))

        gravity_result = collector.gravity_in_tool0()
        gravity = np.asarray(gravity_result[0], dtype=float)
        quaternion = np.asarray(gravity_result[1], dtype=float)

        print(f"Current joints: {current}")
        print(f"Reference joints: {REFERENCE}")
        print(f"Maximum reference error: {reference_error:.6f} rad")
        print(f"Gravity in tool0: {gravity}")
        print(f"base->tool0 quaternion: {quaternion}")

        print_plan(sequence)

        if not args.execute:
            print("\nCHECK ONLY: no movement was commanded.")
            return

        if reference_error > 0.02:
            raise RuntimeError(
                "Robot is not sufficiently close to the reference pose. "
                f"Maximum error is {reference_error:.6f} rad."
            )

        rows = []

        print("\nExecution starts in 5 seconds...")
        spin_for(collector, 5.0)

        for index, (name, wrist1_deg, wrist2_deg) in enumerate(
            sequence, start=1
        ):
            target = target_for(wrist1_deg, wrist2_deg)

            print(
                f"\n[{index}/{len(sequence)}] {name}: "
                f"Wrist1={wrist1_deg:.1f}, Wrist2={wrist2_deg:.1f}"
            )

            input(
                "\nConfirm the complete 1051 g assembly is REMOVED. "
                "Press Enter to permit movement to this pose: "
            )

            collector.move(target)
            print(
                f"Trajectory succeeded; settling for "
                f"{SETTLING_SECONDS:.0f} seconds..."
            )
            spin_for(collector, SETTLING_SECONDS)

            spin_for(collector, 1.0)
            actual_joint = np.asarray(
                collector.latest_joint, dtype=float
            )

            gravity_result = collector.gravity_in_tool0()
            gravity = np.asarray(gravity_result[0], dtype=float)
            quaternion = np.asarray(gravity_result[1], dtype=float)

            input(
                "\nAssembly must still be REMOVED. "
                "Do not touch or move the robot. "
                "Press Enter to record unloaded_before: "
            )
            before_mean, before_std = capture_wrench(collector)
            print(f"  unloaded_before mean: {before_mean}")
            print(f"  unloaded_before std:  {before_std}")

            input(
                "\nAttach the COMPLETE 1051 g assembly using the same clip. "
                "Let it hang freely without touching anything. "
                "Press Enter when settled to record loaded: "
            )
            loaded_mean, loaded_std = capture_wrench(collector)
            print(f"  loaded mean:           {loaded_mean}")
            print(f"  loaded std:            {loaded_std}")

            input(
                "\nRemove the COMPLETE 1051 g assembly. "
                "Do not touch or move the robot afterward. "
                "Press Enter when settled to record unloaded_after: "
            )
            after_mean, after_std = capture_wrench(collector)
            print(f"  unloaded_after mean:   {after_mean}")
            print(f"  unloaded_after std:    {after_std}")

            reference_mean = 0.5 * (before_mean + after_mean)
            raw_delta = loaded_mean - reference_mean

            expected_force_delta = MASS_KG * gravity
            raw_force_delta = raw_delta[:3]
            raw_torque_delta = raw_delta[3:]

            row = {
                "pose": name,
                "wrist1_deg": wrist1_deg,
                "wrist2_deg": wrist2_deg,
                "mass_kg": MASS_KG,
                "sample_count": SAMPLE_COUNT,
            }

            for joint_name, value in zip(
                (
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ),
                actual_joint,
            ):
                row[f"actual_{joint_name}"] = float(value)

            row.update(vector_fields("unloaded_before_mean", before_mean))
            row.update(vector_fields("unloaded_before_std", before_std))
            row.update(vector_fields("loaded_mean", loaded_mean))
            row.update(vector_fields("loaded_std", loaded_std))
            row.update(vector_fields("unloaded_after_mean", after_mean))
            row.update(vector_fields("unloaded_after_std", after_std))
            row.update(vector_fields("bracketed_raw_delta", raw_delta))

            row.update(force_fields("gravity_tool", gravity))
            row.update(force_fields(
                "expected_force_delta", expected_force_delta
            ))

            row.update({
                "base_tool0_qx": float(quaternion[0]),
                "base_tool0_qy": float(quaternion[1]),
                "base_tool0_qz": float(quaternion[2]),
                "base_tool0_qw": float(quaternion[3]),
                "raw_force_delta_norm": float(
                    np.linalg.norm(raw_force_delta)
                ),
                "expected_force_delta_norm": float(
                    np.linalg.norm(expected_force_delta)
                ),
                "raw_torque_delta_norm": float(
                    np.linalg.norm(raw_torque_delta)
                ),
            })

            rows.append(row)
            write_rows(args.output, rows)

            print(f"  Bracketed raw force delta: {raw_force_delta} N")
            print(
                f"  Expected force delta:      "
                f"{expected_force_delta} N"
            )
            print(
                f"  Raw force magnitude ratio: "
                f"{np.linalg.norm(raw_force_delta) / np.linalg.norm(expected_force_delta):.6f}"
            )
            print(f"  Saved: {args.output}")

        input(
            "\nAll measurements are complete. Confirm the 1051 g assembly "
            "is REMOVED, then press Enter to return to reference: "
        )

        collector.move(REFERENCE)
        spin_for(collector, SETTLING_SECONDS)

        print("\nKnown-mass orientation collection completed.")
        print(f"Final file: {args.output}")

    finally:
        collector.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
