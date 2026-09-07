#!/usr/bin/env python3

import csv
import importlib.util
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


SAMPLE_COUNT = 600
OUTPUT_TOPIC = "/hex_e/compensated_wrench"
ALPHAS = [0.10, 0.20, 0.30, 0.50]


class Recorder(Node):
    def __init__(self):
        super().__init__("hex_e_filter_characterization_recorder")
        self.samples = []
        self.times = []

        self.subscription = self.create_subscription(
            WrenchStamped,
            OUTPUT_TOPIC,
            self.callback,
            10,
        )

    def callback(self, message):
        force = np.asarray(
            (
                message.wrench.force.x,
                message.wrench.force.y,
                message.wrench.force.z,
            ),
            dtype=float,
        )

        if np.all(np.isfinite(force)):
            self.samples.append(force)
            self.times.append(time.monotonic())


def ema(data, alpha):
    result = np.empty_like(data)
    result[0] = data[0]

    for index in range(1, len(data)):
        result[index] = (
            alpha * data[index]
            + (1.0 - alpha) * result[index - 1]
        )

    return result


def report(name, data):
    norms = np.linalg.norm(data, axis=1)

    print()
    print(name)
    print(f"  Samples:              {len(data)}")
    print(f"  Mean force:           {np.mean(data, axis=0)} N")
    print(
        "  Component std:       "
        f"{np.std(data, axis=0, ddof=1)} N"
    )
    print(f"  RMS force norm:       {np.sqrt(np.mean(norms**2)):.6f} N")
    print(f"  Median norm:          {np.median(norms):.6f} N")
    print(f"  95th percentile norm: {np.percentile(norms, 95):.6f} N")
    print(f"  99th percentile norm: {np.percentile(norms, 99):.6f} N")
    print(f"  Maximum norm:         {np.max(norms):.6f} N")


def main():
    if len(sys.argv) != 2:
        raise SystemExit(
            "Usage: hex_e_characterize_compensation_filters.py SESSION"
        )

    session = Path(sys.argv[1])
    diagnostic_path = Path.home() / "hex_e_compensation_diagnostic.py"

    specification = importlib.util.spec_from_file_location(
        "hex_e_compensation_diagnostic",
        diagnostic_path,
    )

    if specification is None or specification.loader is None:
        raise RuntimeError(
            f"Could not load {diagnostic_path}"
        )

    diagnostic_module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(diagnostic_module)

    rclpy.init()

    compensation = diagnostic_module.HexECompensationDiagnostic()
    recorder = Recorder()

    executor = SingleThreadedExecutor()
    executor.add_node(compensation)
    executor.add_node(recorder)

    print("Keep the complete tool assembly unloaded.")
    print("Do not touch or move the robot.")
    print("The first approximately five seconds are used for tare.")
    print(
        f"After tare, {SAMPLE_COUNT} compensated samples "
        "will be collected."
    )
    print("No robot-motion commands are published.")

    deadline = time.monotonic() + 90.0

    try:
        while (
            rclpy.ok()
            and len(recorder.samples) < SAMPLE_COUNT
            and time.monotonic() < deadline
        ):
            executor.spin_once(timeout_sec=0.2)

            count = len(recorder.samples)
            if count and count % 100 == 0:
                print(
                    f"Collected {count}/{SAMPLE_COUNT} samples",
                    flush=True,
                )

    finally:
        executor.remove_node(recorder)
        executor.remove_node(compensation)
        recorder.destroy_node()
        compensation.destroy_node()
        rclpy.shutdown()

    if len(recorder.samples) < SAMPLE_COUNT:
        raise RuntimeError(
            "Timed out before collecting the requested number of samples: "
            f"{len(recorder.samples)}/{SAMPLE_COUNT}"
        )

    data = np.vstack(recorder.samples[:SAMPLE_COUNT])
    timestamps = np.asarray(recorder.times[:SAMPLE_COUNT])

    intervals = np.diff(timestamps)
    median_dt = float(np.median(intervals))
    measured_rate = 1.0 / median_dt

    csv_path = (
        session / "hex_e_compensation_filter_characterization_samples.csv"
    )

    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow((
            "elapsed_s",
            "unfiltered_Fx",
            "unfiltered_Fy",
            "unfiltered_Fz",
        ))

        initial_time = next(iter(timestamps))

        for stamp, force in zip(timestamps, data):
            fx, fy, fz = force
            writer.writerow((
                stamp - initial_time,
                fx,
                fy,
                fz,
            ))

    print()
    print("========== FILTER CHARACTERIZATION ==========")
    print(f"Median sample interval: {median_dt:.6f} s")
    print(f"Measured sample rate:   {measured_rate:.6f} Hz")

    report("UNFILTERED", data)

    results = {}

    for alpha in ALPHAS:
        filtered = ema(data, alpha)

        # Exclude the initial five seconds so that initialization does not
        # artificially improve or worsen a filter's statistics.
        discard = min(50, len(filtered) // 4)
        evaluated = filtered[discard:]

        cutoff = (
            -math.log(1.0 - alpha)
            / (2.0 * math.pi * median_dt)
        )

        results[alpha] = evaluated

        report(
            (
                f"EMA alpha={alpha:.2f}, "
                f"approximate cutoff={cutoff:.3f} Hz"
            ),
            evaluated,
        )

    print()
    print(f"Saved full-rate unfiltered samples: {csv_path}")
    print("No robot motion was commanded.")


if __name__ == "__main__":
    main()
