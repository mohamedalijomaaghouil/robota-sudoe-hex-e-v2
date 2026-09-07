#!/usr/bin/env python3
import csv
import time
import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped

TOPIC = "/bus0/ft_sensor0/ft_sensor_readings/wrench"
N = 100                       # 10 seconds at approximately 10 Hz
EXPECTED = 0.418 * 9.80665    # 4.09917970 N

samples = []
collecting = False
rows = []

rclpy.init()
node = rclpy.create_node("hex_e_repeatability_test")

def callback(msg):
    if collecting:
        samples.append(np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ], dtype=float))

sub = node.create_subscription(WrenchStamped, TOPIC, callback, 10)

def spin_for(seconds):
    deadline = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

def capture(label):
    global samples, collecting

    # Flush messages accumulated while waiting at the prompt.
    collecting = False
    spin_for(1.0)

    samples = []
    collecting = True
    deadline = time.monotonic() + 15.0

    while rclpy.ok() and len(samples) < N and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    collecting = False

    if len(samples) < 80:
        raise RuntimeError(
            f"{label}: received only {len(samples)} samples; expected at least 80"
        )

    x = np.vstack(samples)
    mean = x.mean(axis=0)
    std = x.std(axis=0, ddof=1)

    rows.append([label, len(x), *mean, *std])

    print(
        f"{label}: n={len(x)}, "
        f"F=[{mean[0]:.4f}, {mean[1]:.4f}, {mean[2]:.4f}] N, "
        f"T=[{mean[3]:.4f}, {mean[4]:.4f}, {mean[5]:.4f}] Nm"
    )
    return mean

try:
    input(
        "\nRobot stationary and 418 g object REMOVED.\n"
        "Wait until everything is still, then press Enter: "
    )
    unloaded_previous = capture("U0")

    deltas = []

    for trial in range(1, 6):
        input(
            f"\nTrial {trial}: attach the 418 g object, let it hang freely, "
            "wait until completely still, then press Enter: "
        )
        loaded = capture(f"L{trial}")

        input(
            f"Trial {trial}: remove the object without moving the robot, "
            "wait until completely still, then press Enter: "
        )
        unloaded_next = capture(f"U{trial}")

        baseline = 0.5 * (unloaded_previous + unloaded_next)
        delta = loaded - baseline
        deltas.append(delta)

        force_magnitude = np.linalg.norm(delta[:3])
        torque_magnitude = np.linalg.norm(delta[3:])

        print(
            f"Trial {trial} delta: "
            f"dF=[{delta[0]:.4f}, {delta[1]:.4f}, {delta[2]:.4f}] N, "
            f"|dF|={force_magnitude:.4f} N, "
            f"|dT|={torque_magnitude:.4f} Nm"
        )

        unloaded_previous = unloaded_next

    deltas = np.vstack(deltas)
    magnitudes = np.linalg.norm(deltas[:, :3], axis=1)

    print("\n========== SUMMARY ==========")
    for i, magnitude in enumerate(magnitudes, 1):
        print(f"Trial {i}: {magnitude:.6f} N")

    print(f"Expected for 418 g:       {EXPECTED:.6f} N")
    print(f"Measured mean:            {magnitudes.mean():.6f} N")
    print(f"Between-trial std dev:    {magnitudes.std(ddof=1):.6f} N")
    print(f"Measured/expected ratio:  {magnitudes.mean()/EXPECTED:.6f}")
    print(f"Diagnostic correction:    {EXPECTED/magnitudes.mean():.6f}")
    print("Do not apply the diagnostic correction yet.")

    output = "/home/kairos/hex_e_repeatability.csv"
    header = [
        "label", "samples",
        "Fx_mean", "Fy_mean", "Fz_mean",
        "Tx_mean", "Ty_mean", "Tz_mean",
        "Fx_std", "Fy_std", "Fz_std",
        "Tx_std", "Ty_std", "Tz_std",
    ]

    with open(output, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)

    print(f"Saved phase statistics to {output}")

finally:
    node.destroy_node()
    rclpy.shutdown()
