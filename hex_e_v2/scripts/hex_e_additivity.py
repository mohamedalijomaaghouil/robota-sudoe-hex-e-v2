import csv
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped

TOPIC = "/bus0/ft_sensor0/ft_sensor_readings/wrench"
SAMPLES_PER_PHASE = 100
TRIALS = 5

MASS_A = 1.331
MASS_B = 0.418
G = 9.80665

EXPECTED_A = MASS_A * G
EXPECTED_B = MASS_B * G
EXPECTED_AB = (MASS_A + MASS_B) * G

samples = []
rows = []

rclpy.init()
node = rclpy.create_node("hex_e_additivity_test")


def callback(msg):
    samples.append([
        msg.wrench.force.x,
        msg.wrench.force.y,
        msg.wrench.force.z,
        msg.wrench.torque.x,
        msg.wrench.torque.y,
        msg.wrench.torque.z,
    ])


subscription = node.create_subscription(
    WrenchStamped, TOPIC, callback, 50
)


def capture(label):
    samples.clear()
    deadline = time.monotonic() + 20.0

    while rclpy.ok() and len(samples) < SAMPLES_PER_PHASE:
        if time.monotonic() >= deadline:
            break
        rclpy.spin_once(node, timeout_sec=0.2)

    if len(samples) < 80:
        raise RuntimeError(
            f"{label}: received only {len(samples)} samples"
        )

    data = np.asarray(samples, dtype=float)
    mean = data.mean(axis=0)
    std = data.std(axis=0)

    print(
        f"{label}: n={len(data)}, "
        f"F=[{mean[0]:.4f}, {mean[1]:.4f}, {mean[2]:.4f}] N, "
        f"T=[{mean[3]:.4f}, {mean[4]:.4f}, {mean[5]:.4f}] Nm"
    )

    rows.append([label, len(data), *mean, *std])
    return mean


try:
    input(
        "\nRobot stationary; both added objects REMOVED.\n"
        "Wait until still, then press Enter: "
    )
    previous_u = capture("U0")

    results = []

    for trial in range(1, TRIALS + 1):
        input(
            f"\nTrial {trial}: place only the 1331 g object on the handler, "
            "wait until still, then press Enter: "
        )
        a_before = capture(f"A{trial}_before")

        input(
            f"Trial {trial}: add the 418 g object without removing the "
            "1331 g object, wait until still, then press Enter: "
        )
        ab = capture(f"AB{trial}")

        input(
            f"Trial {trial}: remove only the 418 g object, leaving the "
            "1331 g object in place; wait until still, then press Enter: "
        )
        a_after = capture(f"A{trial}_after")

        input(
            f"Trial {trial}: remove the 1331 g object too, wait until still, "
            "then press Enter: "
        )
        current_u = capture(f"U{trial}")

        u_reference = 0.5 * (previous_u + current_u)
        a_reference = 0.5 * (a_before + a_after)

        d_a = a_reference[:3] - u_reference[:3]
        d_b = ab[:3] - a_reference[:3]
        d_ab = ab[:3] - u_reference[:3]

        mag_a = np.linalg.norm(d_a)
        mag_b = np.linalg.norm(d_b)
        mag_ab = np.linalg.norm(d_ab)

        results.append([trial, mag_a, mag_b, mag_ab])

        print(
            f"Trial {trial} results:\n"
            f"  1331 g response:             {mag_a:.6f} N\n"
            f"  Additional 418 g response:   {mag_b:.6f} N\n"
            f"  Combined 1749 g response:    {mag_ab:.6f} N"
        )

        previous_u = current_u

    results = np.asarray(results, dtype=float)

    print("\n========== ADDITIVITY SUMMARY ==========")
    print(f"Expected 1331 g:             {EXPECTED_A:.6f} N")
    print(f"Measured 1331 g mean:        {results[:,1].mean():.6f} N")
    print(f"1331 g ratio:                {results[:,1].mean()/EXPECTED_A:.6f}")
    print()
    print(f"Expected added 418 g:        {EXPECTED_B:.6f} N")
    print(f"Measured added 418 g mean:   {results[:,2].mean():.6f} N")
    print(f"Added 418 g ratio:           {results[:,2].mean()/EXPECTED_B:.6f}")
    print()
    print(f"Expected combined 1749 g:    {EXPECTED_AB:.6f} N")
    print(f"Measured combined mean:      {results[:,3].mean():.6f} N")
    print(f"Combined ratio:              {results[:,3].mean()/EXPECTED_AB:.6f}")

    output = Path.home() / "hex_e_additivity.csv"
    with output.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([
            "label", "samples",
            "Fx_mean", "Fy_mean", "Fz_mean",
            "Tx_mean", "Ty_mean", "Tz_mean",
            "Fx_std", "Fy_std", "Fz_std",
            "Tx_std", "Ty_std", "Tz_std",
        ])
        writer.writerows(rows)

        writer.writerow([])
        writer.writerow([
            "trial",
            "measured_1331_N",
            "measured_incremental_418_N",
            "measured_combined_1749_N",
        ])
        writer.writerows(results.tolist())

    print(f"\nSaved results to {output}")

finally:
    node.destroy_node()
    rclpy.shutdown()
