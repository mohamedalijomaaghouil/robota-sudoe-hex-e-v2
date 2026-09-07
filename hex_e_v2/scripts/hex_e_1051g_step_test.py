import csv
import time
import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped

TOPIC = "/bus0/ft_sensor0/ft_sensor_readings/wrench"
MASS = 1.051
EXPECTED = MASS * 9.80665
SAMPLE_RATE = 10.0

rows = []
current_phase = "idle"

rclpy.init()
node = rclpy.create_node("hex_e_625g_step_test")

def callback(msg):
    if current_phase != "idle":
        rows.append([
            time.monotonic(),
            current_phase,
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ])

node.create_subscription(WrenchStamped, TOPIC, callback, 50)

def record_phase(name, duration):
    global current_phase
    current_phase = name
    start = time.monotonic()
    next_report = start + 30.0

    print(f"\nRecording {name} for {duration:.0f} seconds...")

    while rclpy.ok() and time.monotonic() - start < duration:
        rclpy.spin_once(node, timeout_sec=0.2)

        now = time.monotonic()
        if now >= next_report:
            phase_rows = [r for r in rows if r[1] == name]
            if phase_rows:
                x = np.asarray([r[2:] for r in phase_rows], dtype=float)
                print(
                    f"  {now-start:5.0f} s: "
                    f"F={x[:, :3].mean(axis=0)} N, "
                    f"T={x[:, 3:].mean(axis=0)} Nm"
                )
            next_report += 30.0

    current_phase = "idle"

def phase_array(name):
    return np.asarray(
        [r[2:] for r in rows if r[1] == name],
        dtype=float
    )

input(
    "\nRobot stationary; complete 1051 g assembly REMOVED.\n"
    "Do not move or touch the robot. Press Enter to begin: "
)
record_phase("unloaded_before", 60.0)

input(
    "\nAttach the complete 1051 g assembly by its clip. "
    "Let it hang freely, then press Enter: "
)
record_phase("loaded", 180.0)

input(
    "\nRemove the complete 1051 g assembly. "
    "Do not move the robot, then press Enter: "
)
record_phase("unloaded_after", 180.0)

node.destroy_node()
rclpy.shutdown()

with open("/home/kairos/hex_e_1051g_step.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "monotonic_time", "phase",
        "Fx", "Fy", "Fz", "Tx", "Ty", "Tz"
    ])
    writer.writerows(rows)

before = phase_array("unloaded_before")
loaded = phase_array("loaded")
after = phase_array("unloaded_after")

# Use settled portions of each phase.
before_mean = before[-300:].mean(axis=0)
loaded_mean = loaded[-600:].mean(axis=0)
after_mean = after[-600:].mean(axis=0)
reference = 0.5 * (before_mean + after_mean)

delta_before = loaded_mean[:3] - before_mean[:3]
delta_after = loaded_mean[:3] - after_mean[:3]
delta_bracketed = loaded_mean[:3] - reference[:3]

print("\n========== 1051 g STEP-TEST SUMMARY ==========")
print(f"Expected gravitational force: {EXPECTED:.6f} N")
print(f"Unloaded-before mean F:       {before_mean[:3]} N")
print(f"Settled loaded mean F:        {loaded_mean[:3]} N")
print(f"Unloaded-after mean F:        {after_mean[:3]} N")
print()
print(f"Delta versus before:          {delta_before} N")
print(f"|Delta versus before|:        {np.linalg.norm(delta_before):.6f} N")
print(f"Delta versus after:           {delta_after} N")
print(f"|Delta versus after|:         {np.linalg.norm(delta_after):.6f} N")
print(f"Bracketed delta:              {delta_bracketed} N")
print(f"|Bracketed delta|:            {np.linalg.norm(delta_bracketed):.6f} N")
print(
    f"Bracketed measured/expected:  "
    f"{np.linalg.norm(delta_bracketed)/EXPECTED:.6f}"
)
print("Saved: /home/kairos/hex_e_1051g_step.csv")
