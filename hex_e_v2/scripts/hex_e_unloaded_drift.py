import csv
import time
import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped

TOPIC = "/bus0/ft_sensor0/ft_sensor_readings/wrench"
DURATION = 300.0       # 5 minutes
WINDOW = 30.0          # report every 30 seconds
rows = []

rclpy.init()
node = rclpy.create_node("hex_e_unloaded_drift")

def callback(msg):
    rows.append([
        time.monotonic(),
        msg.wrench.force.x,
        msg.wrench.force.y,
        msg.wrench.force.z,
        msg.wrench.torque.x,
        msg.wrench.torque.y,
        msg.wrench.torque.z,
    ])

sub = node.create_subscription(WrenchStamped, TOPIC, callback, 50)

print("Recording unloaded sensor drift for 5 minutes.")
print("Do not move the robot, touch the handler, or zero the sensor.")

start = time.monotonic()
next_report = WINDOW

while rclpy.ok() and time.monotonic() - start < DURATION:
    rclpy.spin_once(node, timeout_sec=0.2)
    elapsed = time.monotonic() - start

    if elapsed >= next_report:
        recent = [r for r in rows if r[0] - start >= next_report - WINDOW]
        if recent:
            x = np.asarray([r[1:] for r in recent])
            print(
                f"{next_report:5.0f} s: n={len(x):3d}, "
                f"F=[{x[:,0].mean():.4f}, {x[:,1].mean():.4f}, "
                f"{x[:,2].mean():.4f}] N, "
                f"T=[{x[:,3].mean():.4f}, {x[:,4].mean():.4f}, "
                f"{x[:,5].mean():.4f}] Nm"
            )
        next_report += WINDOW

node.destroy_node()
rclpy.shutdown()

if not rows:
    raise SystemExit("ERROR: No wrench samples received")

t0 = rows[0][0]
with open("/home/kairos/hex_e_unloaded_drift.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time_s", "Fx_N", "Fy_N", "Fz_N",
                     "Tx_Nm", "Ty_Nm", "Tz_Nm"])
    for row in rows:
        writer.writerow([row[0] - t0] + row[1:])

x = np.asarray([r[1:] for r in rows])
n = max(1, int(len(x) * 0.1))
first = x[:n].mean(axis=0)
last = x[-n:].mean(axis=0)
delta = last - first

print("\n========== UNLOADED DRIFT SUMMARY ==========")
print(f"Samples: {len(x)}")
print(f"First 10% mean F: {first[:3]} N")
print(f"Last 10% mean F:  {last[:3]} N")
print(f"Force drift:      {delta[:3]} N")
print(f"First 10% mean T: {first[3:]} Nm")
print(f"Last 10% mean T:  {last[3:]} Nm")
print(f"Torque drift:     {delta[3:]} Nm")
print("Saved: /home/kairos/hex_e_unloaded_drift.csv")
