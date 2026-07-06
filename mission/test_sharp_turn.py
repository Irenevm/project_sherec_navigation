#!/usr/bin/env python3
"""
test_sharp_turn.py — Test instrumentado para diagnosticar Bug B (crash por giro brusco)

Envía un FollowPath directo (sin A*, sin obstáculos) con un giro de ~135 grados
a velocidad de crucero, y loguea motion_reference/twist + ground_truth/pose
durante la maniobra para confirmar si la saturación proporcional del PID
(proportional_limitation=true) arrastra el eje Z cuando el error lateral satura.

Uso:
  python3 test_sharp_turn.py
"""

import csv
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSPresetProfiles
from as2_msgs.msg import YawMode
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.modules.follow_path_module import FollowPathModule
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path

TAKEOFF_HEIGHT = 1.0
NAV_SPEED = 1.5
LOG_PATH = "/root/sherec_nav/sharp_turn_telemetry.csv"

# Ruta con giro brusco: de (-9,0) -> (-3,0) [tramo recto este] -> (-1,-3.5) [giro ~135 grados
# hacia sureste] -> (2,-5) [continúa]. Sin obstáculos, sin A*: FollowPath directo.
WAYPOINTS = [
    (-9.0, 0.0, TAKEOFF_HEIGHT),
    (-3.0, 0.0, TAKEOFF_HEIGHT),
    (-1.0, -3.5, TAKEOFF_HEIGHT),
    (2.0, -5.0, TAKEOFF_HEIGHT),
]


class TestDrone(DroneInterface):
    def __init__(self):
        super().__init__("drone0", verbose=False, use_sim_time=True)
        self.follow_path = FollowPathModule(drone=self)


def make_path(frame_id="earth"):
    path = Path()
    path.header.frame_id = frame_id
    for i, (x, y, z) in enumerate(WAYPOINTS[1:]):  # skip start (drone's own spawn)
        p = PoseStamped()
        p.header.frame_id = frame_id
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        p.pose.orientation.w = 1.0
        p.header.stamp.sec = i  # id used by FollowPath as sequence marker
        path.poses.append(p)
    return path


def run_test():
    rclpy.init()
    drone = TestDrone()
    executor = MultiThreadedExecutor()
    executor.add_node(drone)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    telemetry = []
    t0 = [None]

    def twist_cbk(msg):
        if t0[0] is None:
            return
        t = time.time() - t0[0]
        telemetry.append({
            "t": round(t, 3),
            "topic": "motion_reference/twist",
            "vx": msg.twist.linear.x,
            "vy": msg.twist.linear.y,
            "vz": msg.twist.linear.z,
        })

    def pose_cbk(msg):
        if t0[0] is None:
            return
        t = time.time() - t0[0]
        telemetry.append({
            "t": round(t, 3),
            "topic": "ground_truth/pose",
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
        })

    drone.create_subscription(
        TwistStamped, "motion_reference/twist", twist_cbk,
        QoSPresetProfiles.SENSOR_DATA.value)
    drone.create_subscription(
        PoseStamped, "ground_truth/pose", pose_cbk,
        QoSPresetProfiles.SENSOR_DATA.value)

    try:
        time.sleep(1.0)
        print("[1/4] Offboard...")
        drone.offboard()
        time.sleep(1.0)

        print("[2/4] Arm...")
        drone.arm()
        time.sleep(1.0)

        print(f"[3/4] Takeoff a {TAKEOFF_HEIGHT}m...")
        drone.takeoff(height=TAKEOFF_HEIGHT, speed=0.5)
        time.sleep(2.0)

        print("[4/4] FollowPath directo con giro brusco (~135 deg)...")
        print(f"  Waypoints: {WAYPOINTS}")
        t0[0] = time.time()

        path = make_path()
        drone.follow_path(
            path, speed=NAV_SPEED, yaw_mode=YawMode.PATH_FACING, yaw_angle=0.0, wait=False)

        deadline = time.time() + 40
        while drone.follow_path.is_running() and time.time() < deadline:
            pos = drone.position
            print(f"  t={time.time()-t0[0]:5.1f}s pos=[{pos[0]:6.2f},{pos[1]:6.2f},{pos[2]:5.2f}]", end="\r")
            time.sleep(0.2)

        elapsed = time.time() - t0[0]
        pos = drone.position
        print(f"\n\n--- Resultado ---")
        print(f"  Tiempo total: {elapsed:.1f}s")
        print(f"  Posición final: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")

        # Write telemetry to CSV
        with open(LOG_PATH, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t", "topic", "a", "b", "c"])
            for row in telemetry:
                if row["topic"] == "motion_reference/twist":
                    writer.writerow([row["t"], row["topic"], row["vx"], row["vy"], row["vz"]])
                else:
                    writer.writerow([row["t"], row["topic"], row["x"], row["y"], row["z"]])
        print(f"  Telemetría guardada en {LOG_PATH} ({len(telemetry)} muestras)")

        # Quick min-z / max-|vz| summary
        z_values = [r["z"] for r in telemetry if r["topic"] == "ground_truth/pose"]
        vz_values = [r["vz"] for r in telemetry if r["topic"] == "motion_reference/twist"]
        if z_values:
            print(f"  z_min={min(z_values):.3f}  z_max={max(z_values):.3f}")
        if vz_values:
            print(f"  vz_min={min(vz_values):.3f}  vz_max={max(vz_values):.3f}")

    except KeyboardInterrupt:
        print("\n  Interrumpido")
    finally:
        print("\nAterrizando...")
        drone.land(speed=0.5)
        time.sleep(2.0)
        drone.disarm()
        drone.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    run_test()
