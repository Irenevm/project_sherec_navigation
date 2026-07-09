#!/usr/bin/env python3
"""
Test aislado: dron en hover en zona abierta, spawnea una caja delante con
linea de vision directa (sin paredes), y mide con precision cuando la
celda del mapa de ocupacion correspondiente cruza los umbrales de deteccion.

No usa path_planner ni FollowPath: solo takeoff + hover + spawn + sample.
"""
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import OccupancyGrid
from as2_python_api.drone_interface import DroneInterface

WORLD = "nav_test_world"
HOVER_X, HOVER_Y, HOVER_Z = -9.0, 0.0, 1.0
OBS_X, OBS_Y, OBS_Z = -6.0, 0.0, 1.0   # 3m delante del dron, corredor abierto
OBS_SIZE = (1.0, 1.0, 2.0)
# Muestreamos la CARA CERCANA de la caja (borde que da al dron), no el centro:
# el centro de un obstaculo solido nunca es alcanzado directamente por un rayo
# LiDAR (queda ocluido por su propia cara cercana) — confirmado empiricamente.
SAMPLE_X, SAMPLE_Y = OBS_X - OBS_SIZE[0] / 2.0, OBS_Y


def spawn_obstacle(name, x, y, z, sx, sy, sz, color=(0.8, 0.2, 0.2)):
    r, g, b = color
    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="col">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
      <visual name="vis">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
    t_cmd = time.time()
    result = subprocess.run(
        ["ros2", "run", "ros_gz_sim", "create",
         "-world", WORLD, "-string", sdf,
         "-x", str(x), "-y", str(y), "-z", str(z)],
        capture_output=True, text=True, timeout=10
    )
    ok = result.returncode == 0
    return t_cmd, ok, result.stderr.strip()


class TestDrone(DroneInterface):
    def __init__(self):
        super().__init__("drone0", verbose=False, use_sim_time=True)


class CellSampler:
    """Sample the occupancy value of the cell under the obstacle over time."""
    def __init__(self, node, obs_x, obs_y):
        self.obs_x = obs_x
        self.obs_y = obs_y
        self.events = []  # (t, value)
        self.last_val = None
        self.t_spawn = None
        self.sub = node.create_subscription(OccupancyGrid, "/drone0/map", self.cb, 1)

    def cb(self, msg):
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        res = msg.info.resolution
        w, h = msg.info.width, msg.info.height
        cx = int((self.obs_x - ox) / res)
        cy = int((self.obs_y - oy) / res)
        val = None
        if 0 <= cx < w and 0 <= cy < h:
            val = msg.data[cy * w + cx]
        t = time.time()
        if val != self.last_val:
            self.events.append((t, val))
            self.last_val = val


def main():
    rclpy.init()
    drone = TestDrone()
    executor = MultiThreadedExecutor()
    executor.add_node(drone)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    sampler = CellSampler(drone, SAMPLE_X, SAMPLE_Y)

    try:
        time.sleep(1.0)
        print("[1/3] Offboard...")
        drone.offboard()
        time.sleep(1.0)
        print("[2/3] Arm...")
        drone.arm()
        time.sleep(1.0)
        print(f"[3/3] Takeoff a {HOVER_Z}m...")
        drone.takeoff(height=HOVER_Z, speed=0.5)
        time.sleep(3.0)

        print(f"Dron en hover ~({HOVER_X},{HOVER_Y},{HOVER_Z}). Estabilizando 5s...")
        time.sleep(5.0)

        print(f"Spawneando obstaculo en ({OBS_X},{OBS_Y},{OBS_Z})...")
        t_cmd, ok, err = spawn_obstacle("measure_obs2", OBS_X, OBS_Y, OBS_Z, *OBS_SIZE)
        sampler.t_spawn = t_cmd
        print(f"  spawn ok={ok} t_cmd={t_cmd:.3f}" + (f" err={err}" if not ok else ""))

        print("Monitorizando celda del mapa durante 8s tras el spawn...")
        time.sleep(8.0)

        print("\n--- Eventos de la celda (t relativo al comando de spawn, valor) ---")
        for t, val in sampler.events:
            rel = t - sampler.t_spawn
            marker = ""
            if val is not None and val > 30 and marker == "":
                marker = "  <-- OCUPADA (>30)"
            print(f"  t={rel:+7.3f}s  value={val}{marker}")

        # Reportar tiempo hasta > 30
        t_occ = None
        for t, val in sampler.events:
            if val is not None and val > 30 and t >= sampler.t_spawn:
                t_occ = t - sampler.t_spawn
                break
        if t_occ is not None:
            print(f"\n>>> Tiempo desde spawn hasta value>30: {t_occ:.3f}s")
        else:
            print("\n>>> La celda NUNCA superó el umbral 30 en la ventana de 8s.")

    finally:
        print("\nAterrizando...")
        try:
            drone.land(speed=0.5)
        except Exception as e:
            print(f"land error: {e}")
        try:
            drone.disarm()
        except Exception:
            pass
        drone.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
