#!/usr/bin/env python3
"""
Estudio propio e independiente de latencia M_t vs M_g.

Dron en hover en zona abierta, spawnea una caja delante con linea de vision
directa, y muestrea simultaneamente tres topics de ocupacion sobre la MISMA
celda (cara cercana del obstaculo):

  - /drone0/map_instant   -> M_t: mapa instantaneo por medida (nuevo, sin acumular)
  - /drone0/map           -> M_g crudo: acumulado con hit/miss_confidence (el que usa A*)
  - /drone0/map_filtered  -> M_g filtrado: acumulado + cierre morfologico

Objetivo: medir, con datos propios y frescos, el hueco Delta(t1->t2) entre que
M_t marca la celda ocupada y que cada version de M_g cruza el umbral (>30,
el mismo que usa a_star.cpp:occupancy_value). No se asume el resultado del
estudio previo (measure_map_delay.py) -- se remide de cero.
"""
import subprocess
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import OccupancyGrid
from as2_python_api.drone_interface import DroneInterface

WORLD = "nav_test_world"
HOVER_X, HOVER_Y, HOVER_Z = -9.0, -2.0, 1.0
OBS_X, OBS_Y, OBS_Z = -6.0, -2.0, 1.0
OBS_SIZE = (1.0, 1.0, 2.0)
SAMPLE_X, SAMPLE_Y = OBS_X - OBS_SIZE[0] / 2.0, OBS_Y
OCC_THRESHOLD = 30


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
    """Sample the occupancy value of one fixed world cell on a given topic."""
    def __init__(self, node, topic, obs_x, obs_y):
        self.topic = topic
        self.obs_x = obs_x
        self.obs_y = obs_y
        self.events = []
        self.last_val = None
        self.t_spawn = None
        self.sub = node.create_subscription(OccupancyGrid, topic, self.cb, 1)

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

    def first_crossing(self, threshold, op):
        """op: '==' for M_t hit (100), '>' for M_g occupancy threshold."""
        for t, val in self.events:
            if val is None or self.t_spawn is None or t < self.t_spawn:
                continue
            if (op == '==' and val == threshold) or (op == '>' and val > threshold):
                return t - self.t_spawn
        return None


def main():
    rclpy.init()
    drone = TestDrone()
    executor = MultiThreadedExecutor()
    executor.add_node(drone)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    s_instant = CellSampler(drone, "/drone0/map_instant", SAMPLE_X, SAMPLE_Y)
    s_raw = CellSampler(drone, "/drone0/map", SAMPLE_X, SAMPLE_Y)
    s_filtered = CellSampler(drone, "/drone0/map_filtered", SAMPLE_X, SAMPLE_Y)
    samplers = [s_instant, s_raw, s_filtered]

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
        t_cmd, ok, err = spawn_obstacle(f"latency_own_obs_{int(time.time())}", OBS_X, OBS_Y, OBS_Z, *OBS_SIZE)
        for s in samplers:
            s.t_spawn = t_cmd
        print(f"  spawn ok={ok} t_cmd={t_cmd:.3f}" + (f" err={err}" if not ok else ""))

        print("Monitorizando 3 topics durante 8s tras el spawn...")
        time.sleep(8.0)

        for name, s in [("map_instant (M_t)", s_instant),
                         ("map (M_g crudo)", s_raw),
                         ("map_filtered (M_g filtrado)", s_filtered)]:
            print(f"\n--- {name} — eventos de la celda ---")
            for t, val in s.events:
                rel = t - s.t_spawn
                print(f"  t={rel:+7.3f}s  value={val}")

        t1 = s_instant.first_crossing(100, '==')
        t2_raw = s_raw.first_crossing(OCC_THRESHOLD, '>')
        t2_filtered = s_filtered.first_crossing(OCC_THRESHOLD, '>')

        print("\n=== RESULTADO ===")
        print(f"t1 (M_t primer hit ==100):            {t1}")
        print(f"t2_raw (map >30, el que usa A*):        {t2_raw}")
        print(f"t2_filtered (map_filtered >30):          {t2_filtered}")
        if t1 is not None and t2_raw is not None:
            print(f"Delta(t1 -> t2_raw):      {t2_raw - t1:.3f}s")
        if t1 is not None and t2_filtered is not None:
            print(f"Delta(t1 -> t2_filtered): {t2_filtered - t1:.3f}s")

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
