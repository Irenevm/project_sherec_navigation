#!/usr/bin/env python3
"""
mission_nav_test.py — Test de navegación punto a punto con posiciones fijas

Uso:
  python3 mission_nav_test.py <caso>

Casos:
  1  Posición libre desconocida        (x= 9.0, y=-4.0)  → espera SUCCESS
  2  Sala cerrada inaccesible          (x=10.0, y= 6.0)  → espera ABORT
  3  Dentro de obstáculo sólido        (x=-6.0, y=-4.5)  → espera closest_free_point
  5  Dos obstáculos dinámicos          (x= 9.0, y=-4.0)  → SUCCESS validado

El log queda en: /root/sherec_nav/nav_planner_diag.log
"""

import os
import shutil
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from as2_msgs.msg import YawMode
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.modules.navigate_to_module import NavigateToModule
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from std_msgs.msg import String
from rcl_interfaces.msg import Log

TAKEOFF_HEIGHT = 1.0
NAV_SPEED = 1.5
MAX_WAIT_S = 120
WORLD = "nav_test_world"


def spawn_obstacle(name="dynamic_obstacle", x=0.0, y=0.0, z=1.0,
                   sx=1.0, sy=1.0, sz=2.0, color=(0.8, 0.2, 0.2)):
    """Spawn a static box in Ignition Gazebo via ros_gz_sim."""
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
    result = subprocess.run(
        ["ros2", "run", "ros_gz_sim", "create",
         "-world", WORLD,
         "-string", sdf,
         "-x", str(x), "-y", str(y), "-z", str(z)],
        capture_output=True, text=True, timeout=10
    )
    if result.returncode == 0:
        print(f"\n  [SPAWN] Obstáculo '{name}' aparecido en ({x}, {y}, {z})")
    else:
        print(f"\n  [SPAWN ERROR] {result.stderr.strip()}")


def remove_obstacle(name="dynamic_obstacle"):
    """Remove a previously spawned entity via Ignition service."""
    result = subprocess.run(
        ["ign", "service", "-s", f"/world/{WORLD}/remove",
         "--reqtype", "ignition.msgs.Entity",
         "--reptype", "ignition.msgs.Boolean",
         "--timeout", "3000",
         "--req", f'name: "{name}" type: 2'],
        capture_output=True, text=True, timeout=10
    )
    if "data: true" not in result.stdout:
        print(f"\n  [REMOVE] Para borrar la caja reinicia la simulacion (ign service no disponible)")


TEST_CASES = {
    1: {
        "name":   "Libre desconocido — rodear pared divisoria",
        "x": 9.0,  "y": -4.0,  "z": TAKEOFF_HEIGHT,
        "expect": "SUCCESS: el dron rodea la pared por el hueco central y llega",
    },
    2: {
        "name":   "Inaccesible — sala cerrada NE",
        "x": 10.0, "y":  6.0,  "z": TAKEOFF_HEIGHT,
        "expect": "ABORT: supera MAX_REPLANS o frontera en dirección incorrecta",
    },
    3: {
        "name":   "Dentro de obstáculo sólido",
        "x": -6.0, "y": -4.5,  "z": TAKEOFF_HEIGHT,
        "expect": "PARCIAL: llega al borde del bloque (closest_free_point), no aborta",
    },
    5: {
        "name":   "Dos obstáculos dinámicos en ruta",
        "x": 9.0,  "y": -4.0,  "z": TAKEOFF_HEIGHT,
        "expect": (
            "2 cajas aparecen casi simultáneamente (trigger 4m):\n"
            "  OBS-1 (6.5,-3.8) rojo    — bloquea ruta directa sur\n"
            "  OBS-2 (8.0,-3.5) naranja — bloquea la curva norte hacia el goal\n"
            "Validado: LiDAR frena en 2 ocasiones, A*/MAP_CHECK replantea (6 replans),\n"
            "el dron rodea ambos obstáculos por el norte y llega SUCCESS en ~50s."
        ),
        # OBS-1 en (6.5,-3.8): inflado x=5.5-7.5, y=-4.8 to -2.8.
        # OBS-2 en (8.0,-3.5): inflado x=7.0-9.0, y=-4.5 to -2.5.
        # Juntos bloquean el corredor norte (y≈-2.85). A* debe ir a y>-2.5 para pasar.
        "obstacles": [
            {
                "name": "obs_sur",
                "pos":  (6.5,  -3.8,  1.0),
                "size": (1.0,  1.0,   2.0),
                "trigger_dist": 4.0,
                "color": (0.9, 0.1, 0.1),   # rojo
            },
            {
                "name": "obs_este",
                "pos":  (8.0,  -3.5,  1.0),
                "size": (1.0,  1.0,   2.0),
                "trigger_dist": 4.0,
                "color": (0.9, 0.5, 0.1),   # naranja
            },
        ],
    },
}


class TestDrone(DroneInterface):
    def __init__(self):
        super().__init__("drone0", verbose=False, use_sim_time=True)
        self.navigate_to = NavigateToModule(drone=self)


def _make_spawn_thread(drone, obs):
    """Crea y devuelve un hilo que espera proximidad al obstáculo y lo spawnea."""
    import math
    ox, oy, oz = obs["pos"]
    sx, sy, sz = obs.get("size", (1.0, 1.0, 2.0))
    trigger = obs.get("trigger_dist", 4.0)
    name = obs["name"]
    color = obs.get("color", (0.8, 0.2, 0.2))

    t_start = time.time()

    def _run():
        # Guard mínimo: esperar 5s desde el inicio de la misión antes de monitorizar.
        # Evita disparo prematuro si la posición publicada es residual de un run anterior.
        while time.time() - t_start < 5.0:
            time.sleep(0.1)
        print("[SPAWN-" + name + "] Esperando dist <= " + str(trigger) + "m ...")
        while True:
            p = drone.position
            d = math.sqrt((p[0] - ox) ** 2 + (p[1] - oy) ** 2)
            if d <= trigger:
                break
            time.sleep(0.05)
        p = drone.position
        d2 = math.sqrt((p[0] - ox) ** 2 + (p[1] - oy) ** 2)
        print(
            "[SPAWN-" + name + "] dist=" + str(round(d2, 2))
            + " dron=(" + str(round(p[0], 2)) + "," + str(round(p[1], 2)) + ")"
            + " -> spawning at (" + str(ox) + "," + str(oy) + "," + str(oz) + ")"
        )
        spawn_obstacle(name, ox, oy, oz, sx=sx, sy=sy, sz=sz, color=color)

    return threading.Thread(target=_run, daemon=True)


class _Tee:
    """Duplica todo lo escrito en stdout también a un fichero de log."""

    def __init__(self, stream, logfile):
        self._stream = stream
        self._logfile = logfile

    def write(self, data):
        self._stream.write(data)
        self._logfile.write(data)

    def flush(self):
        self._stream.flush()
        self._logfile.flush()


def run_test(case_num: int) -> None:
    case = TEST_CASES[case_num]

    log_dir = "/root/sherec_nav/logs"
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(
        log_dir,
        f"mission_case{case_num}_{time.strftime('%Y%m%d_%H%M%S')}.log")
    latest_path = os.path.join(log_dir, "last_manual_run.log")
    log_file = open(log_path, "w")
    original_stdout = sys.stdout
    sys.stdout = _Tee(original_stdout, log_file)

    print(f"[LOG] Guardando esta ejecución en: {log_path}")
    print(f"      (también copiado a {latest_path} al terminar)")

    print(f"\n{'='*60}")
    print(f"  CASO {case_num}: {case['name']}")
    print(f"  Objetivo : x={case['x']:.1f}, y={case['y']:.1f}, z={case['z']:.1f}")
    print(f"  Esperado : {case['expect']}")
    print(f"{'='*60}\n")

    rclpy.init()
    drone = TestDrone()
    executor = MultiThreadedExecutor()
    executor.add_node(drone)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Modo de la capa reactiva (NONE/DECEL/HARD), publicado por el planner en
    # "reactive_mode" como "<modo>:<distancia>". Se muestra siempre, incluso
    # cuando no hay reacción (NONE), para verlo en directo en esta terminal.
    reactive_state = {"text": "NONE:-1"}

    def _reactive_mode_cbk(msg: String) -> None:
        reactive_state["text"] = msg.data

    reactive_mode_sub = drone.create_subscription(
        String, "reactive_mode", _reactive_mode_cbk, 10)

    # Vía alternativa que funciona en CUALQUIER commit, exista o no el topic
    # "reactive_mode": todo RCLCPP_WARN/INFO del planner se publica también en
    # /rosout (comportamiento estándar de ROS2, no depende de este código).
    # Filtramos por nombre de nodo y contenido para imprimir en esta misma
    # terminal en cuanto el planner entra en LIDAR_SAFETY o cambia de modo,
    # sin tener que mirar tmux ni el log aparte.
    def _rosout_cbk(msg: Log) -> None:
        if "path_planner" in msg.name and (
                "LIDAR_SAFETY" in msg.msg or "REACTIVE_MODE" in msg.msg):
            print(f"\n  [PLANNER] {msg.msg}")

    rosout_sub = drone.create_subscription(Log, "/rosout", _rosout_cbk, 10)

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

        print(f"[4/4] Navigate to [{case['x']}, {case['y']}, {case['z']}]...")
        t_start = time.time()

        if "obstacles" in case:
            for obs in case["obstacles"]:
                _make_spawn_thread(drone, obs).start()

        try:
            drone.navigate_to(
                case["x"], case["y"], case["z"],
                speed=NAV_SPEED,
                yaw_mode=YawMode.PATH_FACING,
                wait=False,
            )
        except BehaviorHandler.GoalRejected:
            print("  [!] Goal rechazado inmediatamente por el servidor")

        deadline = time.time() + MAX_WAIT_S
        while drone.navigate_to.is_running() and time.time() < deadline:
            elapsed = time.time() - t_start
            pos = drone.position
            print(
                f"  t={elapsed:5.1f}s  pos=[{pos[0]:6.2f}, {pos[1]:6.2f}, {pos[2]:5.2f}]"
                f"  reactive={reactive_state['text']:16s}",
                end="\r",
            )
            time.sleep(0.5)

        elapsed = time.time() - t_start
        pos = drone.position
        print(f"\n\n--- Resultado ---")
        print(f"  Tiempo total : {elapsed:.1f}s")
        print(f"  Posición final: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
        if time.time() >= deadline:
            print(f"  Estado: TIMEOUT ({MAX_WAIT_S}s superados sin terminar)")
        else:
            print(f"  Estado: behavior terminado (éxito, abort, o rechazo)")
        print(f"\n  >>>  'caso {case_num} terminado'")

    except KeyboardInterrupt:
        print("\n  Interrumpido")
    finally:
        print("\nAtterrizando...")
        drone.land(speed=0.5)
        time.sleep(2.0)
        drone.disarm()
        drone.shutdown()
        rclpy.shutdown()

        sys.stdout = original_stdout
        log_file.close()
        shutil.copyfile(log_path, latest_path)
        print(f"[LOG] Log de esta ejecución en: {log_path} (y en {latest_path})")


if __name__ == "__main__":
    valid_cases = [str(k) for k in TEST_CASES.keys()]
    if len(sys.argv) != 2 or sys.argv[1] not in valid_cases:
        print(__doc__)
        sys.exit(1)

    run_test(int(sys.argv[1]))
