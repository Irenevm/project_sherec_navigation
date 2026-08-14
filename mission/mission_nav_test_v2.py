#!/usr/bin/env python3
"""
mission_nav_test_v2.py — Test de navegación punto a punto en zonas nuevas del mapa

Complementa a mission_nav_test.py (que cubre casi todo en la zona SE, ruta
sur hacia goals cerca de (9,-4)). Aquí se prueba una ruta en una zona del
mapa no cubierta antes: el corredor suroeste, junto al bloque sólido.

Referencia del mundo (nav_test_world.sdf):
  Sala: x=[-13,13], y=[-8,8]. Spawn del dron: x=-9, y=0.
  Pared divisoria en x=0 con HUECO en y=[-1.5, 1.5] (único paso este-oeste).
  Sala cerrada NE inaccesible: x=[7,13], y=[4,8].
  Bloque sólido SW: x=[-8,-4], y=[-6,-3].
  Caja estática NW: centro (-5,4), 1.5x1.0m.
  Caja estática SE: centro (6,-5.5), 1.2x1.0m.

Uso:
  python3 mission_nav_test_v2.py <caso>

Casos:
  1  Corredor SW ajustado (goal -11,-6.5)   → paso estrecho junto a west_wall y solid_block

El log queda en: /root/sherec_nav/nav_planner_diag.log
"""

import os
import shutil
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


TEST_CASES = {
    1: {
        "name":   "Corredor SW ajustado",
        "x": -11.0, "y": -6.5, "z": TAKEOFF_HEIGHT,
        "expect": (
            "Prueba que A* encuentre el paso estrecho entre west_wall (x=-13) y\n"
            "solid_block (x=[-8,-4], y=[-6,-3]) para llegar a un goal pegado a la\n"
            "esquina SW."
        ),
    },
}


class TestDrone(DroneInterface):
    def __init__(self):
        super().__init__("drone0", verbose=False, use_sim_time=True)
        self.navigate_to = NavigateToModule(drone=self)


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
        f"mission_case{case_num}v2_{time.strftime('%Y%m%d_%H%M%S')}.log")
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
