#!/usr/bin/env python3
"""
mission_nav_test.py — Test de navegación reactiva con posiciones fijas

Uso:
  python3 mission_nav_test.py <caso>

Casos:
  1  Posición libre desconocida  (x= 9.0, y=-4.0)  → espera SUCCESS
  2  Sala cerrada inaccesible    (x=10.0, y= 6.0)  → espera ABORT
  3  Dentro de obstáculo sólido  (x=-6.0, y=-4.5)  → espera closest_free_point

El log queda en: /root/sherec_nav/nav_planner_diag.log
"""

import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from as2_msgs.msg import YawMode
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.modules.navigate_to_module import NavigateToModule
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler

TAKEOFF_HEIGHT = 1.0
NAV_SPEED = 1.5
MAX_WAIT_S = 120

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
}


class TestDrone(DroneInterface):
    def __init__(self):
        super().__init__("drone0", verbose=False, use_sim_time=True)
        self.navigate_to = NavigateToModule(drone=self)


def run_test(case_num: int) -> None:
    case = TEST_CASES[case_num]

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
                f"  t={elapsed:5.1f}s  pos=[{pos[0]:6.2f}, {pos[1]:6.2f}, {pos[2]:5.2f}]",
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


if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in ("1", "2", "3"):
        print(__doc__)
        sys.exit(1)

    run_test(int(sys.argv[1]))
