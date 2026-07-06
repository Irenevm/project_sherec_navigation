#!/usr/bin/env python3
"""
mission_nav_test.py — Test de navegación reactiva con posiciones fijas

Uso:
  python3 mission_nav_test.py <caso>

Casos:
  1  Posición libre desconocida        (x= 9.0, y=-4.0)  → espera SUCCESS
  2  Sala cerrada inaccesible          (x=10.0, y= 6.0)  → espera ABORT
  3  Dentro de obstáculo sólido        (x=-6.0, y=-4.5)  → espera closest_free_point
  4  Obstáculo dinámico único          (x= 9.0, y=-4.0)  → LiDAR frena, mapa replantea, SUCCESS
  5  Dos obstáculos dinámicos          (x= 9.0, y=-4.0)  → SUCCESS validado
  6  Obstáculo en ruta norte          (goal 9,-2)        → obstáculo a mitad de camino, trigger 3.0m
  7  Obstáculo en ruta corta          (goal 4,-4)        → obstáculo a mitad de camino, trigger 3.0m
  8  Obstáculo de reacción rápida     (trigger 2.5m)     → LiDAR con margen de aviso reducido
  9  Obstáculo en tramo intermedio-avanzado (trigger 2.5m) → ya no pegado al goal (antes 0.4m)
  10 Corredor de tres obstáculos      (3 cajas seguidas)  → cadena de replans, stress del bucle
  11 Obstáculo ancho bloqueo total    (caja 4m ancho)     → fuerza rodeo largo, sin paso directo
  12 Obstáculo margen mínimo          (trigger 1.2m)     → caso más exigente pero físicamente plausible

El log queda en: /root/sherec_nav/nav_planner_diag.log
"""

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
    4: {
        "name":   "Obstáculo dinámico único en ruta",
        "x": 9.0,  "y": -4.0,  "z": TAKEOFF_HEIGHT,
        "expect": (
            "Una caja aparece a t≈20s en (6.5,-3.8): bloquea la ruta directa al sur.\n"
            "El LiDAR frena el dron antes de colisión (~1.5m del obstáculo).\n"
            "El mapa confirma el obstáculo en ~1.5s → MAP_CHECK replantea hacia el norte (y≈-2.85).\n"
            "El dron rodea la caja y llega al goal en SUCCESS."
        ),
        "obstacles": [
            {
                "name": "obs_sur",
                "pos":  (6.5,  -3.8,  1.0),
                "size": (1.0,  1.0,   2.0),
                "trigger_dist": 4.0,
                "color": (0.9, 0.1, 0.1),   # rojo
            },
        ],
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
    # ── Casos de estrés con goals distintos y obstáculos a distancia realista ─
    # Todos disparan el obstáculo con margen de reacción razonable (2.5-3.5m,
    # nunca "encima" del dron ni pegado al propio goal) — simula mejor el
    # escenario real que motivó la capa LiDAR: el obstáculo ya está ahí, el
    # dron se acerca de forma normal, y la sorpresa es el retraso del mapa,
    # no una aparición imposible a quemarropa.
    6: {
        "name":   "Obstáculo en ruta norte (goal 9, -2)",
        "x": 9.0,  "y": -2.0,  "z": TAKEOFF_HEIGHT,
        "expect": (
            "Goal más al norte que el habitual (9,-4). Caja en (5.0,-2.0), trigger_dist=3.0m:\n"
            "aparece con margen de reacción normal a mitad de camino. Prueba que el replan\n"
            "funcione igual de bien en una ruta distinta a la ya validada hacia (9,-4)."
        ),
        "obstacles": [
            {
                "name": "obs_north",
                "pos":  (5.0,  -2.0,  1.0),
                "size": (1.0,  1.0,   2.0),
                "trigger_dist": 3.0,
                "color": (0.9, 0.1, 0.1),
            },
        ],
    },
    7: {
        "name":   "Obstáculo en ruta corta (goal 4, -4)",
        "x": 4.0,  "y": -4.0,  "z": TAKEOFF_HEIGHT,
        "expect": (
            "Goal más cercano (4,-4), ruta más corta. Caja en (0.5,-2.3), trigger_dist=3.0m:\n"
            "aparece a mitad de una ruta más simple. Prueba si el replan se dispara antes o\n"
            "de forma distinta cuando hay menos margen de distancia total hasta el goal."
        ),
        "obstacles": [
            {
                "name": "obs_short",
                "pos":  (0.5,  -2.3,  1.0),
                "size": (1.0,  1.0,   2.0),
                "trigger_dist": 3.0,
                "color": (0.9, 0.5, 0.1),
            },
        ],
    },
    # ── Casos de estrés adicionales para la capa LiDAR + replan ───────────────
    8: {
        "name":   "Obstáculo de reacción rápida (trigger 2.5m)",
        "x": 9.0,  "y": -4.0,  "z": TAKEOFF_HEIGHT,
        "expect": (
            "Caja en (5.0,-3.0) con trigger_dist=2.5m (vs 4.0m habitual): margen de reacción\n"
            "reducido pero realista, no a quemarropa. Prueba el frenado del LiDAR con menos\n"
            "aviso de lo normal (lidar_danger_distance=1.5m, lidar_stop_distance=0.4m). Debe\n"
            "frenar/parar sin colisionar (d_min_approach > safety_distance=0.5m) y replantear."
        ),
        "obstacles": [
            {
                "name": "obs_fast",
                "pos":  (5.0,  -3.0,  1.0),
                "size": (1.0,  1.0,   2.0),
                "trigger_dist": 2.5,
                "color": (0.9, 0.1, 0.1),
            },
        ],
    },
    9: {
        "name":   "Obstáculo en tramo intermedio-avanzado",
        "x": 9.0,  "y": -4.0,  "z": TAKEOFF_HEIGHT,
        "expect": (
            "Caja en (7.0,-3.6), a ~2.3m del goal (9.0,-4.0) — ya no pegada al objetivo como\n"
            "en la versión anterior de este caso (evitaba margen real de reacción y el dron\n"
            "prácticamente chocaba). trigger_dist=2.5m: aparece avanzada la ruta, con margen\n"
            "de frenado real. Prueba que el replan funcione bien cerca del tramo final sin\n"
            "que closest_free_point/FRONTIER se confundan con el propio objetivo."
        ),
        "obstacles": [
            {
                "name": "obs_late",
                "pos":  (7.0,  -3.6,  1.0),
                "size": (1.0,  1.0,   2.0),
                "trigger_dist": 2.5,
                "color": (0.6, 0.1, 0.8),   # morado
            },
        ],
    },
    10: {
        "name":   "Corredor de tres obstáculos en cadena",
        "x": 9.0,  "y": -4.0,  "z": TAKEOFF_HEIGHT,
        "expect": (
            "3 cajas alineadas a lo largo de la ruta directa, cada una con su propio trigger,\n"
            "de forma que aparecen de manera escalonada conforme el dron avanza. Prueba la\n"
            "cadena de replans repetidos (varias activaciones de MAP_CHECK/LiDAR seguidas)\n"
            "sin degradar en oscilación ni superar max_replans=15."
        ),
        "obstacles": [
            {
                "name": "obs_chain1",
                "pos":  (2.5,  -2.0,  1.0),
                "size": (1.0,  1.0,   2.0),
                "trigger_dist": 3.0,
                "color": (0.9, 0.1, 0.1),
            },
            {
                "name": "obs_chain2",
                "pos":  (5.5,  -2.8,  1.0),
                "size": (1.0,  1.0,   2.0),
                "trigger_dist": 3.0,
                "color": (0.9, 0.5, 0.1),
            },
            {
                "name": "obs_chain3",
                "pos":  (8.0,  -3.8,  1.0),
                "size": (1.0,  1.0,   2.0),
                "trigger_dist": 3.0,
                "color": (0.9, 0.8, 0.1),
            },
        ],
    },
    11: {
        "name":   "Obstáculo ancho — bloqueo total del corredor",
        "x": 9.0,  "y": -4.0,  "z": TAKEOFF_HEIGHT,
        "expect": (
            "Caja de 4m de ancho (y de -5.5 a -1.5) centrada en la ruta directa: no hay hueco\n"
            "estrecho para colarse, obliga a un rodeo largo por uno de los extremos. Prueba\n"
            "que A* encuentre el rodeo correcto y que el LiDAR no confunda un borde lejano\n"
            "del bloque ancho con el corredor libre."
        ),
        "obstacles": [
            {
                "name": "obs_wide",
                "pos":  (6.5,  -3.5,  1.0),
                "size": (1.0,  4.0,   2.0),
                "trigger_dist": 4.0,
                "color": (0.1, 0.3, 0.9),   # azul
            },
        ],
    },
    12: {
        "name":   "Obstáculo con margen de reacción mínimo (trigger 1.2m)",
        "x": 9.0,  "y": -4.0,  "z": TAKEOFF_HEIGHT,
        "expect": (
            "Caja en (6.5,-3.8) con trigger_dist=1.2m: deja un margen de frenado real (más\n"
            "que lidar_danger_distance=1.5m mide desde el borde, no desde el centro) en vez\n"
            "de la versión anterior (0.5m), que prácticamente garantizaba la colisión al\n"
            "activarse dentro del propio radio de frenado del LiDAR. Sigue siendo el caso más\n"
            "exigente de margen de reacción, pero ahora es físicamente plausible."
        ),
        "obstacles": [
            {
                "name": "obs_point_blank",
                "pos":  (6.5,  -3.8,  1.0),
                "size": (1.0,  1.0,   2.0),
                "trigger_dist": 1.2,
                "color": (0.9, 0.0, 0.4),   # rosa fuerte
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

        # Lanzar un hilo por obstáculo, cada uno esperando su trigger_dist propio.
        # Cualquier caso con "obstacles" (o el legacy "obstacle_pos") activa el spawn dinámico.
        if "obstacles" in case or "obstacle_pos" in case:
            obstacles = case.get("obstacles", [])
            # Compatibilidad hacia atrás: si no hay lista, usar obstacle_pos
            if not obstacles and "obstacle_pos" in case:
                ox, oy, oz = case["obstacle_pos"]
                obstacles = [{
                    "name": "dynamic_obstacle",
                    "pos": (ox, oy, oz),
                    "size": case.get("obstacle_size", (1.0, 1.0, 2.0)),
                    "trigger_dist": case.get("obstacle_trigger_dist", 4.0),
                }]
            for obs in obstacles:
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
    valid_cases = [str(k) for k in TEST_CASES.keys()]
    if len(sys.argv) != 2 or sys.argv[1] not in valid_cases:
        print(__doc__)
        sys.exit(1)

    run_test(int(sys.argv[1]))
