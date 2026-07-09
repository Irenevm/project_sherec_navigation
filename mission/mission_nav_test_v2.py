#!/usr/bin/env python3
"""
mission_nav_test_v2.py — Test de navegación reactiva en zonas nuevas del mapa

Complementa a mission_nav_test.py (que cubre casi todo en la zona SE, ruta
sur hacia goals cerca de (9,-4)). Aquí se prueban rutas y obstáculos en
zonas del mapa NO cubiertas antes: norte, noroeste, el propio hueco central
de la pared divisoria, y la esquina suroeste (bloque sólido).

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
  1  Obstáculo dinámico en ruta norte-oeste (goal -9,6.5)   → dron sube en línea recta por el oeste, caja le corta el paso
  2  Obstáculo tras el hueco, rumbo NE (goal 5,5)            → cruza el hueco central y se topa con una caja al girar al norte
  3  Bloqueo dinámico del propio hueco central (goal 9,0)    → única vía este-oeste cortada por una caja, replan obligatorio
  4  Bloque SW + obstáculo dinámico (goal -9,-6.5)           → rodea el bloque sólido y luego aparece una caja en la salida
  5  Corredor NW estrecho (goal -2,6)                        → caja estática box_nw + caja dinámica cercana, paso estrecho
  6  Cadena norte: hueco + tramo NE (goal 9,3)                → dos obstáculos en cadena en una ruta norte nunca probada
  7  Corredor SW ajustado (solo estático, goal -11,-6.5)     → paso estrecho junto a west_wall y solid_block, sin dinámico
  8  Diagonal SW→NE con doble bloqueo (goal 6,6)             → cruce largo del mapa, dos cajas de reacción rápida

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
        "name":   "Ruta norte-oeste — obstáculo en línea recta",
        "x": -9.0, "y": 6.5,  "z": TAKEOFF_HEIGHT,
        "expect": (
            "El dron sube en línea recta desde el spawn (-9,0) hacia el norte, por la\n"
            "mitad oeste del mapa (nunca antes probado). Caja en (-9.0,3.0), trigger 3.5m:\n"
            "corta el paso directo. Debe frenar con el LiDAR, el mapa confirma y replantea\n"
            "un rodeo (este u oeste) evitando la caja estática box_nw más al norte (-5,4)."
        ),
        "obstacles": [
            {
                "name": "obs_nw_path",
                "pos":  (-9.0, 3.0,  1.0),
                "size": (1.2,  1.2,  2.0),
                "trigger_dist": 3.5,
                "color": (0.9, 0.1, 0.1),
            },
        ],
    },
    2: {
        "name":   "Tras el hueco central, rumbo NE — obstáculo al girar",
        "x": 5.0,  "y": 5.0,  "z": TAKEOFF_HEIGHT,
        "expect": (
            "El dron debe cruzar el ÚNICO paso este-oeste (hueco en x=0, y en [-1.5,1.5])\n"
            "y girar al norte hacia (5,5), ruta nunca probada. Caja en (2.0,3.0), trigger\n"
            "3.0m: aparece justo al girar tras cruzar el hueco. Prueba el replan en una\n"
            "zona distinta al corredor sur ya validado."
        ),
        "obstacles": [
            {
                "name": "obs_post_gap",
                "pos":  (2.0,  3.0,  1.0),
                "size": (1.0,  1.0,  2.0),
                "trigger_dist": 3.0,
                "color": (0.9, 0.5, 0.1),
            },
        ],
    },
    3: {
        "name":   "Bloqueo dinámico del hueco central",
        "x": 9.0,  "y": 0.0,  "z": TAKEOFF_HEIGHT,
        "expect": (
            "Goal directo al este (9,0): la única vía posible es el hueco en x=0,\n"
            "y=[-1.5,1.5]. Caja de 1x1.4m centrada en (0.0,0.0), trigger 3.0m: bloquea\n"
            "por completo el único paso este-oeste del mapa. Caso crítico nunca probado:\n"
            "no hay rodeo posible sin ir hasta y=+/-4.75 (bordes de las paredes divisorias)\n"
            "y esperar a que A* encuentre el hueco libre o el frontier más cercano.\n"
            "Puede acabar en SUCCESS tras un rodeo largo, o en ABORT si no hay margen —\n"
            "en ambos casos, ver cómo se comporta el replanner ante un choke point cerrado."
        ),
        "obstacles": [
            {
                "name": "obs_gap_block",
                "pos":  (0.0,  0.0,  1.0),
                "size": (1.0,  1.4,  2.0),
                "trigger_dist": 3.0,
                "color": (0.8, 0.0, 0.8),   # magenta
            },
        ],
    },
    4: {
        "name":   "Bloque sólido SW + obstáculo dinámico a la salida",
        "x": -9.0, "y": -6.5, "z": TAKEOFF_HEIGHT,
        "expect": (
            "El dron rodea el bloque sólido estático (x=[-8,-4], y=[-6,-3]) para llegar\n"
            "a (-9,-6.5), zona nunca antes usada como goal. Caja dinámica en (-6.0,-6.5),\n"
            "trigger 3.0m: aparece justo en la salida del rodeo, cuando el dron ya ha\n"
            "pasado el bloque. Prueba un replan encadenado (estático + dinámico) en la\n"
            "esquina SW en vez de la SE ya validada."
        ),
        "obstacles": [
            {
                "name": "obs_sw_exit",
                "pos":  (-6.0, -6.5,  1.0),
                "size": (1.0,  1.0,   2.0),
                "trigger_dist": 3.0,
                "color": (0.1, 0.6, 0.9),   # azul claro
            },
        ],
    },
    5: {
        "name":   "Corredor NW estrecho — estático + dinámico",
        "x": -2.0, "y": 6.0,  "z": TAKEOFF_HEIGHT,
        "expect": (
            "Goal en (-2,6), zona norte-oeste. La caja estática box_nw (-5,4, 1.5x1.0m)\n"
            "ya reduce el corredor; una caja dinámica en (-3.0,5.0), trigger 3.0m,\n"
            "lo estrecha más. Prueba que A*/LiDAR no confundan el hueco libre restante\n"
            "con espacio ocupado, y que el rodeo final sea correcto en un paso estrecho\n"
            "combinado (nunca probado: siempre había un solo obstáculo por caso)."
        ),
        "obstacles": [
            {
                "name": "obs_nw_narrow",
                "pos":  (-3.0, 5.0,   1.0),
                "size": (1.0,  1.0,   2.0),
                "trigger_dist": 3.0,
                "color": (0.6, 0.1, 0.8),   # morado
            },
        ],
    },
    6: {
        "name":   "Cadena norte: hueco + tramo NE",
        "x": 9.0,  "y": 3.0,  "z": TAKEOFF_HEIGHT,
        "expect": (
            "Goal en (9,3), justo al oeste de la sala cerrada NE (accesible, no dentro).\n"
            "Ruta nunca probada: cruza el hueco central y avanza por el norte en vez del\n"
            "sur. Dos obstáculos en cadena:\n"
            "  OBS-1 (1.5, 2.0) — cerca de la salida del hueco, trigger 3.0m\n"
            "  OBS-2 (5.5, 3.5) — a mitad de la recta final, trigger 3.0m\n"
            "Prueba varios replans seguidos en una zona distinta al corredor sur."
        ),
        "obstacles": [
            {
                "name": "obs_chain_n1",
                "pos":  (1.5,  2.0,   1.0),
                "size": (1.0,  1.0,   2.0),
                "trigger_dist": 3.0,
                "color": (0.9, 0.1, 0.1),
            },
            {
                "name": "obs_chain_n2",
                "pos":  (5.5,  3.5,   1.0),
                "size": (1.0,  1.0,   2.0),
                "trigger_dist": 3.0,
                "color": (0.9, 0.8, 0.1),
            },
        ],
    },
    7: {
        "name":   "Corredor SW ajustado — solo estático",
        "x": -11.0, "y": -6.5, "z": TAKEOFF_HEIGHT,
        "expect": (
            "Sin obstáculo dinámico: solo prueba que A* encuentre el paso estrecho entre\n"
            "west_wall (x=-13) y solid_block (x=[-8,-4], y=[-6,-3]) para llegar a un goal\n"
            "muy pegado a la esquina SW, nunca usado antes. Sirve de caso de control (sin\n"
            "capa reactiva) para comparar con el caso 4, que añade dinámico en la misma zona."
        ),
    },
    8: {
        "name":   "Diagonal SW→NE con doble bloqueo de reacción rápida",
        "x": 6.0,  "y": 6.0,  "z": TAKEOFF_HEIGHT,
        "expect": (
            "Cruce más largo del mapa: de (-9,0) a (6,6), pegado al borde oeste de la\n"
            "sala cerrada NE (sin entrar en ella). Dos cajas con trigger reducido (2.0m,\n"
            "reacción rápida) en zonas nunca combinadas:\n"
            "  OBS-1 (0.5, 1.0)  — justo al cruzar el hueco central\n"
            "  OBS-2 (4.5, 5.0)  — cerca del goal final, junto a enc_west_wall (x=7)\n"
            "Caso más exigente del set: replan encadenado + margen de reacción corto en\n"
            "una ruta diagonal completa."
        ),
        "obstacles": [
            {
                "name": "obs_diag1",
                "pos":  (0.5,  1.0,   1.0),
                "size": (1.0,  1.0,   2.0),
                "trigger_dist": 2.0,
                "color": (0.9, 0.0, 0.4),   # rosa fuerte
            },
            {
                "name": "obs_diag2",
                "pos":  (4.5,  5.0,   1.0),
                "size": (1.0,  1.0,   2.0),
                "trigger_dist": 2.0,
                "color": (0.1, 0.3, 0.9),   # azul
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
