#!/usr/bin/env python3
"""
mission_detection_probe.py — Latencia de DETECCIÓN pura, desacoplada del vuelo

A diferencia de mission_latency_study.py (que mide todo el pipeline incluida
la reacción/evitación del dron en movimiento), este script aísla solo la
pregunta: "desde que un obstáculo aparece delante del dron, ¿cuánto tarda el
sistema en SABER que está ahí (celda ocupada / MAP_CHECK lo detecta)?" — sin
que la dinámica de vuelo, la replanificación o el riesgo de colisión influyan
en la medida.

Mecánica:
  - El dron despega y se queda prácticamente parado (solo mantiene un
    navigate_to activo hacia un goal lejano para que el MAP_CHECK del
    planner siga corriendo — sin eso no hay [LAT] DETECT que medir).
  - El obstáculo, ya aparcado a z=5m (fuera de la banda del LiDAR) desde el
    principio, se suelta a una distancia FIJA delante del punto de spawn muy
    poco después de despegar (el dron apenas se ha movido en ese intervalo,
    así que la distancia de soltado ≈ distancia real).
  - Se aterriza en cuanto se ha capturado la detección (o tras un margen),
    sin esperar a que el dron llegue a ningún sitio ni le importe si
    colisionaría — no es relevante para esta medida.

Uso:
  python3 mission_detection_probe.py <distance> [label]

  distance : distancia (m) desde el spawn a la que se coloca el obstáculo
  label    : sufijo opcional para la repetición

Salida: mismo esquema de CSV/JSON/cellmon que mission_latency_study.py,
reutilizable directamente por analyze_latency_study.py (mismo formato de
resumen), con "case"="probe" y "drop_dist_requested"=distance.
"""

import json
import math
import os
import subprocess
import sys
import time

import rclpy
from as2_msgs.msg import YawMode
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler

sys.path.insert(0, os.path.dirname(__file__))
from mission_latency_study import (  # noqa: E402
    DROP_Z, OBSTACLE_SIZE, PARK_Z, TestDrone, WORLD,
    drop_obstacle, make_set_pose_client, sim_now,
)
from spawn_utils import spawn_static_box  # noqa: E402
from telemetry_logger import TelemetryLogger  # noqa: E402

TAKEOFF_HEIGHT = 1.0
SPAWN_XY = (-9.0, 0.0)
GOAL = (2.0, 0.0, TAKEOFF_HEIGHT)   # goal lejano, solo para mantener MAP_CHECK activo
NAV_SPEED = 0.3                      # deliberadamente lento: minimiza recorrido mientras
                                      # se captura la detección (ver STOP_ON_DETECT)
DROP_DELAY_S = 2.0                   # soltar poco después de iniciar navigate_to
CAPTURE_WINDOW_S = 10.0              # margen MÁXIMO (sim-time) tras el drop si nunca
                                      # se detecta (p.ej. distancia fuera de rango)
OBSTACLE_NAME = "probe_obstacle"


def run_probe(distance: float, label: str) -> None:
    ox, oy = SPAWN_XY[0] + distance, SPAWN_XY[1]
    session_label = f"detprobe_d{distance}_{label}"

    print(f"\n{'='*70}")
    print(f"  SONDA DE DETECCIÓN — distancia={distance}m  label={label}")
    print(f"  obstáculo en ({ox},{oy})  (dron prácticamente parado en el spawn)")
    print(f"{'='*70}\n")

    rclpy.init()
    drone = TestDrone()
    set_pose_client = make_set_pose_client(drone)
    logger = TelemetryLogger(drone)

    drop_result = {}
    cellmon_proc = None
    cellmon_json = ""

    try:
        time.sleep(1.0)

        print("[PARK] Spawn estático del obstáculo a z=%.1fm..." % PARK_Z)
        spawn_static_box(
            WORLD, OBSTACLE_NAME, ox, oy, PARK_Z,
            sx=OBSTACLE_SIZE[0], sy=OBSTACLE_SIZE[1], sz=OBSTACLE_SIZE[2],
        )
        time.sleep(1.0)

        csv_path = logger.start(GOAL[0], GOAL[1], GOAL[2], session_label)
        cellmon_json = os.path.splitext(csv_path)[0] + "_cellmon.json"
        # Cara del obstáculo que mira al dron (viene desde -x)
        face_x = ox - OBSTACLE_SIZE[0] / 2.0
        cellmon_proc = subprocess.Popen(
            ["python3", os.path.join(os.path.dirname(__file__), "latency_cell_monitor.py"),
             str(face_x), str(oy), cellmon_json, str(CAPTURE_WINDOW_S + DROP_DELAY_S + 15)],
        )

        print("[1/4] Offboard...")
        drone.offboard()
        time.sleep(1.0)
        print("[2/4] Arm...")
        drone.arm()
        time.sleep(1.0)
        print(f"[3/4] Takeoff a {TAKEOFF_HEIGHT}m...")
        drone.takeoff(height=TAKEOFF_HEIGHT, speed=0.5)
        time.sleep(2.0)

        logger.mark_event("NAV_START")

        # Detectar el [LAT] DETECT leyendo directamente nav_planner_diag.log
        # en vez de suscribirse a /rosout: se probó la suscripción ROS y,
        # aunque SÍ recibía mensajes normales (MAP_CHECK tick a 0.5s), el
        # burst de ~8-10 mensajes que se publican casi simultáneos justo en
        # el instante del replan (DETECT+REPLAN_START+ASTAR_DONE+MODIFY+DIAG)
        # se perdía — el spin_once() no bloqueante de DroneInterfaceBase
        # (20Hz) no drena toda la ráfaga a tiempo. Leer el fichero es
        # inmune a ese problema.
        DIAG_LOG_PATH = "/root/sherec_nav/nav_planner_diag.log"

        def check_detect_in_log() -> bool:
            try:
                with open(DIAG_LOG_PATH, "r", errors="replace") as f:
                    return "[LAT] DETECT" in f.read()
            except FileNotFoundError:
                return False

        print(f"[4/4] Navigate to {GOAL} a {NAV_SPEED} m/s (solo para activar MAP_CHECK)...")
        try:
            drone.navigate_to(
                GOAL[0], GOAL[1], GOAL[2],
                speed=NAV_SPEED, yaw_mode=YawMode.PATH_FACING, wait=False,
            )
        except BehaviorHandler.GoalRejected:
            print("  [!] Goal rechazado")

        print(f"[DROP] Esperando {DROP_DELAY_S}s antes de soltar (dron apenas se mueve)...")
        time.sleep(DROP_DELAY_S)

        pos = drone.position
        real_dist = math.hypot(pos[0] - ox, pos[1] - oy)
        print(f"  Posición dron al soltar: [{pos[0]:.2f},{pos[1]:.2f}] "
              f"(distancia real al obstáculo: {real_dist:.2f}m)")

        t_send, t_ack = drop_obstacle(drone, set_pose_client, OBSTACLE_NAME, ox, oy, DROP_Z)
        drop_result["t_drop_sim"] = t_send
        drop_result["t_drop_sim_ack"] = t_ack
        drop_result["drop_dist_actual"] = real_dist
        drop_result["drone_pos_at_drop"] = [pos[0], pos[1], pos[2]]
        logger.mark_event("DROP")

        # Usar tiempo de SIMULACIÓN para la ventana de captura, no reloj real:
        # Gazebo puede correr con factor tiempo-real <1 bajo carga (varios
        # replans, etc.), así que "8s de reloj real" no siempre cubren "8s
        # de tiempo simulado" — un piloto de validación mostró un DETECT a
        # t_sim=5.3s que mi ventana de 8s en reloj real no llegó a capturar
        # porque el sim iba más lento que el reloj de la máquina.
        t_capture_end_sim = t_send + CAPTURE_WINDOW_S
        print(f"[CAPTURE] Esperando detección (máx {CAPTURE_WINDOW_S}s sim-time)...")
        t_wall_start = time.time()
        wall_deadline = t_wall_start + CAPTURE_WINDOW_S * 4  # cinturón de seguridad
        stopped_early = False
        while sim_now(drone) < t_capture_end_sim and time.time() < wall_deadline:
            p = drone.position
            detected = check_detect_in_log()
            print(
                f"  t_sim={sim_now(drone)-t_send:5.1f}s (wall={time.time()-t_wall_start:5.1f}s)"
                f" pos=[{p[0]:6.2f},{p[1]:6.2f},{p[2]:5.2f}] detected={detected}",
                end="\r")
            if detected and not stopped_early:
                print(f"\n[CAPTURE] [LAT] DETECT visto en t_sim={sim_now(drone)-t_send:.2f}s"
                      f" — frenando navigate_to ya")
                try:
                    drone.navigate_to.stop()
                except Exception as exc:
                    print(f"  [WARN] fallo al frenar navigate_to: {exc}")
                stopped_early = True
                # Un pequeño margen extra tras frenar para que el cellmon
                # capture también t_cell_100 si tarda un poco más.
                time.sleep(1.0)
                break
            time.sleep(0.2)
        print()

    finally:
        csv_final = logger.stop()
        if cellmon_proc is not None:
            cellmon_proc.terminate()
            try:
                cellmon_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                cellmon_proc.kill()

        print("\nAterrizando (sin esperar a llegar a ningún sitio)...")
        try:
            drone.navigate_to.stop()
        except Exception:
            pass
        try:
            drone.land(speed=0.5)
            time.sleep(2.0)
            drone.disarm()
        except Exception as exc:
            print(f"  [WARN] fallo en aterrizaje/disarm: {exc}")
        drone.shutdown()
        rclpy.shutdown()

        summary = {
            "case": "probe",
            "speed": NAV_SPEED,
            "drop_dist_requested": distance,
            "label": label,
            "obstacle_xy": [ox, oy],
            "goal": list(GOAL),
            "collision_dist_threshold": 1.0,
            "outcome": "N/A_PROBE",
            "min_dist_to_obstacle": drop_result.get("drop_dist_actual"),
            "csv_path": csv_final,
            "cellmon_json_path": cellmon_json,
            **drop_result,
        }
        json_path = os.path.splitext(csv_final)[0] + ".json" if csv_final else ""
        if json_path:
            with open(json_path, "w") as f:
                json.dump(summary, f, indent=2)
            print(f"[SUMMARY] {json_path}")


if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        print(__doc__)
        sys.exit(1)
    distance_arg = float(sys.argv[1])
    label_arg = sys.argv[2] if len(sys.argv) == 3 else "run"
    run_probe(distance_arg, label_arg)
