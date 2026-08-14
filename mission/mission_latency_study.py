#!/usr/bin/env python3
"""
mission_latency_study.py — Estudio de latencia de la cadena de detección/replan

Objetivo: para un caso de ruta y una velocidad dados, barrer la distancia a la
que se "suelta" un obstáculo ya pre-spawneado (en vez de crearlo de cero) sobre
la trayectoria del dron, y registrar todos los datos crudos necesarios para
luego calcular (en analyze_latency_study.py):
  - la distancia mínima real a la que se puede soltar el obstáculo sin choque,
    por velocidad,
  - el desglose de tiempos de cada eslabón: LiDAR ve el obstáculo -> su celda
    en el mapa que usa A* queda ocupada -> MAP_CHECK lo detecta -> replan ->
    modify aceptado -> reacción física del dron.

Mecanismo de "mover" el obstáculo (no crear de nuevo): se spawnea una caja
estática UNA VEZ al principio, aparcada a z=5m sobre su posición objetivo
(fuera de la banda 0-1m que ve el LiDAR -> no dela "fantasma" en el mapa), y
se teletransporta a z=1m vía el servicio ROS2 nativo de aerostack2
`/world/<world>/set_pose` (ros_gz_interfaces/srv/SetEntityPose), expuesto por
el nodo `set_entity_pose_bridge` de as2_gazebo_assets. Requiere que el yaml de
mundo declare `world_bridges: [set_entity_pose]` (ya añadido a world_test.yaml).

Uso:
  python3 mission_latency_study.py <caso> <speed> <drop_dist> [label]

  caso       : "recto" (ruta este en línea recta) | "giro" (ruta con giro)
  speed      : velocidad de crucero m/s (p.ej. 0.5, 1.0, 1.5)
  drop_dist  : distancia dron-obstáculo (m) a la que se suelta el objeto
  label      : sufijo opcional para identificar la repetición (p.ej. "rep1")

Salida:
  CSV de telemetría  : /root/sherec_nav/sherec_nav_<ts>_latstudy_....csv
  JSON de resumen     : mismo nombre con extensión .json (t_drop, outcome,
                        min_dist_to_obstacle, etc.) — lo consume
                        analyze_latency_study.py junto al CSV y al
                        nav_planner_diag.log (logs [LAT] del planner).
"""

import json
import math
import os
import subprocess
import sys
import threading
import time

import rclpy
from as2_msgs.msg import YawMode
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.modules.navigate_to_module import NavigateToModule
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity

sys.path.insert(0, os.path.dirname(__file__))
from telemetry_logger import TelemetryLogger  # noqa: E402
from spawn_utils import spawn_static_box  # noqa: E402

WORLD = "nav_test_world"
TAKEOFF_HEIGHT = 1.0
MAX_WAIT_S = 150  # subido de 60s: el caso "abierto" recorre ~18.4m (vs ~11m de recto/giro)
PARK_Z = 5.0          # aparcado fuera de la banda 0-1m que ve el LiDAR
DROP_Z = 1.0          # altura real del obstáculo al soltarlo
COLLISION_DIST = 1.0  # radio_caja(0.5) + safety_distance(0.5) -> despeje mínimo
OBSTACLE_SIZE = (1.0, 1.0, 2.0)
STUCK_TIMEOUT_S = 20.0
STUCK_PROGRESS_EPS_M = 0.3
CRASH_Z_FLOOR_M = 0.25
CRASH_STARTUP_GRACE_S = 4.0
CRASH_PERSISTENCE_S = 0.3

CASES = {
    "recto": {
        "goal": (2.0, 0.0, TAKEOFF_HEIGHT),
        "obstacle_xy": (-4.0, 0.0),
        "desc": "Ruta este en línea recta (y=0 constante)",
    },
    "giro": {
        "goal": (2.0, 3.0, TAKEOFF_HEIGHT),
        "obstacle_xy": (0.5, 2.5),
        "desc": "Ruta con giro tras el hueco central hacia el norte",
    },
    "abierto": {
        "goal": (9.0, -4.0, TAKEOFF_HEIGHT),
        "obstacle_xy": (4.0, -4.0),
        "desc": (
            "Meta lejana en zona inicialmente sin explorar (fuerza modo frontera "
            "al despegar, como el caso 1 de mission_nav_test.py), tras cruzar el "
            "hueco central. Obstáculo en (4,-4), en espacio abierto: >=1.9m de "
            "despeje a solid_block/box_se/div_wall_south/south_wall en cualquier "
            "dirección — deliberadamente SIN el pasillo estrecho de 'recto'/'giro'."
        ),
    },
    "abierto_temprano": {
        "goal": (9.0, -4.0, TAKEOFF_HEIGHT),
        "obstacle_xy": (-4.0, 0.0),
        "desc": (
            "Mismo goal lejano/no explorado que 'abierto' (fuerza modo frontera "
            "al despegar) pero el obstáculo está en la PRIMERA recta, nada mas "
            "salir (mismo punto validado en 'recto': (-4,0), sobre el primer "
            "tramo recto y=0 que comparten las tres rutas) — no cronometrado "
            "para coincidir con el instante del replan al frontier. Objetivo: "
            "ver si esquivar un obstaculo temprano interactua mal con el modo "
            "frontera de un goal desconocido (2026-07-28, tras el fix de "
            "reintento en frontier_stuck)."
        ),
    },
    "abierto_var1": {
        "goal": (9.0, -4.0, TAKEOFF_HEIGHT),
        "obstacle_xy": (5.0, -1.0),
        "desc": (
            "Mismo goal lejano/frontera que 'abierto', obstáculo en otra posición "
            "SOBRE la curva real de la ruta hacia (9,-4) (comprobado en vuelo: la "
            "ruta pasa por (4.9,-0.8)/(5.3,-0.9) a esa altura), lejos de "
            "solid_block/box_se/div_wall_south (>=3.7m). Corrige la posición "
            "(0,-6) original, que quedaba a >5m de la trayectoria real y no "
            "probaba nada (2026-08-14)."
        ),
    },
    "abierto_var2": {
        "goal": (9.0, -4.0, TAKEOFF_HEIGHT),
        "obstacle_xy": (7.0, -3.0),
        "desc": (
            "Mismo goal lejano/frontera que 'abierto', obstáculo en otra posición "
            "SOBRE la curva real de la ruta (comprobado en vuelo: pasa por "
            "(6.9,-2.5)/(7.45,-3.08) a esa altura), más cerca del tramo final "
            "junto a box_se (~2m de despeje) que 'abierto_var1'. Segunda posición "
            "para verificar robustez espacial, ya sí sobre la trayectoria "
            "(2026-08-14)."
        ),
    },
}


class TestDrone(DroneInterface):
    def __init__(self):
        super().__init__("drone0", verbose=False, use_sim_time=True)
        self.navigate_to = NavigateToModule(drone=self)


def sim_now(node) -> float:
    return node.get_clock().now().nanoseconds * 1e-9


def make_set_pose_client(node):
    client = node.create_client(SetEntityPose, f"/world/{WORLD}/set_pose")
    if not client.wait_for_service(timeout_sec=10.0):
        print(
            f"\n  [ERROR] Servicio /world/{WORLD}/set_pose no disponible.\n"
            f"  Comprueba que config/gazebo/world_test.yaml declara "
            f"'world_bridges: [set_entity_pose]' y que el mundo se relanzó."
        )
        sys.exit(1)
    return client


def drop_obstacle(node, client, name: str, x: float, y: float, z: float) -> float:
    """Teletransporta la entidad ya spawneada a (x, y, z).

    Devuelve (t_send, t_ack) en sim-time. El objeto se mueve físicamente en
    Gazebo en torno al instante de ENVÍO de la petición, no al de recepción
    del ACK — el piloto mostró un [LAT] DETECT del planner ANTES del t_ack
    registrado, confirmando que el ACK llega con retraso respecto al cambio
    físico real. t_send es la referencia correcta para medir L_lidar/L_cell/
    L_mapcheck; (t_ack - t_send) es la latencia del propio servicio ROS/GZ,
    útil como diagnóstico aparte.
    """
    req = SetEntityPose.Request()
    req.entity = Entity()
    req.entity.name = name
    req.entity.type = Entity.MODEL
    req.pose.position.x = x
    req.pose.position.y = y
    req.pose.position.z = z
    req.pose.orientation.w = 1.0
    t_send = sim_now(node)
    future = client.call_async(req)
    # No usar rclpy.spin_until_future_complete(node, ...): crearía otro
    # executor temporal sobre el mismo nodo (ver nota en run_study()). El
    # spin_thread interno de DroneInterfaceBase ya procesa la respuesta del
    # servicio; solo hay que esperar a que el future se resuelva.
    t_wait_start = time.time()
    while not future.done() and time.time() - t_wait_start < 3.0:
        time.sleep(0.01)
    t_ack = sim_now(node)
    if future.done() and future.result() is not None and future.result().success:
        print(
            f"  [DROP] set_pose OK -> ({x},{y},{z}) "
            f"t_send={t_send:.3f} t_ack={t_ack:.3f} (rtt={t_ack - t_send:.3f}s)")
    else:
        print(f"  [DROP][WARN] set_pose no confirmó éxito (future={future.done()})")
    return t_send, t_ack


def make_drop_thread(node, client, name, ox, oy, drop_dist, drone, result: dict):
    """Hilo que espera a que el dron esté a <= drop_dist del obstáculo y lo suelta."""

    def _run():
        t_start = time.time()
        while time.time() - t_start < 3.0:
            time.sleep(0.05)
        print(f"[DROP-{name}] Esperando dist <= {drop_dist}m ...")
        while True:
            p = drone.position
            d = math.hypot(p[0] - ox, p[1] - oy)
            if d <= drop_dist:
                break
            time.sleep(0.02)
        p = drone.position
        d2 = math.hypot(p[0] - ox, p[1] - oy)
        print(
            f"[DROP-{name}] dist={d2:.2f} dron=({p[0]:.2f},{p[1]:.2f}) -> soltando"
        )
        t_send, t_ack = drop_obstacle(node, client, name, ox, oy, DROP_Z)
        result["t_drop_sim"] = t_send
        result["t_drop_sim_ack"] = t_ack
        result["drop_dist_actual"] = d2
        result["drone_pos_at_drop"] = [p[0], p[1], p[2]]

    return threading.Thread(target=_run, daemon=True)


def run_study(case_name: str, speed: float, drop_dist: float, label: str) -> None:
    case = CASES[case_name]
    ox, oy = case["obstacle_xy"]
    gx, gy, gz = case["goal"]
    obstacle_name = "study_obstacle"

    session_label = f"latstudy_{case_name}_v{speed}_d{drop_dist}_{label}"
    print(f"\n{'='*70}")
    print(f"  ESTUDIO DE LATENCIA — caso={case_name} ({case['desc']})")
    print(f"  speed={speed} m/s   drop_dist={drop_dist} m   label={label}")
    print(f"  goal=({gx},{gy},{gz})   obstacle=({ox},{oy})")
    print(f"{'='*70}\n")

    rclpy.init()
    drone = TestDrone()
    # DroneInterfaceBase.__init__ ya crea su propio executor interno y arranca
    # su propio spin_thread (as2_python_api/drone_interface_base.py) a 20Hz.
    # Añadir un SEGUNDO executor sobre el mismo nodo (como hacía
    # mission_nav_test.py) hace que ambos compitan por las callbacks del
    # mismo nodo — bajo carga (muchos replans, muchos mensajes) una de las
    # dos deja de procesar, y drone.position se congela mientras el nodo C++
    # del planner (con su propia suscripción) sigue viendo la pose real
    # actualizarse. Esto reproduce el "Hallazgo 4" ya documentado
    # (telemetría Python desincronizada tras el colapso). No crear un
    # executor propio aquí: basta con el interno de DroneInterfaceBase.

    set_pose_client = make_set_pose_client(drone)
    logger = TelemetryLogger(drone)

    drop_result: dict = {}
    outcome = "UNKNOWN"
    min_dist_to_obstacle = float("inf")
    cellmon_proc = None
    cellmon_json = ""

    try:
        time.sleep(1.0)

        print("[PARK] Spawn estático del obstáculo a z=%.1fm (fuera del scan)..." % PARK_Z)
        spawn_static_box(
            WORLD, obstacle_name, ox, oy, PARK_Z,
            sx=OBSTACLE_SIZE[0], sy=OBSTACLE_SIZE[1], sz=OBSTACLE_SIZE[2],
        )
        time.sleep(1.0)

        csv_path = logger.start(gx, gy, gz, session_label)
        cellmon_json = os.path.splitext(csv_path)[0] + "_cellmon.json"
        # Medir sobre la CARA que mira al dron, no el centro de la caja: el
        # centro queda ocluido por la cara frontal en cuanto esta empieza a
        # devolver impactos y nunca llega a marcarse ocupado (lección ya
        # documentada — ver memoria project-map-latency-study). El dron se
        # aproxima desde el oeste (spawn en x=-9) en ambos casos, así que la
        # cara cercana es la del lado -x.
        face_x = ox - OBSTACLE_SIZE[0] / 2.0
        face_y = oy
        cellmon_proc = subprocess.Popen(
            ["python3", os.path.join(os.path.dirname(__file__), "latency_cell_monitor.py"),
             str(face_x), str(face_y), cellmon_json, str(MAX_WAIT_S + 30)],
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
        t_start_sim = sim_now(drone)
        t_start_wall = time.time()

        drop_thread = make_drop_thread(
            drone, set_pose_client, obstacle_name, ox, oy, drop_dist, drone, drop_result)
        drop_thread.start()

        print(f"[4/4] Navigate to [{gx}, {gy}, {gz}] a {speed} m/s...")
        try:
            drone.navigate_to(
                gx, gy, gz,
                speed=speed,
                yaw_mode=YawMode.PATH_FACING,
                wait=False,
            )
        except BehaviorHandler.GoalRejected:
            print("  [!] Goal rechazado inmediatamente por el servidor")
            outcome = "REJECTED"

        # Dar margen a que el behavior server acepte el goal antes de fiarse
        # de is_running(): con wait=False hay una ventana breve (acción
        # async) en la que is_running() puede devolver False todavía —
        # comprobarlo inmediatamente después de navigate_to() puede
        # declarar "ABORTED" en <1s con el dron aún en el spawn (visto en
        # batch2/rep2 y otra vez con margen de 2s en fixval/rep2 — la
        # ventana real puede superar 2s alguna vez). Reintentar hasta 5s, y
        # si sigue sin arrancar, reintentar el propio navigate_to() una vez
        # antes de rendirse.
        if outcome == "UNKNOWN":
            t_accept_deadline = time.time() + 5.0
            while not drone.navigate_to.is_running() and time.time() < t_accept_deadline:
                time.sleep(0.05)
            if not drone.navigate_to.is_running():
                print("  [!] is_running() seguía en False tras 5s — reintentando navigate_to()")
                try:
                    drone.navigate_to(
                        gx, gy, gz, speed=speed, yaw_mode=YawMode.PATH_FACING, wait=False)
                except BehaviorHandler.GoalRejected:
                    print("  [!] Goal rechazado en el reintento")
                    outcome = "REJECTED"
                t_accept_deadline = time.time() + 3.0
                while not drone.navigate_to.is_running() and time.time() < t_accept_deadline:
                    time.sleep(0.05)

        deadline = time.time() + MAX_WAIT_S
        last_progress_t = time.time()
        best_dist_to_goal = float("inf")
        crash_since = None
        navigating_since = time.time()

        while outcome == "UNKNOWN" and drone.navigate_to.is_running() and time.time() < deadline:
            elapsed = time.time() - t_start_wall
            pos = drone.position
            d_goal = math.hypot(pos[0] - gx, pos[1] - gy)
            d_obs = math.hypot(pos[0] - ox, pos[1] - oy)
            min_dist_to_obstacle = min(min_dist_to_obstacle, d_obs)

            if d_goal < best_dist_to_goal - STUCK_PROGRESS_EPS_M:
                best_dist_to_goal = d_goal
                last_progress_t = time.time()
            if time.time() - last_progress_t > STUCK_TIMEOUT_S:
                outcome = "STUCK"
                break

            if time.time() - navigating_since > CRASH_STARTUP_GRACE_S:
                if pos[2] < CRASH_Z_FLOOR_M:
                    if crash_since is None:
                        crash_since = time.time()
                    elif time.time() - crash_since > CRASH_PERSISTENCE_S:
                        outcome = "CRASH"
                        break
                else:
                    crash_since = None

            if min_dist_to_obstacle < COLLISION_DIST and "drop_dist_actual" in drop_result:
                outcome = "COLLISION"
                break

            print(
                f"  t={elapsed:5.1f}s pos=[{pos[0]:6.2f},{pos[1]:6.2f},{pos[2]:5.2f}]"
                f" d_goal={d_goal:5.2f} d_obs={d_obs:5.2f} min_d_obs={min_dist_to_obstacle:5.2f}",
                end="\r",
            )
            time.sleep(0.1)

        elapsed = time.time() - t_start_wall
        pos = drone.position
        d_goal_final = math.hypot(pos[0] - gx, pos[1] - gy)

        if outcome == "UNKNOWN":
            if time.time() >= deadline:
                outcome = "TIMEOUT"
            elif d_goal_final < 1.0:
                outcome = "SUCCESS"
            else:
                outcome = "ABORTED"

        logger.mark_event(f"END_{outcome}")
        print(f"\n\n--- Resultado ---")
        print(f"  Outcome            : {outcome}")
        print(f"  Tiempo total        : {elapsed:.1f}s")
        print(f"  Posición final      : [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
        print(f"  Dist. final a goal  : {d_goal_final:.2f}m")
        print(f"  Min dist obstáculo  : {min_dist_to_obstacle:.2f}m")
        if "t_drop_sim" in drop_result:
            print(f"  t_drop (sim)        : {drop_result['t_drop_sim']:.3f}")

    finally:
        csv_final = logger.stop()
        if cellmon_proc is not None:
            cellmon_proc.terminate()
            try:
                cellmon_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                cellmon_proc.kill()
        print("\nAterrizando...")
        try:
            drone.land(speed=0.5)
            time.sleep(2.0)
            drone.disarm()
        except Exception as exc:
            print(f"  [WARN] fallo en aterrizaje/disarm: {exc}")
        drone.shutdown()
        rclpy.shutdown()

        summary = {
            "case": case_name,
            "speed": speed,
            "drop_dist_requested": drop_dist,
            "label": label,
            "obstacle_xy": [ox, oy],
            "goal": [gx, gy, gz],
            "collision_dist_threshold": COLLISION_DIST,
            "outcome": outcome,
            "min_dist_to_obstacle": min_dist_to_obstacle,
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
    if len(sys.argv) not in (4, 5) or sys.argv[1] not in CASES:
        print(__doc__)
        sys.exit(1)
    case_arg = sys.argv[1]
    speed_arg = float(sys.argv[2])
    drop_dist_arg = float(sys.argv[3])
    label_arg = sys.argv[4] if len(sys.argv) == 5 else "run"
    run_study(case_arg, speed_arg, drop_dist_arg, label_arg)
