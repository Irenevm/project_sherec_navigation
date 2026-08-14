#!/usr/bin/env python3
"""
latency_study_replan_v3case5.py — Estudio de retraso de replanificación real
(no sintético) para mission_nav_test_v3.py caso 5.

A diferencia de latency_study_mt_mg.py (que vuela su propio dron y spawnea su
propia caja en hover, midiendo solo M_t-vs-M_g), este script es un MONITOR
PASIVO: no controla el dron ni spawnea nada. Se ejecuta EN PARALELO mientras
Irene lanza `mission_nav_test_v3.py 5` (ruta recta ESTE, spawn(-9,0) ->
goal(2,0), obstáculo fijo en (-4.0,0.0) trigger_dist=3.5m, margen teórico del
LiDAR +1.00s). Escucha los mismos topics que ve el planner C++ y correlaciona:

  1. Cuándo el occupancy grid ve el objeto por primera vez:
       - M_t (map_instant, crudo, sin acumular) en la celda de la cara del
         obstáculo más cercana al dron (-4.5, 0.0).
       - M_g (map_filtered, acumulado) cruzando occ_threshold=50 en esa misma
         celda -> retraso M_g-menos-M_t (ya estudiado para un caso sintético
         en project_map_latency_study.md; aquí se remide para el objeto real
         de esta ruta).
  2. Si el cono de detección de la capa reactiva LLEGA A VER el objeto:
       parsea las líneas [LIDAR_DEBUG] corridor(require_new=true) raw=N ...
       del nodo path_planner vía /rosout. raw>0 significa que M_t tenía algo
       ocupado DENTRO del sector de búsqueda del LiDAR (aunque luego el
       filtro M_t-vs-M_g lo descarte) — si esto nunca aparece, el cono nunca
       encontró el objeto, sea cual sea el resto del pipeline.
  3. El retraso real de REPLANIFICACIÓN: parsea los waypoints del primer plan
       (al iniciar la misión, antes de que exista el objeto) y de cada replan
       posterior (bloque [DIAG replan] Waypoint[k/N] : [x, y] ...). Como la
       ruta es recta en y=0, cualquier replan cuyos waypoints se aparten de
       y=0 cerca de x=-4 es, por construcción, el primer plan que "tiene en
       cuenta" el obstáculo. Se reporta el instante de ese replan relativo a
       T0 (primer hit de M_t) — este es EL número que le interesa a Irene:
       cuánto tarda el sistema en pasar de "plan recto de antes" a "plan que
       lo esquiva", no solo cuánto tarda en darse cuenta de que existe.
  4. Duración pura del algoritmo A* en cada replan: delta entre "Activating
       A* plugin" y "Publishing path"/"Reduced path size" del mismo evento
       (aísla el cómputo del A* del resto de la latencia de detección).
  5. Transiciones de REACTIVE_MODE (si el LiDAR llega a frenar/decelerar) y
       eventos de MAP_CHECK (bloqueo/segmento ocupado/replanificación).

Todos los timestamps se toman de los campos `stamp` de los propios mensajes
(rcl_interfaces/Log.stamp para /rosout, OccupancyGrid.header.stamp para los
mapas) — no del reloj de pared de este proceso ni del de mission_nav_test_v3,
así que no hay problema de reloj-de-simulación-vs-wall-clock entre procesos
(ver limitación de este tipo ya documentada en project_map_latency_study.md).

Uso:
  Terminal A (dentro del container):
    python3 mission/mission_nav_test_v3.py 5
  Terminal B (dentro del container, EN PARALELO, lanzar justo antes o a la vez):
    python3 latency_study_replan_v3case5.py

Corre durante RUN_DURATION_S segundos (por defecto 100, suficiente para todo
el caso 5 con margen) y al final imprime el informe. Ctrl+C también fuerza el
informe con lo capturado hasta ese momento.
"""
import math
import re
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from rcl_interfaces.msg import Log

RUN_DURATION_S = 100.0

# Obstáculo fijo del caso v3 este (ver mission_nav_test_v3.py: OBSTACLE_EAST_POS,
# tamaño (1,1,2)). El dron viaja de x=-9 a x=2 por y=0, así que la cara del
# obstáculo que ve primero es la más cercana en -x.
OBS_X, OBS_Y = -4.0, 0.0
OBS_SIZE_X = 1.0
SAMPLE_X, SAMPLE_Y = OBS_X - OBS_SIZE_X / 2.0, OBS_Y  # (-4.5, 0.0)
OCC_THRESHOLD = 50  # occ_threshold_ en behavior_default.yaml


def stamp_to_float(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class ReplanLatencyMonitor(Node):

    def __init__(self):
        super().__init__("latency_study_replan_monitor")

        self.t0 = None          # primer hit M_t en la celda del obstáculo
        self.events = []        # (t_rel_o_none, tag, detail)

        self.mt_hit_t = None
        self.mg_threshold_t = None
        self.mt_last_val = None
        self.mg_last_val = None

        self.corridor_first_t = None
        self.corridor_events = []

        self.reactive_transitions = []
        self.last_reactive_text = None

        self.replans = []       # lista de dicts por replan: {t_astar_start, t_path_published, waypoints:[(x,y),...]}
        self.current_replan = None
        self.initial_plan_waypoints = None
        self.initial_plan_t = None

        self.map_check_events = []

        self.create_subscription(
            OccupancyGrid, "/drone0/map_instant", self._map_instant_cbk, 10)
        self.create_subscription(
            OccupancyGrid, "/drone0/map_filtered", self._map_filtered_cbk, 10)
        self.create_subscription(
            String, "/drone0/reactive_mode", self._reactive_mode_cbk, 10)
        self.create_subscription(
            Log, "/rosout", self._rosout_cbk, 50)

        self.get_logger().info(
            f"Monitor activo. Muestreando celda del obstáculo en "
            f"({SAMPLE_X}, {SAMPLE_Y}). Esperando datos...")

    # ── Occupancy sampling ──────────────────────────────────────────────
    def _sample_cell(self, msg: OccupancyGrid):
        info = msg.info
        if info.width == 0 or info.height == 0:
            return None
        ox = info.origin.position.x
        oy = info.origin.position.y
        res = info.resolution
        cx = int((SAMPLE_X - ox) / res)
        cy = int((SAMPLE_Y - oy) / res)
        if not (0 <= cx < info.width and 0 <= cy < info.height):
            return None
        idx = cy * info.width + cx
        if idx >= len(msg.data):
            return None
        return msg.data[idx]

    def _map_instant_cbk(self, msg: OccupancyGrid):
        val = self._sample_cell(msg)
        if val is None or val == self.mt_last_val:
            return
        self.mt_last_val = val
        t = stamp_to_float(msg.header.stamp)
        if val == 100 and self.mt_hit_t is None:
            self.mt_hit_t = t
            self.t0 = t
            self._log_event(t, "MT_HIT", f"M_t=100 en ({SAMPLE_X},{SAMPLE_Y}) -> T0")

    def _map_filtered_cbk(self, msg: OccupancyGrid):
        val = self._sample_cell(msg)
        if val is None or val == self.mg_last_val:
            return
        self.mg_last_val = val
        t = stamp_to_float(msg.header.stamp)
        if val is not None and val >= OCC_THRESHOLD and self.mg_threshold_t is None:
            self.mg_threshold_t = t
            self._log_event(t, "MG_THRESHOLD", f"M_g={val} cruza umbral {OCC_THRESHOLD}")

    # ── Reactive mode ───────────────────────────────────────────────────
    def _reactive_mode_cbk(self, msg: String):
        if msg.data == self.last_reactive_text:
            return
        self.last_reactive_text = msg.data
        t = time.time()  # este topic no lleva stamp -> wall clock, solo referencia relativa
        self.reactive_transitions.append((t, msg.data))
        self._log_event(None, "REACTIVE_MODE", msg.data)

    # ── /rosout parsing ─────────────────────────────────────────────────
    def _rosout_cbk(self, msg: Log):
        if "path_planner" not in msg.name:
            return
        text = msg.msg
        t = stamp_to_float(msg.stamp)

        if "[LIDAR_DEBUG] corridor(" in text:
            m = re.search(
                r"raw=(\d+) skipped_not_confirmed_free=(\d+) hits=(\d+)", text)
            if m:
                raw, skipped, hits = (int(x) for x in m.groups())
                if raw > 0 and self.corridor_first_t is None:
                    self.corridor_first_t = t
                    self._log_event(t, "CORRIDOR_FIRST_RAW_HIT", text)
                self.corridor_events.append((t, raw, skipped, hits))
            return

        if "[MAP_CHECK] Path segment occupied" in text or \
           "[MAP_CHECK] Path to goal" in text and "blocked" in text or \
           "[MAP_CHECK] Direct path to goal" in text:
            self.map_check_events.append((t, text))
            self._log_event(t, "MAP_CHECK", text)
            return

        if "Activating A* plugin" in text:
            # No pisar un current_replan ya abierto por "Replanning to original
            # goal" (que llama a on_activate() justo después, en el mismo
            # tick síncrono) — solo rellenar t_astar_start si falta.
            if self.current_replan is None:
                self.current_replan = {"t_astar_start": t, "waypoints": []}
            elif self.current_replan.get("t_astar_start") is None:
                self.current_replan["t_astar_start"] = t
            return

        if ("Reduced path size" in text or "Path size:" in text) and self.current_replan is not None:
            self.current_replan.setdefault("t_path_published", t)
            return

        if "DIAG replan] Path type" in text:
            if self.current_replan is None:
                self.current_replan = {"t_astar_start": None, "waypoints": []}
            self.current_replan["t_diag"] = t
            m = re.search(r"waypoints:\s*(\d+)", text)
            if m:
                self.current_replan["n_waypoints"] = int(m.group(1))
            return

        if "DIAG replan] Waypoint" in text:
            m = re.search(r"\[([\-\d.]+), ([\-\d.]+)\]\s*\|", text)
            if m and self.current_replan is not None:
                x, y = float(m.group(1)), float(m.group(2))
                self.current_replan["waypoints"].append((x, y))
            return

        if "Replanning to original goal" in text:
            # cierra el replan anterior (si lo había) y abre contexto para el nuevo
            if self.current_replan is not None and self.current_replan.get("waypoints"):
                self.replans.append(self.current_replan)
            self.current_replan = {"t_astar_start": None, "waypoints": [], "t_trigger": t}
            self._log_event(t, "REPLANNING_TO_GOAL", text)
            return

        if "Sending goal to FollowPath" in text and self.initial_plan_t is None:
            # Primer envío de FollowPath = plan inicial, antes de que exista el obstáculo.
            self.initial_plan_t = t
            if self.current_replan is not None:
                self.initial_plan_waypoints = list(self.current_replan.get("waypoints", []))
            return

    def _log_event(self, t, tag, detail):
        rel = f"{t - self.t0:+7.3f}s" if (t is not None and self.t0 is not None) else "  ??s  "
        print(f"[{rel}] {tag:22s} {detail}")

    def finalize(self):
        if self.current_replan is not None and self.current_replan.get("waypoints"):
            self.replans.append(self.current_replan)

    # ── Informe final ───────────────────────────────────────────────────
    def report(self):
        print("\n" + "=" * 70)
        print("INFORME — retraso de replanificación real (caso v3-5)")
        print("=" * 70)

        if self.t0 is None:
            print("No se detectó M_t=100 en la celda del obstáculo "
                  f"({SAMPLE_X},{SAMPLE_Y}) durante la ejecución. "
                  "Comprueba que el monitor arrancó a la vez que la misión "
                  "y que el caso realmente spawneó el obstáculo.")
            return

        print(f"\nT0 (M_t ve el obstáculo por primera vez): sim_t={self.t0:.3f}")

        if self.initial_plan_t is not None:
            print(
                f"Plan inicial (sin obstáculo, enviado a FollowPath al inicio "
                f"de la misión): sim_t={self.initial_plan_t:.3f} "
                f"({self.initial_plan_t - self.t0:+.3f}s respecto a T0, "
                f"lógicamente negativo si el objeto aún no existía)")

        if self.mg_threshold_t is not None:
            print(
                f"M_g cruza occ_threshold={OCC_THRESHOLD}: "
                f"+{self.mg_threshold_t - self.t0:.3f}s tras T0  "
                "(retraso del mapa global — cuánto tarda en 'creerse' lo que "
                "el sensor ya vio)")
        else:
            print("M_g NUNCA cruzó el umbral en la celda muestreada — revisar "
                  "resolución de muestreo o si el A* ve el obstáculo por otra celda.")

        if self.corridor_first_t is not None:
            print(
                f"Cono LiDAR ve algo por primera vez (raw>0): "
                f"+{self.corridor_first_t - self.t0:.3f}s tras T0  "
                "(SÍ hay hits crudos dentro del sector — el problema, si lo hay, "
                "está en el filtro M_t-vs-M_g o en la persistencia, no en el FOV)")
        else:
            print("El cono LiDAR NUNCA registró un solo hit crudo "
                  "(raw_occupied_in_corridor siempre 0) — el objeto nunca cayó "
                  "dentro del sector de búsqueda en ningún tick, sea cual sea "
                  "el resto de la lógica de filtrado.")

        if self.reactive_transitions:
            print("\nTransiciones de REACTIVE_MODE (tiempo de pared, solo orden relativo):")
            t_ref = self.reactive_transitions[0][0]
            for t, text in self.reactive_transitions:
                print(f"  +{t - t_ref:6.2f}s  {text}")
        else:
            print("\nREACTIVE_MODE nunca cambió de NONE — la capa reactiva no actuó.")

        if self.map_check_events:
            print("\nEventos MAP_CHECK relevantes:")
            for t, text in self.map_check_events:
                print(f"  [{t - self.t0:+7.3f}s] {text}")

        print(f"\nReplans detectados (con waypoints vía [DIAG replan]): {len(self.replans)}")
        first_avoiding = None
        for i, r in enumerate(self.replans):
            wps = r.get("waypoints", [])
            max_abs_y = max((abs(y) for _, y in wps), default=0.0)
            avoids = max_abs_y > 0.3
            t_trigger = r.get("t_trigger")
            t_astar_start = r.get("t_astar_start")
            t_published = r.get("t_path_published") or r.get("t_diag")
            algo_dt = (
                (t_published - t_astar_start)
                if (t_published is not None and t_astar_start is not None) else None
            )
            rel = (t_trigger - self.t0) if t_trigger is not None else None
            tag = "ESQUIVA (y != 0)" if avoids else "recto (y ~ 0)"
            print(
                f"  replan #{i+1}: t_trigger={'%+.3fs' % rel if rel is not None else '??'} "
                f"tras T0 | max|y|={max_abs_y:.2f} -> {tag} | "
                f"dur_algoritmo={'%.3fs' % algo_dt if algo_dt is not None else '??'} | "
                f"waypoints={wps}"
            )
            if avoids and first_avoiding is None:
                first_avoiding = r

        if first_avoiding is not None and first_avoiding.get("t_trigger") is not None:
            delta = first_avoiding["t_trigger"] - self.t0
            print(
                f"\n>>> RETRASO CLAVE: primer plan que esquiva el obstáculo "
                f"llega +{delta:.3f}s después de que el sensor lo vio por "
                f"primera vez (T0). Esto es lo que la posición del dron avanza "
                f"'a ciegas' con el plan recto viejo antes de tener una ruta "
                f"nueva.")
        else:
            print(
                "\n>>> Ningún replan capturado se desvía de y=0 con margen "
                ">0.3m — o el obstáculo nunca provocó un replan detectado, o "
                "el detour real es más sutil que el umbral usado aquí (ajustar "
                "0.3 si hace falta).")

        print("=" * 70)


def main():
    rclpy.init()
    node = ReplanLatencyMonitor()

    stop = {"flag": False}

    def _sigint(_sig, _frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, _sigint)

    start_wall = time.time()
    try:
        while rclpy.ok() and not stop["flag"] and (time.time() - start_wall) < RUN_DURATION_S:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.finalize()
        node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
