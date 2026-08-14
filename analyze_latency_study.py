#!/usr/bin/env python3
"""
analyze_latency_study.py — agrega todas las repeticiones de
mission_latency_study.py y calcula:

  1. Desglose de latencia por etapa (mediana/media/min/max/p95), en segundos
     desde el instante de soltar el obstáculo (t_drop, sim-time):
       L_lidar     : hasta que el LiDAR ve el obstáculo (proxy: primera caída
                     sostenida de lidar_fwd_min en la telemetría, CSV 10Hz).
       L_cell      : hasta que la celda del obstáculo supera el umbral de
                     ocupación del A* (>30) en el mapa "map" (cell monitor).
       L_mapcheck  : hasta que el MAP_CHECK del planner detecta el segmento
                     ocupado (log [LAT] DETECT).
       L_replan    : desde DETECT hasta el inicio del replan ([LAT]
                     REPLAN_START) — normalmente ~0, el mismo tick.
       L_astar     : duración de A* ([LAT] REPLAN_START -> ASTAR_DONE).
       L_modify    : desde ASTAR_DONE hasta que FollowPath acepta/rechaza el
                     modify ([LAT] MODIFY_ACCEPTED/REJECTED).
  2. Curva Dmin(v): para cada velocidad, la distancia de soltado (drop_dist)
     mínima probada con 0 colisiones sobre todas las repeticiones.
  3. Tasa de colisión por combinación (caso x velocidad x drop_dist).

Uso:
  python3 analyze_latency_study.py [data_dir] [diag_log_path]

  data_dir      : carpeta con los CSV/JSON de mission_latency_study.py
                  (default: /root/sherec_nav/results/)
  diag_log_path : nav_planner_diag.log del planner
                  (default: /root/sherec_nav/nav_planner_diag.log)
"""

import csv
import glob
import json
import os
import re
import statistics
import sys
from collections import defaultdict

LAT_RE = re.compile(
    r"\[LAT\]\s+(DETECT|REPLAN_START|ASTAR_DONE|MODIFY_ACCEPTED|MODIFY_REJECTED|GOAL_SENT"
    r"|FRONTIER_ENTER|FRONTIER_EXIT)"
    r".*?t=([0-9.]+)"
)


def parse_lat_log(diag_log_path: str):
    """Devuelve lista de (event, t_sim) en orden de aparición en el log."""
    events = []
    if not os.path.exists(diag_log_path):
        print(f"[WARN] No existe {diag_log_path} — sin datos de [LAT] del planner.")
        return events
    with open(diag_log_path, "r", errors="replace") as f:
        for line in f:
            m = LAT_RE.search(line)
            if m:
                events.append((m.group(1), float(m.group(2))))
    return events


def first_after(events, event_name: str, t_ref: float, window_s: float = 20.0):
    for name, t in events:
        if name == event_name and t_ref - 0.5 <= t <= t_ref + window_s:
            return t
    return None


def is_frontier_blind_at(events, t_ref: float) -> bool:
    """True si el planner está en modo frontier (is_intermediate_goal_=true) en
    t_ref — en ese modo, MAP_CHECK no detecta obstáculos dinámicos nuevos (ver
    hallazgo del piloto 2: FRONTIER_ENTER deshabilita segment-check y
    full-block hasta el siguiente FRONTIER_EXIT o fin de misión)."""
    in_frontier = False
    for name, t in events:
        if t > t_ref:
            break
        if name == "FRONTIER_ENTER":
            in_frontier = True
        elif name == "FRONTIER_EXIT":
            in_frontier = False
    return in_frontier


def compute_lidar_sees(csv_path: str, t_drop: float):
    """Primera caída sostenida (2 muestras seguidas) de lidar_fwd_min tras t_drop."""
    rows = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)

    baseline_vals = [
        float(r["lidar_fwd_min"]) for r in rows
        if r["timestamp_s"] and float(r["timestamp_s"]) < t_drop and r["lidar_fwd_min"]
    ]
    baseline = statistics.median(baseline_vals) if baseline_vals else None

    consec = 0
    for r in rows:
        if not r["timestamp_s"] or float(r["timestamp_s"]) <= t_drop:
            continue
        if not r["lidar_fwd_min"]:
            consec = 0
            continue
        val = float(r["lidar_fwd_min"])
        hit = (baseline is None) or (val < baseline - 1.0)
        if hit:
            consec += 1
            if consec >= 2:
                return float(r["timestamp_s"])
        else:
            consec = 0
    return None


def load_runs(data_dir: str, label: str = None):
    runs = []
    patterns = ["sherec_nav_*latstudy_*.json", "sherec_nav_*detprobe_*.json"]
    paths = sorted({p for pat in patterns for p in glob.glob(os.path.join(data_dir, pat))})
    for json_path in paths:
        if json_path.endswith("_cellmon.json"):
            continue
        with open(json_path) as f:
            summary = json.load(f)
        if label is not None and summary.get("label") != label:
            continue
        runs.append(summary)
    return runs


def compute_run_latencies(events, runs):
    """Calcula las latencias por-run dada una lista de eventos [LAT] (de UN
    log de diagnóstico) y una lista de resúmenes de misión. Separado de
    analyze() para poder reusarlo cuando cada repetición tiene su propio
    log (el reloj de simulación se resetea en cada reinicio completo del
    stack, así que no se pueden mezclar logs de repeticiones distintas)."""
    per_run_latencies = []
    for run in runs:
        t_drop = run.get("t_drop_sim")
        csv_path = run.get("csv_path")
        if t_drop is None or not csv_path or not os.path.exists(csv_path):
            print(f"[SKIP] Run incompleto (sin t_drop o csv): {run.get('label')}")
            continue

        t_lidar = compute_lidar_sees(csv_path, t_drop)

        cellmon_path = run.get("cellmon_json_path", "")
        t_cell = None
        if cellmon_path and os.path.exists(cellmon_path):
            with open(cellmon_path) as f:
                cellmon = json.load(f)
            t_cell = cellmon.get("t_cell_gt_thresh_sim")

        t_detect = first_after(events, "DETECT", t_drop)
        t_replan_start = first_after(events, "REPLAN_START", t_detect) if t_detect else None
        t_astar_done = first_after(events, "ASTAR_DONE", t_replan_start) if t_replan_start else None
        t_modify = None
        if t_astar_done:
            t_modify = first_after(events, "MODIFY_ACCEPTED", t_astar_done) \
                or first_after(events, "MODIFY_REJECTED", t_astar_done) \
                or first_after(events, "GOAL_SENT", t_astar_done)

        def d(a, b):
            return (a - b) if (a is not None and b is not None) else None

        # Usar la distancia REAL de soltado (drop_dist_actual), no la nominal
        # pedida (drop_dist_requested): el piloto mostró hasta ~0.9m de
        # diferencia entre ambas por el polling del hilo de drop (0.02s a
        # 1.5m/s ya deja margen, pero la latencia de lectura de posición bajo
        # carga puede ampliarlo). Agrupar/derivar Dmin(v) por la nominal
        # mezclaría repeticiones que en realidad soltaron a distancias
        # distintas.
        frontier_blind = is_frontier_blind_at(events, t_drop)

        lat = {
            "case": run["case"], "speed": run["speed"],
            "drop_dist": round(run.get("drop_dist_actual", run["drop_dist_requested"]), 2),
            "drop_dist_requested": run["drop_dist_requested"], "label": run["label"],
            "outcome": run["outcome"], "min_dist_to_obstacle": run["min_dist_to_obstacle"],
            "frontier_blind_at_drop": frontier_blind,
            "L_lidar": d(t_lidar, t_drop),
            "L_cell": d(t_cell, t_drop),
            "L_mapcheck": d(t_detect, t_drop),
            "L_replan": d(t_replan_start, t_detect),
            "L_astar": d(t_astar_done, t_replan_start),
            "L_modify": d(t_modify, t_astar_done),
            "L_total_to_modify": d(t_modify, t_drop),
        }
        per_run_latencies.append(lat)
    return per_run_latencies


def analyze(data_dir: str, diag_log_path: str):
    events = parse_lat_log(diag_log_path)
    runs = load_runs(data_dir)
    if not runs:
        print(f"[WARN] No se encontraron resúmenes de misión en {data_dir}")
        return

    per_run_latencies = compute_run_latencies(events, runs)
    _print_stage_table(per_run_latencies)
    _print_dmin_curve(per_run_latencies)
    _write_csv(per_run_latencies, os.path.join(data_dir, "latency_study_analysis.csv"))


def _stats(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    values_sorted = sorted(values)
    p95_idx = min(len(values_sorted) - 1, int(round(0.95 * (len(values_sorted) - 1))))
    return {
        "n": len(values),
        "median": statistics.median(values),
        "mean": statistics.mean(values),
        "min": min(values),
        "max": max(values),
        "p95": values_sorted[p95_idx],
    }


def _print_stage_table(rows):
    print("\n" + "=" * 78)
    print("  DESGLOSE DE LATENCIA POR ETAPA (segundos desde t_drop)")
    print("=" * 78)
    stages = ["L_lidar", "L_cell", "L_mapcheck", "L_replan", "L_astar", "L_modify",
              "L_total_to_modify"]
    header = f"{'etapa':18s}{'n':>4s}{'mediana':>10s}{'media':>10s}{'min':>10s}{'max':>10s}{'p95':>10s}"
    print(header)
    print("-" * len(header))
    for stage in stages:
        s = _stats([r[stage] for r in rows])
        if s is None:
            print(f"{stage:18s}  (sin datos)")
            continue
        print(
            f"{stage:18s}{s['n']:4d}{s['median']:10.3f}{s['mean']:10.3f}"
            f"{s['min']:10.3f}{s['max']:10.3f}{s['p95']:10.3f}"
        )


def _print_dmin_curve(rows):
    print("\n" + "=" * 78)
    print("  OUTCOME POR (caso, velocidad, drop_dist) Y CURVA Dmin(v)")
    print("=" * 78)
    grouped = defaultdict(list)
    for r in rows:
        grouped[(r["case"], r["speed"], r["drop_dist"])].append(r)

    by_case_speed = defaultdict(dict)
    for (case, speed, drop_dist), group_rows in sorted(grouped.items()):
        outcomes = [r["outcome"] for r in group_rows]
        n = len(outcomes)
        n_collision = sum(1 for o in outcomes if o == "COLLISION")
        n_blind = sum(1 for r in group_rows if r["frontier_blind_at_drop"])
        print(
            f"  caso={case:6s} v={speed:.2f}  drop_dist={drop_dist:.2f}m  "
            f"n={n}  colisiones={n_collision}/{n}  frontier_blind={n_blind}/{n}  "
            f"outcomes={outcomes}"
        )
        by_case_speed[(case, speed)][drop_dist] = n_collision

    n_blind_total = sum(1 for r in rows if r["frontier_blind_at_drop"])
    n_blind_collision = sum(
        1 for r in rows if r["frontier_blind_at_drop"] and r["outcome"] == "COLLISION")
    if n_blind_total:
        print(
            f"\n  [BUG FRONTIER-BLIND] {n_blind_total}/{len(rows)} repeticiones tenían "
            f"is_intermediate_goal_=true (detección de obstáculo deshabilitada) en el "
            f"instante del drop; de esas, {n_blind_collision} acabaron en COLLISION."
        )

    print("\n  --- Dmin(v) por caso (menor drop_dist probado con 0 colisiones) ---")
    for (case, speed), dd_map in sorted(by_case_speed.items()):
        safe_dists = sorted(dd for dd, n_coll in dd_map.items() if n_coll == 0)
        unsafe_dists = sorted(dd for dd, n_coll in dd_map.items() if n_coll > 0)
        if safe_dists:
            dmin = safe_dists[0]
            warn = ""
            if unsafe_dists and max(unsafe_dists) > dmin:
                warn = "  [!] hay drop_dist mayor con colisión — revisar, no monótono"
            print(f"  caso={case:6s} v={speed:.2f} m/s  Dmin≈{dmin:.2f}m{warn}")
        else:
            print(f"  caso={case:6s} v={speed:.2f} m/s  sin drop_dist seguro probado todavía")


def _write_csv(rows, out_path):
    if not rows:
        return
    fields = list(rows[0].keys())
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f"\n[CSV] Análisis por repetición escrito en: {out_path}")


def analyze_batch(data_dir: str, label_diaglog_pairs):
    """Analiza N repeticiones, cada una con su PROPIO log de diagnóstico —
    imprescindible cuando cada repetición reinicia el stack completo, ya que
    el reloj de simulación se resetea y los timestamps de logs de
    repeticiones distintas no son comparables entre sí ni concatenables."""
    all_rows = []
    for label, diag_log_path in label_diaglog_pairs:
        events = parse_lat_log(diag_log_path)
        runs = load_runs(data_dir, label=label)
        if not runs:
            print(f"[WARN] Sin resumen de misión para label={label}")
            continue
        all_rows.extend(compute_run_latencies(events, runs))

    if not all_rows:
        print("[WARN] Ninguna repetición produjo datos analizables.")
        return

    _print_stage_table(all_rows)
    _print_dmin_curve(all_rows)
    _write_csv(all_rows, os.path.join(data_dir, "latency_study_analysis_batch.csv"))


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "batch":
        # Uso: analyze_latency_study.py batch <data_dir> <label1>:<diaglog1> [<label2>:<diaglog2> ...]
        data_dir_arg = sys.argv[2]
        pairs = []
        for arg in sys.argv[3:]:
            label, diag_log = arg.split(":", 1)
            pairs.append((label, diag_log))
        analyze_batch(data_dir_arg, pairs)
    else:
        data_dir_arg = sys.argv[1] if len(sys.argv) > 1 else "/root/sherec_nav/results/"
        diag_log_arg = sys.argv[2] if len(sys.argv) > 2 else "/root/sherec_nav/nav_planner_diag.log"
        analyze(data_dir_arg, diag_log_arg)
