#!/usr/bin/env python3
"""Build one master CSV/JSON with every single repetition run this session."""
import glob
import json
import os
import re
import sys

sys.path.insert(0, "/root/sherec_nav")
from analyze_latency_study import (  # noqa: E402
    parse_lat_log, compute_run_latencies, load_runs,
)

DATA_DIR = "/root/sherec_nav/results"

# batch -> (valid, description)
BATCH_INFO = {
    "pilot": (True, "Pilotos iniciales (validación mecanismo drop/set_pose, sistema original)"),
    "batch1": (True, "Batería 1 — 5 reps recto v1.5 d4.0 (sistema original)"),
    "batch2": (True, "Batería 2 — 5 reps recto v1.5 d4.0 (sistema original, tras fix executor)"),
    "matA": (True, "Matriz 1 (parte A) — confirmación bug en otras velocidades/distancias"),
    "matB": (True, "Matriz 1 (parte B) — barrido de distancia a v=1.5"),
    "fixval": (False, "Validación del fix experimental unknown_as_free=true (DESCARTADO)"),
    "diag": (False, "Diagnóstico ASTAR_FAIL con fix experimental aplicado (DESCARTADO)"),
    "clean": (False, "Primeras 7 reps de la batería final, aún con el fix aplicado (DESCARTADO)"),
    "orig": (True, "Batería final (27 reps) — sistema original, tras revertir el fix"),
    "hi": (True, "Extensión de la batería final (18 reps) — distancias mayores v=1.0/0.5"),
    "d": (True, "Sonda de detección pura (24 reps, dron casi estático)"),
    "pkillfix": (True, "Prueba de validación del fix de pkill (operativo, no de investigación)"),
}


def classify_label(label):
    if label.startswith("pilot"):
        return "pilot"
    if label.startswith("batch1"):
        return "batch1"
    if label.startswith("batch2"):
        return "batch2"
    if label.startswith("matA"):
        return "matA"
    if label.startswith("matB"):
        return "matB"
    if label.startswith("fixval"):
        return "fixval"
    if label.startswith("diag_v1.5") or label.startswith("diag_v1"):
        return "diag"
    if label.startswith("clean_"):
        return "clean"
    if label.startswith("orig_"):
        return "orig"
    if label.startswith("hi_"):
        return "hi"
    if re.match(r"^d\d+_rep", label):
        return "d"
    if label.startswith("pkillfix"):
        return "pkillfix"
    return "other"


def main():
    all_json = sorted(
        set(glob.glob(os.path.join(DATA_DIR, "sherec_nav_*latstudy_*.json")))
        | set(glob.glob(os.path.join(DATA_DIR, "sherec_nav_*detprobe_*.json")))
    )
    all_json = [p for p in all_json if not p.endswith("_cellmon.json")]

    # Deduplicar por label: durante la validación de mission_detection_probe.py
    # se reutilizó la misma etiqueta (p.ej. "d3_rep1") en varios intentos antes
    # de que el script funcionara bien. Los ficheros llevan timestamp en el
    # nombre (sherec_nav_YYYYMMDD_HHMMSS_...) -> quedarnos con el más reciente
    # por label, que es la ejecución real de la batería (los intentos previos
    # eran validación/depuración, no datos de la batería).
    def label_of(path):
        try:
            return json.load(open(path)).get("label", "")
        except Exception:
            return None

    latest_by_label = {}
    for jpath in all_json:
        lbl = label_of(jpath)
        if lbl is None:
            continue
        # el timestamp está en el nombre de fichero, ordenable como string
        if lbl not in latest_by_label or jpath > latest_by_label[lbl]:
            latest_by_label[lbl] = jpath
    all_json = sorted(latest_by_label.values())

    master = []
    for jpath in all_json:
        try:
            summary = json.load(open(jpath))
        except Exception:
            continue
        label = summary.get("label", "")
        batch = classify_label(label)
        valid, batch_desc = BATCH_INFO.get(batch, (True, "otro"))

        diag_path = os.path.join(DATA_DIR, f"nav_planner_diag_{label}.log")
        lat = {}
        if os.path.exists(diag_path) and summary.get("t_drop_sim") is not None:
            events = parse_lat_log(diag_path)
            rows = compute_run_latencies(events, [summary])
            if rows:
                lat = rows[0]

        row = {
            "label": label,
            "batch": batch,
            "batch_desc": batch_desc,
            "valid_for_conclusions": valid,
            "case": summary.get("case"),
            "speed": summary.get("speed"),
            "drop_dist_requested": summary.get("drop_dist_requested"),
            "drop_dist_actual": summary.get("drop_dist_actual"),
            "obstacle_xy": summary.get("obstacle_xy"),
            "goal": summary.get("goal"),
            "outcome": summary.get("outcome"),
            "min_dist_to_obstacle": summary.get("min_dist_to_obstacle"),
            "t_drop_sim": summary.get("t_drop_sim"),
            "csv_path": summary.get("csv_path"),
            "frontier_blind_at_drop": lat.get("frontier_blind_at_drop"),
            "L_lidar": lat.get("L_lidar"),
            "L_cell": lat.get("L_cell"),
            "L_mapcheck": lat.get("L_mapcheck"),
            "L_replan": lat.get("L_replan"),
            "L_astar": lat.get("L_astar"),
            "L_modify": lat.get("L_modify"),
            "L_total_to_modify": lat.get("L_total_to_modify"),
            "reconstructed_from_text": summary.get("reconstructed_from_text", False),
        }
        master.append(row)

    with open(os.path.join(DATA_DIR, "master_dataset.json"), "w") as f:
        json.dump(master, f, indent=2)

    import csv
    fields = list(master[0].keys()) if master else []
    with open(os.path.join(DATA_DIR, "master_dataset.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in master:
            w.writerow(r)

    print(f"Total repeticiones: {len(master)}")
    from collections import Counter
    print("Por batch:", Counter(r["batch"] for r in master))
    print("Válidas para conclusiones:", sum(1 for r in master if r["valid_for_conclusions"]))
    print("Descartadas:", sum(1 for r in master if not r["valid_for_conclusions"]))


if __name__ == "__main__":
    main()
