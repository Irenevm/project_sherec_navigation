#!/usr/bin/env python3
"""
reconstruct_summary_json.py — genera el JSON de resumen de una repetición de
mission_latency_study.py a partir de su output de texto crudo, para cuando el
proceso se mata (o se cuelga, bug ya documentado tras imprimir "Resultado")
antes de llegar a escribirlo él mismo (aterrizaje+desarme+shutdown tardan
unos segundos más tras el bloque "--- Resultado ---").

No sobreescribe si el JSON ya existe (idempotente).

Uso:
  python3 reconstruct_summary_json.py <case> <speed> <drop_dist_requested> <label> <raw_txt_path> <data_dir>
"""
import glob
import json
import os
import re
import sys

CASES = {
    "recto": {"goal": [2.0, 0.0, 1.0], "obstacle_xy": [-4.0, 0.0]},
    "giro": {"goal": [2.0, 3.0, 1.0], "obstacle_xy": [0.5, 2.5]},
    "abierto": {"goal": [9.0, -4.0, 1.0], "obstacle_xy": [4.0, -4.0]},
    "abierto_temprano": {"goal": [9.0, -4.0, 1.0], "obstacle_xy": [-4.0, 0.0]},
}


def main():
    if len(sys.argv) != 7:
        print(__doc__)
        sys.exit(1)
    case, speed, drop_req, label, raw_txt_path, data_dir = sys.argv[1:7]
    speed = float(speed)
    drop_req = float(drop_req)

    existing = glob.glob(os.path.join(
        data_dir, f"sherec_nav_*latstudy_{case}_v{speed}_d{drop_req}_{label}.json"))
    if existing:
        print(f"[SKIP] JSON ya existe: {existing[0]}")
        return

    if not os.path.exists(raw_txt_path):
        print(f"[WARN] No existe {raw_txt_path}, no se puede reconstruir")
        return
    text = open(raw_txt_path, errors="replace").read()

    m_outcome = re.search(r"Outcome\s*:\s*(\S+)", text)
    m_mindist = re.search(r"Min dist obst.culo\s*:\s*([\d.]+)", text)
    m_drop = re.search(
        r"\[DROP\] set_pose OK.*?t_send=([\d.]+) t_ack=([\d.]+)", text)
    m_dropdist = re.search(r"\[DROP-\S+\] dist=([\d.]+)", text)

    if not (m_outcome and m_dropdist and m_drop):
        print(f"[WARN] No se pudo parsear {raw_txt_path} (faltan campos clave)")
        return

    m_csv = re.search(r"Recording (?:started|saved) .* .(/root/sherec_nav/\S+\.csv)", text)
    csv_glob = glob.glob(os.path.join(
        data_dir, f"sherec_nav_*latstudy_{case}_v{speed}_d{drop_req}_{label}.csv"))
    csv_path = csv_glob[0] if csv_glob else (m_csv.group(1) if m_csv else "")
    base = os.path.splitext(csv_path)[0] if csv_path else \
        f"/root/sherec_nav/sherec_nav_UNKNOWN_latstudy_{case}_v{speed}_d{drop_req}_{label}"

    case_info = CASES[case]
    summary = {
        "case": case,
        "speed": speed,
        "drop_dist_requested": drop_req,
        "label": label,
        "obstacle_xy": case_info["obstacle_xy"],
        "goal": case_info["goal"],
        "collision_dist_threshold": 1.0,
        "outcome": m_outcome.group(1),
        "min_dist_to_obstacle": float(m_mindist.group(1)) if m_mindist else None,
        "csv_path": csv_path,
        "cellmon_json_path": base + "_cellmon.json",
        "t_drop_sim": float(m_drop.group(1)),
        "t_drop_sim_ack": float(m_drop.group(2)),
        "drop_dist_actual": float(m_dropdist.group(1)),
        "reconstructed_from_text": True,
    }
    out_path = base + ".json"
    with open(out_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"[RECONSTRUCTED] {out_path}")


if __name__ == "__main__":
    main()
