#!/usr/bin/env python3
"""
mission_brake_test.py — Mide la deceleración real del dron (a_max)

Necesario para diseñar una capa reactiva basada en distancia de frenado real
(v²/(2·a_max)) en vez de un margen ajustado a ojo. Sin esto no se puede
calcular si "hay tiempo físico de parar antes de llegar al obstáculo".

Mecánica: vuelo recto a velocidad de crucero constante, sin obstáculos, y a
mitad de camino se manda una orden de parada (cancelar FollowPath) — se
registra la telemetría (speed_norm a 10Hz) antes y después de la orden para
poder ajustar la curva de deceleración real.

Uso:
  python3 mission_brake_test.py <speed> [label]

  speed : velocidad de crucero [m/s] desde la que se frena
  label : sufijo opcional para la repetición
"""

import json
import os
import sys
import time

import rclpy
from as2_msgs.msg import YawMode

sys.path.insert(0, os.path.dirname(__file__))
from mission_latency_study import TestDrone, sim_now  # noqa: E402
from telemetry_logger import TelemetryLogger  # noqa: E402

TAKEOFF_HEIGHT = 1.0
SPAWN_XY = (-9.0, 0.0)
GOAL = (9.0, 0.0, TAKEOFF_HEIGHT)   # recorrido largo, recto, sin obstáculos
BRAKE_AFTER_S = 7.0                  # esperar a que se asiente en velocidad de crucero —
                                      # subido de 4.0: con v bajas (0.5 m/s) 4s no bastaban
                                      # para superar la fase de despegue/aceleración
CAPTURE_AFTER_BRAKE_S = 6.0          # margen tras la orden de frenado


def run_brake_test(speed: float, label: str) -> None:
    session_label = f"braketest_v{speed}_{label}"
    print(f"\n{'='*70}")
    print(f"  TEST DE FRENADA — v_crucero={speed} m/s  label={label}")
    print(f"{'='*70}\n")

    rclpy.init()
    drone = TestDrone()
    logger = TelemetryLogger(drone)

    try:
        time.sleep(1.0)
        csv_path = logger.start(GOAL[0], GOAL[1], GOAL[2], session_label)

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
        print(f"[4/4] Navigate to {GOAL} a {speed} m/s...")
        drone.navigate_to(
            GOAL[0], GOAL[1], GOAL[2],
            speed=speed, yaw_mode=YawMode.PATH_FACING, wait=False,
        )

        print(f"[BRAKE] Esperando {BRAKE_AFTER_S}s antes de frenar (asentar crucero)...")
        time.sleep(BRAKE_AFTER_S)

        t_brake_sim = sim_now(drone)
        pos_at_brake = drone.position
        print(f"  Posición al frenar: {pos_at_brake}  t_sim={t_brake_sim:.2f}")
        logger.mark_event("BRAKE_CMD")
        drone.navigate_to.stop()

        print(f"[CAPTURE] Registrando {CAPTURE_AFTER_BRAKE_S}s tras la orden de frenado...")
        # Menos polling/print que antes (cada 0.5s, no 0.1s): evita competir por
        # el GIL con el timer de telemetría a 10Hz, que en el primer intento
        # solo llegó a loguear ~1.6 muestras/s en vez de 10.
        t_wall_start = time.time()
        while time.time() - t_wall_start < CAPTURE_AFTER_BRAKE_S:
            time.sleep(0.5)
        print(f"  Posición final: {drone.position}")

    finally:
        csv_final = logger.stop()
        print("\nAterrizando...")
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
            "case": "brake_test",
            "speed": speed,
            "label": label,
            "goal": list(GOAL),
            "t_brake_sim": t_brake_sim,
            "pos_at_brake": list(pos_at_brake),
            "csv_path": csv_final,
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
    speed_arg = float(sys.argv[1])
    label_arg = sys.argv[2] if len(sys.argv) == 3 else "run"
    run_brake_test(speed_arg, label_arg)
