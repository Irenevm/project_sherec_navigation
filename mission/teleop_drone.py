#!/usr/bin/env python3
"""
teleop_drone.py — Control manual del drone0 por teclado

Uso:
  python3 teleop_drone.py

Controles (cada tecla FIJA la velocidad, no acumula):
  tk / la   : takeoff / land
  w / s     : adelante / atrás
  a / d     : izquierda / derecha (traslación lateral)
  q / e     : subir / bajar
  z / c     : girar izquierda / girar derecha (yaw)
  SPACE     : parar todo
  Ctrl+D    : salir
"""

import time
import rclpy
from as2_python_api.drone_interface_teleop import DroneInterfaceTeleop as DroneInterface

SPEED   = 0.5   # m/s traslación
YAW     = 0.5   # rad/s giro

CONTROLS = """
╔══════════════════════════════════════╗
║  tk       takeoff      la    land    ║
║  w/s      adelante/atrás             ║
║  a/d      izquierda/derecha          ║
║  q/e      subir/bajar                ║
║  z/c      girar izq/der (yaw)        ║
║  SPACE    PARAR TODO                 ║
║  Ctrl+D   salir                      ║
╚══════════════════════════════════════╝
"""


def print_status(vx, vy, vz, az):
    parts = []
    if vx > 0: parts.append("adelante")
    elif vx < 0: parts.append("atrás")
    if vy > 0: parts.append("izquierda")
    elif vy < 0: parts.append("derecha")
    if vz > 0: parts.append("subiendo")
    elif vz < 0: parts.append("bajando")
    if az > 0: parts.append("girando izq")
    elif az < 0: parts.append("girando der")
    if not parts: parts.append("PARADO")
    print(f"  → {', '.join(parts)}  (vx={vx:+.1f} vy={vy:+.1f} vz={vz:+.1f} yaw={az:+.1f})")


if __name__ == "__main__":
    rclpy.init()
    drone = DroneInterface("drone0", verbose=False, use_sim_time=True)
    drone.offboard()
    drone.arm()

    vx, vy, vz, az = 0.0, 0.0, 0.0, 0.0
    print(CONTROLS)

    while rclpy.ok():
        try:
            cmd = input("> ").strip().lower()
        except EOFError:
            break

        if cmd == "tk":
            drone.takeoff(height=1.0, speed=0.5)
            print("  [takeoff a 1m]")
            continue
        elif cmd == "la":
            drone.land(speed=0.3)
            print("  [land]")
            continue

        # Cada tecla FIJA la velocidad (no acumula)
        if cmd == "w":
            vx, vy, vz, az = SPEED, 0.0, 0.0, 0.0
        elif cmd == "s":
            vx, vy, vz, az = -SPEED, 0.0, 0.0, 0.0
        elif cmd == "a":
            vx, vy, vz, az = 0.0, SPEED, 0.0, 0.0
        elif cmd == "d":
            vx, vy, vz, az = 0.0, -SPEED, 0.0, 0.0
        elif cmd == "q":
            vx, vy, vz, az = 0.0, 0.0, SPEED, 0.0
        elif cmd == "e":
            vx, vy, vz, az = 0.0, 0.0, -SPEED, 0.0
        elif cmd == "z":
            vx, vy, vz, az = 0.0, 0.0, 0.0, YAW
        elif cmd == "c":
            vx, vy, vz, az = 0.0, 0.0, 0.0, -YAW
        elif cmd in ("", " "):
            vx, vy, vz, az = 0.0, 0.0, 0.0, 0.0
        else:
            print("  [?] no reconocido — SPACE para parar")
            continue

        print_status(vx, vy, vz, az)
        drone.motion_ref_handler.speed.send_speed_command_with_yaw_speed(
            [vx, vy, vz], twist_frame_id='base_link', yaw_speed=az)
        time.sleep(0.1)

    print("BYE")
    rclpy.shutdown()
