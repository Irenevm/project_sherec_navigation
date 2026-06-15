#!/usr/bin/env python3
"""Spawn or remove a dynamic obstacle in the Ignition Gazebo simulation.

Usage:
  python3 spawn_obstacle.py spawn [delay_seconds]   # default delay: 0
  python3 spawn_obstacle.py remove
"""
import subprocess
import time
import sys

WORLD = "nav_test_world"
OBSTACLE_NAME = "dynamic_obstacle"

# Position: in the corridor between div_wall_north (y=4.75) and div_wall_south (y=-4.75),
# east of centre — the drone must pass here to reach the east side of the map.
OBSTACLE_X = 4.0
OBSTACLE_Y = 0.0
OBSTACLE_Z = 1.0


def ign_service(service, reqtype, reptype, req):
    result = subprocess.run(
        ["ign", "service", "-s", service,
         "--reqtype", reqtype,
         "--reptype", reptype,
         "--timeout", "5000",
         "--req", req],
        capture_output=True, text=True
    )
    return result.stdout, result.stderr


def spawn():
    sdf = (
        f'<sdf version=\\"1.6\\">'
        f'<model name=\\"{OBSTACLE_NAME}\\">'
        f'<static>true</static>'
        f'<link name=\\"link\\">'
        f'<collision name=\\"col\\"><geometry><box><size>1.0 1.0 2.0</size></box></geometry></collision>'
        f'<visual name=\\"vis\\"><geometry><box><size>1.0 1.0 2.0</size></box></geometry></visual>'
        f'</link></model></sdf>'
    )
    req = f'sdf: "{sdf}" pose: {{position: {{x: {OBSTACLE_X}, y: {OBSTACLE_Y}, z: {OBSTACLE_Z}}}}}'
    stdout, stderr = ign_service(
        f"/world/{WORLD}/create",
        "ignition.msgs.EntityFactory",
        "ignition.msgs.Boolean",
        req
    )
    if "data: true" in stdout:
        print(f"[OK] Obstacle '{OBSTACLE_NAME}' spawned at ({OBSTACLE_X}, {OBSTACLE_Y}, {OBSTACLE_Z})")
    else:
        print(f"[FAIL] Could not spawn obstacle.\nstdout: {stdout}\nstderr: {stderr}")


def remove():
    stdout, stderr = ign_service(
        f"/world/{WORLD}/remove",
        "ignition.msgs.Entity",
        "ignition.msgs.Boolean",
        f'name: "{OBSTACLE_NAME}" type: 2'
    )
    if "data: true" in stdout:
        print(f"[OK] Obstacle '{OBSTACLE_NAME}' removed.")
    else:
        print(f"[FAIL] Could not remove obstacle.\nstdout: {stdout}\nstderr: {stderr}")


if __name__ == "__main__":
    action = sys.argv[1] if len(sys.argv) > 1 else "spawn"

    if action == "remove":
        remove()
    elif action == "spawn":
        delay = int(sys.argv[2]) if len(sys.argv) > 2 else 0
        if delay > 0:
            print(f"Waiting {delay}s before spawning obstacle...")
            time.sleep(delay)
        spawn()
    else:
        print(__doc__)
