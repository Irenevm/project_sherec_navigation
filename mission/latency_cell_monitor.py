#!/usr/bin/env python3
"""
latency_cell_monitor.py — monitor pasivo: cuándo queda "ocupada" la celda del
obstáculo en el mapa que usa A* (topic /drone0/map, el M_g acumulado).

No controla el dron. Se lanza en paralelo a mission_latency_study.py (que lo
arranca automáticamente como subproceso) y escribe, al recibir SIGTERM o tras
`duration_s`, un JSON con:
  - t_cell_gt_thresh_sim : primer instante (sim-time) en que la celda del
    obstáculo supera el umbral que usa el A* (>30, ver a_star_searcher.hpp).
  - t_cell_100_sim       : primer instante en que la celda queda consolidada
    a 100 (hit_confidence acumulado + "keep obstacles" > 80 -> 100).
  - last_value           : último valor visto en esa celda.

Uso:
  python3 latency_cell_monitor.py <obstacle_x> <obstacle_y> <output_json> [duration_s]
"""

import json
import signal
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

OCC_THRESHOLD = 30  # a_star_searcher.hpp default thresh


class CellMonitor(Node):
    def __init__(self, ox: float, oy: float, output_path: str):
        super().__init__("latency_cell_monitor", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        ])
        self._ox = ox
        self._oy = oy
        self._output_path = output_path
        self._t_gt_thresh = None
        self._t_100 = None
        self._last_value = None
        # Topic completo, no relativo: este nodo no tiene namespace drone0
        # propio, y "map" relativo resolvía a /map (nunca publicado) en vez
        # de /drone0/map -> el monitor nunca veía nada (bug encontrado en el
        # piloto: t_cell_gt_thresh_sim siempre None pese a que el A* sí
        # reaccionaba al obstáculo).
        self.create_subscription(OccupancyGrid, "/drone0/map", self._cbk, 1)

    def _cbk(self, msg: OccupancyGrid) -> None:
        info = msg.info
        col = int((self._ox - info.origin.position.x) / info.resolution)
        row = int((self._oy - info.origin.position.y) / info.resolution)
        if not (0 <= col < info.width and 0 <= row < info.height):
            return
        idx = row * info.width + col
        if idx >= len(msg.data):
            return
        value = msg.data[idx]
        self._last_value = value
        t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if value > OCC_THRESHOLD and self._t_gt_thresh is None:
            self._t_gt_thresh = t_sim
            self.get_logger().info(f"[CELLMON] t_cell_gt_thresh={t_sim:.3f} value={value}")
        if value >= 100 and self._t_100 is None:
            self._t_100 = t_sim
            self.get_logger().info(f"[CELLMON] t_cell_100={t_sim:.3f}")

    def dump(self) -> None:
        summary = {
            "obstacle_xy": [self._ox, self._oy],
            "t_cell_gt_thresh_sim": self._t_gt_thresh,
            "t_cell_100_sim": self._t_100,
            "last_value": self._last_value,
        }
        with open(self._output_path, "w") as f:
            json.dump(summary, f, indent=2)
        print(f"[CELLMON] Escrito {self._output_path}: {summary}")


def main():
    if len(sys.argv) not in (4, 5):
        print(__doc__)
        sys.exit(1)
    ox = float(sys.argv[1])
    oy = float(sys.argv[2])
    output_path = sys.argv[3]
    duration_s = float(sys.argv[4]) if len(sys.argv) == 5 else 90.0

    rclpy.init()
    node = CellMonitor(ox, oy, output_path)

    def _handle_sigterm(signum, frame):
        node.dump()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _handle_sigterm)
    signal.signal(signal.SIGINT, _handle_sigterm)

    end_time = node.get_clock().now().nanoseconds * 1e-9 + duration_s
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.get_clock().now().nanoseconds * 1e-9 > end_time:
                break
    finally:
        node.dump()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
