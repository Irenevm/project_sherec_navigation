#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import time

OBS_X, OBS_Y = 6.5, -3.8

class Sampler(Node):
    def __init__(self):
        super().__init__('occ_cell_sampler')
        self.t0 = time.time()
        self.sub = self.create_subscription(OccupancyGrid, '/drone0/map', self.cb, 1)
        self.last_val = None

    def cb(self, msg):
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        res = msg.info.resolution
        w = msg.info.width
        h = msg.info.height
        cx = int((OBS_X - ox) / res)
        cy = int((OBS_Y - oy) / res)
        if 0 <= cx < w and 0 <= cy < h:
            idx = cy * w + cx
            val = msg.data[idx]
        else:
            val = None
        t = time.time() - self.t0
        if val != self.last_val:
            print(f"t={t:.2f}s value={val}", flush=True)
            self.last_val = val

def main():
    rclpy.init()
    node = Sampler()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
