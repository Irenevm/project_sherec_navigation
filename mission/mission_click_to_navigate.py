#!/bin/python3

"""
mission_click_to_navigate.py
"""

import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from as2_msgs.msg import YawMode
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.modules.navigate_to_module import NavigateToModule
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from geometry_msgs.msg import PointStamped

from telemetry_logger import TelemetryLogger


class Drone(DroneInterface):
    def __init__(self, drone_id: str = "drone0", verbose: bool = False, use_sim_time: bool = False) -> None:
        super().__init__(drone_id, verbose, use_sim_time)

        self.navigate_to = NavigateToModule(drone=self)
        self._tel_logger = TelemetryLogger(drone=self, output_dir='/root/sherec_nav/')

        cbk_group = MutuallyExclusiveCallbackGroup()
        # cbk_group = None
        self.create_subscription(PointStamped, "/clicked_point", self.clicked_point_callback,
                                 10, callback_group=cbk_group)

        while not self.arm():
            self.get_logger().info("Waiting for arming...")
            self.sleep(1.0)
        self.offboard()
        self.takeoff(height=1.0, speed=0.5, wait=True)
        self.keep_running = False

    def clicked_point_callback(self, msg: PointStamped):
        self.get_logger().info(f"Clicked point: {msg.point.x}, {msg.point.y}, {msg.point.z}")
        z = 1.0
        self._tel_logger.start(msg.point.x, msg.point.y, z, session_label='click_nav')
        try:
            self._tel_logger.mark_event('NAV_START')
            self.navigate_to(msg.point.x, msg.point.y, z, speed=2.0,
                             yaw_mode=YawMode.PATH_FACING, wait=False)
            threading.Thread(
                target=self._monitor_nav_end,
                daemon=True
            ).start()
        except BehaviorHandler.GoalRejected:
            self.get_logger().info("Goal rejected")
            self._tel_logger.mark_event('NAV_REJECTED')
            self._tel_logger.stop()
        else:
            self.get_logger().info("Navigate to started (reactive loop active)")

    def _monitor_nav_end(self) -> None:
        """Background thread: waits for the navigate_to behavior to finish, then stops the logger."""
        # Wait until the behavior is confirmed running (goal accepted)
        deadline = time.time() + 5.0
        while not self.navigate_to.is_running() and time.time() < deadline:
            time.sleep(0.1)

        # Wait until the behavior ends (goal reached or aborted)
        while self.navigate_to.is_running():
            time.sleep(0.2)

        self._tel_logger.mark_event('NAV_END')
        self._tel_logger.stop()


if __name__ == '__main__':
    rclpy.init()

    uav = Drone("drone0", verbose=True, use_sim_time=True)

    executor = MultiThreadedExecutor()
    executor.add_node(uav)
    try:
        uav.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        uav.get_logger().info('Keyboard interrupt, shutting down.\n')

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)