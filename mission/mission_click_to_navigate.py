#!/bin/python3

"""
mission_click_to_navigate.py
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from as2_msgs.msg import YawMode
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.modules.navigate_to_module import NavigateToModule
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from geometry_msgs.msg import PointStamped


class Drone(DroneInterface):
    def __init__(self, drone_id: str = "drone0", verbose: bool = False, use_sim_time: bool = False) -> None:
        super().__init__(drone_id, verbose, use_sim_time)

        self.navigate_to = NavigateToModule(drone=self)

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
        try:
            # z = msg.point.z
            # if msg.point.z == 0.0:
            #     z = 2.0
            z = 1.0
            self.navigate_to(msg.point.x, msg.point.y, z, speed=2.0,
                             yaw_mode=YawMode.PATH_FACING, wait=False)
        except BehaviorHandler.GoalRejected:
            self.get_logger().info("Goal rejected")
        else:
            self.get_logger().info("Navigate to done")


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