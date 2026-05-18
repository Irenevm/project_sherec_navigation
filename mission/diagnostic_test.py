#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from as2_msgs.action import Takeoff
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from as2_msgs.msg import PlatformInfo
import time

class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__('diagnostics_node')
        self.namespace = '/drone0'
        
        # Suscribers for State
        self.pose = None
        self.platform_info = None
        
        self.create_subscription(PoseStamped, f'{self.namespace}/self_localization/pose', self.pose_callback, 10)
        self.create_subscription(PlatformInfo, f'{self.namespace}/platform/info', self.info_callback, 10)
        
    def pose_callback(self, msg):
        self.pose = msg
        
    def info_callback(self, msg):
        self.platform_info = msg
        
    def run_diagnostics(self):
        self.get_logger().info("--- INICIANDO DIAGNÓSTICO AEROSTACK2 ---")
        
        # 1. Esperar un poco para recibir topics
        time.sleep(2)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.pose is None:
            self.get_logger().error("FAIL: No se reciben datos de pose en /drone0/self_localization/pose")
        else:
            self.get_logger().info(f"OK: Pose recibida (z = {self.pose.pose.position.z:.2f})")
            
        if self.platform_info is None:
            self.get_logger().error("FAIL: No se reciben datos de /drone0/platform/info")
        else:
            self.get_logger().info(f"OK: PlatformInfo recibida. Estado: {self.platform_info.status.state}")
            
        # 2. Check Actions and Services
        self.get_logger().info("\n--- VERIFICANDO ACCIONES Y SERVICIOS ---")
        action_names_and_types = self.get_action_names_and_types()
        actions = [a[0] for a in action_names_and_types]
        
        expected_actions = [f'{self.namespace}/TakeoffBehavior', f'{self.namespace}/GoToBehavior', f'{self.namespace}/FollowPathBehavior', f'{self.namespace}/path_planner']
        for ea in expected_actions:
            if ea in actions:
                self.get_logger().info(f"OK: Acción {ea} disponible.")
            else:
                self.get_logger().error(f"FAIL: Acción {ea} NO encontrada.")
                
        service_names_and_types = self.get_service_names_and_types()
        services = [s[0] for s in service_names_and_types]
        
        pause_service = f'{self.namespace}/TakeoffBehavior/_behavior/pause'
        if pause_service in services:
             self.get_logger().info("OK: Servicios internos de Behavior (_behavior/pause) encontrados.")
        else:
             self.get_logger().warn(f"WARN: Servicio {pause_service} NO encontrado. ¿Faltan los internal services de los behaviors?")
             
        self.get_logger().info("------------------------------------------")
        self.get_logger().info("DIAGNÓSTICO TERMINADO. Presiona Ctrl+C para salir.")

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        node.run_diagnostics()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
