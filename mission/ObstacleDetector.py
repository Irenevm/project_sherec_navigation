import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.modules.navigate_to_module import NavigateToModule
import math
import time

class ObstacleDetector(DroneInterface):
    def __init__(self):
        super().__init__("drone0", use_sim_time=True)
        self.navigate_to = NavigateToModule(drone=self)
        self.map_sub = self.create_subscription(OccupancyGrid, '/drone0/map', self.map_callback, 10)
        self.latest_map = None

    def map_callback(self, msg):
        self.latest_map = msg

    def get_nearest_obstacle_y(self):
        if not self.latest_map: return None
        info = self.latest_map.info
        res = info.resolution
        # Filtro de pasillo: X entre 5.5 y 6.5, Y adelante del dron
        for i, value in enumerate(self.latest_map.data):
            if value == 100:
                grid_x, grid_y = i % info.width, i // info.width
                real_x = (grid_x * res) + info.origin.position.x
                real_y = (grid_y * res) + info.origin.position.y
                if 5.5 < real_x < 6.5 and real_y > self.position[1]:
                    return real_y
        return None

def main():
    rclpy.init()
    uav = ObstacleDetector()
    
    # --- FASE 0: Preparación con reintentos ---
    print("Preparando sistemas...")
    uav.offboard()
    time.sleep(1)
    if not uav.arm():
        print("ERROR: No se pudo armar el dron. Revisa Gazebo.")
        return

    # --- FASE 1: Despegue y Esquina ---
    if uav.takeoff(1.5, speed=1.0):
        print("Despegue exitoso.")
        
        print("Navegando a la esquina del pasillo (Punto seguro)...")
        try:
            # AJUSTE: Probamos 5.5 en lugar de 6.0 para estar más lejos de la pared exterior
            uav.navigate_to(5.5, 4.0, 1.5, speed=0.5, wait=True)
        except Exception as e:
            print(f"La navegación a la esquina falló: {e}")
            # Si falla la navegación inteligente, probamos un movimiento directo 'ciego'
            print("Intentando movimiento directo de emergencia...")
            uav.go_to(5.5, 4.0, 1.5, speed=0.5, wait=True)

    # --- FASE 2: Tramo vertical y parada de precisión ---
    print("Iniciando tramo vertical. Buscando cilindro...")
    # Navegamos hacia el fondo sin bloquear
    uav.navigate_to(5.5, 10.0, 1.5, speed=0.4, wait=False)
    
    safety_distance = 0.6 # Aumentamos un poco el margen para que se vea claro
    
    while rclpy.ok():
        rclpy.spin_once(uav, timeout_sec=0.1)
        obs_y = uav.get_nearest_obstacle_y()
        if obs_y:
            target_stop_y = obs_y - safety_distance
            if uav.position[1] >= (target_stop_y - 0.5):
                print(f"¡FRENAZO! Obstáculo en Y={obs_y:.2f}. Parando en Y={target_stop_y:.2f}")
                uav.go_to(5.5, target_stop_y, 1.5, speed=0.3, wait=True)
                break

    uav.land()
    uav.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
