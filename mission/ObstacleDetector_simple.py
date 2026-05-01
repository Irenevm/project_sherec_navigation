import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from as2_python_api.drone_interface import DroneInterface
import math

class ObstacleDetector(DroneInterface):
    def __init__(self):
        super().__init__("drone0", use_sim_time=True)
        # Suscriptor al mapa
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/drone0/map',
            self.map_callback,
            10)
        self.latest_map = None

    def map_callback(self, msg):
        self.latest_map = msg

    def get_nearest_obstacle(self):
        if not self.latest_map:
            return None
        
        info = self.latest_map.info
        data = self.latest_map.data
        res = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        
        # Guardaremos el obstáculo más cercano que esté delante
        nearest_x = float('inf')
        nearest_coords = None

        for i, value in enumerate(data):
            if value == 100:
                grid_x = i % info.width
                grid_y = i // info.width
                real_x = (grid_x * res) + origin_x
                real_y = (grid_y * res) + origin_y
                
                # FILTRO: Solo nos interesan obstáculos delante (X > 0.5)
                # y que estén más o menos en nuestra trayectoria (Y cerca de 0)
                if real_x > 0.5 and abs(real_y) < 1.0:
                    if real_x < nearest_x:
                        nearest_x = real_x
                        nearest_coords = (real_x, real_y)
        
        return nearest_coords

def main():
    rclpy.init()
    uav = ObstacleDetector()
    
    # 1. Despegue
    uav.offboard()
    uav.arm()
    uav.takeoff(1.0, speed=1.0)
    
    print("Escaneando mapa en busca de obstáculos...")
    # Esperamos a que el mapa tenga datos (necesitas rotar el dron en RViz)
    found_x, found_y = (None, None)
    while rclpy.ok() and found_x is None:
        rclpy.spin_once(uav, timeout_sec=1.0)
        coords = uav.get_nearest_obstacle()
        if coords:
            found_x, found_y = coords
            print(f"Obstáculo encontrado en: X={found_x:.2f}, Y={found_y:.2f}")

    # 2. Navegación inteligente
    if found_x:
        safety_margin = 1.0  # Nos quedamos a 1 metro
        target_x = found_x - safety_margin
        print(f"Navegando a posición segura: X={target_x:.2f}")
        
        uav.go_to(target_x, 0.0, 1.0, speed=0.5, wait=True)
        print("Llegada completada.")

    uav.land()
    uav.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
