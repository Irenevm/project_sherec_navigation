#!/bin/python3

"""
mission.py
"""

from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.modules.navigate_to_module import NavigateToModule


class Drone(DroneInterface):
    def __init__(self, drone_id: str = "drone0", verbose: bool = False, use_sim_time: bool = False) -> None:
        super().__init__(drone_id, verbose, use_sim_time)

        self.navigate_to = NavigateToModule(drone=self)


def drone_run(drone_interface: Drone):
    """ Misión corregida para evitar Goal Rejected """

    takeoff_height = 1.0
    sleep_time = 2.0

    print("--- Iniciando Fase de Exploración ---")

    # 1. Preparación técnica
    drone_interface.offboard()
    sleep(1.0)
    drone_interface.arm()
    sleep(1.0)

    # 2. Despegue (Verifica que en Gazebo suba 1 metro)
    print("Despegando...")
    if drone_interface.takeoff(takeoff_height, speed=1.0):
        print("Vuelo estacionario alcanzado.")
    else:
        print("ERROR: Despegue fallido")
        return
    
    sleep(sleep_time)

    # 3. Navegación hasta DELANTE del obstáculo
    # Ponemos x=1.5 o 2.0 para que el sensor lo detecte sin chocar
    print("Navegando hacia el obstáculo...")
    try:
        # Usamos wait=True para que el programa espere a que llegue antes de seguir
        drone_interface.navigate_to(1.0, 0.0, 1.0, speed=0.5, wait=True)
        print("Llegada a zona segura pre-obstáculo.")
    except Exception as e:
        print(f"La navegación fue rechazada: {e}")

    # 4. Esperar para observar en RViz cómo se 'ensucia' el mapa
    print("Esperando 10 segundos para escaneo de sensores...")
    sleep(10.0)

    # 5. Aterrizaje seguro
    print("Aterrizando...")
    drone_interface.land(speed=0.5)
    print("Misión finalizada.")


if __name__ == '__main__':
    rclpy.init()

    uav = Drone("drone0", verbose=False, use_sim_time=True)

    drone_run(uav)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
