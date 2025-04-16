import sys
import time
import math
import rclpy
from rclpy.node import Node

# Añadir ruta del cliente ZMQ
sys.path.append('/home/mateo/CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04/programming/zmqRemoteApi/clients/python/src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class BallMover(Node):
    def __init__(self):
        super().__init__('ball_node')

        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.sim.setStepping(True)

        # Obtener handle de la esfera
        self.ball_handle = self.sim.getObject('/Sphere')
        self.get_logger().info('🎱 Nodo conectado a la esfera')

        # Guardar la posición inicial de la pelota
        self.initial_position = self.sim.getObjectPosition(self.ball_handle, -1)
        self.get_logger().info(f'Posición inicial de la pelota: {self.initial_position}')


        # Esperar a que la simulación inicie
        self.get_logger().info('⏳ Esperando a que inicie la simulación...')
        while self.sim.getSimulationState() == 0:
            time.sleep(0.1)

        self.get_logger().info('▶️ Simulación iniciada, comenzando movimiento de la esfera...')
        
        # Parámetros del movimiento
        self.t = 0.0
        self.amplitud = 1.5  # Mayor distancia (metros)
        self.frecuencia = 0.05 
        #self.frecuencia = 0.1  # Menor frecuencia → movimiento más lento

        #CREACIÓN DE RUTA SENOIDAL
        #self.timer = self.create_timer(0.05, self.move_ball)  # 20 Hz aprox

        #CREACIÓN DE RUTA CIRCULAR
        self.timer = self.create_timer(0.05, self.move_ball_circle)  # 20 Hz aprox

    def move_ball(self):
        try:
            y = self.amplitud * math.sin(2 * math.pi * self.frecuencia * self.t)
            position = [1.0, y, 0.2]  # x, y, z (ajusta z si la pelota no está flotando correctamente)

            self.sim.setObjectPosition(self.ball_handle, -1, position)
            self.sim.step()

            self.t += 0.05  # avanzar tiempo

        except Exception as e:
            self.get_logger().error(f'❌ Error moviendo la esfera: {str(e)}')

    def move_ball_circle(self):
        try:
            # Centro del círculo
            xc = 0.0
            yc = 0.0
            r = self.amplitud  # Radio del círculo

            # Posiciones circulares
            x = xc + r * math.cos(2 * math.pi * self.frecuencia * self.t)
            y = yc + r * math.sin(2 * math.pi * self.frecuencia * self.t)
            z = 0.2  # Altura fija

            self.sim.setObjectPosition(self.ball_handle, -1, [x, y, z])
            self.sim.step()

            self.t += 0.05  # avanzar tiempo

        except Exception as e:
            self.get_logger().error(f'❌ Error moviendo la esfera: {str(e)}')


    def reset_ball_position(self):
        # Restablecer la pelota a su posición inicial
        self.sim.setObjectPosition(self.ball_handle, -1, self.initial_position)
        self.get_logger().info(f'Pelota restablecida a la posición inicial: {self.initial_position}')

    def on_shutdown(self):
        # Restablecer la posición cuando se apaga el nodo
        self.reset_ball_position()


def main(args=None):
    rclpy.init(args=args)
    node = BallMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
