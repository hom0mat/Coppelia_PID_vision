import sys
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

# Añadir ruta al cliente ZMQ de CoppeliaSim
sys.path.append('/home/mateo/CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04/programming/zmqRemoteApi/clients/python/src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class CoppeliaCameraNode(Node):
    def __init__(self):
        super().__init__('coppelia_camera_node')

        # Publicador de la posición de la pelota
        self.publisher = self.create_publisher(Point, 'ball_position', 10)

        # Conexión a CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        # STEPPING MANUAL
        self.sim.setStepping(True)

        self.cam_handle = self.sim.getObject('/PioneerP3DX/visionSensor')

        self.get_logger().info('📷 Nodo conectado al visionSensor de PioneerP3DX')

        # Esperar a que la simulación esté corriendo
        self.get_logger().info('⏳ Esperando a que inicie la simulación...')
        while self.sim.getSimulationState() == 0:
            time.sleep(0.1)

        self.get_logger().info('▶️ Simulación iniciada, comenzando a capturar imágenes...')
        self.timer = self.create_timer(0.2, self.capture_and_display)

    def capture_and_display(self):
        try:
            img, resolution = self.sim.getVisionSensorImg(self.cam_handle, 0)
            resX, resY = resolution

            img = np.frombuffer(img, dtype=np.uint8).reshape((resY, resX, 3))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            img = cv2.flip(img, 0)  # Voltear imagen verticalmente

            # Detección de pelota (verde)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lower_green = np.array([40, 70, 70])
            upper_green = np.array([80, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                (x, y), radius = cv2.minEnclosingCircle(largest_contour)
                center = (int(x), int(y))

                # Dibuja la pelota
                cv2.circle(img, center, int(radius), (0, 255, 0), 2)
                cv2.circle(img, center, 2, (0, 255, 255), -1)

                # Publicar la posición de la pelota
                msg = Point()
                msg.x = float(x)
                msg.y = float(y)
                msg.z = 0.0
                self.publisher.publish(msg)
            else:
                self.get_logger().info('🎯 Pelota no detectada')

            # Mostrar la imagen
            cv2.imshow('Imagen desde visionSensor', img)
            cv2.waitKey(1)

            # LLAMADA A STEP
            self.sim.step()

        except Exception as e:
            self.get_logger().error(f'❌ Error capturando imagen: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
