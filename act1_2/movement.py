import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
sys.path.append('/home/mateo/CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04/programming/zmqRemoteApi/clients/python/src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class CoppeliaTeleopBridge(Node):
    def __init__(self):
        super().__init__('coppelia_teleop_bridge')

        # Conexión a CoppeliaSim usando ZMQ Remote API
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.sim.setStepping(True)  # permite paso a paso con trigger
            self.get_logger().info('✅ Connected to CoppeliaSim via ZMQ Remote API')
        except Exception as e:
            self.get_logger().error(f'❌ Could not connect to CoppeliaSim: {str(e)}')
            raise

        # Obtener handles de los motores
        self.left_motor = self.sim.getObject('/PioneerP3DX/leftMotor')
        self.right_motor = self.sim.getObject('/PioneerP3DX/rightMotor')

        # Parámetros del robot
        self.wheel_radius = 0.0975  # radios en metros
        self.wheel_base = 0.381     # distancia entre ruedas

        # Subscriptor a cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('🚀 Subscribed to /cmd_vel')

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # Cinemática diferencial
        v_left = (v - w * self.wheel_base / 2.0) / self.wheel_radius
        v_right = (v + w * self.wheel_base / 2.0) / self.wheel_radius

        # Enviar velocidades a CoppeliaSim
        self.sim.setJointTargetVelocity(self.left_motor, v_left)
        self.sim.setJointTargetVelocity(self.right_motor, v_right)

        self.sim.step()  # Paso del simulador

        self.get_logger().info(f'Enviando velocidades: left={v_left:.2f}, right={v_right:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaTeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
