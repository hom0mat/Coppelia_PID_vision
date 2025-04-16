import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class PIDFollower(Node):
    def __init__(self):
        super().__init__('pid_follower')

        # Parámetros del PID angular
        
        self.Kp = 0.05
        self.Ki = 0.05
        self.Kd = 0.5

        self.error_prev = 0.0
        self.error_sum = 0.0

        self.image_center_x = 128  # Imagen de 256 x 256
        self.sub = self.create_subscription(Point, 'ball_position', self.listener_callback, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info('🧠 PID Follower inicializado y escuchando a la esfera')

    def listener_callback(self, msg):
        ball_x = msg.x

        # Error = qué tanto se ha desviado del centro
        error = self.image_center_x - ball_x
        self.error_sum += error
        delta_error = error - self.error_prev

        # PID
        angular_z = self.Kp * error + self.Ki * self.error_sum + self.Kd * delta_error
        self.error_prev = error

        twist = Twist()
        twist.linear.x = 0.3  # constante mientras no esté demasiado lejos
        twist.angular.z = angular_z

        self.pub.publish(twist)
        self.get_logger().info(f'🌀 Publicando: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = PIDFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
