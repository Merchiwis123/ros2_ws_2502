import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math

class ControlWall(Node):
    def __init__(self):
        super().__init__('control_wall')

        # Subscripciones
        self.create_subscription(Float32, '/error', self.error_callback, 10)
        self.create_subscription(Float32, '/alfa', self.alfa_callback, 10)

        # Publicador
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_ctrl', 10)

        # Variables de control
        self.error = 0.0
        self.prev_error = 0.0
        self.alfa = 0.0
        self.velocidad_x = 0.2  # velocidad base
        self.kp = 3.0
        self.kd = 1.5

    def error_callback(self, msg: Float32):
        self.error = msg.data
        self.publish_velocity()

    def alfa_callback(self, msg: Float32):
        self.alfa = msg.data  # Guardamos el valor actual de alfa (en radianes)

        # Ajuste dinámico de la velocidad lineal
        alfa_deg = math.degrees(self.alfa)
        if -5.0 < alfa_deg < 5.0:
            self.velocidad_x = min(self.velocidad_x + 0.02, 1.0)
        else:
            self.velocidad_x = 0.1

    def publish_velocity(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.velocidad_x

        # Si el ángulo alfa es muy grande, detener giro
        salida_angular = self.compute_pd()
        salida_angular = max(min(salida_angular, 1.0), -1.0)

        if self.alfa > 0.35 and salida_angular > 0:
            salida_angular = 0.0
            twist_msg.linear.x = 0.4
        elif self.alfa < -0.35 and salida_angular < 0:
            salida_angular = 0.0
            twist_msg.linear.x = 0.4

        twist_msg.angular.z = salida_angular
        self.cmd_pub.publish(twist_msg)

        self.get_logger().debug(
            f"Error: {self.error:.3f}, Alfa: {self.alfa:.3f}, Vel_x: {self.velocidad_x:.2f}, Angular: {salida_angular:.3f}"
        )

    def compute_pd(self):
        derivative = self.error - self.prev_error
        salida = self.kp * self.error + self.kd * derivative
        self.prev_error = self.error
        return salida

def main(args=None):
    rclpy.init(args=args)
    node = ControlWall()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

