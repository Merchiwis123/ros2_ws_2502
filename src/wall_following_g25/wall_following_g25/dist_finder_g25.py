import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math

class ErrorScan(Node):
    def __init__(self):
        super().__init__('error_scan')

        # Parámetros configurables
        self.target_distance = 1.0   # Distancia deseada a la pared (m)
        self.angle_a = 90           # Índice de rayo "a"
        self.angle_b_offset = 45   # Diferencia de ángulo entre a y b

        # Subscripciones
        self.create_subscription(Odometry, '/odom', self.velocity_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        # Publicadores
        self.error_pub = self.create_publisher(Float32, '/error', 10)
        self.alfa_pub = self.create_publisher(Float32, '/alfa', 10)

        # Variables internas
        self.velocity_x = 0.0
        self.last_time = self.get_clock().now()
        self.error = 0.0
        self.alfa = 0.0

    def laser_callback(self, msg: LaserScan):
        try:
            now = self.get_clock().now()
            delta_time = (now - self.last_time).nanoseconds / 1e9
            self.last_time = now

            a, b, theta = self.extract_ranges(msg.ranges)

            if any(math.isnan(val) or math.isinf(val) for val in [a, b, theta]):
                self.get_logger().warn("Lectura inválida del LIDAR (NaN o Inf).")
                return

            denominator = a * math.sin(theta)
            if abs(denominator) < 1e-6:
                self.get_logger().warn("Denominador muy pequeño para calcular alfa.")
                return

            self.alfa = math.atan2((a * math.cos(theta) - b), denominator)

            if math.isnan(self.alfa) or math.isinf(self.alfa):
                self.get_logger().warn("Ángulo alfa no válido.")
                return

            distance_AB = b * math.cos(self.alfa)
            distance_CB = distance_AB + self.velocity_x * delta_time
            self.error = self.target_distance - distance_CB

            # Publicar inmediatamente
            self.error_pub.publish(Float32(data=self.error))
            self.alfa_pub.publish(Float32(data=self.alfa))

        except IndexError:
            self.get_logger().warn("Índices fuera de rango en los datos del LIDAR.")
        except Exception as e:
            self.get_logger().error(f"Error inesperado: {str(e)}")

    def velocity_callback(self, msg: Odometry):
        self.velocity_x = msg.twist.twist.linear.x

    def extract_ranges(self, ranges):
        idx_a = self.angle_a
        idx_b = self.angle_a + self.angle_b_offset

        if idx_b >= len(ranges):
            raise IndexError("El índice del segundo rayo excede el tamaño del arreglo.")

        range_b = ranges[idx_a]
        range_a = ranges[idx_b]
        theta_rad = math.radians(self.angle_b_offset)

        return range_a, range_b, theta_rad

def main(args=None):
    rclpy.init(args=args)
    node = ErrorScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
