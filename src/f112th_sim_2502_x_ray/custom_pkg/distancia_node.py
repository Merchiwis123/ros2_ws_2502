import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ClosestObstacleNode(Node):
    def __init__(self):
        super().__init__('closest_obstacle_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        self.get_logger().info('Nodo de distancia mínima inicializado.')

    def laser_callback(self, msg):
        # Filtra los valores válidos: dentro del rango permitido y no infinitos
        valid_ranges = [
            r for r in msg.ranges
            if msg.range_min < r < msg.range_max and not math.isinf(r)
        ]

        if valid_ranges:
            min_distance = min(valid_ranges)
            self.get_logger().info(f'Distancia mínima detectada: {min_distance:.2f} m')
        else:
            # No se detectó nada: todos los rayos dieron infinito
            self.get_logger().info('Distancia mínima: inf (sin objetos detectados)')