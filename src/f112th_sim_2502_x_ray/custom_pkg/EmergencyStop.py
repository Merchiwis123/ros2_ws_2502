import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class EmergencySStop(Node):
    def __init__(self):
        super().__init__('closest_obstacle_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'emergency', 10)
        self.flagstop = 0
        timer_period = 0.25
        self.timer = self.create_timer(timer_period, self.action_stop)

    def laser_callback(self, msg):
        # Filtra los valores válidos: dentro del rango permitido y no infinitos
        valid_ranges = [
            r for r in msg.ranges
            if msg.range_min < r < msg.range_max and not math.isinf(r)
        ]
        valid_ranges2 = valid_ranges[144:214]
        if valid_ranges2:
            min_distance = min(valid_ranges2)
            if min_distance <= 0.32:
                self.flagstop = 1
            else:
                self.flagstop = 0
        else:
             self.flagstop = 0
    def action_stop(self):
        movin = Twist()
        if self.flagstop == 1:
            movin.linear.x = -0.1
            movin.angular.z = 0.0
            self.publisher.publish(movin)
            self.get_logger().info('Cerca de colisión')
            
        
