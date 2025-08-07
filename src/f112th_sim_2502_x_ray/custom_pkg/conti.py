import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class continuonode(Node):
    def __init__(self):
        super().__init__('closest_obstacle_node')
        self.publisher = self.create_publisher(Twist, 'normal', 10)
        self.flagstop = 0
        timer_period = 0.25
        self.timer = self.create_timer(timer_period, self.action_stop)

    def action_stop(self):
        movin = Twist()
        movin.linear.x = 0.5
        movin.angular.z = 0.0
        self.publisher.publish(movin)
        