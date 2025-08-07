#!/usr/bin/env python3

import rclpy
from custom_pkg.distancia_node import ClosestObstacleNode

def main(args=None):
    rclpy.init(args=args)
    node = ClosestObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()