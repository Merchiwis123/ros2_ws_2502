#!/usr/bin/env python3

import rclpy
from custom_pkg.conti import continuonode

def main(args=None):
    rclpy.init(args=args)
    node = continuonode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()