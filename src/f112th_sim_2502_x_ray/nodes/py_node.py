#!/usr/bin/env python3

import rclpy
from custom_pkg.module_to_import import JoystickToTwistNode

def main(args=None):
    rclpy.init(args=args)
    node = JoystickToTwistNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()