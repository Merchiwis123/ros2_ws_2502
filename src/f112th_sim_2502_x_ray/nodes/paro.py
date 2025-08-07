#!/usr/bin/env python3

import rclpy
from custom_pkg.EmergencyStop import EmergencySStop

def main(args=None):
    rclpy.init(args=args)
    node = EmergencySStop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()