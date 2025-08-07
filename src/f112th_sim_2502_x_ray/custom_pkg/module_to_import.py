#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickToTwistNode(Node):
    def __init__(self):
        super().__init__('joystick_to_twist')

        # Parámetros desde YAML o launch
        self.linear_axis = self.declare_parameter('axis_linear.x', 1).value
        self.linear_scale = self.declare_parameter('scale_linear.x', 0.5).value
        self.angular_axis = self.declare_parameter('axis_angular.yaw', 2).value
        self.angular_scale = self.declare_parameter('scale_angular.yaw', 0.5).value
        self.deadzone = self.declare_parameter('deadzone', 0.05).value
        self.enable_button = self.declare_parameter('enable_button', 4).value
        self.require_enable_button = self.declare_parameter('require_enable_button', True).value

        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bandera = 0


    def joy_callback(self, msg):
        twist = Twist()

        # Validación de botón habilitador
        if self.require_enable_button:
                if len(msg.buttons) <= self.enable_button or msg.buttons[self.enable_button] != 1:
                        if self.bandera == 1:
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            self.bandera = 0
                            self.publisher.publish(twist) 
                            return

                elif len(msg.axes) > max(self.linear_axis, self.angular_axis):
                    self.bandera = 1
                    linear_input = msg.axes[1]
                    angular_input = msg.axes[0]

                    # Aplicar zona muerta
                    linear_input = 0.0 if abs(linear_input) < self.deadzone else linear_input
                    angular_input = 0.0 if abs(angular_input) < self.deadzone else angular_input

                    twist.linear.x = self.linear_scale * linear_input
                    twist.angular.z = self.angular_scale * angular_input

                    self.publisher.publish(twist)
                    self.get_logger().debug(f"cmd_vel -> linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")
                else:
                    self.get_logger().warn("Mensaje Joy no contiene suficientes ejes.")
