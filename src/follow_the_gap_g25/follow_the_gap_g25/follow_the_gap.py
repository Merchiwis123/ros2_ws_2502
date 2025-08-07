#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
# ros2 launch f112th_sim_2502_x_ray launch_sim.launch.py world:=$HOME/ros2_ws_2502/src/f112th_sim_2502_x_ray/worlds/Carrera.world
class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_follow_gap')

        self.bubble_radius = 0.4  # metros
        self.wheelbase = 0.325   # metros
        self.block_threshold = 0.40  # m: distancia mínima para considerar que hay una pared al frente
        self.last_turn_direction = 0  # -1 = izquierda, 1 = derecha
        self.stuck_counter = 0

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_ctrl', 10)

    def preprocess_lidar(self, ranges):
        ranges = np.array(ranges)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0, neginf=0.0)
        max_range = 4.2
        ranges[ranges > max_range] = max_range

        kernel_size = 3
        kernel = np.ones(kernel_size) / kernel_size
        ranges_smoothed = np.convolve(ranges, kernel, mode='same')
        return ranges_smoothed.tolist()
    def find_max_gap(self, free_space_ranges):
        """
        Encuentra el mejor gap según prioridad:
        1. Gap suficientemente ancho para el robot.
        2. Gap más ancho.
        3. Gap más lejano (distancia media).
        4. Gap más centrado (menor desviación del centro).
        """
        min_gap_width = 0.4  # ancho mínimo necesario
        angle_increment = math.radians(1.0)
        center_idx = len(free_space_ranges) // 2

        best_gap = (0, 0)
        best_criteria = (-1, -1, float('inf'), float('inf'))  # (width, r, center_offset, prefer_left)

        i = 0
        while i < len(free_space_ranges):
            if free_space_ranges[i] > 0.3:
                start = i
                while i < len(free_space_ranges) and free_space_ranges[i] > 0.1:
                    i += 1
                end = i - 1

                gap_size = end - start + 1
                if gap_size < 2:
                    continue

                # Calcular ancho físico del gap
                gap_angle = gap_size * angle_increment
                r = np.mean(free_space_ranges[start:end + 1])
                width = 2 * r * math.sin(gap_angle / 2.0)

                if width >= min_gap_width:
                    mid_idx = (start + end) // 2
                    center_offset = abs(mid_idx - center_idx)

                    # Criterios: mayor width, luego mayor r, luego menor center_offset
                    prefer_right = mid_idx  # mientras más grande el índice, más a la derecha
                    criteria = (width, r, -center_offset, prefer_right)

                    if criteria > best_criteria:
                        best_criteria = criteria
                        best_gap = (start, end)
            else:
                i += 1

        if best_gap == (0, 0):
            # fallback si no se encuentra un gap válido
            mid = len(free_space_ranges) // 2
            return mid, mid

        return best_gap


    def find_best_point(self, start_i, end_i, ranges):
        """
        Encuentra el mejor punto dentro del gap considerando:
        1. Que tenga suficiente espacio libre (el robot quepa con margen).
        2. Que esté lo más alejado posible (r grande).
        3. Que esté lo más centrado posible (menos esfuerzo de giro).
        4. Que no esté muy cerca del borde del escaneo.
        """
        if start_i >= end_i or start_i < 0 or end_i >= len(ranges):
            return len(ranges) // 2

        best_index = (start_i + end_i) // 2
        best_score = -float('inf')
        robot_half_width = 0.35
        angle_increment = math.radians(1.0)
        center = len(ranges) // 2

        for i in range(start_i, end_i + 1):
            r = ranges[i]
            if r < 0.1:
                continue  # descartar puntos inválidos

            # Cálculo del ángulo necesario para pasar
            angle_width = 2 * math.atan(robot_half_width / r)
            index_width = int(angle_width / angle_increment)

            left = max(i - index_width // 2, 0)
            right = min(i + index_width // 2, len(ranges) - 1)

            # Verificar que haya suficiente espacio alrededor del punto
            if min(ranges[left:right + 1]) < r - 0.2:
                continue

            # === Puntaje compuesto ===

            # 1. Espacioso (espacio válido ya está garantizado arriba)
            clearance_score = r  # mayor r, mejor

            # 2. Lejanía (ya cubierta con clearance)
            # No se agrega otro término

            # 3. Centración
            center_offset = abs(i - center)
            centered_score = -0.01 * center_offset

            # 4. Penalización por estar en el borde
            border_penalty = -0.05 * (abs(i - center) / center)

            # 4. Preferencia por la derecha (bono sutil)
            prefer_right_score = 0.02 * i

            # Puntaje total
            score = clearance_score + centered_score + border_penalty+ prefer_right_score 

            if score > best_score:
                best_score = score
                best_index = i

        return best_index




    def zero_out_safety_bubble(self, ranges, center_index, bubble_radius):
        center_dist = ranges[center_index]
        ranges[center_index] = 0.0

        i = center_index + 1
        while i < len(ranges) and ranges[i] < center_dist + bubble_radius:
            ranges[i] = 0.0
            i += 1

        i = center_index - 1
        while i >= 0 and ranges[i] < center_dist + bubble_radius:
            ranges[i] = 0.0
            i -= 1

    def lidar_callback(self, data):
        ranges = np.array(data.ranges)

        # Limitamos el ángulo entre -90° y +90° (índices 89 a 179 en un escáner de 1°)
        proc_ranges = self.preprocess_lidar(ranges[89:269])
        left_side = proc_ranges[:45]
        right_side = proc_ranges[-45:]
        lateral_ajuste = 0.0
        if min(left_side) < 0.35:
            lateral_ajuste = 0.8  # gira a la derecha
        elif min(right_side) < 0.35:
            lateral_ajuste = -0.8  # gira a la izquierda

        # === VERIFICAR SI EL CAMINO FRONTAL ESTÁ BLOQUEADO ===
        center_index = len(proc_ranges) // 2
        offset = 0 
        center_window = proc_ranges[center_index - 45 : center_index + 45]

        # Si la mayoría de los valores están por debajo del umbral, es una pared
        if sum(d < self.block_threshold for d in center_window) > 7:
            twist_msg = Twist()
            twist_msg.linear.x = 0.0

            best_index = self.find_best_point(0, len(proc_ranges) - 1, proc_ranges)
            center_index = len(proc_ranges) // 2
            offset = best_index - center_index

            # Decidir nueva dirección solo si no stamos "congelados"
            if self.stuck_counter == 0:
                self.last_turn_direction = -1 if offset < 0 else 1
                self.stuck_counter = 45  # mantener giro por 10 ciclos

            # Aplicar la dirección decidida
            twist_msg.angular.z = 0.4 * -self.last_turn_direction

            self.cmd_vel_pub.publish(twist_msg)
            self.stuck_counter -= 1
            return

        # === FOLLOW THE GAP ===
        closest_index = int(np.argmin(proc_ranges))
        if abs(offset) > 30:
            self.zero_out_safety_bubble(proc_ranges, closest_index, self.bubble_radius * 2)
        else:
            self.zero_out_safety_bubble(proc_ranges, closest_index, self.bubble_radius)

        start_i, end_i = self.find_max_gap(proc_ranges)
        best_index = self.find_best_point(start_i, end_i, proc_ranges)

        # Convertimos índice a ángulo
        angle = data.angle_min + (89 + best_index) * data.angle_increment  # compensar el recorte
        angle = max(min(angle, 1.5708), -1.5708)

        twist_msg = Twist()
        twist_msg.linear.x = 0.1 if abs(angle) > 0.7 else 0.7
        twist_msg.angular.z = twist_msg.linear.x * math.tan(angle) / self.wheelbase + lateral_ajuste
        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()