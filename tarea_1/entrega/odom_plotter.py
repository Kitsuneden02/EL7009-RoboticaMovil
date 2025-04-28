#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import numpy as np

class OdomPlotter(Node):
    def __init__(self):
        super().__init__('odom_plotter')
        
        # Configuración del gráfico
        plt.ion()  # Modo interactivo
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-', label='Trayectoria Odom')
        self.markers = self.ax.plot([], [], 'ro', label='Marcadores Odom')[0]
        self.gt_markers = self.ax.plot([], [], 'go', label='Ground Truth')[0]
        
        self.orientation_arrows = []  # Flechas de orientación de trayectoria
        self.marker_arrows = []       # Flechas de orientación de marcadores
        self.gt_arrows = []           # Flechas de orientación de ground truth
        
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Trayectoria del Robot')
        self.ax.grid(True)
        self.ax.legend()
        
        # Datos
        self.x_data = []
        self.y_data = []
        self.theta_data = []  # Guardar orientación trayectoria
        
        self.marker_x = []
        self.marker_y = []
        self.marker_theta = []
        
        self.gt_x = []
        self.gt_y = []
        self.gt_theta = []
        
        # Suscriptor a odometría
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # Suscriptor a ground truth
        self.gt_sub = self.create_subscription(
            Point,
            '/ground_truth_point',
            self.ground_truth_callback,
            10)
        
        # Servicio
        self.srv_mark = self.create_service(
            Empty,
            'mark_position',
            self.mark_position_callback)
        
        # Timer para actualizar el gráfico
        self.timer = self.create_timer(0.05, self.update_plot)
        
        self.get_logger().info('Odom Plotter listo. Servicios:')
        self.get_logger().info('/mark_position - Marca posición actual (rojo)')
        self.get_logger().info('Ground truth ahora llega por tópico: /ground_truth_point (tipo Point: x, y, theta en radianes)')

    def odom_callback(self, msg):
        x = round(msg.pose.pose.position.x, 2)
        y = round(msg.pose.pose.position.y, 2)
        
        # Obtener theta (yaw) desde cuaternión
        q = msg.pose.pose.orientation
        theta = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        
        self.x_data.append(x)
        self.y_data.append(y)
        self.theta_data.append(theta)
        
        self.current_pose = (x, y, theta)

    def quaternion_to_yaw(self, x, y, z, w):
        """Convierte un cuaternión a yaw."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def mark_position_callback(self, request, response):
        if hasattr(self, 'current_pose'):
            x, y, theta = self.current_pose
            self.marker_x.append(x)
            self.marker_y.append(y)
            self.marker_theta.append(theta)
            self.get_logger().info(f'Marcador Odom añadido en: ({x:.2f}, {y:.2f}, theta={np.degrees(theta):.1f}°)')
        else:
            self.get_logger().warn('No hay datos de odometría disponibles aún')
        return response

    def ground_truth_callback(self, msg):
        x = round(msg.x, 2)
        y = round(msg.y, 2)
        theta = round(msg.z, 2)  # Usamos 'z' como theta
        
        self.gt_x.append(x)
        self.gt_y.append(y)
        self.gt_theta.append(theta)
        
        self.get_logger().info(f'Marcador GT añadido en: ({x:.2f}, {y:.2f}, theta={np.degrees(theta):.1f}°)')

    def update_plot(self):
        if len(self.x_data) > 0:
            self.line.set_data(self.x_data, self.y_data)
            
            if len(self.marker_x) > 0:
                self.markers.set_data(self.marker_x, self.marker_y)
            
            if len(self.gt_x) > 0:
                self.gt_markers.set_data(self.gt_x, self.gt_y)
            
            # Eliminar flechas viejas
            for arrow in self.orientation_arrows + self.marker_arrows + self.gt_arrows:
                arrow.remove()
            self.orientation_arrows.clear()
            self.marker_arrows.clear()
            self.gt_arrows.clear()

            # Dibujar flechas de orientación actuales
            for (x, y, theta) in zip(self.x_data, self.y_data, self.theta_data):
                arrow = self.ax.arrow(
                    x, y,
                    0.12 * np.cos(theta), 0.12 * np.sin(theta),
                    head_width=0.04, head_length=0.04, fc='k', ec='k')
                self.orientation_arrows.append(arrow)

            # Flechas para marcadores
            for (x, y, theta) in zip(self.marker_x, self.marker_y, self.marker_theta):
                arrow = self.ax.arrow(
                    x, y,
                    0.15 * np.cos(theta), 0.15 * np.sin(theta),
                    head_width=0.05, head_length=0.05, fc='r', ec='r')
                self.marker_arrows.append(arrow)

            # Flechas para ground truth
            for (x, y, theta) in zip(self.gt_x, self.gt_y, self.gt_theta):
                arrow = self.ax.arrow(
                    x, y,
                    0.15 * np.cos(theta), 0.15 * np.sin(theta),
                    head_width=0.05, head_length=0.05, fc='g', ec='g')
                self.gt_arrows.append(arrow)

            self.ax.relim()
            self.ax.autoscale_view()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = OdomPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

