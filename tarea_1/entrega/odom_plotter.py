#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
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
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Trayectoria del Robot')
        self.ax.grid(True)
        self.ax.legend()
        
        # Datos
        self.x_data = []
        self.y_data = []
        self.marker_x = []
        self.marker_y = []
        
        # Suscriptor
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Servicio
        self.srv = self.create_service(
            Empty,
            'mark_position',
            self.mark_position_callback)
        
        # Timer para actualizar el gráfico
        self.timer = self.create_timer(0.05, self.update_plot)
        
        self.get_logger().info('Odom Plotter listo. Use el servicio /mark_position para marcar la posición actual.')

    def odom_callback(self, msg):
        self.x_data.append(round(msg.pose.pose.position.x, 2))
        self.y_data.append(round(msg.pose.pose.position.y, 2))
        self.current_pose = msg.pose.pose.position  # Guardar la posición actual

    def mark_position_callback(self, request, response):
        if hasattr(self, 'current_pose'):
            self.marker_x.append(round(self.current_pose.x, 2))
            self.marker_y.append(round(self.current_pose.y, 2))
            self.get_logger().info(f'Marcador añadido en: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})')
        else:
            self.get_logger().warn('No hay datos de odometría disponibles aún')
        return response

    def update_plot(self):
        if len(self.x_data) > 0:
            self.line.set_data(self.x_data, self.y_data)
            
            # Actualizar marcadores si hay alguno
            if len(self.marker_x) > 0:
                self.markers.set_data(self.marker_x, self.marker_y)
            
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