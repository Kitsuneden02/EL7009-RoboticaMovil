#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import matplotlib.pyplot as plt
import numpy as np
import time
import os
from collections import deque

class PosePlotter(Node):
    def __init__(self):
        super().__init__('pose_plotter')
        
        # Configuración de matplotlib
        plt.style.use('seaborn')
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(16, 7))
        self.fig.suptitle('AMCL vs Ground Truth - Comparación en Tiempo Real', fontsize=14)
        
        # Configurar gráfico de trayectoria
        self.ax1.set_title('Trayectorias')
        self.ax1.set_xlabel('Posición X (m)')
        self.ax1.set_ylabel('Posición Y (m)')
        self.ax1.grid(True)
        
        # Configurar gráfico de error
        self.ax2.set_title('Error de Posición')
        self.ax2.set_xlabel('Muestras')
        self.ax2.set_ylabel('Error (m)')
        self.ax2.grid(True)
        
        # Inicializar líneas
        self.line_amcl, = self.ax1.plot([], [], 'r-', label='AMCL', alpha=0.7, linewidth=2)
        self.line_gt, = self.ax1.plot([], [], 'b-', label='Ground Truth', alpha=0.7, linewidth=2)
        self.marker_amcl, = self.ax1.plot([], [], 'ro', markersize=8, label='AMCL Actual')
        self.marker_gt, = self.ax1.plot([], [], 'bs', markersize=8, label='GT Actual')
        self.line_error, = self.ax2.plot([], [], 'g-', label='Error')
        
        # Leyendas
        self.ax1.legend(loc='upper right')
        self.ax2.legend(loc='upper right')
        
        # Almacenamiento de datos
        self.max_points = 1000
        self.amcl_x = deque(maxlen=self.max_points)
        self.amcl_y = deque(maxlen=self.max_points)
        self.gt_x = deque(maxlen=self.max_points)
        self.gt_y = deque(maxlen=self.max_points)
        self.errors = deque(maxlen=self.max_points)
        
        # Variables para márgenes dinámicos
        self.x_margin = 1.0  # Margen inicial en X
        self.y_margin = 1.0  # Margen inicial en Y
        self.error_margin = 0.1  # Margen para el gráfico de error
        
        # Suscriptores
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10)
        
        self.gt_sub = self.create_subscription(
            PoseStamped,
            '/ground_truth_pose',
            self.gt_callback,
            10)
        
        # Configurar modo interactivo
        plt.ion()
        plt.show(block=False)
        
        # Timer para actualización del gráfico
        self.timer = self.create_timer(0.05, self.update_plot)
        
        self.get_logger().info("Visualización iniciada. Mueve el robot para ver los datos.")
    
    def amcl_callback(self, msg):
        try:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.amcl_x.append(x)
            self.amcl_y.append(y)
        except Exception as e:
            self.get_logger().error(f"Error en AMCL callback: {str(e)}")
    
    def gt_callback(self, msg):
        try:
            x = msg.pose.position.x
            y = msg.pose.position.y
            self.gt_x.append(x)
            self.gt_y.append(y)
            
            # Calcular error si tenemos datos de AMCL
            if len(self.amcl_x) > 0 and len(self.amcl_y) > 0:
                error = np.sqrt((x - self.amcl_x[-1])**2 + (y - self.amcl_y[-1])**2)
                self.errors.append(error)
        except Exception as e:
            self.get_logger().error(f"Error en GT callback: {str(e)}")
    
    def update_plot(self):
        try:
            # Actualizar trayectorias
            if len(self.amcl_x) > 0:
                self.line_amcl.set_data(self.amcl_x, self.amcl_y)
                self.marker_amcl.set_data([self.amcl_x[-1]], [self.amcl_y[-1]])
            
            if len(self.gt_x) > 0:
                self.line_gt.set_data(self.gt_x, self.gt_y)
                self.marker_gt.set_data([self.gt_x[-1]], [self.gt_y[-1]])
            
            # Actualizar error
            if len(self.errors) > 0:
                self.line_error.set_data(range(len(self.errors)), self.errors)
                
                # Ajustar límites del gráfico de error dinámicamente
                if len(self.errors) > 0:
                    min_error = max(0, min(self.errors) - self.error_margin)
                    max_error = max(self.errors) + self.error_margin
                    self.ax2.set_xlim(0, len(self.errors))
                    self.ax2.set_ylim(min_error, max_error)
            
            # Ajustar límites del gráfico de trayectoria dinámicamente
            if len(self.amcl_x) > 0 or len(self.gt_x) > 0:
                # Combinar todos los puntos de ambas trayectorias
                all_x = list(self.amcl_x) + list(self.gt_x)
                all_y = list(self.amcl_y) + list(self.gt_y)
                
                if all_x and all_y:  # Verificar que hay datos
                    # Calcular márgenes dinámicos basados en el rango de datos
                    x_range = max(all_x) - min(all_x)
                    y_range = max(all_y) - min(all_y)
                    
                    # Ajustar márgenes proporcionalmente al rango de datos
                    self.x_margin = max(1.0, x_range * 0.2)  # 20% del rango o 1.0m mínimo
                    self.y_margin = max(1.0, y_range * 0.2)  # 20% del rango o 1.0m mínimo
                    
                    # Establecer límites con márgenes dinámicos
                    self.ax1.set_xlim(min(all_x) - self.x_margin, max(all_x) + self.x_margin)
                    self.ax1.set_ylim(min(all_y) - self.y_margin, max(all_y) + self.y_margin)
                    
                    # Mantener relación de aspecto igual (1:1)
                    self.ax1.set_aspect('equal', adjustable='datalim')
            
            # Redibujar
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            self.get_logger().error(f"Error al actualizar gráfico: {str(e)}")
    
    def save_plot(self):
        try:
            os.makedirs('plots', exist_ok=True)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"plots/amcl_vs_gt_{timestamp}.png"
            self.fig.savefig(filename, dpi=300, bbox_inches='tight')
            self.get_logger().info(f"Gráfico guardado como: {filename}")
            return filename
        except Exception as e:
            self.get_logger().error(f"Error al guardar gráfico: {str(e)}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = PosePlotter()
    
    try:
        while rclpy.ok() and plt.fignum_exists(node.fig.number):
            rclpy.spin_once(node, timeout_sec=0.05)
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        saved_file = node.save_plot()
        if saved_file:
            print(f"\nGráfico guardado en: {saved_file}")
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()