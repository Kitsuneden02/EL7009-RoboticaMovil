#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math
import time

class DifferentialController(Node):
    def __init__(self):
        super().__init__('differential_controller')
        
        # Parámetros del robot
        self.r = 0.05  # Radio de las ruedas (m)
        self.l = 0.32  # Distancia entre ruedas (m)
        
        # Ganancias del controlador
        self.k1 = 0.5  # Ganancia para velocidad lineal
        self.k2 = 0.4  # Ganancia para orientación hacia el objetivo
        self.k3 = 1.1  # Ganancia para orientación final
        
        # Umbrales de control
        self.position_threshold = 0.005  # 5 cm
        self.orientation_threshold = 0.017  # ~5 grados (0.087 rad)
        
        # Límites de velocidad
        self.max_linear_vel = 0.5  # m/s
        self.max_angular_vel = 1.0  # rad/s
        
        # Estado actual del robot
        self.current_pose = None
        self.target_pose = None
        self.goal_reached = False
        self.stable_count = 0
        self.stable_threshold = 10  # Número de iteraciones estables para considerar objetivo alcanzado
        
        # Suscriptores
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Publicadores para las ruedas
        self.left_wheel_pub = self.create_publisher(Float64, 'left_wheel_cmd_vel', 10)
        self.right_wheel_pub = self.create_publisher(Float64, 'right_wheel_cmd_vel', 10)
        
        # Temporizador para el control
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Final Differential Controller initialized")

    def odom_callback(self, msg):
        """Callback para actualizar la pose actual del robot"""
        self.current_pose = msg.pose.pose
        
    def set_target(self, x, y, theta):
        """Establece la pose objetivo"""
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        # Convertir theta a cuaternión (simplificado)
        self.target_pose.pose.orientation.z = math.sin(theta/2)
        self.target_pose.pose.orientation.w = math.cos(theta/2)
        self.get_logger().info(f"Nuevo objetivo establecido: ({x}, {y}, {theta})")
        self.goal_reached = False
        self.stable_count = 0
        self.start_time = time.time()

    def control_loop(self):
        """Bucle principal de control con condición de terminación"""
        if self.current_pose is None or self.target_pose is None or self.goal_reached:
            return
            
        # Obtener posición actual
        x_robot = self.current_pose.position.x
        y_robot = self.current_pose.position.y
        
        # Obtener orientación actual
        q = self.current_pose.orientation
        theta_robot = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        
        # Obtener posición objetivo
        x_target = self.target_pose.pose.position.x
        y_target = self.target_pose.pose.position.y
        
        # Obtener orientación objetivo
        q_target = self.target_pose.pose.orientation
        theta_target = math.atan2(2*(q_target.w*q_target.z + q_target.x*q_target.y), 
                                 1-2*(q_target.y*q_target.y + q_target.z*q_target.z))
        
        # Calcular diferencias
        delta_x = x_target - x_robot
        delta_y = y_target - y_robot
        distance = math.sqrt(delta_x**2 + delta_y**2)
        
        # Ángulo hacia el objetivo
        theta_to_target = math.atan2(delta_y, delta_x)
        
        # Diferencia de ángulo para orientación final
        delta_theta_final = theta_target - theta_robot
        # Normalizar el ángulo a [-π, π]
        delta_theta_final = math.atan2(math.sin(delta_theta_final), math.cos(delta_theta_final))
        
        # Verificar condición de terminación
        if (distance < self.position_threshold and 
            abs(delta_theta_final) < self.orientation_threshold):
            self.stable_count += 1
            if self.stable_count >= self.stable_threshold:
                self.goal_reached = True
                self.stop_robot()
                self.get_logger().info("¡Objetivo alcanzado con éxito!")
                self.get_logger().info(f"Posición final: ({x_robot:.2f}, {y_robot:.2f}, {theta_robot:.2f} rad)")
                self.get_logger().info(f"Tiempo demorado: {time.time() - self.start_time}s")
                return
        else:
            self.stable_count = 0
        
        # Controlador mejorado con cambio de modos
        if distance > self.position_threshold:
            # Modo 1: Alcanzar la posición objetivo
            v = min(self.k1 * distance, self.max_linear_vel)
            
            # Diferencia de ángulo para apuntar al objetivo
            delta_theta = theta_to_target - theta_robot
            # Normalizar el ángulo a [-π, π]
            delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))
            
            omega = self.k2 * delta_theta
        else:
            # Modo 2: Ajustar la orientación final
            v = 0.0
            omega = self.k3 * delta_theta_final
        
        # Limitar la velocidad angular
        omega = max(min(omega, self.max_angular_vel), -self.max_angular_vel)
        
        # Aplicar cinemática inversa
        left_wheel_vel, right_wheel_vel = self.inverse_kinematics(v, omega)
        
        # Publicar velocidades
        left_msg = Float64()
        left_msg.data = left_wheel_vel
        self.left_wheel_pub.publish(left_msg)
        
        right_msg = Float64()
        right_msg.data = right_wheel_vel
        self.right_wheel_pub.publish(right_msg)
        
        # Loggear estado actual (cada 10 iteraciones para no saturar)
        if self.stable_count % 2 == 0:
            self.get_logger().info(
                f"Pos: ({x_robot:.2f}, {y_robot:.2f}), θ: {math.degrees(theta_robot):.1f}° | "
                f"Dist: {distance:.2f}m, Delta_θ: {math.degrees(delta_theta_final):.1f}° | "
                f"Vel: {v:.2f}m/s, ω: {math.degrees(omega):.1f}°/s | "
                f"Estable: {self.stable_count}/{self.stable_threshold}")

    def stop_robot(self):
        """Detiene completamente el robot"""
        left_msg = Float64()
        left_msg.data = 0.0
        self.left_wheel_pub.publish(left_msg)
        
        right_msg = Float64()
        right_msg.data = 0.0
        self.right_wheel_pub.publish(right_msg)

    def inverse_kinematics(self, v, omega):
        """Cinemática inversa para robot diferencial con limitación de velocidad"""
        left_vel = (2*v - omega*self.l) / (2*self.r)
        right_vel = (2*v + omega*self.l) / (2*self.r)
        self.get_logger().info(f"Left_Vel: {left_vel}, Right_Vel: {right_vel}")
        
        # Limitar las velocidades de las ruedas
        max_wheel_vel = 10.0  # rad/s
        left_vel = max(min(left_vel, max_wheel_vel), -max_wheel_vel)
        right_vel = max(min(right_vel, max_wheel_vel), -max_wheel_vel)
        
        return left_vel, right_vel

def main(args=None):
    rclpy.init(args=args)
    
    controller = DifferentialController()
    

    controller.set_target(2.0, 3.0, math.pi/2)

    while not controller.goal_reached:
        rclpy.spin_once(controller) 
    
    controller.get_logger().info("Deteniendo el nodo...")
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()