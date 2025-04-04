#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from collections import deque
from std_srvs.srv import Empty

import math

class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')
        self.node = rclpy.create_node('wheel_controller')
        self.mark_position_client = self.node.create_client(Empty, '/mark_position')
        
        self.left_pub = self.create_publisher(Float64, 'left_wheel_cmd_vel', 10)
        self.right_pub = self.create_publisher(Float64, 'right_wheel_cmd_vel', 10)
        
        self.get_logger().info("Controller initialized. Ready for commands.")
        
        # Sistema de cola de acciones
        self.action_queue = deque()
        self.current_action = None
        self.action_timer = None
        
        # Parámetros del robot según lo calculado en la parte 1)
        self.r = 0.05
        self.l = 0.32

        # Timer
        self.process_timer = self.create_timer(0.1, self.process_queue)

    # Avanzar en linea recta
    def move_forward(self, duration: float, speed: float = 1.0):
        self.action_queue.append(('FORWARD', duration, speed))
        self.get_logger().info(f"Added FORWARD action: {speed} m/s for {duration}s")

    # Rotar
    def rotate(self, duration: float, clockwise: bool = True):
        self.action_queue.append(('ROTATE', duration, clockwise))
        direction = "clockwise" if clockwise else "counter-clockwise"
        self.get_logger().info(f"Added ROTATION action: {direction} for {duration}s")

    # Cinemática inversa, asume vel angular fija y cte = |3.0| para desplazamientos y vel = 0.5 para rotaciones
    # Calcula el tiempo necesario para llegar a los puntos necesarios.
    # Se basa en el sistema de coordenadas del ROBOT.
    def inverse_kinematics(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0, fwd_vel: float = 3.0, rt_vel: float = 0.5):
        # Por convención: 1) Rotar hacia el punto de destino, 2) Traslación al punto, 3) Rotar hacia la pose
        def rotation_time(theta):
            k_r = 0.97595
            self.get_logger().info(f"Tiempo estimado de rotación: {(theta*self.l)*k_r/(2*rt_vel*self.r)}")
            return (theta*self.l)*k_r/(2*rt_vel*self.r)
        def forward_time(x, theta, y_axis: bool = False):
            k_f = 1.00802
            self.get_logger().info(f"Tiempo estimado de traslación: {x*k_f/(fwd_vel*self.r*math.cos(theta)) if not y_axis \
                                                                    else (x*k_f)/(fwd_vel*self.r*math.sin(theta))}")
            return k_f*(x)/(fwd_vel*self.r*math.cos(theta)) if not y_axis \
                   else k_f*(x)/(fwd_vel*self.r*math.sin(theta))

        # Verificar si existe un punto (x,y) de lo contrario, no tendría sentido realizar los pasos 1) y 2)
        if (x != 0.0) or (y != 0.0):
            # 1)
            temp_theta = math.atan2(y, x) # Ángulo temporal para mirar en dirección al punto deseado
            self.rotate(abs(rotation_time(abs(temp_theta))), True if temp_theta > 0 else False)
            # 2)
            self.move_forward(abs(forward_time(abs(x), abs(temp_theta))), 3.0) if x!=0.0 \
            else self.move_forward(abs(forward_time(abs(y), abs(temp_theta), True)), 3.0)
        # 3)
        if theta != 0.0:
            self.rotate(rotation_time(abs(theta)), True if theta > 0 else False)


    def process_queue(self):
        if self.current_action is None and self.action_queue:
            # Comenzar nueva acción
            self.current_action = self.action_queue.popleft()
            action_type, duration, param = self.current_action
            
            if action_type == 'FORWARD':
                speed = param
                msg = Float64()
                msg.data = speed
                self.left_pub.publish(msg)
                self.right_pub.publish(msg)
                self.get_logger().info(f"Executing FORWARD: {speed} m/s")
                
            elif action_type == 'ROTATE':
                clockwise = param
                base_speed = 0.5
                left_msg = Float64()
                right_msg = Float64()
                left_msg.data = -base_speed if clockwise else base_speed
                right_msg.data = base_speed if clockwise else -base_speed
                
                self.left_pub.publish(left_msg)
                self.right_pub.publish(right_msg)
                direction = "clockwise" if clockwise else "counter-clockwise"
                self.get_logger().info(f"Executing ROTATE: {direction}")
            
            # Timer para la acción
            self.action_timer = self.create_timer(duration, self.complete_current_action)

    def complete_current_action(self):
        if self.action_timer:
            self.action_timer.cancel()
        
        stop_msg = Float64()
        stop_msg.data = 0.0
        self.left_pub.publish(stop_msg)
        self.right_pub.publish(stop_msg)
        
        self.get_logger().info(f"Completed action: {self.current_action[0]}")
        if self.current_action[0] == "FORWARD":
            self.call_mark_position_service()        
        self.current_action = None
        self.action_timer = None

    def clean_shutdown(self):
        self.process_timer.cancel()
        if self.action_timer:
            self.action_timer.cancel()
        
        # Detener cualquier movimiento
        stop_msg = Float64()
        stop_msg.data = 0.0
        self.left_pub.publish(stop_msg)
        self.right_pub.publish(stop_msg)
        
        self.destroy_node()
        rclpy.shutdown()

    # No aporta funcionalidad al controlador, sólo ayuda a graficar mejor
    def call_mark_position_service(self):
        if self.mark_position_client.wait_for_service(timeout_sec=1.0):
            request = Empty.Request()
            self.mark_position_client.call_async(request)
        else:
            self.node.get_logger().warn('Servicio /mark_position no disponible')

def main(args=None):
    rclpy.init(args=args)
    controller = WheelController()
    
    """
    Movimientos para el gráfico de la Parte 3)
    """
    # controller.move_forward(duration=4.0, speed=1.0)
    # controller.rotate(duration=4.0, clockwise=True)
    # controller.move_forward(duration=4.0, speed=1.0)
    # controller.rotate(duration=4.0, clockwise=False)
    # controller.move_forward(duration=4.0, speed=-1.0)
    # controller.rotate(duration=4.0, clockwise=True)
    # controller.move_forward(duration=4.0, speed=1.0)
    # controller.move_forward(duration=4.0, speed=1.0)
    # controller.rotate(duration=4.0, clockwise=False)
    # controller.move_forward(duration=4.0, speed=-1.0)
    # controller.rotate(duration=4.0, clockwise=False)
    # controller.move_forward(duration=4.0, speed=-1.0)

    """
    Movimientos para la Parte 4) (ejecutar script 10 veces o agregar ciclo for)
    """
    controller.inverse_kinematics(x=0.0, y=3.75)
    controller.inverse_kinematics(x=0.0, y=2.9)
    controller.inverse_kinematics(x=0.0, y=3.75)
    controller.inverse_kinematics(x=0.0, y=2.9)
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.clean_shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()