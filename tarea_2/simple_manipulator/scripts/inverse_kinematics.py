#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

import numpy as np
from fk_solution import fk_solution

class IKPublisher(Node):

    def __init__(self):
        super().__init__('fk_publisher')

        self._position_jacobian, self._fk = fk_solution()
        self.get_logger().info('IK Publisher iniciado')

        self._ik_publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribers
        self._joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_callback,
            10)

        self._cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10)
        
        # Timer
        timer_period = 0.02
        self._ik_timer = self.create_timer(timer_period, self._timer_callback)

        # Variables auxiliares
        self._joint_states = None  
        self._delta_signs = np.zeros(3)
        self._delta_pos_gain = 0.005  
        self._nb_iterations = 10
        
        # Variable de control para logging
        self._log_counter = 0


    def _cmd_vel_callback(self, msg):
        prev_signs = self._delta_signs.copy()
        
        self._delta_signs = np.array([
            np.sign(msg.linear.x) if abs(msg.linear.x) > 0.01 else 0,
            np.sign(msg.linear.y) if abs(msg.linear.y) > 0.01 else 0,
            np.sign(msg.linear.z) if abs(msg.linear.z) > 0.01 else 0
        ])
        if abs(msg.linear.y) > 0:
            self.get_logger().info(f'Comando Y recibido: {msg.linear.y:.5f}')
        
        if not np.array_equal(prev_signs, self._delta_signs):
            self.get_logger().info(f'Comando recibido: x={msg.linear.x:.3f}, y={msg.linear.y:.3f}, z={msg.linear.z:.3f}')
            self.get_logger().info(f'Delta signs: {self._delta_signs}')


    def _joint_states_callback(self, msg):
        if len(msg.position) >= 6:
            if self._joint_states is None:
                self.get_logger().info(f'Posiciones iniciales de articulaciones: {msg.position[:6]}')
            self._joint_states = np.array(msg.position[:6])


    def _timer_callback(self):
        if self._joint_states is None:
            return
        
        if np.all(self._delta_signs == 0):
            return
        
        self._log_counter += 1
        if self._log_counter >= 50:
            self._log_counter = 0
            self.get_logger().info(f'Estado actual: {self._joint_states}')
            self.get_logger().info(f'Comandos activos: {self._delta_signs}')
        
        try:
            current_eef_position = self._fk(*self._joint_states)
            
            if not isinstance(current_eef_position, np.ndarray):
                current_eef_position = np.array(current_eef_position)
            
            if current_eef_position.shape == (3,):
                current_eef_position = current_eef_position.reshape(3, 1)
            
            delta_pos = np.array([
                self._delta_pos_gain * self._delta_signs[0],
                self._delta_pos_gain * self._delta_signs[1], 
                self._delta_pos_gain * self._delta_signs[2]
            ]).reshape(3, 1)
            
            target_eef_position = current_eef_position + delta_pos
            
            self.get_logger().debug(f'Posición actual: {current_eef_position.flatten()}')
            self.get_logger().debug(f'Delta: {delta_pos.flatten()}')
            self.get_logger().debug(f'Posición objetivo: {target_eef_position.flatten()}')
            
            ik_result = self._compute_ik(target_eef_position, self._joint_states)

            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = ["shoulder_yaw_joint", "shoulder_pitch_joint", "elbow_joint", 
                               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
            joint_state.position = ik_result.tolist()
            
            self._ik_publisher.publish(joint_state)
            
        except Exception as e:
            self.get_logger().error(f'Error en timer_callback: {str(e)}')


    def _compute_ik(self, eef_position, initial_guess=None):
        if initial_guess is None:
            theta = np.zeros(6)
        else:
            theta = initial_guess.copy()
        
        if hasattr(eef_position, 'shape'):
            if eef_position.shape != (3, 1):
                eef_position = np.array(eef_position).reshape(3, 1)
        else:
            eef_position = np.array(eef_position).reshape(3, 1)
        
        self.get_logger().debug(f'Iniciando IK con theta inicial: {theta}')
        
        for i in range(self._nb_iterations):
            current_position = self._fk(*theta)
            
            if hasattr(current_position, 'shape'):
                if current_position.shape != (3, 1):
                    current_position = np.array(current_position).reshape(3, 1)
            else:
                current_position = np.array(current_position).reshape(3, 1)
            
            error = eef_position - current_position
            
            self.get_logger().debug(f'Iteración {i}: Error: {np.linalg.norm(error):.6f}')
            
            if np.linalg.norm(error) < 1e-5:
                self.get_logger().debug(f'IK convergió en {i} iteraciones')
                break
            
            J = self._position_jacobian(*theta)
            
            if not isinstance(J, np.ndarray):
                J = np.array(J, dtype=float)
            
            try:
                J_pinv = np.linalg.pinv(J)
                
                delta_theta = J_pinv @ error
                
                max_step = 0.1
                if np.max(np.abs(delta_theta)) > max_step:
                    delta_theta = delta_theta * max_step / np.max(np.abs(delta_theta))
                
                theta = theta + delta_theta.flatten()
                
                theta = np.clip(theta, -np.pi, np.pi)
                
            except np.linalg.LinAlgError as e:
                self.get_logger().warn(f'Error en la inversión de matriz: {str(e)}')
                step_size = 0.01
                random_step = np.random.uniform(-step_size, step_size, theta.shape)
                theta = theta + random_step
        
        self.get_logger().debug(f'IK resultado: {theta}')
        return theta


def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger('rclpy')
    logger.set_level(rclpy.logging.LoggingSeverity.INFO)
    ik_publisher = IKPublisher()
    rclpy.spin(ik_publisher)
    ik_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()