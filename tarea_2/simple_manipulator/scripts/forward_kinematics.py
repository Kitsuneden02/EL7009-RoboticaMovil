#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from tf_transformations import quaternion_from_matrix, euler_from_quaternion

import numpy as np
from math import cos, sin

class FKPublisher(Node):

    def __init__(self):
        super().__init__('fk_publisher')

        # Publisher for the eef_pose according to forward kinematics
        self._fk_publisher = self.create_publisher(PoseStamped, 'eef_pose', 10)

        # Subscriber for joint states
        self._joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_callback,
            10)
        
        # Timer to publish the eef_pose according to forward kinematics at a fixed frequency
        timer_period = 0.02
        self._fk_timer = self.create_timer(timer_period, self._timer_callback)

        # Auxiliary variables
        self._joint_states = None

        # Dimensiones del brazo
        self.L_base = 0.1
        self.L_shoulder_y = 0.15  # x_length_shoulder_link/2 + x_length_arm_link/2
        self.L_shoulder_z = 0.2   # z_length_shoulder_link - 0.1
        self.L_arm_y = -0.1       # -x_length_arm_link/2 - x_length_forearm_link/2
        self.L_arm_z = 0.8        # z_length_arm_link - 0.1
        self.L_forearm_y = 0.1    # x_length_wrist_1_link/2 + x_length_wrist_2_link/2
        self.L_forearm_z = 0.8    # z_length_forearm_link - 0.1
        self.L_wrist1_x = -0.1    # -x_length_wrist_2_link/2 - x_length_wrist_3_link/2
        self.L_wrist1_z = 0.3     # z_length_wrist_1_link - 0.1
        self.L_wrist2_z = 0.2     # z_length_wrist_2_link
        self.L_wrist3_z = 0.1     # z_length_wrist_3_link


    def _joint_states_callback(self, msg):
        self._joint_states = msg.position[:6]

    def _timer_callback(self):

        if self._joint_states is None:
            return 

        eef_position, eef_quaternion = self._compute_fk(self._joint_states)
        eef_pose = PoseStamped()

        # Configurar el encabezado
        eef_pose.header.stamp = self.get_clock().now().to_msg()
        eef_pose.header.frame_id = 'base_link'
        
        # Configurar posición
        eef_pose.pose.position.x = float(eef_position[0])
        eef_pose.pose.position.y = float(eef_position[1])
        eef_pose.pose.position.z = float(eef_position[2])
        
        # Configurar orientación (cuaternión)
        eef_pose.pose.orientation.x = float(eef_quaternion[0])
        eef_pose.pose.orientation.y = float(eef_quaternion[1])
        eef_pose.pose.orientation.z = float(eef_quaternion[2])
        eef_pose.pose.orientation.w = float(eef_quaternion[3])

        self._fk_publisher.publish(eef_pose)


    def _compute_fk(self, joint_states):

        theta_1, theta_2 ,theta_3, theta_4, theta_5, theta_6 = joint_states

        # Matriz base a shoulder_yaw_joint (traslación en z)
        T_base_shoulder_yaw = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.L_base],
            [0, 0, 0, 1]
        ])
        
        # Matriz shoulder_yaw_joint a shoulder_link (rotación en z)
        c1, s1 = cos(theta_1), sin(theta_1)
        T_shoulder_yaw_shoulder = np.array([
            [c1, -s1, 0, 0],
            [s1,  c1, 0, 0],
            [0,   0, 1, 0],
            [0,   0, 0, 1]
        ])
        
        # Matriz shoulder_link a shoulder_pitch_joint (traslación en y y z)
        T_shoulder_shoulder_pitch = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, self.L_shoulder_y],
            [0, 0, 1, self.L_shoulder_z],
            [0, 0, 0, 1]
        ])
        
        # Matriz shoulder_pitch_joint a arm_link (rotación en y)
        c2, s2 = cos(theta_2), sin(theta_2)
        T_shoulder_pitch_arm = np.array([
            [c2,  0, s2, 0],
            [0,   1,  0, 0],
            [-s2, 0, c2, 0],
            [0,   0,  0, 1]
        ])
        
        # Matriz arm_link a elbow_joint (traslación en y y z)
        T_arm_elbow = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, self.L_arm_y],
            [0, 0, 1, self.L_arm_z],
            [0, 0, 0, 1]
        ])
        
        # Matriz elbow_joint a forearm_link (rotación en y)
        c3, s3 = cos(theta_3), sin(theta_3)
        T_elbow_forearm = np.array([
            [c3,  0, s3, 0],
            [0,   1,  0, 0],
            [-s3, 0, c3, 0],
            [0,   0,  0, 1]
        ])
        
        # Matriz forearm_link a wrist_1_joint (traslación en y y z)
        T_forearm_wrist1 = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, self.L_forearm_y],
            [0, 0, 1, self.L_forearm_z],
            [0, 0, 0, 1]
        ])
        
        # Matriz wrist_1_joint a wrist_1_link (rotación en y)
        c4, s4 = cos(theta_4), sin(theta_4)
        T_wrist1_wrist1_link = np.array([
            [c4,  0, s4, 0],
            [0,   1,  0, 0],
            [-s4, 0, c4, 0],
            [0,   0,  0, 1]
        ])
        
        # Matriz wrist_1_link a wrist_2_joint (traslación en x y z)
        T_wrist1_link_wrist2 = np.array([
            [1, 0, 0, self.L_wrist1_x],
            [0, 1, 0, 0],
            [0, 0, 1, self.L_wrist1_z],
            [0, 0, 0, 1]
        ])
        
        # Matriz wrist_2_joint a wrist_2_link (rotación en x)
        c5, s5 = cos(theta_5), sin(theta_5)
        T_wrist2_wrist2_link = np.array([
            [1,   0,    0, 0],
            [0,  c5,  -s5, 0],
            [0,  s5,   c5, 0],
            [0,   0,    0, 1]
        ])
        
        # Matriz wrist_2_link a wrist_3_joint (traslación en z)
        T_wrist2_link_wrist3 = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.L_wrist2_z],
            [0, 0, 0, 1]
        ])
        
        # Matriz wrist_3_joint a wrist_3_link (rotación en z)
        c6, s6 = cos(theta_6), sin(theta_6)
        T_wrist3_wrist3_link = np.array([
            [c6, -s6, 0, 0],
            [s6,  c6, 0, 0],
            [0,   0, 1, 0],
            [0,   0, 0, 1]
        ])
        
        # Matriz wrist_3_link a eef (traslación en z)
        T_wrist3_link_eef = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.L_wrist3_z],
            [0, 0, 0, 1]
        ])

        # Multiplicación de todas las matrices de transformación
        T = T_base_shoulder_yaw @ T_shoulder_yaw_shoulder @ T_shoulder_shoulder_pitch @ \
            T_shoulder_pitch_arm @ T_arm_elbow @ T_elbow_forearm @ T_forearm_wrist1 @ \
            T_wrist1_wrist1_link @ T_wrist1_link_wrist2 @ T_wrist2_wrist2_link @ \
            T_wrist2_link_wrist3 @ T_wrist3_wrist3_link @ T_wrist3_link_eef
        
        # Extraer posición y orientación

        eef_position = T[:3, 3]
        eef_quaternion = quaternion_from_matrix(T)

        return eef_position, eef_quaternion



def main(args=None):
    rclpy.init(args=args)

    fk_publisher = FKPublisher()

    rclpy.spin(fk_publisher)

    fk_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
