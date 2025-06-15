#!/usr/bin/env python3
from typing import assert_never
import rclpy
import numpy as np
from rclpy.node import Node
import math

import rosidl_parser
import threading
import std_msgs.msg
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from scipy.signal import convolve
#import tf
from tf_transformations import quaternion_from_euler, quaternion_multiply
import os

from ament_index_python.packages import get_package_share_directory

import numpy as np

def conv_circ( signal, ker ):
    '''
        signal: real 1D array
        ker: real 1D array
        signal and ker must have same shape
    '''
    return np.real(np.fft.ifft( np.fft.fft(signal)*np.fft.fft(ker) ))

class EkfSlam(Node):

    def __init__(self):
        super().__init__('ekf_slam')
        self.laser_subscriber_ = self.create_subscription(LaserScan, 'scan', self.laser_callback_, 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, 'odom', self.odom_callback_, 10)
        self.marker_publisher_ = self.create_publisher(Marker, 'marker', 10)
        self.map_marker_publisher = self.create_publisher(MarkerArray, 'map_marker', 10)
        self.publish_timer_ = self.create_timer(0.1, self.publish_timer_callback_)
        self.estimated_pose_publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'ekf_pose', 10)
        self.estimated_pose_ = PoseWithCovarianceStamped()
        
        # Estado del EKF, use estas variables para implementar el filtro
        # self.estimated_state_ = np.array([0.0, 0.0, 0.0]) # x, y, theta
        # self.estimated_covariance_ = np.matrix([[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.05] ])

        self.estimated_state_ = np.array([0.0, 0.0, 0.0 ]).transpose() # x, y, theta 
        self.estimated_covariance_ = np.matrix([[0.1, 0.0, 0.0 ], 
                                                [0.0, 0.1, 0.0],
                                                [0.0, 0.0, 0.05]])

        self.estimated_pose_.header.frame_id = 'map'
        self.estimated_pose_.pose.pose.position.x = 0.0
        self.estimated_pose_.pose.pose.position.y = 0.0
        self.estimated_pose_.pose.pose.position.z = 0.0
        self.estimated_pose_.pose.pose.orientation.x = 0.0
        self.estimated_pose_.pose.pose.orientation.y = 0.0
        self.estimated_pose_.pose.pose.orientation.z = 0.0
        self.estimated_pose_.pose.pose.orientation.w = 1.0


        
        
    def publish_timer_callback_(self):
        self.estimated_pose_.pose.pose.position.x = self.estimated_state_[0]
        self.estimated_pose_.pose.pose.position.y = self.estimated_state_[1]
        q = quaternion_from_euler(0, 0, self.estimated_state_[2])
        self.estimated_pose_.pose.pose.orientation.x = q[0]
        self.estimated_pose_.pose.pose.orientation.y = q[1]
        self.estimated_pose_.pose.pose.orientation.z = q[2]
        self.estimated_pose_.pose.pose.orientation.w = q[3]

        self.estimated_pose_.pose.covariance[0] = self.estimated_covariance_[0,0]
        self.estimated_pose_.pose.covariance[1] = self.estimated_covariance_[0,1]
        self.estimated_pose_.pose.covariance[6] = self.estimated_covariance_[1,0]
        self.estimated_pose_.pose.covariance[7] = self.estimated_covariance_[1,1]
        self.estimated_pose_.pose.covariance[35] = self.estimated_covariance_[2,2]

        self.estimated_pose_publisher_.publish(self.estimated_pose_)

        map_marker_array = MarkerArray()
        num_map_points = round((len(self.estimated_state_) - 3) / 2)
        
        for i in range(num_map_points):
            map_point = self.estimated_state_[3+i*2:3+i*2+2]
            covariance = self.estimated_covariance_[3+i*2:3+i*2+2, 3+i*2:3+i*2+2]
            
            eigenvalues, eigenvectors = np.linalg.eig(covariance)
            
            direction = eigenvectors[:,0].flatten()
            direction /= np.linalg.norm(direction)
            
            angle = np.arctan2(direction[1], direction[0])
            q = quaternion_from_euler(0, 0, angle)

            map_marker = Marker()
            map_marker.header.frame_id = 'map'
            map_marker.ns = 'map'
            map_marker.id = i
            map_marker.type = Marker.CYLINDER
            map_marker.action = Marker.ADD
            map_marker.pose.position.x = map_point[0]
            map_marker.pose.position.y = map_point[1]
            map_marker.pose.position.z = 0.0
            map_marker.pose.orientation.x = q[0]
            map_marker.pose.orientation.y = q[1]
            map_marker.pose.orientation.z = q[2]
            map_marker.pose.orientation.w = q[3]
            map_marker.scale.x = np.sqrt(eigenvalues[0]) * 2  # Multiplicado por 2 para mejor visualización
            map_marker.scale.y = np.sqrt(eigenvalues[1]) * 2
            map_marker.scale.z = 0.1
            map_marker.color.a = 1.0
            map_marker.color.r = 1.0
            map_marker.color.g = 0.0
            map_marker.color.b = 0.0

            map_marker_array.markers.append(map_marker)
        
        self.map_marker_publisher.publish(map_marker_array)

            
                



    def detect_corners(self, msg: LaserScan):

        marker_points = Marker()
        marker_points.header = msg.header
        marker_points.ns = 'detected_points'
        marker_points.id = 0
        marker_points.type = marker_points.SPHERE_LIST
        marker_points.action = marker_points.ADD
        marker_points.pose.position.x = 0.0
        marker_points.pose.position.y = 0.0
        marker_points.pose.position.z = 0.0
        marker_points.pose.orientation.x = 0.0
        marker_points.pose.orientation.y = 0.0
        marker_points.pose.orientation.z = 0.0
        marker_points.pose.orientation.w = 1.0
        marker_points.scale.x = 0.1
        marker_points.scale.y = 0.1
        marker_points.scale.z = 0.1
        marker_points.color.a = 1.0
        marker_points.color.r = 1.0
        marker_points.color.g = 0.0
        marker_points.color.b = 0.0

        
        # detect corners with infinity 
        i = 0
        indices = []
        while i < len(msg.ranges):
            if math.isinf( msg.ranges[i] ) and not math.isinf( msg.ranges[(i+1)%len(msg.ranges)] )  and  msg.ranges[(i+1)%len(msg.ranges)] > 0.0 and msg.ranges[(i+1)%len(msg.ranges)]< msg.range_max*0.9 :
                point = Point()
                point.x = msg.ranges[(i+1)%len(msg.ranges)]*np.cos(msg.angle_min + msg.angle_increment*(i+1)%len(msg.ranges))
                point.y = msg.ranges[(i+1)%len(msg.ranges)]*np.sin(msg.angle_min + msg.angle_increment*(i+1)%len(msg.ranges))
                point.z = 0.0
                indices.append(i+1)
                marker_points.points.append(point)
                i += 1
            elif not math.isinf( msg.ranges[i] ) and msg.ranges[i] > 0.0  and msg.ranges[i] <msg.range_max*0.9 and math.isinf( msg.ranges[(i+1)%len(msg.ranges)] ):
                point = Point()
                point.x = msg.ranges[i]*np.cos(msg.angle_min + msg.angle_increment*i)
                point.y = msg.ranges[i]*np.sin(msg.angle_min + msg.angle_increment*i)
                point.z = 0.0
                marker_points.points.append(point)
                indices.append(i)
            i += 1

        # detect corners
        # kernel = np.pad(np.array([1,1,1,1,1,-10,1,1,1,1,1]), (0, len(msg.ranges)-11), 'constant', constant_values=(0.0,))
        # diffrange  = conv_circ(msg.ranges, kernel)
        kernel = np.array([1.0,1.0,1.0,-6.0,1.0,1.0,1.0])
        diffrange = convolve(msg.ranges, kernel, mode='valid')
        curvature = diffrange * diffrange / (np.array(msg.ranges[3:-3]) **2 )

        # sort by curvature
        sorted_indices = np.argsort(curvature)[::-1]


        curvature_num_points = 0

        for index in sorted_indices:
            if curvature_num_points > 20:
                break
            if curvature[index] < 0.1:
                break
            if  math.isfinite(curvature[index] ) and math.isfinite(msg.ranges[(index+3)%len(msg.ranges)] ):
                is_close = False
                for idx in indices:
                    if abs(index - idx) < 4:
                        is_close = True
                        break
                if not is_close:
                    indices.append(index)
                    point = Point()
                    point.x = msg.ranges[(index+3)%len(msg.ranges)]*np.cos(msg.angle_min + msg.angle_increment*((index+3)%len(msg.ranges)))
                    point.y = msg.ranges[(index+3)%len(msg.ranges)]*np.sin(msg.angle_min + msg.angle_increment*((index+3)%len(msg.ranges)))
                    point.z = 0.3* (curvature_num_points+1)
                    marker_points.points.append(point)
                    curvature_num_points += 1
        return marker_points

    def laser_callback_(self, msg: LaserScan):
        marker_points = self.detect_corners(msg)
        self.marker_publisher_.publish(marker_points)

        current_pose = self.estimated_state_[0:3]
        num_map_points = round((len(self.estimated_state_) - 3) / 2)
        
        # Predecir mediciones para landmarks existentes (igual que en ekf_localization.py)
        predicted_measurements = []
        for i in range(num_map_points):
            map_point = self.estimated_state_[3+i*2:3+i*2+2]
            dx = map_point[0] - current_pose[0]
            dy = map_point[1] - current_pose[1]
            
            # Mismo modelo de observación que en ekf_localization.py
            z_pred = np.array([
                dx * np.cos(current_pose[2]) + dy * np.sin(current_pose[2]),
                -dx * np.sin(current_pose[2]) + dy * np.cos(current_pose[2])
            ])
            predicted_measurements.append(z_pred)

        # Umbral para asociación de datos (puedes usar el mismo que en ekf_localization.py)
        association_threshold = 0.5
        
        for point in marker_points.points:
            z = np.array([point.x, point.y])
            
            # Buscar el landmark más cercano (igual que antes)
            min_dist = float('inf')
            best_match = None
            
            for i, pred in enumerate(predicted_measurements):
                dist = np.linalg.norm(z - pred)
                if dist < min_dist and dist < association_threshold:
                    min_dist = dist
                    best_match = i

            if best_match is not None:
                # Corrección para un landmark existente (similar a ekf_localization.py pero extendido)
                map_point_idx = best_match
                
                # Jacobiano de la función de medición (similar pero extendido para SLAM)
                H = np.zeros((2, len(self.estimated_state_)))
                # Partes del Jacobiano correspondientes a la pose del robot (igual que antes)
                H[0, 0] = -np.cos(current_pose[2])
                H[0, 1] = -np.sin(current_pose[2])
                H[0, 2] = -dx * np.sin(current_pose[2]) + dy * np.cos(current_pose[2])
                H[1, 0] = np.sin(current_pose[2])
                H[1, 1] = -np.cos(current_pose[2])
                H[1, 2] = -dx * np.cos(current_pose[2]) - dy * np.sin(current_pose[2])
                
                # Partes del Jacobiano correspondientes al landmark
                H[0, 3+map_point_idx*2] = np.cos(current_pose[2])
                H[0, 3+map_point_idx*2+1] = np.sin(current_pose[2])
                H[1, 3+map_point_idx*2] = -np.sin(current_pose[2])
                H[1, 3+map_point_idx*2+1] = np.cos(current_pose[2])
                
                # Misma covarianza del sensor que en ekf_localization.py
                R = np.diag([0.1, 0.1])
                
                # Resto de la corrección igual que antes
                y = z - predicted_measurements[best_match]
                S = H @ self.estimated_covariance_ @ H.T + R
                K = self.estimated_covariance_ @ H.T @ np.linalg.inv(S)
                
                self.estimated_state_ = self.estimated_state_ + K @ y
                self.estimated_covariance_ = (np.eye(len(self.estimated_state_)) - K @ H) @ self.estimated_covariance_
                
            else:
                # Agregar nuevo landmark (esto es nuevo para SLAM)
                # Transformar la medición a coordenadas globales (igual que antes)
                global_x = current_pose[0] + z[0] * np.cos(current_pose[2]) - z[1] * np.sin(current_pose[2])
                global_y = current_pose[1] + z[0] * np.sin(current_pose[2]) + z[1] * np.cos(current_pose[2])
                
                # Extender el vector de estado
                new_state = np.zeros(len(self.estimated_state_) + 2)
                new_state[0:len(self.estimated_state_)] = self.estimated_state_
                new_state[len(self.estimated_state_):] = [global_x, global_y]
                self.estimated_state_ = new_state
                
                # Extender la matriz de covarianza
                n = len(self.estimated_state_)
                new_cov = np.zeros((n, n))
                new_cov[0:n-2, 0:n-2] = self.estimated_covariance_
                
                # Inicialización de covarianza para el nuevo landmark
                # Usamos el mismo principio que en ekf_localization.py pero extendido
                G = np.zeros((2, n))
                G[0, 0] = 1
                G[0, 2] = -z[0] * np.sin(current_pose[2]) - z[1] * np.cos(current_pose[2])
                G[1, 1] = 1
                G[1, 2] = z[0] * np.cos(current_pose[2]) - z[1] * np.sin(current_pose[2])
                G[0, n-2] = np.cos(current_pose[2])
                G[0, n-1] = -np.sin(current_pose[2])
                G[1, n-2] = np.sin(current_pose[2])
                G[1, n-1] = np.cos(current_pose[2])
                
                # Covarianza inicial del landmark (puedes usar valores similares a los de ekf_localization.py)
                R_landmark = np.diag([0.5, 0.5])
                new_cov[n-2:n, n-2:n] = G @ new_cov @ G.T + R_landmark
                
                self.estimated_covariance_ = new_cov


    def odom_callback_(self, msg: Odometry):
        # Toma delta t desde el tiempo entre callbacks
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if hasattr(self, 'last_time'):
            dt = current_time - self.last_time
        else:
            dt = 0.1  # valor por defecto
        self.last_time = current_time

        theta = self.estimated_state_[2]
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        # Predicción del nuevo estado (solo para la pose del robot)
        dx = v * math.cos(theta) * dt
        dy = v * math.sin(theta) * dt
        dtheta = w * dt

        # Actualizar solo la parte de la pose del robot (primeros 3 elementos)
        self.estimated_state_[0] += dx
        self.estimated_state_[1] += dy
        self.estimated_state_[2] = (self.estimated_state_[2] + dtheta + np.pi) % (2 * np.pi) - np.pi

        # Matriz Jacobiana del movimiento (solo para la parte de la pose)
        F = np.eye(len(self.estimated_state_))  # Matriz identidad del tamaño del estado completo
        F[0, 2] = -v * math.sin(theta) * dt
        F[1, 2] = v * math.cos(theta) * dt

        # Ruido del modelo de movimiento (solo para la parte de la pose)
        Q = np.zeros((len(self.estimated_state_), len(self.estimated_state_)))
        Q[0:3, 0:3] = np.diag([0.05, 0.05, 0.02])  # Mismos valores que en ekf_localization.py

        # Actualizar la covarianza
        self.estimated_covariance_ = F @ self.estimated_covariance_ @ F.T + Q










def main():
    rclpy.init()
    node = EkfSlam()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()

