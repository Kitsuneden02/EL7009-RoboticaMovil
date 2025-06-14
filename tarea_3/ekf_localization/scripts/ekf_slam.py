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


        estimated_state_shape = self.estimated_state_.shape
        
        estimated_covariance_shape = self.estimated_covariance_.shape
        assert estimated_state_shape[0] == estimated_covariance_shape[0]
        assert estimated_covariance_shape[0] == estimated_covariance_shape[1]


        map_marker_array = MarkerArray()

        
        num_map_points = round((len( self.estimated_state_) -3 )/2 )
        # print(f"num_map_points: {num_map_points}")
        for i in range(num_map_points):
            map_point = self.estimated_state_[3+i*2:3+i*2+2]
            # print(f"i: {i}")
            # print(f"map_point: {map_point}")
            covariance = self.estimated_covariance_[3+i*2:3+i*2+2,3+i*2:3+i*2+2]
            
            # print(f"covariance: {covariance}")
            eigenvalues, eigenvectors = np.linalg.eig(covariance)

            direction = eigenvectors[:,0]
            # print(f"direction: {direction}")
            direction /= np.linalg.norm(direction)

            # print(f"direction: {direction}")
            angle = np.arctan2(direction[1,0], direction[0,0])
            q = quaternion_from_euler(0, 0, angle)

            map_marker = Marker()
            map_marker.header.frame_id = 'map'
            map_marker.ns = 'map'
            map_marker.id = i
            map_marker.type = map_marker.SPHERE
            map_marker.action = map_marker.ADD
            map_marker.pose.position.x = map_point[0]
            map_marker.pose.position.y = map_point[1]
            map_marker.pose.position.z = 0.0
            map_marker.pose.orientation.x = q[0]
            map_marker.pose.orientation.y = q[1]
            map_marker.pose.orientation.z = q[2]
            map_marker.pose.orientation.w = q[3]
            map_marker.scale.x = 0.2 #np.sqrt(eigenvalues[0])
            map_marker.scale.y = 0.2 #np.sqrt(eigenvalues[1])
            map_marker.scale.z = 0.1
            map_marker.color.a = 1.0
            hue = i / float(num_map_points) if num_map_points > 0 else 0
            map_marker.color.r = (1.0 - hue)
            map_marker.color.g = hue
            map_marker.color.b = 0.5

            # Markers para elipses de covarianza
            ellipse_marker = Marker()
            ellipse_marker.header.frame_id = 'map'
            ellipse_marker.ns = 'covariance_ellipses'
            ellipse_marker.id = i*2 + 1 
            ellipse_marker.type = Marker.SPHERE  
            ellipse_marker.action = Marker.ADD
            ellipse_marker.pose.position.x = map_point[0]
            ellipse_marker.pose.position.y = map_point[1]
            ellipse_marker.pose.position.z = 0.0
            
            eigenvalues, eigenvectors = np.linalg.eig(covariance)
            angle = np.arctan2(eigenvectors[1,0], eigenvectors[0,0])
            q = quaternion_from_euler(0, 0, angle)
            ellipse_marker.pose.orientation.x = q[0]
            ellipse_marker.pose.orientation.y = q[1]
            ellipse_marker.pose.orientation.z = q[2]
            ellipse_marker.pose.orientation.w = q[3]
            
            ellipse_marker.scale.x = 1.0 * np.sqrt(eigenvalues[0])
            ellipse_marker.scale.y = 1.0 * np.sqrt(eigenvalues[1])
            ellipse_marker.scale.z = 0.01  
            
            ellipse_marker.color.a = 0.3 
            ellipse_marker.color.r = 0.0
            ellipse_marker.color.g = 0.0
            ellipse_marker.color.b = 1.0 

            map_marker_array.markers.append(map_marker)
            map_marker_array.markers.append(ellipse_marker)
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

        current_pose = np.array(self.estimated_state_[:3], dtype=np.float64).flatten()
        if current_pose.size != 3:
            current_pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        num_map_points = round((len(self.estimated_state_) - 3) / 2)
        predicted_measurements = []
        
        for i in range(num_map_points):
            map_point = self.estimated_state_[3+i*2:3+i*2+2]
            dx = map_point[0] - current_pose[0]
            dy = map_point[1] - current_pose[1]
            predicted = np.array([
                dx * np.cos(current_pose[2]) + dy * np.sin(current_pose[2]),
                -dx * np.sin(current_pose[2]) + dy * np.cos(current_pose[2])
            ])
            predicted_measurements.append(predicted)

        for point in marker_points.points:
            min_dist = float('inf')
            best_match = None
            
            for i, pred in enumerate(predicted_measurements):
                dist = np.linalg.norm(np.array([point.x, point.y]) - pred)
                if dist < min_dist and dist < 0.5:
                    min_dist = dist
                    best_match = i

            if best_match is not None:
                map_point = self.estimated_state_[3+best_match*2:3+best_match*2+2]
                z = np.array([point.x, point.y])
                z_pred = predicted_measurements[best_match]
                
                dx = map_point[0] - current_pose[0]
                dy = map_point[1] - current_pose[1]
                
                state_size = len(self.estimated_state_)
                H = np.zeros((2, state_size))
                
                H[:, :3] = np.array([
                    [-np.cos(current_pose[2]), -np.sin(current_pose[2]), 
                    -dx*np.sin(current_pose[2]) + dy*np.cos(current_pose[2])],
                    [np.sin(current_pose[2]), -np.cos(current_pose[2]), 
                    -dx*np.cos(current_pose[2]) - dy*np.sin(current_pose[2])]
                ])
                
                landmark_start = 3 + best_match * 2
                H[:, landmark_start:landmark_start+2] = np.array([
                    [np.cos(current_pose[2]), np.sin(current_pose[2])],
                    [-np.sin(current_pose[2]), np.cos(current_pose[2])]
                ])
                
                R = np.diag([0.4, 0.4])
                y = z - z_pred
                S = H @ self.estimated_covariance_ @ H.T + R
                K = self.estimated_covariance_ @ H.T @ np.linalg.inv(S)
                
                # Actualización
                correction = np.squeeze(np.asarray(K @ y))
                
                # Asegurar que correction tenga la dimensión correcta
                if correction.ndim == 0:
                    correction = np.array([correction])
                if len(correction) != state_size:
                    full_correction = np.zeros(state_size)
                    full_correction[:len(correction)] = correction
                    correction = full_correction
                
                self.estimated_state_ = self.estimated_state_ + correction
                self.estimated_covariance_ = (np.eye(state_size) - K @ H) @ self.estimated_covariance_
                
                # Normalizar ángulo
                self.estimated_state_[2] = (self.estimated_state_[2] + np.pi) % (2 * np.pi) - np.pi
                
            else:
                global_x = current_pose[0] + point.x * np.cos(current_pose[2]) - point.y * np.sin(current_pose[2])
                global_y = current_pose[1] + point.x * np.sin(current_pose[2]) + point.y * np.cos(current_pose[2])
                
                new_landmark = np.array([global_x, global_y])
                self.estimated_state_ = np.concatenate([self.estimated_state_, new_landmark])
                
                old_size = self.estimated_covariance_.shape[0]
                new_size = old_size + 2
                
                new_covariance = np.zeros((new_size, new_size))
                new_covariance[:old_size, :old_size] = self.estimated_covariance_
                
                J = np.zeros((2, 3))
                J[0, 0] = 1.0  # dx/dx_robot
                J[1, 1] = 1.0  # dy/dy_robot
                J[0, 2] = -point.x * np.sin(current_pose[2]) - point.y * np.cos(current_pose[2])  # dx/dtheta
                J[1, 2] = point.x * np.cos(current_pose[2]) - point.y * np.sin(current_pose[2])   # dy/dtheta
                
                R_landmark = np.diag([0.4, 0.4])
                landmark_cov = J @ self.estimated_covariance_[:3, :3] @ J.T + R_landmark
                
                cross_cov = J @ self.estimated_covariance_[:3, :]
                
                new_covariance[old_size:, old_size:] = landmark_cov
                new_covariance[old_size:, :old_size] = cross_cov
                new_covariance[:old_size, old_size:] = cross_cov.T
                
                self.estimated_covariance_ = np.matrix(new_covariance)

    def odom_callback_(self, msg: Odometry):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if hasattr(self, 'last_time'):
            dt = current_time - self.last_time
        else:
            dt = 0.1
        self.last_time = current_time

        theta = self.estimated_state_[2]
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        dx = v * math.cos(theta) * dt
        dy = v * math.sin(theta) * dt
        dtheta = w * dt

        self.estimated_state_[0] += dx
        self.estimated_state_[1] += dy
        self.estimated_state_[2] += dtheta
        self.estimated_state_[2] = (self.estimated_state_[2] + np.pi) % (2 * np.pi) - np.pi

        state_size = len(self.estimated_state_)
        F = np.eye(state_size)
        F[0, 2] = -v * math.sin(theta) * dt
        F[1, 2] = v * math.cos(theta) * dt

        Q = np.zeros((state_size, state_size))
        Q[:3, :3] = np.diag([0.045, 0.045, 0.02])

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

