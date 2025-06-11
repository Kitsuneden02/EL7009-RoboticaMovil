#!/usr/bin/env python
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
import tf
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
            map_marker.type = map_marker.CYLINDER
            map_marker.action = map_marker.ADD
            map_marker.pose.position.x = map_point[0]
            map_marker.pose.position.y = map_point[1]
            map_marker.pose.position.z = 0.0
            map_marker.pose.orientation.x = q[0]
            map_marker.pose.orientation.y = q[1]
            map_marker.pose.orientation.z = q[2]
            map_marker.pose.orientation.w = q[3]
            map_marker.scale.x = np.sqrt(eigenvalues[0])
            map_marker.scale.y = np.sqrt(eigenvalues[1])
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

        # Aqui use los puntos detectados ( en marker_points.points ) para realizar la correccion usando EKF

        num_map_points = round((len( self.estimated_state_) -3 )/2 )
        # print(f"num_map_points: {num_map_points}")
        predicted_measurements = np.zeros((num_map_points, 2))
        for i in range(num_map_points):
            map_point = self.estimated_state_[3+i*2:3+i*2+2]
            # print(f"i: {i}")
            # print(f"map_point: {map_point}")

            ## prediga la posicion del punto vista desde la pose del robot
            ## YOUR CODE HERE:
            prediction = np.array([0.0, 0.0])
            # print(f"prediction: {prediction}")
            predicted_measurements[i] = prediction
            pass

        for point in marker_points.points:
            ## busca el punto mas cercano en la lista de predicciones
            ## YOUR CODE HERE: 


            ## actualiza la posicion estimada con la medicion:
            ## YOUR CODE HERE:
            pass


    def odom_callback_(self, msg: Odometry):
        # use odometry to do the prediction step of the ekf
        # YOUR CODE HERE
        #   self.estimated_pose_2d = XXXX
        #   self.covariance_2d =  self.covariance_2d +  XXXX
        pass










def main():
    rclpy.init()
    node = EkfSlam()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()

