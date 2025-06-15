#!/usr/bin/env python3
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
from visualization_msgs.msg import Marker
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

class EkfLocalization(Node):

    def __init__(self):
        super().__init__('ekf_localization')
        self.laser_subscriber_ = self.create_subscription(LaserScan, 'scan', self.laser_callback_, 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, 'odom', self.odom_callback_, 10)
        self.marker_publisher_ = self.create_publisher(Marker, 'marker', 10)
        self.publish_timer_ = self.create_timer(0.1, self.publish_timer_callback_)
        self.estimated_pose_publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'ekf_pose', 10)
        self.estimated_pose_ = PoseWithCovarianceStamped()
        
        # Estado del EKF, use estas variables para implementar el filtro
        self.estimated_pose_2d = np.array([0.0, 0.0, 0.0]) # x, y, theta
        self.covariance_2d = np.matrix([[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.05] ])

        self.estimated_pose_.header.frame_id = 'map'
        self.estimated_pose_.pose.pose.position.x = 0.0
        self.estimated_pose_.pose.pose.position.y = 0.0
        self.estimated_pose_.pose.pose.position.z = 0.0
        self.estimated_pose_.pose.pose.orientation.x = 0.0
        self.estimated_pose_.pose.pose.orientation.y = 0.0
        self.estimated_pose_.pose.pose.orientation.z = 0.0
        self.estimated_pose_.pose.pose.orientation.w = 1.0

        map_path = os.path.join(get_package_share_directory('amcl_launch'),'map','map_ekf.txt')
        self.map_ = np.loadtxt(map_path)

        
        
    def publish_timer_callback_(self):
        self.estimated_pose_.pose.pose.position.x = self.estimated_pose_2d[0]
        self.estimated_pose_.pose.pose.position.y = self.estimated_pose_2d[1]
        q = quaternion_from_euler(0, 0, self.estimated_pose_2d[2])
        self.estimated_pose_.pose.pose.orientation.x = q[0]
        self.estimated_pose_.pose.pose.orientation.y = q[1]
        self.estimated_pose_.pose.pose.orientation.z = q[2]
        self.estimated_pose_.pose.pose.orientation.w = q[3]

        self.estimated_pose_.pose.covariance[0] = self.covariance_2d[0,0]
        self.estimated_pose_.pose.covariance[1] = self.covariance_2d[0,1]
        self.estimated_pose_.pose.covariance[6] = self.covariance_2d[1,0]
        self.estimated_pose_.pose.covariance[7] = self.covariance_2d[1,1]
        self.estimated_pose_.pose.covariance[35] = self.covariance_2d[2,2]

        self.estimated_pose_publisher_.publish(self.estimated_pose_)

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

        current_pose = np.array(self.estimated_pose_2d, dtype=np.float64).flatten()
        if current_pose.size != 3:
            current_pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        predicted_measurements = []
        for map_point in self.map_:
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
                map_point = self.map_[best_match]
                z = np.array([point.x, point.y])
                z_pred = predicted_measurements[best_match]
                
                dx = map_point[0] - current_pose[0]
                dy = map_point[1] - current_pose[1]
                
                H = np.array([
                    [-np.cos(current_pose[2]), -np.sin(current_pose[2]), -dx*np.sin(current_pose[2]) + dy*np.cos(current_pose[2])],
                    [np.sin(current_pose[2]), -np.cos(current_pose[2]), -dx*np.cos(current_pose[2]) - dy*np.sin(current_pose[2])]
                ])
                
                R = np.diag([0.1, 0.1])
                y = z - z_pred
                S = H @ self.covariance_2d @ H.T + R
                K = self.covariance_2d @ H.T @ np.linalg.inv(S)
                
                correction = np.squeeze(np.asarray(K @ y))
                if correction.size == 1:
                    correction = np.array([0.0, 0.0, correction.item()])
                elif correction.size == 2:
                    correction = np.append(correction, 0.0)
                
                new_pose = np.array([
                    current_pose[0] + correction[0],
                    current_pose[1] + correction[1],
                    (current_pose[2] + correction[2] + np.pi) % (2 * np.pi) - np.pi
                ])

                self.estimated_pose_2d = new_pose
                self.covariance_2d = (np.eye(3) - K @ H) @ self.covariance_2d
                print("K shape:", K.shape)
                print("y shape:", y.shape)
                print("correction shape:", (K @ y).shape)
                print("estimated_pose shape:", self.estimated_pose_2d.shape)


    def odom_callback_(self, msg: Odometry):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if hasattr(self, 'last_time'):
            dt = current_time - self.last_time
        else:
            dt = 0.1 
        self.last_time = current_time

        theta = self.estimated_pose_2d[2]
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        dx = v * math.cos(theta) * dt
        dy = v * math.sin(theta) * dt
        dtheta = w * dt

        self.estimated_pose_2d += np.array([dx, dy, dtheta])
        self.estimated_pose_2d[2] = (self.estimated_pose_2d[2] + np.pi) % (2 * np.pi) - np.pi  # normalizar ángulo

        F = np.array([
            [1, 0, -v * math.sin(theta) * dt],
            [0, 1,  v * math.cos(theta) * dt],
            [0, 0, 1]
        ])

        Q = np.diag([0.05, 0.05, 0.02])

        self.covariance_2d = F @ self.covariance_2d @ F.T + Q

def main():
    rclpy.init()
    node = EkfLocalization()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()

