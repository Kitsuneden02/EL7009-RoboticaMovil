#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

#import tf_transformations

import numpy as np


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
        timer_period = 0.02  # seconds
        self._fk_timer = self.create_timer(timer_period, self._timer_callback)

        # Auxiliary variables
        self._joint_states = None


    def _joint_states_callback(self, msg):
        # TODO: get the joint states position accessing the 'msg'
        pass

    def _timer_callback(self):

        if self._joint_states is None:
            return 
        
        # Get the eef position (x,y,z) and orientation (rx, ry, rz, rw)

        eef_position, eef_quaternion = self._compute_fk(self._joint_states)
        
        # TODO: construct a message for a pose stamped and publish it
        eef_pose = PoseStamped()

        # TODO: your code goes here

        #self._fk_publisher.publish(eef_pose)


    def _compute_fk(self, joint_states):
        
        # TODO: program a method that receives a vector of six variables (joint_states)
        # an by computing the forward kinematics of the manipulator, returns the end effector
        # position (x,y,z) and orientation as a quaternion (rx, ry, rz, rw)

        eef_position = None
        eef_quaternion = None

        return eef_position, eef_quaternion



def main(args=None):
    rclpy.init(args=args)

    fk_publisher = FKPublisher()

    rclpy.spin(fk_publisher)

    fk_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
