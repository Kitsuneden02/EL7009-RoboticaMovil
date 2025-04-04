#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class OdomPublisher(Node):

    def __init__(self):
        super().__init__('example_controller')
        # Subscriber a las velocidades de las ruedas
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        # TF
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.update_odometry) # Timer para la TF
        # Odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Parámetros
        self.r = 0.05 # Radio de las ruedas
        self.l = 0.32 # Longitud del eje de las ruedas
        
        # Estado Inicial
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Velocidades de las ruedas
        self.left_vel = 0.0
        self.right_vel = 0.0

    def joint_state_callback(self, msg):
        left_idx = msg.name.index('left_wheel_joint') # Obtenido al hacer echo al tópico /joint_states
        right_idx = msg.name.index('right_wheel_joint') 
        self.left_wheel_vel = msg.velocity[left_idx]
        self.right_wheel_vel = msg.velocity[right_idx]
    
    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0:
            return
        
        # Cinemática directa
        v_left = self.left_wheel_vel * self.r
        v_right = self.right_wheel_vel * self.r
        
        # Vel. lineal y angular
        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.l
        
        # Actualizar la posición y orientación
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = omega * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalizar el ángulo entre -pi y pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publicar la transformación odom->base_link
        self.publish_odom_tf()
        
        # Opcional: publicar mensaje de odometría
        self.publish_odom_msg(current_time)

    def publish_odom_tf(self):
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = self.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

    def publish_odom_msg(self, current_time):
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        q = self.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        self.odom_pub.publish(odom)

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        ]
        
        return q

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()