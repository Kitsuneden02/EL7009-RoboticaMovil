#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math

class ControlExample(Node):
    def __init__(self):
        super().__init__('control_example')
        self.target_pub = self.create_publisher(PoseStamped, 'target_pose', 10)
        
        # Primer objetivo: (2.0, 0.0) con 0°
        self.send_target(2.0, 0.0, 0.0)
        
        # Segundo objetivo: después de 10 segundos
        self.create_timer(10.0, lambda: self.send_target(0.0, 2.0, math.pi/2))
        
        # Tercer objetivo: después de 20 segundos
        self.create_timer(20.0, lambda: self.send_target(0.0, 0.0, math.pi))

    def send_target(self, x, y, theta):
        """Envía una nueva pose objetivo (x, y, theta)"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Posición
        msg.pose.position = Point(x=x, y=y, z=0.0)
        
        # Orientación convertida a cuaternión
        msg.pose.orientation = self.quaternion_from_euler(0.0, 0.0, theta)
        
        self.target_pub.publish(msg)
        self.get_logger().info(
            f"🎯 Objetivo enviado: x={x:.2f}, y={y:.2f}, θ={math.degrees(theta):.1f}°"
        )

    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convierte ángulos de Euler (en radianes) a cuaternión"""
        q = Quaternion()
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

def main(args=None):
    rclpy.init(args=args)
    example = ControlExample()
    
    try:
        rclpy.spin(example)
    except KeyboardInterrupt:
        pass
    finally:
        example.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
