#!/usr/bin/env python
import rclpy
from rclpy.node import Node

import rosidl_parser
import threading
from std_msgs import msg
from std_msgs.msg import   Float64


class ExampleControllerMinimal(Node):

    def __init__(self):
        super().__init__('example_controller')
        self.publisher_left_ = self.create_publisher(Float64, 'left_wheel_cmd_vel', 10)
        self.publisher_right_ = self.create_publisher(Float64, 'right_wheel_cmd_vel', 10)
        self._loop_rate = self.create_rate(20.0, self.get_clock())

def main(args=None):
    rclpy.init(args=args)

    # Spin in a separate thread so that the clock is updated
    minimal_publisher = ExampleControllerMinimal()
    thread = threading.Thread(target=rclpy.spin, args=(minimal_publisher,))
    thread.start()

    # Publish speeds for 4 seconds
    minimal_publisher.get_logger().info("Publishing speeds for 4 seconds")
    for i in range(40): 
        msg = Float64()
        msg.data = 1.0
        minimal_publisher.publisher_left_.publish(msg)
        msg.data = 1.0
        minimal_publisher.publisher_right_.publish(msg)
        minimal_publisher.get_logger().info("publishing speeds")
        minimal_publisher._loop_rate.sleep()

    # Stop the robot
    minimal_publisher.get_logger().info("Stopping the robot")
    msg = Float64()
    msg.data = 0.0
    minimal_publisher.publisher_left_.publish(msg)
    minimal_publisher.publisher_right_.publish(msg)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

