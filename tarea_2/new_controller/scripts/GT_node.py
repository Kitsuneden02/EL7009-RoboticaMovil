import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration

class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_publisher')
        
        # Buffer y listener para acceder a las TFs
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publicador para el tópico personalizado
        self.gt_pose_pub = self.create_publisher(PoseStamped, '/ground_truth/pose', 10)
        
        # Timer para publicar la pose periódicamente
        self.timer = self.create_timer(0.1, self.publish_ground_truth)  # 10 Hz

    def publish_ground_truth(self):
        try:
            # Obtener la transformación world → base_link
            transform = self.tf_buffer.lookup_transform(
                'world',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            
            # Crear mensaje PoseStamped
            gt_pose = PoseStamped()
            gt_pose.header.stamp = self.get_clock().now().to_msg()
            gt_pose.header.frame_id = 'world'
            gt_pose.pose.position.x = transform.transform.translation.x
            gt_pose.pose.position.y = transform.transform.translation.y
            gt_pose.pose.position.z = transform.transform.translation.z
            gt_pose.pose.orientation = transform.transform.rotation
            
            # Publicar
            self.gt_pose_pub.publish(gt_pose)
            self.get_logger().info(f"Publicada pose: {gt_pose.pose.position}")
            
        except Exception as e:
            self.get_logger().warn(f"No se pudo obtener la transformación: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()