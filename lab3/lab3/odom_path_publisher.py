import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class OdomPathPublisher(Node):
    def __init__(self):
        super().__init__("odom_path_publisher")

        # ПАРАМЕТРИ
        self.odom_topic = "/model/vehicle_purple/odometry"
        self.path_topic = "/path"
        self.frame_id = "odom"  
        self.child_frame = "base_link"

        self.sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.pub = self.create_publisher(Path, self.path_topic, 10)
        
        # Ця штука сама створює зв'язок координат, щоб RViz не тупив
        self.tf_broadcaster = TransformBroadcaster(self)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        self.get_logger().info(f"Path Publisher started. Topic: {self.path_topic}")

    def on_odom(self, msg: Odometry):
        # 1. Транслюємо TF вручну 
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # 2. Додаємо точку в шлях
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose = msg.pose.pose

        self.path_msg.header.stamp = pose.header.stamp
        self.path_msg.poses.append(pose)

        if len(self.path_msg.poses) > 2000:
            self.path_msg.poses.pop(0)

        self.pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()