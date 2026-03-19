import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class DeadReckoningNode(Node):
    def __init__(self):
        super().__init__("dead_reckoning")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("ground_truth_topic", "/odom")
        self.declare_parameter("path_dr_topic", "/path_dr")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("max_poses", 2000)

        cmd_topic = self.get_parameter("cmd_vel_topic").value
        gt_topic = self.get_parameter("ground_truth_topic").value
        path_topic = self.get_parameter("path_dr_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.max_poses = int(self.get_parameter("max_poses").value)

        # Стан робота (Dead Reckoning)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None

        # Стан Ground Truth (для порівняння)
        self.gt_x = 0.0
        self.gt_y = 0.0

        self.create_subscription(TwistStamped, cmd_topic, self.cmd_callback, 10)
        self.create_subscription(Odometry, gt_topic, self.gt_callback, 10)
        self.pub_path = self.create_publisher(Path, path_topic, 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        self.get_logger().info("Dead Reckoning Node initialized.")

    def cmd_callback(self, msg: TwistStamped):
        now = self.get_clock().now()
        
        if self.last_time is None:
            self.last_time = now
            return

        # 1. Рахуємо Delta T
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # 2. Отримуємо швидкості
        v = msg.twist.linear.x
        w = msg.twist.angular.z

        # 3. Оновлюємо позу (Dead Reckoning Formulas)
        # x_new = x + v * cos(theta) * dt
        # y_new = y + v * sin(theta) * dt
        # theta_new = theta + w * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        # 4. Публікуємо шлях для RViz
        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        # Для простоти орієнтацію в PoseStamped можна не заповнювати ідеально, 
        # або перетворити theta в Quaternion
        
        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > self.max_poses:
            self.path_msg.poses.pop(0)

        self.path_msg.header.stamp = now.to_msg()
        self.pub_path.publish(self.path_msg)

        # 5. Рахуємо похибку (Error calculation)
        error = math.sqrt((self.x - self.gt_x)**2 + (self.y - self.gt_y)**2)
        self.get_logger().info(f"DR Pose: ({self.x:.2f}, {self.y:.2f}) | Error: {error:.3f}m", throttle_duration_sec=1.0)

    def gt_callback(self, msg: Odometry):
        # Зберігаємо реальну позицію з Gazebo для порівняння
        self.gt_x = msg.pose.pose.position.x
        self.gt_y = msg.pose.pose.position.y

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()