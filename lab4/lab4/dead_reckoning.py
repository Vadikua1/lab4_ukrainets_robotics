    import math
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import PoseStamped

    class DeadReckoningNode(Node):
        def __init__(self):
            super().__init__("dead_reckoning")

            # Параметри топіків
            self.declare_parameter("odom_topic", "/odom")
            self.declare_parameter("path_dr_topic", "/path_dr")
            self.declare_parameter("path_gt_topic", "/path_gt")
            self.declare_parameter("frame_id", "odom")
            self.declare_parameter("max_poses", 2000)

            odom_topic = self.get_parameter("odom_topic").value
            path_dr_topic = self.get_parameter("path_dr_topic").value
            path_gt_topic = self.get_parameter("path_gt_topic").value
            self.frame_id = self.get_parameter("frame_id").value
            self.max_poses = int(self.get_parameter("max_poses").value)

            # Стан Dead Reckoning
            self.x, self.y, self.th = 0.0, 0.0, 0.0
            self.last_time = None

            # Повідомлення для шляхів
            self.path_dr_msg = Path()
            self.path_dr_msg.header.frame_id = self.frame_id
            
            self.path_gt_msg = Path()
            self.path_gt_msg.header.frame_id = self.frame_id

            # Підписка на одометрію (дані з коліс/енкодерів симулятора)
            self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
            
            # Publishers для візуалізації в RViz
            self.pub_dr_path = self.create_publisher(Path, path_dr_topic, 10)
            self.pub_gt_path = self.create_publisher(Path, path_gt_topic, 10)

            self.get_logger().info(f"🚀 Dead Reckoning (Encoder-based) started on {odom_topic}")

        def odom_callback(self, msg: Odometry):
            now_msg = rclpy.time.Time.from_msg(msg.header.stamp)

            if self.last_time is None:
                self.last_time = now_msg
                return

            dt = (now_msg - self.last_time).nanoseconds / 1e9
            self.last_time = now_msg

            if dt <= 0: return
            dt = min(dt, 0.1) # Обмежуємо крок часу для стабільності

            v = msg.twist.twist.linear.x
            w = msg.twist.twist.angular.z

            self.th += w * dt
            self.x += v * math.cos(self.th) * dt
            self.y += v * math.sin(self.th) * dt

            # Публікуємо розрахований шлях
            dr_pose = PoseStamped()
            dr_pose.header = msg.header
            dr_pose.pose.position.x = self.x
            dr_pose.pose.position.y = self.y
            dr_pose.pose.orientation.z = math.sin(self.th / 2.0)
            dr_pose.pose.orientation.w = math.cos(self.th / 2.0)

            self.path_dr_msg.poses.append(dr_pose)
            if len(self.path_dr_msg.poses) > self.max_poses:
                self.path_dr_msg.poses.pop(0)
            
            self.path_dr_msg.header.stamp = msg.header.stamp
            self.pub_dr_path.publish(self.path_dr_msg)

            # ---GROUND TRUTH ---
            gt_pose = PoseStamped()
            gt_pose.header = msg.header
            gt_pose.pose = msg.pose.pose

            self.path_gt_msg.poses.append(gt_pose)
            if len(self.path_gt_msg.poses) > self.max_poses:
                self.path_gt_msg.poses.pop(0)

            self.path_gt_msg.header.stamp = msg.header.stamp
            self.pub_gt_path.publish(self.path_gt_msg)

            # Вивід похибки в консоль
            actual_x = msg.pose.pose.position.x
            actual_y = msg.pose.pose.position.y
            error = math.sqrt((self.x - actual_x)**2 + (self.y - actual_y)**2)
            self.get_logger().info(f"DR Error: {error:.3f} m", throttle_duration_sec=1.0)

    def main(args=None):
        rclpy.init(args=args)
        node = DeadReckoningNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    if __name__ == "__main__":
        main()