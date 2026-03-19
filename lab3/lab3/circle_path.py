import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class CirclePath(Node):
    def __init__(self):
        super().__init__('circle_path')

        self.declare_parameter("linear_speed", 0.3)
        self.declare_parameter("angular_speed", 0.3)
        self.declare_parameter("rate_hz", 50.0)
   
        self.declare_parameter("time_multiplier", 4.0) 

        self.v = float(self.get_parameter("linear_speed").value)
        self.w = float(self.get_parameter("angular_speed").value)
        self.multiplier = float(self.get_parameter("time_multiplier").value)
        
        # Розрахунок часу з урахуванням множника
        self.duration = abs(2.0 * math.pi / max(abs(self.w), 1e-6)) * self.multiplier
        
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.get_logger().info(f"Target duration: {self.duration:.2f}s (Multiplier: {self.multiplier})")

        self.start_time = None
        self.init_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now()
        
        # Пауза 1 сек перед стартом для стабілізації зв'язку
        if (now - self.init_time).nanoseconds / 1e9 < 1.0:
            return

        if self.start_time is None:
            self.start_time = now
            self.get_logger().info("Moving...")

        elapsed = (now - self.start_time).nanoseconds / 1e9

        if elapsed <= self.duration:
            msg = TwistStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = self.v
            msg.twist.angular.z = self.w
            self.pub.publish(msg)
        else:
            self.stop_robot()

    def stop_robot(self):
        self.pub.publish(TwistStamped())
        self.get_logger().info("Done.")
        self.timer.cancel()
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = CirclePath()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()