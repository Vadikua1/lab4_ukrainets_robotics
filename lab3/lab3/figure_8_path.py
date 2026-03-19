import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class FigureEightPath(Node):
    def __init__(self):
        super().__init__('figure_eight_path')

        # Параметри
        self.declare_parameter("linear_speed", 0.3)
        self.declare_parameter("angular_speed", 0.3)
        self.declare_parameter("time_multiplier", 4.0) 
        self.declare_parameter("rate_hz", 50.0)

        self.v = float(self.get_parameter("linear_speed").value)
        self.w = float(self.get_parameter("angular_speed").value)
        self.multiplier = float(self.get_parameter("time_multiplier").value)
        
        self.circle_duration = abs(2.0 * math.pi / max(abs(self.w), 1e-6)) * self.multiplier
        
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        
        self.state = 1
        self.start_time = None
        self.init_time = self.get_clock().now()
        
        self.get_logger().info(f"Starting Figure-8. Circle duration: {self.circle_duration:.2f}s")
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now()
        
        # 1. Початкова затримка для стабілізації зв'язку
        if (now - self.init_time).nanoseconds / 1e9 < 1.0:
            return

        if self.start_time is None:
            self.start_time = now

        elapsed = (now - self.start_time).nanoseconds / 1e9
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'

        if self.state == 1:
            # Перше коло
            if elapsed <= self.circle_duration:
                msg.twist.linear.x = self.v
                msg.twist.angular.z = self.w
                self.pub.publish(msg)
            else:
                self.get_logger().info("First circle complete. Switching direction...")
                self.state = 2
                self.start_time = now # Скидаємо час для другого кола
        
        elif self.state == 2:
            # Друге коло
            if elapsed <= self.circle_duration:
                msg.twist.linear.x = self.v
                msg.twist.angular.z = -self.w
                self.pub.publish(msg)
            else:
                self.stop_robot()

    def stop_robot(self):
        self.pub.publish(TwistStamped())
        self.get_logger().info("Figure-8 complete! Stopping.")
        self.timer.cancel()
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = FigureEightPath()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()