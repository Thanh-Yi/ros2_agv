#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class SquarePathSimTime(Node):
    def __init__(self):
        super().__init__('square_path_simtime')

        # Luôn bật sim time
        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, 'odom_raw', self.odom_callback, 10)

        # Thông số chuyển động
        self.side_length = 5.0      # mét
        self.linear_speed = 0.5     # m/s
        self.angular_speed = math.pi/20    # rad/s (~28.6°/s)
        self.turn_angle = math.pi/2 # 90°

        # Thời gian cần cho mỗi hành động
        self.drive_time = self.side_length / self.linear_speed
        self.turn_time = self.turn_angle / self.angular_speed

        # State machine
        self.state = "DRIVE"
        self.start_time = None
        self.sides_done = 0

        self.timer = self.create_timer(0.1, self.control_loop)

        # Lưu yaw từ odom
        self.current_yaw = 0.0

    def odom_callback(self, msg: Odometry):
        # Lấy yaw từ quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_yaw = math.degrees(yaw)  # đổi sang độ

    def control_loop(self):
        now = self.get_clock().now()

        if self.start_time is None:
            self.start_time = now
            return

        elapsed = (now - self.start_time).nanoseconds / 1e9
        cmd = Twist()

        if self.state == "DRIVE":
            if elapsed < self.drive_time:
                cmd.linear.x = self.linear_speed
            else:
                self.state = "TURN"
                self.start_time = now
        elif self.state == "TURN":
            if elapsed < self.turn_time:
                cmd.angular.z = -self.angular_speed  # quay phải
            else:
                self.sides_done += 1
                if self.sides_done >= 4:
                    self.get_logger().info(f"✅ Done: Back to start! Final Yaw = {self.current_yaw:.2f}°")
                    # Publish 0 velocity để dừng hẳn
                    self.pub.publish(Twist())
                    self.timer.cancel()
                    return
                self.state = "DRIVE"
                self.start_time = now

        # Xuất yaw mỗi vòng lặp
        self.get_logger().info(f"Yaw: {self.current_yaw:.2f}° | State: {self.state}")
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SquarePathSimTime()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
