#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_pub')
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # Publish initial pose sau 1 giây
        self.timer = self.create_timer(1.0, self.publish_initial_pose)
        self.done = False

    def publish_initial_pose(self):
        if self.done:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"

        # 🚨 Thay đổi vị trí này cho đúng với pose spawn của robot
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0

        # Orientation: 90 độ = yaw = pi/2 → (z=0.707, w=0.707)
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # Covariance nhỏ để AMCL tin tưởng
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        self.publisher_.publish(msg)
        self.get_logger().info("Initial pose published.")
        self.done = True

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin_once(node, timeout_sec=2)  # chỉ chạy 1 lần
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
