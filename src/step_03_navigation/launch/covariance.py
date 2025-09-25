#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class OdomCovStamper(Node):
    def __init__(self):
        super().__init__('odom_cov_stamper')
        self.sub = self.create_subscription(Odometry, 'odom_raw', self.cb, 10)
        self.pub = self.create_publisher(Odometry, 'odom_raw_cov', 10)

        # Covariance cho pose (không dùng => để lớn)
        self.pose_cov = [0] * 36
        # Covariance cho twist
        self.twist_cov = [0] * 36
        self.twist_cov[0]  = 0.01   # vx
        self.twist_cov[1]  = 0.01   # vy
        self.twist_cov[35] = 0.01  # wz sẽ lấy từ IMU, không dùng odom

    def cb(self, msg: Odometry):
        msg.pose.covariance = self.pose_cov
        msg.twist.covariance = self.twist_cov
        self.pub.publish(msg)


class ImuCovStamper(Node):
    def __init__(self):
        super().__init__('imu_cov_stamper')
        self.sub = self.create_subscription(Imu, 'imu', self.cb, 10)
        self.pub = self.create_publisher(Imu, 'imu_cov', 10)

        # orientation (không dùng)
        self.orient_cov = [0.0] * 9
        # angular velocity
        self.ang_cov = [0.0] * 9
        self.ang_cov[8] = 0.02   # chỉ tin angular_velocity.z (yaw rate)
        # linear acceleration (không dùng)
        self.lin_cov = [0.0] * 9

    def cb(self, msg: Imu):
        msg.orientation_covariance = self.orient_cov
        msg.angular_velocity_covariance = self.ang_cov
        msg.linear_acceleration_covariance = self.lin_cov
        self.pub.publish(msg)


def main():
    rclpy.init()
    odom_node = OdomCovStamper()
    imu_node = ImuCovStamper()
    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(odom_node)
        executor.add_node(imu_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        odom_node.destroy_node()
        imu_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
