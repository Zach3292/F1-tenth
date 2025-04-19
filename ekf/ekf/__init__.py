import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np


class Extended_Kalman_Filter(Node):


    def __init__(self):
        super().__init__("ekf_node")
        self.publisher_ = self.create_publisher(Float32, "/autodrive/f1tenth_1/steering_command", 10)
        self.subscription = self.create_subscription(LaserScan, "/autodrive/f1tenth_1/lidar", self.scan_callback, 10)
        self.get_logger().info("EKF node started")
        # self.timer_ = self.create_timer(0.001, self.timer_callback)

    def scan_callback(self, msg):
        return



def main(args=None):
    rclpy.init(args=args)
    ekf_node = Extended_Kalman_Filter()
    rclpy.spin(ekf_node)
    ekf_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
