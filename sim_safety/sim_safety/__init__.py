import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class SafetyNode(Node):

    ranges = []
    prev_ranges = []
    previous_time = 0
    velocity = 0

    def __init__(self):
        super().__init__('safety_node')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscription1 = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.subscription2 = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.get_logger().info('Safety node started')
        # self.timer_ = self.create_timer(0.5, self.timer_callback)


    def scan_callback(self, msg):
        self.ranges = msg.ranges

        if len(self.prev_ranges) == 0:
            self.prev_ranges = self.ranges
            return
        
        delta_ranges = np.zeros(len(self.ranges))
        ittc = np.zeros(len(self.ranges))


        for i in range(len(self.ranges)):
            if self.ranges[i] > msg.range_max or self.ranges[i] < msg.range_min:
                ittc[i] = np.inf
                # self.get_logger().info('Invalid range: %s' % self.ranges[i])
                continue
            
            delta_ranges[i] = self.velocity * np.cos((i*msg.angle_increment) + msg.angle_min)

            if delta_ranges[i] <= 0:
                ittc[i] = np.inf
            else:
                ittc[i] = self.ranges[i]/delta_ranges[i]

        min_ittc = np.nanmin(ittc)

        if min_ittc < 0.25:
            self.get_logger().info('Min ITTc: %s' % min_ittc)
            self.get_logger().info('Emergency stop')
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            self.publisher_.publish(msg)

        self.prev_ranges = self.ranges
        self.previous_time = float(msg.header.stamp.sec)
        
    def odom_callback(self, msg):
        # self.get_logger().info('Safety node received: "%s"' % msg.pose.pose.position)
        self.velocity = msg.twist.twist.linear.x

    

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Safety message'
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()