import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
from runner.PIDController import PIDController as PID


class Gap_Finder(Node):

    ranges = np.array([])
    previous_time, time = 0, 0
    velocity = 0
    setpoint = 0

    angle_increment = 0
    range_max = 0

    def __init__(self):
        super().__init__("runner_node")
        # self.publisher1_ = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.publisher2_ = self.create_publisher(Float32, "/autodrive/f1tenth_1/steering_command", 10)
        self.publisher3_ = self.create_publisher(Float32, "/autodrive/f1tenth_1/throttle_command", 10)
        self.subscription1 = self.create_subscription(LaserScan, "/autodrive/f1tenth_1/lidar", self.scan_callback, 10)
        self.subscription3 = self.create_subscription(Float32, "/autodrive/f1_tenth_1/speed", self.speed_callback, 10)
        self.get_logger().info("Runner node started")
        # self.timer_ = self.create_timer(0.001, self.timer_callback)

    def create_safety_bubble(self, ranges, threshold):

        radius = 0.25
        for i in range(len(ranges)):
            if ranges[i] == np.nan:
                ranges[i] = 0
            if ranges[i] < threshold and ranges[i] > 0:
                bubble_index = int(2 * np.arcsin(radius / (2 * ranges[i])) / self.angle_increment)
                for i in range(i - bubble_index, i + bubble_index):
                    if i >= 0 and i < len(ranges):
                        ranges[i] = 0

        return ranges

    def find_largest_gap(self, ranges):
        gap = []
        gap_start = np.inf
        gap_end = 0
        big_gap_start = 0
        big_gap_end = 0

        for i in range(len(ranges)):
            if gap_start > i:
                if ranges[i] != 0:
                    gap_start = i
            else:
                if ranges[i] == 0:
                    gap_end = i
                    if gap_end - gap_start > big_gap_end - big_gap_start:
                        big_gap_start = gap_start
                        big_gap_end = gap_end
                    gap_start = np.inf

        gap = [big_gap_start, big_gap_end]
        return gap

    def center_of_deepest_gap(self, gap, ranges, threshold):
        for i in range(gap[0], gap[1]):
            if ranges[i] > threshold:
                ranges[i] = threshold

        deep_gap = []
        gap_start = np.inf
        gap_end = 0
        big_gap_start = 0
        big_gap_end = 0

        for i in range(gap[0], gap[1]):
            if gap_start > i:
                if ranges[i] == threshold:
                    gap_start = i
            else:
                if ranges[i] != threshold:
                    gap_end = i
                    if gap_end - gap_start > big_gap_end - big_gap_start:
                        big_gap_start = gap_start
                        big_gap_end = gap_end
                    gap_start = np.inf
        if big_gap_end == 0:
            big_gap_end = gap[1]
        if big_gap_start == 0:
            big_gap_start = gap[0]

        deep_gap = [big_gap_start, big_gap_end]

        return int((deep_gap[1] + deep_gap[0]) / 2)

    def scan_callback(self, msg):
        self.ranges = np.copy(msg.ranges)
        self.angle_increment = msg.angle_increment
        self.range_max = msg.range_max
        # angle_b = np.pi/2
        # reject_index = int((msg.angle_max-angle_b) / msg.angle_increment)

        # for i in range(reject_index):
        #     self.ranges[i] = 0
        # for i in range((len(self.ranges) - reject_index), len(self.ranges)):
        #     self.ranges[i] = 0

        ranges_with_bubble = self.create_safety_bubble(self.ranges, 2)
        gap = self.find_largest_gap(ranges_with_bubble)
        center = self.center_of_deepest_gap(gap, ranges_with_bubble, 2.7)
        angle = (center - len(self.ranges) / 2) * msg.angle_increment

        self.time = self.get_clock().now().nanoseconds / 1e6

        steering_PID = PID(1.1, 0.0001, 1.5)
        throttle_PID = PID(0.05, 0.01, 0.01)

        steering = steering_PID.update(self.setpoint, angle, self.time - self.previous_time)

        steering_index = int((len(self.ranges) / 2) - (angle / self.angle_increment))

        steering_index = min(len(self.ranges) - 1, steering_index)

        if msg.ranges[steering_index] == np.nan:
            speed = 0
        else:
            speed = (msg.ranges[steering_index] / 4) * (np.abs(1 / steering) ** 3)

        throttle = throttle_PID.update(speed, self.velocity, self.time - self.previous_time)
        throttle = min(1.0, throttle)
        # msg = AckermannDriveStamped()
        steering_msg = Float32()
        throttle_msg = Float32()

        throttle_msg.data = throttle
        # msg.drive.speed = speed

        # msg.drive.steering_angle = -control
        steering_msg.data = -steering
        # self.publisher1_.publish(msg)
        self.publisher2_.publish(steering_msg)
        self.publisher3_.publish(throttle_msg)

        self.previous_time = self.time

        # angle_b = np.pi/2
        # angle_a = np.pi/3
        # delta_angle = np.abs(angle_b - angle_a)
        # index_b = int((angle_b-msg.angle_min) / msg.angle_increment)
        # index_a = int((angle_a-msg.angle_min) / msg.angle_increment)
        # a = self.ranges[index_a]
        # b = self.ranges[index_b]

        # alpha = np.arctan((a*np.cos(delta_angle)-b)/(a*np.sin(delta_angle)))

        # measured_distance = b*np.cos(alpha)

        # self.future_measurement = measured_distance + self.velocity*(self.time - self.previous_time)*np.sin(alpha)

    def speed_callback(self, msg):
        self.velocity = msg.data


def main(args=None):
    rclpy.init(args=args)
    runner_node = Gap_Finder()
    rclpy.spin(runner_node)
    runner_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
