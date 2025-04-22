import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu, JointState
import numpy as np


class Extended_Kalman_Filter(Node):

    angular_velocity = 0
    angular_velocity_covariance = 0
    linear_acceleration = np.zeros(2, np.float32)
    linear_acceleration_covariance = np.zeros((2, 2), np.float32)
    left_encoder_position = 0
    right_encoder_position = 0
    previous_left_encoder_position = 0
    previous_right_encoder_position = 0
    left_encoder_velocity = 0
    right_encoder_velocity = 0

    stateMatrix = np.zeros(5, np.float32)

    processCovariance = 0.025
    processCovarianceMatrix = np.eye((5), dtype=np.float32) * processCovariance

    estimatedCovariance = 0.025
    estimatedCovarianceMatrix = np.eye((5), dtype=np.float32) * estimatedCovariance

    encoderCovariance = 0.025
    encoderCovarianceMatrix = np.eye((2), dtype=np.float32) * encoderCovariance

    delta_time = 1 / 60

    wheel_radius = 1.5 * 2.54 / 100
    wheel_base = 0.15

    def __init__(self):
        super().__init__("ekf_node")
        self.publisher_ = self.create_publisher(Pose, "/autodrive/f1tenth_1/ekf_pose", 10)
        self.imu_subscription = self.create_subscription(Imu, "/autodrive/f1tenth_1/imu", self.imu_callback, 10)
        self.left_encoder_subscription = self.create_subscription(
            JointState, "/autodrive/f1tenth_1/left_encoder", self.left_encoder_callback, 10
        )
        self.right_encoder_subscription = self.create_subscription(
            JointState, "/autodrive/f1tenth_1/right_encoder", self.right_encoder_callback, 10
        )

        self.get_logger().info("EKF node started")
        self.timer_ = self.create_timer(self.delta_time, self.main_loop)

    def imu_callback(self, msg):
        self.angular_velocity = msg.angular_velocity.z
        self.angular_velocity_covariance = msg.angular_velocity_covariance[8]
        self.linear_acceleration[0] = msg.linear_acceleration.x
        self.linear_acceleration[1] = msg.linear_acceleration.y
        self.linear_acceleration_covariance[0][0] = msg.linear_acceleration_covariance[0]
        self.linear_acceleration_covariance[1][1] = msg.linear_acceleration_covariance[4]
        return

    def left_encoder_callback(self, msg):
        self.previous_left_encoder_position = self.left_encoder_position
        self.left_encoder_position = msg.position[0]
        self.left_encoder_velocity = (self.left_encoder_position - self.previous_left_encoder_position) / (
            msg.header.stamp.sec
        )
        self.left_encoder_velocity = self.left_encoder_velocity / self.wheel_radius
        return

    def right_encoder_callback(self, msg):
        self.previous_right_encoder_position = self.right_encoder_position
        self.right_encoder_position = msg.position[0]
        self.right_encoder_velocity = (self.right_encoder_position - self.previous_right_encoder_position) / (
            msg.header.stamp.sec
        )
        self.right_encoder_velocity = self.right_encoder_velocity / self.wheel_radius
        return

    def main_loop(self):

        A_matrix = np.array(
            [
                [1, 0, self.delta_time, 0, 0],
                [0, 1, 0, self.delta_time, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1],
            ],
            dtype=np.float32,
        )

        imu_control_matrix = np.array(
            [
                0.5 * self.linear_acceleration[0] * (self.delta_time**2),
                0.5 * self.linear_acceleration[1] * (self.delta_time**2),
                self.linear_acceleration[0] * self.delta_time,
                self.linear_acceleration[1] * self.delta_time,
                self.angular_velocity * self.delta_time,
            ],
            dtype=np.float32,
        )

        imu_covariance_1D = np.array(
            [
                self.linear_acceleration_covariance[0][0],
                self.linear_acceleration_covariance[1][1],
                self.linear_acceleration_covariance[0][0],
                self.linear_acceleration_covariance[1][1],
                self.angular_velocity_covariance,
            ],
            dtype=np.float32,
        )

        imu_covariance_matrix = np.dot(imu_covariance_1D, imu_covariance_1D.T)

        imu_estimated_state = np.dot(A_matrix, self.stateMatrix) + imu_control_matrix + imu_covariance_1D

        va = (self.left_encoder_velocity + self.right_encoder_velocity) / 2

        encoder_control_matrix = np.array(
            [
                va * np.cos(self.stateMatrix[4]) * self.delta_time,
                va * np.sin(self.stateMatrix[4]) * self.delta_time,
                va * np.cos(self.stateMatrix[4]),
                va * np.sin(self.stateMatrix[4]),
                (self.right_encoder_velocity - self.left_encoder_velocity) * self.delta_time / self.wheel_base,
            ],
            dtype=np.float32,
        )

        encoder_covariance_1D = np.ones(5) * self.encoderCovariance

        encoder_covariance_matrix = np.dot(encoder_covariance_1D, encoder_covariance_1D.T)

        encoder_estimated_state = np.dot(A_matrix, self.stateMatrix) + encoder_control_matrix + encoder_covariance_1D

        P = np.dot(np.dot(A_matrix, self.estimatedCovarianceMatrix), A_matrix.T) + imu_covariance_matrix

        K = np.dot(P, np.linalg.inv(P + encoder_covariance_matrix))

        self.stateMatrix = imu_estimated_state + np.dot(K, (encoder_estimated_state - imu_estimated_state))
        self.get_logger().info(f"State Matrix: {self.stateMatrix}")

        self.estimatedCovarianceMatrix = np.dot((np.eye(5, dtype=np.float32) - K), P)

        state_msg = Pose()
        state_msg.position.x = self.stateMatrix[0]
        state_msg.position.y = self.stateMatrix[1]
        state_msg.orientation.w = np.cos(self.stateMatrix[4] / 2)
        state_msg.orientation.z = np.sin(self.stateMatrix[4] / 2)
        state_msg.orientation.x = 0.0
        state_msg.orientation.y = 0.0
        state_msg.position.z = 0.0

        self.publisher_.publish(state_msg)

        return


def main(args=None):
    rclpy.init(args=args)
    ekf_node = Extended_Kalman_Filter()
    rclpy.spin(ekf_node)
    ekf_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
