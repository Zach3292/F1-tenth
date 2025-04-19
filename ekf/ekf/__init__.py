import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu, JointState
from ekf.ekf_msg import State
import numpy as np


class Extended_Kalman_Filter(Node):

    angular_velocity = 0
    angular_velocity_covariance = 0
    linear_acceleration = np.zeros((2, 1))
    linear_acceleration_covariance = np.zeros((2, 2))
    left_encoder_position = np.zeros((3, 1))
    left_encoder_velocity = np.zeros((3, 1))
    right_encoder_position = np.zeros((3, 1))
    right_encoder_velocity = np.zeros((3, 1))

    stateMatrix = np.zeros((5, 1))

    processCovariance = 0.025
    processCovarianceMatrix = np.eye((5)) * processCovariance

    estimatedCovariance = 0.025
    estimatedCovarianceMatrix = np.eye((5)) * estimatedCovariance

    encoderCovariance = 0.025
    encoderCovarianceMatrix = np.eye((2)) * encoderCovariance

    delta_time = 1 / 60

    wheel_radius = 1.5 * 2.54 / 100
    wheel_base = 0.15

    def __init__(self):
        super().__init__("ekf_node")
        self.publisher_ = self.create_publisher(State, "/autodrive/f1tenth_1/state", 10)
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
        self.left_encoder_position[0] = msg.position[0]
        self.left_encoder_position[1] = msg.position[1]
        self.left_encoder_position[2] = msg.position[2]
        self.left_encoder_velocity[0] = msg.velocity[0]
        self.left_encoder_velocity[1] = msg.velocity[1]
        self.left_encoder_velocity[2] = msg.velocity[2]
        return

    def right_encoder_callback(self, msg):
        self.right_encoder_position[0] = msg.position[0]
        self.right_encoder_position[1] = msg.position[1]
        self.right_encoder_position[2] = msg.position[2]
        self.right_encoder_velocity[0] = msg.velocity[0]
        self.right_encoder_velocity[1] = msg.velocity[1]
        self.right_encoder_velocity[2] = msg.velocity[2]
        return

    def main_loop(self):

        A_matrix = np.array(
            [
                [1, 0, self.delta_time, 0, 0],
                [0, 1, 0, self.delta_time, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1],
            ]
        )

        imu_control_matrix = np.array(
            [
                0.5 * self.linear_acceleration[0] * (self.delta_time**2),
                0.5 * self.linear_acceleration[1] * (self.delta_time**2),
                self.linear_acceleration[0] * self.delta_time,
                self.linear_acceleration[1] * self.delta_time,
                self.angular_velocity * self.delta_time,
            ]
        )

        imu_covariance_1D = np.array(
            [
                self.linear_acceleration_covariance[0][0],
                self.linear_acceleration_covariance[1][1],
                self.linear_acceleration_covariance[0][0],
                self.linear_acceleration_covariance[1][1],
                self.angular_velocity_covariance
            ]
        )

        imu_covariance_matrix = np.dot(imu_covariance_1D, imu_covariance_1D.T)

        imu_estimated_state = (
            np.dot(A_matrix, self.stateMatrix)
            + imu_control_matrix
            + imu_covariance_1D
        )
        
        va = (self.left_encoder_velocity[0] + self.right_encoder_velocity[0]) / 2
        
        encoder_control_matrix = np.array(
            [
                va * np.cos(self.stateMatrix[4]) * self.delta_time, 
                va * np.sin(self.stateMatrix[4]) * self.delta_time,
                va * np.cos(self.stateMatrix[4]),
                va * np.sin(self.stateMatrix[4]),
                (self.right_encoder_velocity[0] - self.left_encoder_velocity[0]) * self.delta_time / self.wheel_base
            ]
        )

        encoder_covariance_1D = (np.ones((5, 1)) * self.encoderCovariance)

        encoder_covariance_matrix = np.dot(encoder_covariance_1D, encoder_covariance_1D.T)

        encoder_estimated_state = (
            np.dot(A_matrix, self.stateMatrix)
            + encoder_control_matrix
            + encoder_covariance_1D
        )

        P = np.dot(np.dot(A_matrix, self.estimatedCovarianceMatrix), A_matrix.T) + imu_covariance_matrix

        K = np.dot(P, np.linalg.inv(P + encoder_covariance_matrix))

        self.stateMatrix = imu_estimated_state + np.dot(K, (encoder_estimated_state - imu_estimated_state))

        self.estimatedCovarianceMatrix = np.dot((np.eye(5) - K), P)

        state_msg = State()
        state_msg.x = self.stateMatrix[0]
        state_msg.y = self.stateMatrix[1]
        state_msg.xdot = self.stateMatrix[2]
        state_msg.ydot = self.stateMatrix[3]
        state_msg.theta = self.stateMatrix[4]

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
