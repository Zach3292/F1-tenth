from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    ld = LaunchDescription()

    
    ekf_node = Node(
        package="ekf",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
    )

    ld.add_action(ekf_node)
    return ld
