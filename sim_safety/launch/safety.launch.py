from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    ld = LaunchDescription()
    
    safety_node = Node(
        package='sim_safety',
        executable='safety_node',
        name='safety_node',
        output='screen',
    )
    

    ld.add_action(safety_node)
    return ld