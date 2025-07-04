from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='deconfliction_system',
            executable='mission_input_node',
            name='mission_input_node'
        ),
        Node(
            package='deconfliction_system',
            executable='simulation_manager',
            name='simulation_manager'
        ),
        Node(
            package='deconfliction_system',
            executable='deconfliction_engine',
            name='deconfliction_engine'
        ),
        Node(
            package='deconfliction_system',
            executable='visualization_node',
            name='visualization_node',
            output='screen'  
        )
    ])
