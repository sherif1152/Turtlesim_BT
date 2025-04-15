import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Run the turtlesim node
       Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        # Run the behavior tree runner
        Node(
            package='turtlesim_bt',
            executable='turtle_bt',
            #name='bt_turtlesim',
            output='screen'
        )
    ])
