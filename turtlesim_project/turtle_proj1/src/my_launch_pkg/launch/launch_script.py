from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    id = LaunchDescription()
    first_Node =  Node(
            package='turtlesim',
            executable='turtlesim_node'
        )
    second_Node = Node(
            package='project_nodes',
            executable='turtle_commander'
        )
    
    id.add_action(first_Node)
    id.add_action(second_Node)

    return id
