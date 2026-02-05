from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    publisher_node = Node(
        package='learn_cpp',
        executable='publisher',
        name='publisher_node',
        output='screen'
    )
    
    subscriber_node = Node(
        package='learn_cpp',
        executable='subscriber',
        name='subscriber_node',
        output='screen'
    )
    
    counter_node = Node(
        package='learn_cpp',
        executable='counterpub',
        name='counter_publisher',
        output='screen'
    )
    return LaunchDescription([
        publisher_node,
        subscriber_node,
        counter_node
    ])