from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    publisher = Node(
        package='rcllua_examples',
        executable='simple_publisher'
    )

    subscription = Node(
        package='rcllua_examples',
        executable='simple_subscription'
    )

    return LaunchDescription([
        publisher,
        subscription
    ])

