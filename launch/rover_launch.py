from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rats',
            executable='gps',
            name='gps'
        ),
        Node(
            package='rats',
            executable='camera',
            name='camera'
        ),
        # Node(
        #     package='rats',
        #     executable='camera_reader',
        #     name='camera_reader'
        # ),
        Node(
            package='rats',
            executable='position',
            name='position'
        ),
        Node(
            package='rats',
            executable='tracker',
            name='tracker'
        ),
        Node(
            package='rats',
            executable='navigation',
            name='navigation'
        )
    ])