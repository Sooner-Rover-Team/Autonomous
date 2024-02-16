from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rats',
            executable='simulation',
            name='simulation'
        ),
        Node(
            package='rats',
            executable='viewer',
            name='viewer'
        ),
        Node(
            package='rats',
            executable='camera_reader',
            name='camera_reader'
        ),
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
        ),
        Node(
            package='rats',
            executable='control',
            name='control'
        ),
    ])