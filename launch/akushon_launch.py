from launch import LaunchDescription
from launch_ros.actions import Node;

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='akushon',
            namespace='akushon',
            executable='main',
            name='akushon',
            arguments=['src/akushon/data/action/'],
        ),
        Node(
            package='tachimawari',
            namespace='tachimawari',
            executable='main',
            name='tachimawari',
        )
    ])
