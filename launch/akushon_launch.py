from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    firstPackage = LaunchConfiguration('firstPackage')
    secondPackage = LaunchConfiguration('secondPackage')
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')

    firstPackage_launch_arg = DeclareLaunchArgument(
        'firstPackage',
        default_value='akushon'
    )

    firstNode = Node(
        package=firstPackage,
        executable='main',
        name='akushon',
        arguments=['src/akushon/data/action/']
    )

    run_first_package = ExecuteProcess(
        cmd=[[
            'ros2 run ',
            'akushon ',
            'main ',
            'src/akushon/data/action/'
        ]],
        shell=True
    )

    secondPackage_launch_arg = DeclareLaunchArgument(
        'secondPackage',
        default_value='tachimawari'
    )

    host_launch_arg = DeclareLaunchArgument(
        'host',
        default_value=''
    )

    port_launch_arg = DeclareLaunchArgument(
        'port',
        default_value=''
    )

    secondNode = Node(
        package=secondPackage,
        executable='main',
        arguments=[],
    )

    run_second_package = ExecuteProcess(
        cmd=[[
            'ros2 run ',
            secondPackage,
            ' main ',
            host,
            port
        ]],
        shell=True
    )

    return LaunchDescription([
        firstPackage_launch_arg,
        secondPackage_launch_arg,
        host_launch_arg,
        port_launch_arg,
        firstNode,
        secondNode,
        run_first_package,
        run_second_package
    ])
