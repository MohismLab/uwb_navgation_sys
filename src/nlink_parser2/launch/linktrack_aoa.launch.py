from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyCH343USB0'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='921600',
        description='Serial baud rate'
    )

    linktrack_aoa_node = Node(
        package='nlink_parser2',
        executable='linktrack_aoa_node',
        name='linktrack_aoa0',
        output='screen',
        parameters=[
            {'port_name': LaunchConfiguration('port_name')},
            {'baud_rate': LaunchConfiguration('baud_rate')}
        ]
    )

    return LaunchDescription([
        port_name_arg,
        baud_rate_arg,
        linktrack_aoa_node
    ])
