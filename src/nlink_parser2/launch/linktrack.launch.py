from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyACM0'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='1000000',
        description='Serial baud rate'
    )

    pose_frame_id_arg = DeclareLaunchArgument(
        'pose_frame_id',
        default_value='world',
        description='frame_id of the published tag poses'
    )

    linktrack_node = Node(
        package='nlink_parser2',
        executable='linktrack_node',
        name='linktrack0',
        output='screen',
        parameters=[
            {'port_name': LaunchConfiguration('port_name')},
            {'baud_rate': LaunchConfiguration('baud_rate')},
            # tag with id N is published as <pose_topic_prefix>/<tag_name_prefix>N/pose
            {'pose_topic_prefix': '/uwb'},
            {'tag_name_prefix': 'rm_'},
            {'pose_frame_id': LaunchConfiguration('pose_frame_id')},
        ]
    )

    return LaunchDescription([
        port_name_arg,
        baud_rate_arg,
        pose_frame_id_arg,
        linktrack_node,
    ])
