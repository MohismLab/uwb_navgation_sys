"""LinkTrack + EKF smoothing of the UWB tags, one EKF per robot.

  /uwb/<robot>/pose -> robot_localization ekf_node -> /uwb_ekf/<robot>/pose

  ros2 launch nlink_parser2 linktrack.ekf.launch.py                     # rm_0, rm_1, rm_2
  ros2 launch nlink_parser2 linktrack.ekf.launch.py robots:=rm_0 uwb_std:=0.08

Needs robot_localization (sudo apt install ros-humble-robot-localization).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def ekf_per_robot(context):
    share = get_package_share_directory('nlink_parser2')
    robots = [r.strip() for r in LaunchConfiguration('robots').perform(context).split(',') if r.strip()]
    nodes = []
    for robot in robots:
        nodes += [
            # namespaced so its set_pose / odometry/filtered topics do not collide with other
            # robot_localization instances on the network
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf',
                namespace=f'/uwb_ekf/{robot}',
                output='screen',
                parameters=[os.path.join(share, 'config', 'uwb_ekf.yaml')],
            ),
            Node(
                package='nlink_parser2',
                executable='uwb_ekf_adapter.py',
                output='screen',
                arguments=['--robot', robot, '--std', LaunchConfiguration('uwb_std'),
                           '--floor-z', LaunchConfiguration('floor_z')],
            ),
        ]
    return nodes


def generate_launch_description():
    share = get_package_share_directory('nlink_parser2')

    return LaunchDescription([
        DeclareLaunchArgument('port_name', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='1000000'),
        DeclareLaunchArgument('robots', default_value='rm_0,rm_1,rm_2',
                              description='comma separated tag names, /uwb/<robot>/pose'),
        DeclareLaunchArgument('uwb_std', default_value='0.05',
                              description='UWB position std fed to the EKF [m]'),
        DeclareLaunchArgument('floor_z', default_value='-1.75',
                              description='floor height in the UWB frame (anchors are at z=0) [m]'),

        # world (UWB frame, anchor height) -> uwb_floor: RViz uses the floor as fixed frame so
        # the grid lies on the floor and 2D Goal Pose clicks hit the floor plane
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='uwb_floor_tf',
            arguments=['--frame-id', 'world', '--child-frame-id', 'uwb_floor',
                       '--z', LaunchConfiguration('floor_z')],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'linktrack.launch.py')),
            launch_arguments={
                'port_name': LaunchConfiguration('port_name'),
                'baud_rate': LaunchConfiguration('baud_rate'),
            }.items(),
        ),

        OpaqueFunction(function=ekf_per_robot),
    ])
