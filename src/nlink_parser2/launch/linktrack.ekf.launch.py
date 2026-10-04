"""LinkTrack + Kalman filtering of the UWB tags, one filter (uwb_ekf_adapter.py) per robot.

  /uwb/<robot>/pose [+ robot velocity, config/uwb_velocity.yaml] -> /uwb_ekf/<robot>/pose

  ros2 launch nlink_parser2 linktrack.ekf.launch.py                     # robots of config/uwb_tags.yaml
  ros2 launch nlink_parser2 linktrack.ekf.launch.py robots:=rm_0 uwb_std:=0.08
  ros2 launch nlink_parser2 linktrack.ekf.launch.py tags_file:=/path/to/uwb_tags.yaml
  ros2 launch nlink_parser2 linktrack.ekf.launch.py velocity_file:=''   # UWB position only
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def robot_names(context):
    """robots:=a,b,c, or by default every tag_names entry of the tags file."""
    robots = [r.strip() for r in LaunchConfiguration('robots').perform(context).split(',') if r.strip()]
    if robots:
        return robots
    with open(LaunchConfiguration('tags_file').perform(context)) as f:
        params = next(iter(yaml.safe_load(f).values()))['ros__parameters']
    return list(params.get('tag_names', []))


def velocity_sources(context):
    """robot -> {velocity_topic, heading_topic[, heading_offset]} of the velocity file"""
    path = LaunchConfiguration('velocity_file').perform(context)
    if not path:
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def ekf_per_robot(context):
    vel = velocity_sources(context)
    nodes = []
    for robot in robot_names(context):
        args = ['--robot', robot, '--std', LaunchConfiguration('uwb_std'),
                '--floor-z', LaunchConfiguration('floor_z')]
        src = vel.get(robot)
        if src:
            args += ['--vel-topic', src['velocity_topic'], '--heading-topic', src['heading_topic'],
                     '--mag-state-topic', src.get('mag_state_topic', f'/{robot}/imu/mag_state'),
                     '--velocity-rotation', str(src.get('velocity_rotation', 0.0))]
            if src.get('heading_offset') is not None:
                args += ['--heading-offset', str(src['heading_offset'])]
        nodes.append(Node(package='nlink_parser2', executable='uwb_ekf_adapter.py', output='screen',
                          arguments=args))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('nlink_parser2')

    return LaunchDescription([
        DeclareLaunchArgument('port_name', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='1000000'),
        DeclareLaunchArgument('tags_file', default_value=os.path.join(share, 'config', 'uwb_tags.yaml'),
                              description='tag id -> robot name mapping'),
        DeclareLaunchArgument('robots', default_value='',
                              description='comma separated robot names for the EKF, default: all of tags_file'),
        DeclareLaunchArgument('velocity_file', default_value=os.path.join(share, 'config', 'uwb_velocity.yaml'),
                              description="robots whose body velocity is fused, '' for none"),
        DeclareLaunchArgument('uwb_std', default_value='0.05',
                              description='UWB position std of the filter without velocity [m]'),
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
                'tags_file': LaunchConfiguration('tags_file'),
            }.items(),
        ),

        OpaqueFunction(function=ekf_per_robot),
    ])
