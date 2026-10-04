from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    declare_port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyCH343USB0',  
    )
    declare_baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='921600',
        description='Serial baud rate'
    )

    tofsensem_node = Node(
        package='nlink_parser2',     
        executable='tofsensem_node', 
        name='tofsensem0',          
        output='screen',            
        parameters=[{               
            'port_name': LaunchConfiguration('port_name'),   
            'baud_rate': LaunchConfiguration('baud_rate'),
        }]
    )

    return LaunchDescription([
        declare_port_name_arg,
        declare_baud_rate_arg,
        tofsensem_node,
    ])