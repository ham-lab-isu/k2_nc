from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    gcode_file_arg = DeclareLaunchArgument(
        'gcode_file',
        default_value='',
        description='Path to G-code file'
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.0.2',
        description='Robot IP address'
    )
    
    # Create the node
    gcode_driver_node = Node(
        package='k2_nc',
        executable='gcode_driver',
        name='gcode_driver',
        output='screen',
        parameters=[
            {'robot_ip': LaunchConfiguration('robot_ip')}
        ],
        arguments=[
            {
                LaunchConfiguration('gcode_file')
            }
        ]
    )
    
    return LaunchDescription([
        gcode_file_arg,
        robot_ip_arg,
        gcode_driver_node
    ])
