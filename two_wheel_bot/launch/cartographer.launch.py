from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Path to your cartoslam config.lua
    pkg_share = FindPackageShare('two_wheel_bot').find('two_wheel_bot')
    config_dir = os.path.join(pkg_share, 'config')
    config_file = 'config.lua'

    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', config_file
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
        output='screen'
    )

    # Occupancy grid node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'resolution': 0.05},            # map resolution in meters
            {'publish_period_sec': 1.0},     # update rate for /map
        ],
        output='screen'
    )

    # Return LaunchDescription
    return LaunchDescription([
        use_sim_time_arg,
        cartographer_node,
        occupancy_grid_node,
    ])

