from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to your URDF/Xacro file
    urdf_file = os.path.join(
        get_package_share_directory('two_wheel_bot'),
        'urdf',
        'finalrobo.urdf'   # <-- change to your actual URDF/xacro filename
    )

    # Load robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[urdf_file]
    )

    # Start RViz2
    rviz_config_file = os.path.join(
        get_package_share_directory('two_wheel_bot'),
        'rviz',
        'urdf_config.rviz'   # <-- create this file later if needed
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])

