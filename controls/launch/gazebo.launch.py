from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Set package and URDF file path
    pkg_name = 'controls'
    urdf_file_name = 'simplearm.urdf'
    urdf_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', urdf_file_name)

    # Ensure the URDF file exists before proceeding
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

    # Load URDF as robot_description
    robot_description = open(urdf_path, 'r').read()

    # Start Gazebo with ROS integration
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    # Robot State Publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Controller Manager Node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",   # ✅ fixed
        parameters=[
            {"robot_description": robot_description},
            os.path.join(
                get_package_share_directory("controls"),
                "config",
                "simple_controllers.yaml"
            )
        ],
        output="screen"
    )

    # Joint State Broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # Arm Controller spawner
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_amr',
        arguments=[
            '-entity', 'amr',
            '-file', urdf_path,
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # Joint State Publisher node (useful if you want sliders in RViz)
    #jsp_node = Node(
        #package='joint_state_publisher',
        #executable='joint_state_publisher',
        #name='joint_state_publisher',
        #output='screen'
    #)

    # RViz2 node
    # rviz_config_path = os.path.join(
    #     get_package_share_directory(pkg_name),
    #     'rviz',
    #     'simplearm.rviz'   # Create this RViz config file
    # # )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_path],
    #     output='screen'
    # )

    return LaunchDescription([
        gazebo,
        rsp_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        spawn_entity,
        #jsp_node,
        # rviz_node
    ])
