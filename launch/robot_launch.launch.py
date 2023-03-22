import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher

def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_p3at')
    world = LaunchConfiguration('world')

    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
        ),
        launch_arguments=[
            ('package', 'webots_ros2_p3at'),
            ('executable', 'p3at_driver'),
            ('world', PathJoinSubstitution([package_dir, 'worlds', world])),
        ],
    )
    gps_renamer = Node(
        package='webots_ros2_p3at',
        executable='gps_renamer',
        name='gps_renamer',
        output='screen',
    )

    path_server = Node(
        package='webots_ros2_p3at',
        executable='path_server',
        name='path_server',
        output='screen',
    )

    # The robot state publisher that will publish the robot joints states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )


    return LaunchDescription([
        robot_state_publisher,
        gps_renamer,
        # path_server,
        # ros2_supervisor,
        DeclareLaunchArgument(
            'world',
            default_value='pioneer3at.wbt',
            description='Choose one of the world files from `/webots_ros2_turtlebot/world` directory'
        ),
        webots,
    ])
