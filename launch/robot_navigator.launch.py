# Author: Addison Sears-Collins
# Date: September 10, 2021
# Description: Launch a basic mobile robot using GPS
# https://automaticaddison.com

import os
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  ld = LaunchDescription()

  pkg_share = FindPackageShare(package='navigation_node').find('navigation_node')
  map_yaml_file = LaunchConfiguration('map')
  
  robot_localization_file_path = os.path.join(pkg_share, 'navi.yaml') 

  # Launch the ROS 2 Navigation Stack
  start_ros2_navigation_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
    launch_arguments = {'map': map_yaml_file,
                        'params_file': params_file,
                        'autostart': autostart}.items()

  # map_server_cmd = Node(
  #     package='nav2_map_server',
  #     executable='map_server',
  #     output='screen',
  #     parameters=[map_server_config_path])

  # Create the launch description and populate
  # # Declare the launch options
  return ld
