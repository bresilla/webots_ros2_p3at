import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  pkg_share = FindPackageShare(package='webots_ros2_p3at').find('webots_ros2_p3at')
  robot_localization_file_path = os.path.join(pkg_share, 'ekf.yaml') 
  
  # Start the navsat transform node which converts GPS data into the world coordinate frame
  start_navsat_transform_cmd = Node(
    package='robot_localization',
    executable='navsat_transform_node',
    name='navsat_transform',
    output='screen',
    parameters=[robot_localization_file_path],
    remappings=[('imu/data', 'imu/data'),
                ('gps/fix', 'gps/fix'), 
                ('gps/filtered', 'gps/filtered'),
                ('odometry/gps', 'odometry/gps'),
                ('odometry/filtered', 'odometry/global')])

  # Start robot localization using an Extended Kalman filter...map->odom transform
  start_robot_localization_global_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node_map',
    output='screen',
    parameters=[robot_localization_file_path],
    remappings=[('odometry/filtered', 'odometry/global'), ('/set_pose', '/initialpose')])

  # Start robot localization using an Extended Kalman filter...odom->base_footprint transform
  start_robot_localization_local_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node_odom',
    output='screen',
    parameters=[robot_localization_file_path],
    remappings=[('odometry/filtered', 'odometry/local'), ('/set_pose', '/initialpose')])

  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(start_navsat_transform_cmd)
  ld.add_action(start_robot_localization_global_cmd)
  ld.add_action(start_robot_localization_local_cmd)

  return ld