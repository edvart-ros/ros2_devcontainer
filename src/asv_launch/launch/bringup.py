import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    urdf_file_name = 'urdf/wamv_base.urdf'
    urdf = os.path.join(
        get_package_share_directory('asv_description'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='asv_nav',
            executable='odom_tf',
            name='odom_tf',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),

        Node(
            package='asv_nav',
            executable='cmd_force',
            name='cmd_force',
            output='screen'),

        Node(
            package='asv_nav',
            executable='cmd_vel',
            name='cmd_vel',
            output='screen'),
        # RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'),

        # PointCloud to LaserScan Node
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[('/cloud_in', '/points')]),

        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='map_and_localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'),
        
    ])
