import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    return LaunchDescription([
        # tf static transform from map to odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
        ),

        # tf static transform from baselink to navsat (gps)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='baselink_to_navsat',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'gps']
        ),

        # gnss preprocessing node
        Node(
            package='gnss_preprocessing',
            executable='gnss_preprocessing',
            name='gnss_preprocessing',
            output='screen'
        ),

        # save gnss path node
        Node(
            package='gnss_preprocessing',
            executable='save_gnss_path',
            name='save_gnss_path',
            output='screen'
        ),

        # play the converted_bag.db3 rosbag file
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play', 
                os.path.expanduser('~/ドキュメント/ros2_bag_file/tukuba_1729737929'),
                '--rate', '30.0',
                '--remap', '/ublox/fix:=/ublox_gps_node/fix'
            ],
            output='screen'
        ),

        # display in RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=[
                '-d', 
                FindPackageShare('gnss_preprocessing').find('gnss_preprocessing') + '/rviz/gnss_preprocessing.rviz'
            ]
        ),
    ])

