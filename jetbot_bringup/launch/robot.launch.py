#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    JETBOT_MODEL = os.environ['JETBOT_MODEL']

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    jb_param_dir = LaunchConfiguration(
        'jb_param_dir',
        default=os.path.join(
            get_package_share_directory('jetbot_bringup'),
            'param',
            JETBOT_MODEL + '.yaml'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
           'use_sim_time',
           default_value=use_sim_time,
           description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'jb_param_dir',
            default_value=jb_param_dir,
            description='Full path to jetbot parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/jetbot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([lidar_pkg_dir, '/hlds_laser.launch.py']),
        #     launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        # ),

        Node(
            package='jetbot_node',
            node_executable='jetbot_ros',
            parameters=[jb_param_dir],
            arguments=['-i', usb_port],
            output='screen'),
    ])