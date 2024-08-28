import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    current_dir = get_package_share_directory('gazebo_env')
    spawn = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-file', current_dir + '/model/robot.sdf','-z','0.7',
    ],
    output='screen',
    )
    return LaunchDescription([
        spawn
    ])
