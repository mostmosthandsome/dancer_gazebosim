import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    current_dir = get_package_share_directory('gazebo_env')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': current_dir + '/model/robocup_world.sdf -v 3'}.items(),
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/rotor0@std_msgs/msg/Float64@gz.msgs.Double',
                   ],
        output='screen'
    )
    spawn1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', current_dir + '/model/robot.sdf','-z','0.7','-name', 'robot1',
        ],
        output='screen',
    )

    # spawn2 = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         '-file', current_dir + '/model/robot.sdf','-x', '1.0', '-y','1.0', '-z','0.7', '-name', 'robot2',
    #     ],
    #     output='screen',
    # )
    # watcher = Node(
    #     package='climb',
    #     executable='joint_reader',
    #     output='screen',
    # )
    return LaunchDescription([
        # watcher,
        gz_sim,
        bridge,
        spawn1,
        # spawn2
    ])
