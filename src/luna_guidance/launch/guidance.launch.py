import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('luna_guidance')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when running with the sim clock'),
        Node(
            package='luna_guidance',
            executable='guidance_controller',
            name='guidance_controller',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'guidance.yaml'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
