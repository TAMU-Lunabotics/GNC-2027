import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('luna_navigation'), 'launch')
    tag_launch = os.path.join(launch_dir, 'tag_localizer.launch.py')
    # ekf_launch = os.path.join(launch_dir, 'ekf.launch.py')

    arena_json = LaunchConfiguration('arena_json')
    use_sim_time = LaunchConfiguration('use_sim_time')

    actions = [
        DeclareLaunchArgument('arena_json', description='Path to artemis_arena_<seed>.json'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tag_launch),
            launch_arguments={'arena_json': arena_json, 'use_sim_time': use_sim_time}.items(),
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(ekf_launch),
        #     launch_arguments={'use_sim_time': use_sim_time}.items(),
        # )
    ]
    return LaunchDescription(actions)
