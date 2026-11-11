from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arena_json = LaunchConfiguration('arena_json')
    use_sim_time = LaunchConfiguration('use_sim_time')

    config = PathJoinSubstitution(
        [FindPackageShare('luna_navigation'), 'config', 'tag_localizer.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('arena_json', description='Path to artemis_arena_<seed>.json'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='luna_navigation',
            executable='tag_localizer_node',
            name='tag_localizer_node',
            output='screen',
            parameters=[
                config,
                {'arena_json': arena_json, 'use_sim_time': use_sim_time},
            ],
        ),
    ])
