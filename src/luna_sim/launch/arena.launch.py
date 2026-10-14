"""Generate an arena, then serve its maps.

  ros2 launch luna_sim arena.launch.py                      # random seed (printed)
  ros2 launch luna_sim arena.launch.py seed:=42 rviz:=true
  ros2 launch luna_sim arena.launch.py seed:=7 config:=arena_dense

Publishes:
  /map         known map (walls + column): what nav2 plans on
  /map_truth   full map incl. obstacles: for RViz / oracle mode (publish_truth:=true)

Included by level1.launch.py / level2.launch.py. Parents should pick the seed and pass
it in, so they can compute the same file paths with luna_sim.arena.paths.arena_paths().
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from luna_sim.arena.arena_gen import generate_map
from luna_sim.arena.paths import arena_paths, default_out_dir, resolve_config, resolve_seed


def _setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)

    seed = resolve_seed(arg('seed'))
    config = resolve_config(arg('config'))
    out_dir = arg('out_dir') or str(default_out_dir())
    resolution = float(arg('resolution'))
    use_sim_time = arg('use_sim_time').lower() == 'true'
    publish_truth = arg('publish_truth').lower() == 'true'

    # Runs right now, in the launch process, before any node starts. If the config is
    # bad, launch fails here with the generator's error instead of map_server failing later.
    generate_map(config, seed, out_dir, resolution)
    paths = arena_paths(out_dir, seed)

    actions = [
        LogInfo(msg=f'[arena] seed={seed}  config={config}  out={out_dir}'),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[{'yaml_filename': str(paths['known_map']),
                         'topic_name': 'map',
                         'frame_id': 'map',
                         'use_sim_time': use_sim_time}]),
    ]
    managed = ['map_server']

    if publish_truth:
        actions.append(Node(
            package='nav2_map_server', executable='map_server', name='map_server_truth',
            output='screen',
            parameters=[{'yaml_filename': str(paths['truth_map']),
                         'topic_name': 'map_truth',
                         'frame_id': 'map',
                         'use_sim_time': use_sim_time}]))
        managed.append('map_server_truth')

    # map_server is a lifecycle node: it publishes nothing until configured + activated.
    actions.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_arena', output='screen',
        parameters=[{'autostart': True,
                     'node_names': managed,
                     'use_sim_time': use_sim_time}]))

    if arg('rviz').lower() == 'true':
        rviz_cfg = os.path.join(get_package_share_directory('luna_sim'), 'rviz', 'sim.rviz')
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
            parameters=[{'use_sim_time': use_sim_time}]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='random',
                              description="integer, or 'random' (the chosen seed is logged)"),
        DeclareLaunchArgument('config', default_value='artemis_arena.yaml',
                              description='path, or a name in share/luna_sim/config/arena/'),
        DeclareLaunchArgument('out_dir', default_value='',
                              description="output folder; '' = <src>/luna_sim/generated"),
        DeclareLaunchArgument('resolution', default_value='0.05', description='meters per cell'),
        DeclareLaunchArgument('publish_truth', default_value='true',
                              description='also serve the full map on /map_truth'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when a /clock source is running'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=_setup),
    ])