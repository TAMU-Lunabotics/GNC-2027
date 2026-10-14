"""Level 1 sim

  ros2 launch luna_sim level1.launch.py                     # random seed (logged)
  ros2 launch luna_sim level1.launch.py seed:=42
  ros2 launch luna_sim level1.launch.py seed:=42 rtf:=5.0 rviz:=false #runs at x5 speed
  ros2 launch luna_sim level1.launch.py oracle_localization:=false   # bring your own localization, we can flip the default later when localization exists
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from luna_sim.arena.arena_gen import generate_map
from luna_sim.arena.paths import arena_paths, default_out_dir, resolve_config, resolve_seed


def _setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    is_true = lambda name: arg(name).lower() == 'true'

    sim_share = get_package_share_directory('luna_sim')
    desc_share = get_package_share_directory('luna_description')

    seed = resolve_seed(arg('seed'))
    config = resolve_config(arg('config'))
    out_dir = arg('out_dir') or str(default_out_dir())
    resolution = float(arg('resolution'))
    generate_map(config, seed, out_dir, resolution)
    paths = arena_paths(out_dir, seed)

    sim_time = {'use_sim_time': True}

    actions = [
        LogInfo(msg=f'[level1] seed={seed}  arena={paths["json"]}'),

        # 2. Time source. Everything else runs on /clock.
        Node(package='luna_sim', executable='sim_clock', name='sim_clock', output='screen',
             parameters=[{'use_sim_time': False, 'real_time_factor': float(arg('rtf'))}]),

        # 3. Maps: serve the files generated above (generate:=false, so no second generation).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'arena.launch.py')),
            launch_arguments={
                'seed': str(seed),
                'config': str(config),
                'out_dir': out_dir,
                'generate': 'false',
                'use_sim_time': 'true',
                'rviz': 'false',
            }.items()),

        # 4. Robot description: base_link -> chassis, wheels, sensor frames.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_share, 'launch', 'description.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),

        # 5. The simulated robot: /cmd_vel -> true pose.
        Node(package='luna_sim', executable='kinematic_base', name='kinematic_base',
             output='screen',
             parameters=[sim_time, {'arena_json': str(paths['json'])}]),
    ]

    # 6. Localization: the oracle, unless someone is testing a real estimator.
    if is_true('oracle_localization'):
        actions.append(Node(package='luna_sim', executable='oracle_localizer',
                            name='oracle_localizer', output='screen', parameters=[sim_time]))
    else:
        actions.append(LogInfo(msg='[level1] oracle_localization:=false -> nothing publishes '
                                   'map->odom->base_link until your localization runs'))

    # 7. RViz with the saved layout.
    if is_true('rviz'):
        rviz_cfg = os.path.join(sim_share, 'rviz', 'sim.rviz')
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
            additional_env={'QT_ENABLE_HIGHDPI_SCALING': '0'}, 
            parameters=[sim_time]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='random',
                              description="integer, or 'random' (the chosen seed is logged)"),
        DeclareLaunchArgument('config', default_value='artemis_arena.yaml',
                              description='path, or a name in share/luna_sim/config/arena/'),
        DeclareLaunchArgument('out_dir', default_value='',
                              description="'' = luna_sim/generated"),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('rtf', default_value='1.0',
                              description='real-time factor; >1 runs faster than real time'),
        DeclareLaunchArgument('oracle_localization', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_setup),
    ])