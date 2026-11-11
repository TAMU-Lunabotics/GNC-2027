"""
Run the Level 1 simulator.

ros2 launch luna_sim level1.launch.py  # random seed (logged)
ros2 launch luna_sim level1.launch.py seed:=42
ros2 launch luna_sim level1.launch.py seed:=42 rtf:=5.0 rviz:=false  # five times real time
ros2 launch luna_sim level1.launch.py oracle_localization:=false  # use your own localization
ros2 launch luna_sim level1.launch.py obstacle_sensor:=false  # walls only

The localization default can change once real localization is available.
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
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def is_true(name):
        return arg(name).lower() == 'true'

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
    # 6. Mock wheel encoders and odometry
    if is_true('mock_wheel_odometry'):
        actions.append(Node(
            package='luna_sim',
            executable='mock_wheel_odometry',
            name='mock_wheel_odometry',
            output='screen',
            parameters=[
                os.path.join(
                    sim_share, 'config', 'sensors',
                    'mock_wheel_odometry.yaml'
                ),
                sim_time,
                {
                    'geometry_yaml': os.path.join(
                        desc_share, 'config', 'robot_geometry.yaml'
                    ),
                    'seed': seed,
                },
            ],
        ))

    # Simulated AprilTag cameras
    if is_true('mock_tag_detector'):
        actions.append(Node(
            package='luna_sim',
            executable='mock_tag_detector',
            name='mock_tag_detector',
            output='screen',
            parameters=[
                os.path.join(sim_share, 'config', 'sensors',
                             'mock_tag_detector.yaml'),
                sim_time,
                {
                    'arena_json': str(paths['json']),
                    'geometry_yaml': os.path.join(
                        desc_share, 'config', 'robot_geometry.yaml'
                    ),
                    'seed': seed,
                },
            ],
        ))

    # 7. Localization: the oracle, unless someone is testing a real estimator.
    for name in ('mock_excavation', 'mock_dump'):
        if is_true(name):
            actions.append(Node(
                package='luna_sim',
                executable=name,
                name=name,
                output='screen',
                parameters=[
                    os.path.join(sim_share, 'config', 'actuators', f'{name}.yaml'),
                    sim_time,
                    {'seed': seed},
                ],
            ))
    # Simulated IMU
    if is_true('imu_sensor'):
        actions.append(Node(
            package='luna_sim',
            executable='mock_imu',
            name='mock_imu',
            output='screen',
            parameters=[
                os.path.join(sim_share, 'config', 'sensors', 'mock_imu.yaml'),
                sim_time,
                {
                    'geometry_yaml': os.path.join(
                        desc_share, 'config', 'robot_geometry.yaml'
                    ),
                    'seed': seed,
                },
            ],
        ))

    # 6. Localization: the oracle, unless someone is testing a real estimator.
    if is_true('oracle_localization'):
        actions.append(Node(package='luna_sim', executable='oracle_localizer',
                            name='oracle_localizer', output='screen', parameters=[sim_time]))
    else:
        actions.append(LogInfo(msg='[level1] oracle_localization:=false -> nothing publishes '
                                   'map->odom->base_link until your localization runs'))

    # 8. RViz with the saved layout.
    if is_true('rviz'):
        rviz_cfg = os.path.join(sim_share, 'rviz', 'sim.rviz')
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
            additional_env={'QT_ENABLE_HIGHDPI_SCALING': '0'},
            parameters=[sim_time]))

    # 8. Mock perception: ground truth -> /map_updates, stands in for lidar -> obstacle_detector.
    if is_true('obstacle_sensor'):
        sensors = os.path.join(sim_share, 'config', 'sensors')
        parameters = [
            sim_time,
            os.path.join(sensors, 'mock_obstacle_detector.yaml'),
            {'arena_json': str(paths['json']), 'seed': seed},
        ]
        if arg('obstacle_detector_profile') == 'round2':
            parameters.append(os.path.join(sensors, 'mock_obstacle_detector_round2.yaml'))
        actions.append(Node(
            package='luna_sim', executable='mock_obstacle_detector',
            name='mock_obstacle_detector', output='screen',
            parameters=parameters))
    else:
        actions.append(LogInfo(msg='[level1] obstacle_sensor:=false -> nothing publishes '
                                   '/map_updates; the planner sees walls only'))

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
        DeclareLaunchArgument('obstacle_sensor', default_value='true',
                              description='mock lidar obstacle detection -> /map_updates'),
        DeclareLaunchArgument(
            'obstacle_detector_profile', default_value='default', choices=['default', 'round2'],
            description='Select the mock obstacle detector profile',
        ),
        DeclareLaunchArgument(
            'mock_wheel_odometry',
            default_value='true',
            description='Enable simulated wheel encoder odometry'
        ),
        DeclareLaunchArgument('mock_tag_detector', default_value='true'),
        DeclareLaunchArgument('mock_excavation', default_value='true'),
        DeclareLaunchArgument('mock_dump', default_value='true'),
        DeclareLaunchArgument('imu_sensor', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
