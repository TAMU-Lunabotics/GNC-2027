from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _launch_guidance(context, config):
    planner_yaml = config / 'planner.yaml'
    with planner_yaml.open() as stream:
        planner_params = yaml.safe_load(stream)['path_planner']['ros__parameters']
    controller_type = LaunchConfiguration('controller_type').perform(context)
    footprint_override = LaunchConfiguration('footprint_mode').perform(context)
    default_footprint = ('circle' if controller_type == 'pure_pursuit'
                         else planner_params.get('footprint_mode', 'circle'))
    footprint_mode = footprint_override or default_footprint
    if footprint_mode == 'hybrid' and controller_type == 'pure_pursuit':
        raise ValueError('Set footprint_mode:=circle to launch regular pure pursuit')

    parameters = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'use_map_updates': ParameterValue(LaunchConfiguration('use_map_updates'), value_type=bool),
        'footprint_mode': footprint_mode,
    }
    controller_parameters = {**parameters, 'controller_type': controller_type}
    for name in ('map_frame', 'robot_radius_m', 'robot_length_m', 'robot_width_m',
                 'heading_error_deg', 'occupied_threshold'):
        if name in planner_params:
            controller_parameters[name] = planner_params[name]

    return [
        Node(
            package='luna_guidance',
            executable='guidance_controller',
            name='guidance_controller',
            output='screen',
            parameters=[str(config / 'guidance.yaml'), controller_parameters],
        ),
        Node(
            package='luna_guidance',
            executable='path_planner',
            name='path_planner',
            output='screen',
            parameters=[str(planner_yaml), parameters],
        ),
    ]


def generate_launch_description():
    config = Path(get_package_share_directory('luna_guidance')) / 'config'

    return LaunchDescription([
        DeclareLaunchArgument('controller_type', default_value='regulated_pure_pursuit',
                              choices=['regulated_pure_pursuit', 'pure_pursuit']),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when running with the sim clock'),
        DeclareLaunchArgument('use_map_updates', default_value='false',
                              description='Use Level 1 simulator occupancy patches'),
        DeclareLaunchArgument('footprint_mode', default_value='', choices=['', 'circle', 'hybrid'],
                              description='Override the footprint mode in planner.yaml'),
        OpaqueFunction(function=_launch_guidance, args=[config]),
    ])
